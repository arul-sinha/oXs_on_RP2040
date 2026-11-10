#include "hx711.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "tools.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"

// Reserved 4 KB sector: config at +0, sequencer at +4K, gyro at +8K, this at +12K.
#define CAL_FLASH_OFFSET  (256 * 1024 + 12 * 1024)
#define CAL_MAGIC         0x48583731u

typedef struct {
    uint32_t magic;    // marks the record as valid
    float    factor;   // the value you want to keep
} calRecord_t;

extern bool multicoreIsRunning;   // defined in main.cpp

// filled on core0 from the receiver frames (defined in sbus_out_pwm.cpp)
extern uint16_t rcChannelsUs[16];
// timestamp of the last RC channels frame (defined in crsf_in.cpp)
extern uint32_t lastRcChannels;

// Normally core1 reads the HX711 (hx711Read() in getSensors()).
// During a calibration, core0 must drive the HX711 pins itself; core1 is then
// asked to skip the readings (hx711HoldReads) and confirms it by incrementing
// hx711HoldAck each time it skips one.
static volatile bool     hx711HoldReads = false;
static volatile uint32_t hx711HoldAck   = 0;

static float calFactor = 0.0f;    // 0.0f means "not calibrated yet"

float hx711LoadCalibrationFactor(void) {
    calRecord_t r;
    memcpy(&r, (const void *) (XIP_BASE + CAL_FLASH_OFFSET), sizeof(r));

    if (r.magic == CAL_MAGIC && isfinite(r.factor) && r.factor > 0.0f) {
        calFactor = r.factor;      // valid saved value
    } else {
        calFactor = 0.0f;          // blank or corrupted -> not calibrated
    }
    return calFactor;
}

/* Useless ass function:
float hx711GetCalibrationFactor(void) {
    return calFactor;
}
*/

bool hx711SaveCalibrationFactor(float factor) {
    if (!isfinite(factor) || factor <= 0.0f) return false;   // refuse bad values
    if (factor == calFactor) return true;                    // unchanged -> don't erase

    calRecord_t r = { CAL_MAGIC, factor };

    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));        // erased flash reads as 0xFF
    memcpy(page, &r, sizeof(r));

    watchdog_enable(3000, true);             // cover the erase time
    if (multicoreIsRunning) multicore_lockout_start_blocking();  // pause core1
    uint32_t irq = save_and_disable_interrupts();                // no ISR from flash
    flash_range_erase(CAL_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CAL_FLASH_OFFSET, page, FLASH_PAGE_SIZE);
    restore_interrupts(irq);
    if (multicoreIsRunning) multicore_lockout_end_blocking();

    calFactor = factor;
    return true;
}

// ------------------------------------------------------------------
// GPIO pins used by the HX711.
// Set these 2 variables to the pins you wired (any free GPIO 0..29).
// They stay as simple variables so you can change them at build time
// (or from a test) without touching the rest of the firmware.
// ------------------------------------------------------------------

int hx711DoutPin = 22;   // data output from the HX711 to the RP2040 22
int hx711SckPin  = 20;   // serial clock from the RP2040 to the HX711 26
// The known mass used for the calibration is now HX711_KNOWN_MASS_G in hx711.h

// Channel/gain: raw read is done with 24 bits + 1 extra SCK pulse
// (channel A, gain 128) -> the 25th pulse sets the gain for next read.
static int32_t hx711ReadRaw(void) {
    uint32_t value = 0;

    // Wait for DOUT to go low (HX711 has a new conversion ready).
    uint32_t startUs = microsRp();
    while (gpio_get(hx711DoutPin)) {
        if ((microsRp() - startUs) > 200000) {  // 200 ms timeout -> sensor not ready/disconnected
            return 0;
        }
    }

    // Read the 24 bits, MSB first.
    for (uint8_t i = 0; i < 24; i++) {
        gpio_put(hx711SckPin, 1);
        sleep_us(1);
        value <<= 1;
        if (gpio_get(hx711DoutPin)) {
            value |= 1;
        }
        gpio_put(hx711SckPin, 0);
        sleep_us(1);
    }

    // One extra SCK pulse -> selects channel A, gain 128 for the next read.
    gpio_put(hx711SckPin, 1);
    sleep_us(1);
    gpio_put(hx711SckPin, 0);
    sleep_us(1);

    // Convert the 24-bit two's complement value to a signed 32-bit int.
    if (value & 0x800000) {
        value |= 0xFF000000;
    }
    return (int32_t) value;
}

void hx711Setup(void) {
    hx711LoadCalibrationFactor();  // load from non-volatile memory at boot

    if (hx711DoutPin > 29 || hx711SckPin > 29) {
        return;  // disabled / invalid pins
    }
    gpio_init(hx711DoutPin);
    gpio_set_dir(hx711DoutPin, GPIO_IN);

    gpio_init(hx711SckPin);
    gpio_set_dir(hx711SckPin, GPIO_OUT);
    gpio_put(hx711SckPin, 0);  // power-off pulse to start; DOUT low = ready
}

void hx711Read(void) {
    if (hx711DoutPin > 29 || hx711SckPin > 29) {
        return;
    }

    if (hx711HoldReads) {     // core0 is running a calibration and drives the HX711:
        hx711HoldAck++;       // keep hands off the pins and confirm that we did it
        return;
    }

    int32_t raw = hx711ReadRaw();
    if (calFactor > 0.0f) {
        raw = (int32_t)(fabsf((float)raw) / calFactor);
    }
    // Send the RAW count to core0.
    // We reuse RESERVE6 because no telemetry tables need to be changed.
    sent2Core0(RESERVE6, raw);
}

// returns weight / wheight in newtons (i cant spell)
// waits 0.2 s for the scale to be ready, if not calibration will fail..
int32_t hx711GetRaw(void) {
    if (hx711DoutPin > 29 || hx711SckPin > 29) return 0;
    return hx711ReadRaw();
}

// sleep while still feeding the watchdog: the main loop on core0 is blocked
// while the calibration runs (this function is called by hx711Calibrate).
static void calDelayMs(uint32_t ms) {
    uint32_t startMs = millisRp();
    while ((millisRp() - startMs) < ms) {
        watchdog_update();
        sleep_ms(25);
    }
}

float hx711Calibrate(float knownMassGrams) {
    // pray this never happens
    if (knownMassGrams <= 0.0f) return 0.0f;

    // averages out 5 readings
    int32_t readings[5];
    for (int i = 0; i < 5; i++) {
        readings[i] = hx711GetRaw();
        if (i < 4) calDelayMs(250);   // sleep_ms but feeding the watchdog (loop() is blocked here)
    }

    // send bad readings to the trash
    int32_t sorted[5];
    for (int i = 0; i < 5; i++) sorted[i] = readings[i];
    for (int i = 1; i < 5; i++) {
        int32_t key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; j--; }
        sorted[j + 1] = key;
    }
    int32_t median = sorted[2];

    float limit = 0.10f * fabsf((float) median);   // "a lot" = more than 10%
    if (limit < 2000.0f) limit = 2000.0f;          // with a minimum of 2000 counts

    float sum = 0.0f;
    int kept = 0;
    for (int i = 0; i < 5; i++) {
        if (fabsf((float) (readings[i] - median)) <= limit) {
            sum += (float) readings[i];
            kept++;
        }
    }
    if (kept == 0) return 0.0f;

    // average of the kept readings
    float average = sum / (float) kept;

    // grams -> kg -> force in Newtons (at 984 m) basically taring
    float massKg = knownMassGrams / 1000.0f;
    float force  = 9.81f * powf(6371.0f / (6371.0f + 0.984f), 2.0f) * massKg;
    if (force <= 0.0f || average == 0.0f) return 0.0f;

    // factor = measeured wheight / optimal wheight
    float factor = fabsf(average) / force;
    if (!isfinite(factor) || factor <= 0.0f) return 0.0f;

    // Save to non-volatile memory and use immediately
    if (hx711SaveCalibrationFactor(factor)) {
        return factor;
    }

    // If save failed for some reason, still apply in memory
    calFactor = factor;
    return factor;
}

// ------------------------------------------------------------------
// Calibration requested from the transmitter (FrSky ETHos, ...).
//
// The only data a radio can send to the oXs are the RC channels, so a spare
// channel (HX711_CAL_CHANNEL) is used: when the switch bound to that channel
// is held "up" during HX711_CAL_HOLD_MS, the calibration starts. The switch
// has to go back "down" before a new calibration can be started.
// ------------------------------------------------------------------

// Runs on core0 when the user asked for a calibration from the transmitter.
// (implementation below)
static void hx711RunCalibration(void);

// Called from loop() on core0, once the RC channels are in rcChannelsUs[].
void hx711CheckCalibrationTrigger(void) {
    static uint32_t highSinceMs = 0;  // 0 = switch not (yet) in the "up" position
    static bool     armed       = false; // true once the switch has been seen "down"

    if (hx711DoutPin > 29 || hx711SckPin > 29) return; // HX711 is disabled
    if ((HX711_CAL_CHANNEL < 1) || (HX711_CAL_CHANNEL > 16)) return;

    // keep a fresh RC frame (skip e.g. when the receiver is in failsafe)
    if ((millisRp() - lastRcChannels) > 200) {
        highSinceMs = 0;   // reset the hold time
        return;
    }

    uint16_t chUs = rcChannelsUs[HX711_CAL_CHANNEL - 1];

    if (chUs >= HX711_CAL_HIGH_US) {              // switch is "up"
        if (highSinceMs == 0) {
            highSinceMs = millisRp();              // start measuring the hold time
        } else if (armed && ((millisRp() - highSinceMs) >= HX711_CAL_HOLD_MS)) {
            armed = false;                         // no new start while it stays up
            hx711RunCalibration();
        }
    } else {                                       // switch is not "up" (middle or down)
        highSinceMs = 0;                           // reset the hold time
        armed = true;                              // ready to trigger once it goes up again
    }
}

// Runs on core0 when the user asked for a calibration from the transmitter.
// core1 normally reads the HX711 (hx711Read()); it is asked to skip the
// readings while core0 executes the calibration. hx711Calibrate() takes the
// readings, calculates the factor and stores it in flash (the flash write must
// be done by core0 because core1 is the multicore lockout victim).
// This blocks the main loop for about 1.5 sec.
static void hx711RunCalibration(void) {
    printf("\nHX711 : calibration requested from the transmitter\n");
    printf("HX711 : keep the %.0f g reference mass on the load cell\n", (double) HX711_KNOWN_MASS_G);

    // Ask core1 to stop reading the HX711 and wait for the acknowledge.
    uint32_t ackBefore = hx711HoldAck;
    hx711HoldReads = true;
    uint32_t startMs = millisRp();
    while (hx711HoldAck == ackBefore) {
        watchdog_update();
        if ((millisRp() - startMs) > 500) break;   // core1 did not confirm within 500 msec
    }

    float factor = hx711Calibrate(HX711_KNOWN_MASS_G); // reads, computes and saves the factor

    hx711HoldReads = false;   // core1 may read the HX711 again

    if (factor > 0.0f) {
        printf("HX711 : calibration done; factor = %.6f (stored in flash)\n", (double) factor);
    } else {
        printf("HX711 : calibration failed (check the wiring and the load cell)\n");
    }
}

// another useless ass function - incomplete tare
/*
float fetchDiff(float knownmass) {
    float current_reading = hx711GetCalibrationFactor();

    float expected_reading = 9.81f * powf(6371.0f / (6371.0f + 0.984f), 2.0f) * knownmass;
    float diff = fabs(current_reading - expected_reading);  

    return diff;
}
*/