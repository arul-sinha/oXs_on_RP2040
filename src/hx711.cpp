#include "hx711.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "tools.h"
#include <math.h>
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
//static float knownMassGrams = 500.0f; // 500 grams of known mass attache. CHANGE THIS

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

float hx711Calibrate(float knownMassGrams) {
    // pray this never happens
    if (knownMassGrams <= 0.0f) return 0.0f;

    // averages out 5 readings
    int32_t readings[5];
    for (int i = 0; i < 5; i++) {
        readings[i] = hx711GetRaw();
        if (i < 4) sleep_ms(250);
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

// another useless ass function - incomplete tare
/*
float fetchDiff(float knownmass) {
    float current_reading = hx711GetCalibrationFactor();

    float expected_reading = 9.81f * powf(6371.0f / (6371.0f + 0.984f), 2.0f) * knownmass;
    float diff = fabs(current_reading - expected_reading);  

    return diff;
}
*/