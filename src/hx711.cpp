#include "hx711.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "tools.h"
#include <math.h>

// ------------------------------------------------------------------
// GPIO pins used by the HX711.
// Set these 2 variables to the pins you wired (any free GPIO 0..29).
// They stay as simple variables so you can change them at build time
// (or from a test) without touching the rest of the firmware.
// ------------------------------------------------------------------

// CHANGE THISSSS
int hx711DoutPin = 21;   // data output from the HX711 to the RP2040 (needs change)
int hx711SckPin  = 20;   // serial clock from the RP2040 to the HX711 (needs changeS)

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
    // Send the RAW count to core0.
    // We reuse RESERVE6 because no telemetry tables need to be changed.
    sent2Core0(RESERVE6, raw);
}

// 0 means sensor disconnected or there's no weight / wheight (i cant spell)
// waits 0.2 s for the scale to be ready, if not calibration will fail..
int32_t hx711GetRaw(void) {
    if (hx711DoutPin > 29 || hx711SckPin > 29) return 0;
    return hx711ReadRaw();
}

float hx711Calibrate(float knownMassGrams) {
    if (knownMassGrams <= 0.0f) return 0.0f;

    // 1. take 5 readings, one every 250 ms (~1 s total)
    int32_t readings[5];
    for (int i = 0; i < 5; i++) {
        readings[i] = hx711GetRaw();
        if (i < 4) sleep_ms(250);
    }

    // 2. middle value (median) = reference for spotting outliers
    int32_t sorted[5];
    for (int i = 0; i < 5; i++) sorted[i] = readings[i];
    for (int i = 1; i < 5; i++) {
        int32_t key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; j--; }
        sorted[j + 1] = key;
    }
    int32_t median = sorted[2];

    // 3. keep only readings close to the median, drop the others
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

    // 4. average of the kept readings
    float average = sum / (float) kept;

    // 5. grams -> kg -> force in Newtons (at 984 m)
    float massKg = knownMassGrams / 1000.0f;
    float force  = 9.81f * powf(6371.0f / (6371.0f + 0.984f), 2.0f) * massKg;
    if (force <= 0.0f || average == 0.0f) return 0.0f;

    // 6. factor = raw counts per Newton
    return fabsf(average) / force;
}