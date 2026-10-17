#include "hx711.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "tools.h"

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
