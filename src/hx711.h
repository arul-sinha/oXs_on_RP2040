#pragma once
#include <stdint.h>

// ------------------------------------------------------------------
// HX711 user settings (edit the values here)
// ------------------------------------------------------------------

// Known reference mass used to calibrate the load cell, in grams.
// The mass must be on the load cell when the calibration is started.
// DUMMY VALUE for now: change it to the mass you really use.
#define HX711_KNOWN_MASS_G   500.0f

// RC channel (1..16, as numbered on the receiver/transmitter) that triggers
// the calibration. Bind a switch (ETHos) to this channel.
// Moving the switch up and holding it there during HX711_CAL_HOLD_MS starts
// the calibration. The switch has to leave the "up" position before a new
// calibration can be started.
#define HX711_CAL_CHANNEL    8

// Channel value (in micro sec, 988..2012) above which the switch is
// considered "up" (a normal full-up switch gives about 2000).
#define HX711_CAL_HIGH_US    1800

// Time (in msec) the switch has to stay "up" before the calibration starts.
#define HX711_CAL_HOLD_MS    1000

void hx711Setup(void);  // configure the 2 GPIO pins used by the HX711
void hx711Read(void);   // read one raw sample from the HX711 and send it to core0

int32_t hx711GetRaw(void);
float hx711Calibrate(float knownMassGrams);

float hx711LoadCalibrationFactor(void);
bool  hx711SaveCalibrationFactor(float factor);
// float hx711GetCalibrationFactor(void); USELESS FUNCTION

// Runs on core0 (called from the main loop, after the RC channels have been
// converted in micro sec). Starts the calibration when the switch on
// HX711_CAL_CHANNEL is held "up" during HX711_CAL_HOLD_MS.
void hx711CheckCalibrationTrigger(void);
