#pragma once
#include <stdint.h>

void hx711Setup(void);  // configure the 2 GPIO pins used by the HX711
void hx711Read(void);   // read one raw sample from the HX711 and send it to core0

int32_t hx711GetRaw(void); 
float hx711Calibrate(float knownMassGrams);