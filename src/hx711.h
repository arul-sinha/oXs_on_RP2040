#pragma once

void hx711Setup(void);  // configure the 2 GPIO pins used by the HX711
void hx711Read(void);   // read one raw sample from the HX711 and send it to core0
