#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 MCP4725_DAC;

void setup(void){
  MCP4725_DAC.begin(0x60);
}

void loop(void) {
  //12-bit  resolution, so values can be between 0 - 4095 =  1.263mV per LSB (for USB 5.17 V)
  MCP4725_DAC.setVoltage(158, false); //200 mV
  delay(5000);
  MCP4725_DAC.setVoltage(119, false); //150 mV
  delay(5000);
  MCP4725_DAC.setVoltage(396, false); //500 mV
  delay(5000);
}
