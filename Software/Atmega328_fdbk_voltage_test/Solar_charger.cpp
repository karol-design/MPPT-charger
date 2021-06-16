/*  Name: Solar_charger.cpp
 *  Description: MPPT Solar Charger board with Bluetooth communication | Board library
 *  Author: Karol Wojsław
 *  Date: 08/06/2021 (last release)
 */

#include <Adafruit_MCP4725.h> // Current sensor library
#include "Solar_charger.h"    // Library header file

/* ------------------------------- */
/* Solar_charger class constructor */
Solar_charger::Solar_charger() {
  // Input-output pin mode definition
  pinMode(BAT_VOLT_PIN, INPUT);
  pinMode(PV_VOLT_PIN, INPUT);
  pinMode(PV_CURRENT_PIN, INPUT);
  pinMode(LM_ENABLE_PIN, OUTPUT);
  pinMode(BT_TX_PIN, INPUT);
  pinMode(BT_RX_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Output state change to default on startup
  digitalWrite(LM_ENABLE_PIN, HIGH);  // Turn off LM2576 switching off buck converter by default
  digitalWrite(BUZZER_PIN, LOW);      // Turn off buzzer by default
  digitalWrite(LED_PIN, LOW);         // Turn off LED indicator by default
}

/* ------------------------------------------------------------------------------------- */
/* Method that measure average of n readings from the input_pin and returns value in mV. */
int Solar_charger::get(int command) {
  if(command == 1) {
    input_pin = BAT_VOLT_PIN;
    V_div_R1 = R4;
    V_div_R2 = R5;
  } else if(command == 2) {
    input_pin = PV_VOLT_PIN;
    V_div_R1 = R2;
    V_div_R2 = R3;
  } else if(command == 3) {
    input_pin = PV_CURRENT_PIN;
    V_div_R1 = 0; // No voltage divider for current sensor
    V_div_R2 = 1;
  } else {
    return -1; // Return error (-1) if command is undefined
  }
  
  int n = 100; // Number of measurements
  unsigned long readings_avr_mV, Vin_avr_mV, readings_sum = 0UL;

  for(int i = 0; i < n; i++) { // Take n measurements
    readings_sum += analogRead(input_pin);  // Add a 10-bit (0-1024) value read to readings_sum
    delay(5); //Atmega328 takes ~0.1 ms to read an analog val. Wait 5 ms between readings for better performance 
  }
  readings_avr_mV = map((readings_sum / n), 0, 1024, 0, Vin_mV);
  Vin_avr_mV = (readings_avr_mV *((long)V_div_R1 + V_div_R2))/(V_div_R2);

  return Vin_avr_mV; //Return the average of n readings in mV
}


/* ----------------------------------------------------------------------- */
/* Method that change the charging voltage (LM2576 Vout) to a given value  */
void Solar_charger::setCharging(unsigned long Voltage_mV) {
  Adafruit_MCP4725 MCP4725_DAC;       // Create MCP4725_DAC object
  MCP4725_DAC.begin(0x60);            // MCP4725 digital DAC I2C configuration (Adress: 0x60)
  delay(200);

  MCP4725_DAC.setVoltage(Voltage_mV, false); //12-bit  resolution: values between 0 - 4095;  1.263mV per LSB (example for Vin 5.17 V)
}


/* ------------------------------------------------- */
/* Method that blink with LED indicator on the board */
void Solar_charger::ledBlink() {
  for(int i = 0; i < 3; i++) { // Blink 3 times with LED indicator
    digitalWrite(LED_PIN, HIGH);  
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
}


/* -------------------------------------------------------------- */
/* Method that turn on and off the buzzer on the board three times*/
void Solar_charger::buzzer() {
  for(int i = 0; i < 3; i++) { // Blink 3 times with LED indicator
    tone(BUZZER_PIN, 1000, 250); 
    delay(500);
  }
}


/* --------------------------------------------- */
/* Method for turning on/off the LM Switching IC */
void Solar_charger::LM2576(int state){
  digitalWrite(LM_ENABLE_PIN, state); // Turn on/off LM2576
}