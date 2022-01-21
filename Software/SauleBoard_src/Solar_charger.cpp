/*  Name: Solar_charger.cpp
 *  Description: SauleBoard - Charge controller with MPPT and Bluetooth | Board library
 *  Author: Karol Wojsław
 *  Date: 23/06/2021 (last release)
 */

#include <SoftwareSerial.h>   // Software serial port library
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
uint16_t Solar_charger::get(uint8_t command) {
  uint16_t V_div_R1, V_div_R2;
  uint8_t input_pin;
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
  
  uint8_t n = 100; // Number of measurements
  uint16_t Vin_avr_mV;
  unsigned long readings_avr_mV, readings_sum = 0UL;

  for(uint8_t i = 0; i < n; i++) { // Take n measurements
    readings_sum += analogRead(input_pin);  // Add a 10-bit (0-1024) value read to readings_sum
    delay(5); //Atmega328 takes ~0.1 ms to read an analog val. Wait 5 ms between readings for better performance 
  }
  readings_avr_mV = map((readings_sum / n), 0, 1024, 0, Vin_mV);
  Vin_avr_mV = (readings_avr_mV *((long)V_div_R1 + V_div_R2))/(V_div_R2);

  return Vin_avr_mV; //Return the average of n readings in mV
}


/* ----------------------------------------------------------------------- */
/* Method that change the charging voltage (LM2576 Vout) to a given value  */
uint16_t Solar_charger::setCharging(float Voltage_V) {
  Adafruit_MCP4725 MCP4725_DAC; // Create MCP4725_DAC object
  MCP4725_DAC.begin(0x60);      // MCP4725 digital DAC I2C configuration (Adress: 0x60)

  /* Set the feedback (and hence the output) voltage according to the theoretically calculated fdbk value */
  uint16_t Vbat_desired_mV = (1000.0 * Voltage_V), Vfdbk_MC4725_mV, fdbk_MC4725;
  uint16_t error, Vbat_desired_mV_corrected = Vbat_desired_mV;

  Vfdbk_MC4725_mV = (1309 - (uint16_t) ((float) Vbat_desired_mV * 0.05988)); // Calculate theoretical value of fdbk in mV
  fdbk_MC4725 = map(Vfdbk_MC4725_mV, 0, Vin_mV, 0, 4095);       // Map the value to 12 bit int (12-bit  resolution: values between 0 - 4095)
  MCP4725_DAC.setVoltage(fdbk_MC4725, false);                   // Send the value to the MCP4725 digital DAC
  Solar_charger::LM2576(ON);                                    // Turn on LM2576 voltage regulator

  delay(500);                                                   // Wait for the voltage to stabilise
  error = Vbat_desired_mV - Solar_charger::get(BAT_VOLTAGE);    // Calculate the error

  for(uint8_t i = 0; i < 5 && MOD(error) > 15; i++) {           // For error > 15 mV minimise it using "proportional controller" [Max 5 itterations]
    Vbat_desired_mV_corrected += (error / 2);                   // Add/subtract half of the error to Vbat desired
    if(Vbat_desired_mV_corrected > 6000) {                      // Protect the board from seting the voltage below 6 V and cutting off MCU power
      Vfdbk_MC4725_mV = (1309 - (uint16_t) ((float) Vbat_desired_mV_corrected * 0.05988)); // Calculate theoretical value of fdbk in mV
      fdbk_MC4725 = map(Vfdbk_MC4725_mV, 0, Vin_mV, 0, 4095); 
      MCP4725_DAC.setVoltage(fdbk_MC4725, false);
    }

    delay(500);                                                 // Wait for the voltage to stabilise
    error = Vbat_desired_mV - Solar_charger::get(BAT_VOLTAGE);  // Calculate the current error
  }

  return fdbk_MC4725;                                           // Return the last value of MC4725 fdbk value
}


/* ------------------------------------------------- */
/* Method that blink with LED indicator on the board */
void Solar_charger::ledBlink() {
  for(uint8_t i = 0; i < 3; i++) { // Blink 3 times with LED indicator
    digitalWrite(LED_PIN, HIGH);  
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
  }
}


/* --------------------------------------------------------------- */
/* Method that turn on and off the buzzer on the board three times */
void Solar_charger::buzzer() {
  tone(BUZZER_PIN, 1000, 250); 
  delay(100);
  tone(BUZZER_PIN, 1000, 250);
  delay(100);
  tone(BUZZER_PIN, 1500, 250);
}


/* --------------------------------------------- */
/* Method for turning on/off the LM Switching IC */
void Solar_charger::LM2576(uint8_t state){
  digitalWrite(LM_ENABLE_PIN, state); // Turn on/off LM2576
}


/* ------------------------------------------------------------ */
/* Method for debuging the board and reporting its current state*/
void Solar_charger::report(uint8_t serialPort, bool Bulk) {
  SoftwareSerial btModule(BT_TX_PIN, BT_RX_PIN);  // Create SoftwareSerial object and define Software serial port
  btModule.begin(9600);       // Software Serial port configuration (baud rate: 9600)

  uint16_t _BAT_voltage = Solar_charger::get(BAT_VOLTAGE);  // Calculate Battery voltage
  uint16_t _PV_voltage = Solar_charger::get(PV_VOLTAGE);    // Calculate PV panel voltage
  uint16_t _PV_current = (((Solar_charger::get(PV_CURRENT)-PV_cur_Vofset)*5)/4); // Calculate PV current (1 mA = 0.8 mV)
  static uint8_t n_serial = 1; 
  static uint8_t n_bluetooth = 1; 
  bool _Bulk = Bulk;

  if(serialPort == 1) { // Send report over built-in standard Serial Port
    Serial.print("---------------------------------------------------------------------------------\n");
    Serial.print("MPPT Solar Charger with bluetooth communication\nReport on current status #");
    Serial.print(n_serial);
    Serial.print("\n\n# PV panel illuminated:\t\t");
    Serial.print(_PV_voltage > 10000); //Print 1 if PV panel is illuminated by the sunlight (PV_V > 10.0V)
    Serial.print(" (Illuminated/Shaded)\n# Current charging mode:\t");
    if(_Bulk == 1){
      Serial.print("Bulk mode (MPPT algorithm on)\n# Battery voltage:\t\t");
    }else if(_Bulk == 0){
      Serial.print("Float mode (MPPT algorithm off)\n# Battery voltage:\t\t");
    }
    Serial.print(_BAT_voltage / (float) 1000.0, 3);
    Serial.print(" V \n# Photovoltaic panel voltage:\t");
    Serial.print(_PV_voltage / (float) 1000.0, 3);
    Serial.print(" V \n# Photovoltaic panel current:\t");
    Serial.print(_PV_current / (float) 1000.0, 3);
    Serial.print(" A \n# Power delivered by PV panel:\t");
    Serial.print(((_PV_current * (long) _PV_voltage) / 1000000.0), 3);
    Serial.print(" W \n# Battery SOC (state of charge)\t");
    int8_t bat_SOC = (_BAT_voltage - 11820)/13;
    Serial.print((bat_SOC > 0 && bat_SOC < 100) ? bat_SOC : -1);
    Serial.print(" % of full capacity \n\n");
    n_serial++; //Increment current report number
  }
  
  else if(serialPort == 2) { // Send report over bluetooth
    btModule.print("---------------------------------------------------------------------------------\n");
    btModule.print("MPPT Solar Charger with bluetooth communication\nReport on current status #");
    btModule.print(n_bluetooth);
    btModule.print("\n\n# PV panel illuminated:\t\t");
    btModule.print(_PV_voltage > 10000); //Print 1 if PV panel is illuminated by the sunlight (PV_V > 10.0V)
    btModule.print(" (Illuminated/Shaded)\n# Current charging mode:\t");
    if(_Bulk == 1){
      btModule.print("Bulk mode (MPPT algorithm on)\n# Battery voltage:\t\t");
    }else if(_Bulk == 0){
      btModule.print("Float mode (MPPT algorithm off)\n# Battery voltage:\t\t");
    }
    btModule.print(_BAT_voltage / (float) 1000.0, 3);
    btModule.print(" V \n# Photovoltaic panel voltage:\t");
    btModule.print(_PV_voltage / (float) 1000.0, 3);
    btModule.print(" V \n# Photovoltaic panel current:\t");
    btModule.print(_PV_current / (float) 1000.0, 3);
    btModule.print(" A \n# Power delivered by PV panel:\t");
    btModule.print(((_PV_current * (long) _PV_voltage) / 1000000.0), 3);
    btModule.print(" W \n# Battery SOC (state of charge)\t");
    int8_t bat_SOC = (_BAT_voltage - 11820)/13;
    btModule.print((bat_SOC > -20 && bat_SOC < 120) ? bat_SOC : -1);
    btModule.print(" % of full capacity \n\n");
    n_bluetooth++; //Increment current report number
  }
}