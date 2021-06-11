/*  Name: Solar_charger.h
 *  Description: MPPT Solar Charger board with Bluetooth communication | Board library
 *  Author: Karol Wojsław
 *  Date: 08/06/2021 (last release)
 */

#ifndef Solar_charger_h
#define Solar_charger_h

#include <Arduino.h> // Include standard Arduino header file

/* Define pin allocations of Atmega328 on MPPT Solar Charger board */
#define BAT_VOLT_PIN    A0  // Bat voltage on PC0
#define PV_VOLT_PIN     A1  // PV voltage on PC1
#define PV_CURRENT_PIN  A2  // PV current on PC2
#define LM_ENABLE_PIN   4   // LM enable on PD4
#define BT_TX_PIN       5   // Bluetooth TX (Transmit) on PD5
#define BT_RX_PIN       6   // Bluetooth RX (Receive) on PD6
#define BUZZER_PIN      8   // Buzzer on PB0
#define LED_PIN         9   // LED indicator on PB1

/* Define command number for use with get method */
#define BAT_VOLTAGE     1   // Measuring Bat voltage is associated with command 1
#define PV_VOLTAGE      2   // Measuring PV voltage is associated with command 2
#define PV_CURRENT      3   // Measuring PV current is associated with command 3

/* Define LM2576 operation modes */
#define ON  0 // LM2576 is on when the OFF pin is grounded
#define OFF 1 // LM2576 is turned off when the OFF pin is abouve the threshold voltage

/* Fill in values below after callibration for improved precision of measurements and charger performance */
#define Vin_mV 5090       // Atmega328 supply voltage in mV - increased by 25 mV for more precise calculations (systematic error)
#define PV_cur_Vofset 355 // Voltage in mV to be subtracted from PV_current sensor readings (systematic error)
#define R2  4680          // R2 resistance in Ohms
#define R3  976           // R2 resistance in Ohms
#define R4  4790          // R4 resistance in Ohms
#define R5  976           // R5 resistance in Ohms

class Solar_charger {
  public:
    Solar_charger();
    int get(int command);
    void setCharging(unsigned long Voltage_mV);
    void ledBlink();
    void buzzer();
    void LM2576(int state);

  private:
    int input_pin, V_div_R1, V_div_R2, n;
    unsigned long readings_avr_mV, Vin_avr_mV, readings_sum;
};

#endif