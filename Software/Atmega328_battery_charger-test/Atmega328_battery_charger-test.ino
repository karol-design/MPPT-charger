/*  Name: MPPT Solar Charger with Bluetooth communication | Battery charger test code
 *  Author: Karol Wojsław
 *  Date: 20/04/2021 (last release)
 *  Description: For test purposes only
 */ 

#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

/*Define time intervals for different loops */
#define timeMain 300000UL //Time between main loop iterations (300000 ms = 5 minutes)

/*Define pin allocations on Arduino board */
#define BAT_VOLT_PIN    A0
#define PV_VOLT_PIN     A1
#define PV_CURRENT_PIN  A2
#define LM_ENABLE_PIN   4
#define BT_TX_PIN       5
#define BT_RX_PIN       6

/*Fill in values below after callibration for improved precision of measurements and charger performance */
#define Vin_mV 5020 //Atmega328 supply voltage in mV - increased by 25 mV for more precise calculations (systematic error)
#define Vout_float 170 //Voltage that has to be set on FDBK pin to obtain float mode output voltage (13.70 V)
#define Vout_bulk 80 //Voltage that has to be set on FDBK pin to obtain float mode output voltage (14.65 V)
#define R3  5020 
#define R4  930
#define R5  5070
#define R6  980

Adafruit_MCP4725 MCP4725_DAC; //Create MCP4725_DAC object

unsigned int n = 1; //Variable with current report issue


void setup(){
  //Input-output pin mode definition
  pinMode(BAT_VOLT_PIN, INPUT);
  pinMode(PV_VOLT_PIN, INPUT);
  pinMode(PV_CURRENT_PIN, INPUT);
  pinMode(LM_ENABLE_PIN, OUTPUT);
  pinMode(BT_TX_PIN, INPUT);
  pinMode(BT_RX_PIN, OUTPUT);

  //Output state change to default on startup
  digitalWrite(LM_ENABLE_PIN, HIGH); //Turn off LM2576 switching buck converter by default

  SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN); //Define Software serial port for Bluetooth
  btSerial.begin(9600); //Serial port configuration (baud rate: 9600)
  Serial.begin(9600); //Standard serial port configuration (baud rate: 9600)

  MCP4725_DAC.begin(0x60); //MCP4725 digital DAC I2C configuration (Adress: 0x60)
  SetCharging(Vout_float); //Set the charging voltage to 13.70 V by default (170 mV on FDBK pin)
}


void loop(){
  //Variables declaration
  unsigned long BAT_voltage, PV_current; 
  bool Bulk = false;

  /* Main loop of the code set to regularly check state of charge of the battery */
  while(true){
    Serial.println("Begin main while loop");
    /* Check state of charge of the battery and decide if Bulk or Float charging mode should be executed */
    digitalWrite(LM_ENABLE_PIN, HIGH); //Turn off LM2576 for measurements
    Serial.println("Waiting 30 sec before Bat voltage measurement");
    delay(30000); //Wait 30 seconds
    BAT_voltage = measureAverage(BAT_VOLT_PIN, 100, R5, R6) - 200; //Decrease measured Bat voltage by 200 mV (Bat voltage higher immediatly after charging)
    if(BAT_voltage < 13000){
      Bulk = true; //Set Bulk to 1 if the Battery voltage is lower than 13.0V
    }else{
      Bulk = false; //Set Bulk to 0 if the Battery voltage is higher than 13.0V
    }

    Serial.println("Battery voltage: ");
    Serial.println(BAT_voltage);
 
    /* ------------------------------------------------------------ */
    /* ---------------- Enter bulk charging mode -------------------*/
    while(Bulk){ //While Bulk == 1...
      Serial.println("Mode: Bulk");
      SetCharging(Vout_bulk); //Set the charging voltage to default Bulk voltage (14.65 V)
      digitalWrite(LM_ENABLE_PIN, LOW); //Turn on LM2576
      PV_current = (((measureAverage(PV_CURRENT_PIN, 250, 0, 1)-215)*4)/5); //No voltage div, hence R1=0 & R2=1, 1 mA = 0.8 mV
      if(PV_current < 550){
        Serial.print("BAT_current ");
        Serial.println(PV_current);
      }      
      if(PV_current < 250)
        Bulk = false;
    }

    /* ------------------------------------------------------------ */
    /* -------------- Enter float-storage charging mode ----------- */
    Serial.println("Mode: Float");
    SetCharging(Vout_float); //Set the charging voltage to default Bulk voltage (13.70 V)
    digitalWrite(LM_ENABLE_PIN, LOW); //Turn on LM2576
    PV_current = (((measureAverage(PV_CURRENT_PIN, 250, 0, 1)-215)*4)/5); //No voltage div, hence R1=0 & R2=1, 1 mA = 0.8 mV
    Serial.print("PV_current ");
    Serial.println(PV_current);

    delay(timeMain);
  }
}


/* -------------------------------------------------------------------------------------------------------------- */
/* Function that measure average of n readings from the input_pin and returns value in mV. Execution time:~10*n ms */
int measureAverage(int input_pin, int n, int V_div_R1, int V_div_R2){
  unsigned long readings_avr_mV, Vin_avr_mV, readings_sum = 0UL;
  for(int i = 0; i < n; i++){ //Take n measurements
    readings_sum += analogRead(input_pin); //Add a 10-bit (0-1024) value read to readings_sum
    delay(5); //Atmega328 takes ~0.1 ms to read an analog val. Wait 5 ms between readings for better performance 
  }
  readings_avr_mV = map((readings_sum / n), 0, 1024, 0, Vin_mV);
  Vin_avr_mV = (readings_avr_mV *((long)V_div_R1 + V_div_R2))/(V_div_R2);
  return Vin_avr_mV; //Return the average of n readings in mV
}


/* -------------------------------------------------------------------------------------------------------------- */
/* Function that change the charging voltage (LM2576 Vout) to a given value  */
void SetCharging(unsigned long Voltage_mV){
  MCP4725_DAC.setVoltage(Voltage_mV, false); //12-bit  resolution: values between 0 - 4095;  1.263mV per LSB (example for Vin 5.17 V)
  return;
}
