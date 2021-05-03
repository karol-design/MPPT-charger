/*  Name: MPPT Solar Charger with Bluetooth communication | Battery charger test code
 *  Author: Karol Wojsław
 *  Date: 20/04/2021 (last release)
 *  Description: For test purposes only. This code is intended to be used for the test of 12V battery charger. For the test follow
 *  following procedure: Upload the code --> Connet the power supply --> Connect the battery --> Establish serial connection --> 
 *  --> Turn on the logger and validate first incoming data --> Leave the charger for >10 hours -->  Save and analyse the data
 *  
 *  Before the test change: timeMain, Vin_mV, Vout_float, Vout_bulk, V_ChargingDrop, PV_cur_Vofset
 */ 

#include <Wire.h>
#include <Adafruit_MCP4725.h>

/*Define pin allocations on Arduino board */
#define BAT_VOLT_PIN    A0
#define PV_VOLT_PIN     A1
#define PV_CURRENT_PIN  A2
#define LM_ENABLE_PIN   4

/*Fill in values below after callibration for improved precision of measurements and charger performance */
#define Vin_mV 5000 //Atmega328 supply voltage in mV - increased by - mV for more precise calculations (systematic error)
#define Vout_float 100 //Voltage that has to be set on FDBK pin to obtain float mode output voltage (13.70 V)
#define Vout_bulk 55 //Voltage that has to be set on FDBK pin to obtain float mode output voltage (14.65 V)
#define V_ChargingDrop 200 //Voltage to be subtracted when measureing the actual voltage on the battery
#define PV_cur_Vofset 320 //voltage in mV to be subtracted from PV_current sensor readings (systematic error)
#define R3  5150 
#define R4  980
#define R5  5070
#define R6  980

Adafruit_MCP4725 MCP4725_DAC; //Create MCP4725_DAC object

unsigned int n = 1; //Variable with current report issue

/*Define time intervals for different loops */
const unsigned long timeMain = 600000; //Time between main loop iterations (600000 ms = 600 seconds)


void setup(){
  //Input-output pin mode definition
  pinMode(BAT_VOLT_PIN, INPUT);
  pinMode(PV_VOLT_PIN, INPUT);
  pinMode(PV_CURRENT_PIN, INPUT);
  pinMode(LM_ENABLE_PIN, OUTPUT);

  //Output state change to default on startup
  digitalWrite(LM_ENABLE_PIN, HIGH); //Turn off LM2576 switching buck converter by default

  delay(10000); 

  Serial.begin(9600); //Standard serial port configuration (baud rate: 9600)
  Serial.print("Initialising charger code...\n");

  MCP4725_DAC.begin(0x60); //MCP4725 digital DAC I2C configuration (Adress: 0x60)
  SetCharging(Vout_float); //Set the charging voltage to Vout_float by default
  
  delay(5000);
}


void loop(){
  //Variables declaration
  unsigned long CHA_voltage, BAT_voltage, PV_voltage, PV_current, currentTime, tempTime, timeDifference = 0;
  bool Bulk = false;

  Serial.print("Main loop entered...\n\n"); //Header for logged data
  Serial.print("___________________________________________________________\n"); //Header for logged data
  Serial.print("No.\tMode\tCH_V\tBAT_V\tPV_V\tPV_I\tPower\tSOC\n"); //Header for logged data

  CHA_voltage = measureAverage(BAT_VOLT_PIN, 100, R5, R6); //Measure charging voltage
  /* Check state of charge of the battery before executing while loop and decide if Bulk or Float charging mode should be entered */
  digitalWrite(LM_ENABLE_PIN, HIGH); //Turn off LM2576 for measurements
  delay(1000); //Wait 1 second before the measurement
  BAT_voltage = measureAverage(BAT_VOLT_PIN, 100, R5, R6); //Measure battery voltage
  if(BAT_voltage < 13200){
    Bulk = true; //Set Bulk to 1 if the Battery voltage is lower than 13.2V
  }else{
    Bulk = false; //Set Bulk to 0 if the Battery voltage is higher than 13.2V
  }

  PV_voltage = measureAverage(PV_VOLT_PIN, 100, R3, R4);
  PV_current = (((measureAverage(PV_CURRENT_PIN, 100, 0, 1)-PV_cur_Vofset)*4)/6);
  generateReport(CHA_voltage, BAT_voltage, PV_voltage, PV_current, Bulk); //Send report over USB serial port

  /* Main loop of the code set to regularly check state of charge of the battery */
  while(true){
    currentTime = millis(); //Save current execution time
    tempTime = currentTime; //Set tempTime to currentTime at the beginning of the loop
    timeDifference = 0; //Set timeDifference to 0
    
    /* ------------------------------------------------------------ */
    /* ---------------- Enter bulk charging mode -------------------*/
    while(Bulk == true && timeDifference < timeMain){
      SetCharging(Vout_bulk); //Set the charging voltage to default Bulk voltage (14.65 V)
      digitalWrite(LM_ENABLE_PIN, LOW); //Turn on LM2576
      PV_current = (((measureAverage(PV_CURRENT_PIN, 100, 0, 1)*42)/60)-PV_cur_Vofset); //No voltage div, hence R1=0 & R2=1, 1 mA = 0.8 mV   
      if(PV_current < 300){
        Bulk = false;
      }
      
      delay(1000);

      currentTime = millis();
      timeDifference = currentTime - tempTime;  
    }

    /* ------------------------------------------------------------ */
    /* -------------- Enter float-storage charging mode ----------- */
    while(Bulk == false && timeDifference < timeMain){
      SetCharging(Vout_float); //Set the charging voltage to default float voltage (13.70 V)

      currentTime = millis();
      timeDifference = currentTime - tempTime; 
    }

    CHA_voltage = measureAverage(BAT_VOLT_PIN, 100, R5, R6); //Measure charging voltage
    PV_voltage = measureAverage(PV_VOLT_PIN, 100, R3, R4);
    PV_current = ((((measureAverage(PV_CURRENT_PIN, 100, 0, 1)-PV_cur_Vofset)*4)/6));
    
    /* Check state of charge of the battery and decide if Bulk or Float charging mode should be executed */
    digitalWrite(LM_ENABLE_PIN, HIGH); //Turn off LM2576 for measurements

    if(Bulk){
      delay(30000); //Wait 30 seconds before the measurement if the last mode was Bulk
    }else{
      delay(5000); //Wait 5 seconds before the measurement if the last mode was Float
    }
    
    BAT_voltage = measureAverage(BAT_VOLT_PIN, 100, R5, R6) - V_ChargingDrop; //Decrease measured V_ChargingDrop (Bat voltage higher immediatly after charging)
    if(BAT_voltage < 13100){
      Bulk = true; //Set Bulk to 1 if the Battery voltage is lower than 13.1V
    }else{
      Bulk = false; //Set Bulk to 0 if the Battery voltage is higher than 13.1V
    }

    generateReport(CHA_voltage, BAT_voltage, PV_voltage, PV_current, Bulk); //Send report over USB serial port
  }
}


/* -------------------------------------------------------------------------------------------------------------- */
/* Function that measure average of n readings from the input_pin and returns value in mV. Execution time:~5*n ms */
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


/* -------------------------------------------------------------------------------------------------------------- */
/* Function that send a report on the current status of the charger through USB serial port */
void generateReport(int Cha_voltage, int BAT_voltage, int PV_voltage, int PV_current, bool Bulk){
  Serial.print(n);
  Serial.print("\t");
  Serial.print(Bulk);
  Serial.print("\t");
  Serial.print(Cha_voltage); //Charging voltage [mV]
  Serial.print("\t");
  Serial.print(BAT_voltage); //Battery panel current [mV]
  Serial.print("\t");
  Serial.print(PV_voltage); //PV panel voltage [mV]
  Serial.print("\t");
  Serial.print(PV_current); //PV panel current [mA]
  Serial.print("\t");
  Serial.print((PV_current * (long) PV_voltage) / 1000); //Power delivered [mW]
  Serial.print("\t");
  Serial.print((BAT_voltage - 11820)/13); //State of charge (SOC) [%]
  Serial.print("\n");

  n++; //Increment current report number
  return;
}
