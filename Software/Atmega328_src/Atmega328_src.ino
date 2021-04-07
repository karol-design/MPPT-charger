/*  Name: MPPT Solar Charger with Bluetooth communication | Source file
 *  Author: Karol Wojsław
 *  Date: 04/04/2021 (last release)
 *  Description: Code for Atmega328 microcontroller on MPPT charge controller board [...]
 *  [...] based on LM2576 switching voltage regulator
 */


#include <SoftwareSerial.h>

/*Define time intervals for different loops */
#define timeMain 300000UL //Time between main loop iterations (300000 ms = 5 minutes)

/* MPPT algorithm parameters definition */
#define MPPT_delay_t 5000 //Time between iterations of MPPT algorithm (5000 ms = 5 seconds)

/*Define pin allocations on Arduino board */
#define BAT_VOLT_PIN    A0
#define PV_VOLT_PIN     A1
#define PV_CURRENT_PIN  A2
#define PWM_PIN         3
#define LM_ENABLE_PIN   4
#define BT_TX_PIN       5
#define BT_RX_PIN       6
#define BUZZER_PIN      8
#define LED_PIN         9

/*Fill in values below after callibration for improved precision of measurements and charger performance */
#define Vin_mV 5120 //Atmega328 supply voltage in mV - increased by 120 mV for more precise calculations (systematic error)
#define R3  5020 
#define R4  930
#define R5  5070
#define R6  980
#define R8  12000
#define R9  9000

unsigned int n = 1; //Variable with current report issue

void setup() {
  //Input-output pin mode definition
  pinMode(BAT_VOLT_PIN, INPUT);
  pinMode(PV_VOLT_PIN, INPUT);
  pinMode(PV_CURRENT_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(LM_ENABLE_PIN, OUTPUT);
  pinMode(BT_TX_PIN, INPUT);
  pinMode(BT_RX_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  //Output state change to default on startup
  digitalWrite(LM_ENABLE_PIN, HIGH); //Turn off LM2576 switching buck converter by default
  digitalWrite(BUZZER_PIN, LOW); //Turn off buzzer by default
  digitalWrite(LED_PIN, LOW); //Turn off LED indicator by default
  SetCharging(1300); //Set the charging voltage to 13.0 Volts by default
  
  delay(2000); //Wait 2 seconds before ending the setup for RC filter (connected to PWM) stabilisation
}


void loop() {
  //Variables declaration (time management, measurements, MPPT)
  unsigned long BAT_voltage, PV_voltage, PV_current, currentTime, tempTime, timeDifference = 0; 
  unsigned int Power_new, Power_old = 0, MPPT_Vout = 1300;
  byte MPPT_change = 10;
  bool Bulk = false;

  /* Main loop of the code set to regularly check state of charge of the battery */
  while(true){
    currentTime = millis(); //Save current execution time
    tempTime = currentTime; //Set tempTime to currentTime at the beginning of the loop
    timeDifference = 0; //Set timeDifference to 0

    /* Check state of charge of the battery and decide if Bulk or Float charging mode should be executed */
    digitalWrite(LM_ENABLE_PIN, HIGH); //Turn off LM2576 for measurements
    delay(4000); //Wait 
    if((measureAverage(BAT_VOLT_PIN, 20, R5, R6) - 400) < 1300){ //Decrease measured Bat voltage by 400 mV (Bat voltage higher immediatly after charging)
      Bulk = true; //Set Bulk to 1 if the Battery voltage is lower than 13.0V
    }else{
      Bulk = false; //Set Bulk to 0 if the Battery voltage is higher than 13.0V
    }
    digitalWrite(LM_ENABLE_PIN, LOW); //Turn on LM2576
 
    /* ------------------------------------------------------------ */
    /* Enter bulk charging mode with MPPT algorithm on*/
    while(Bulk == 1 && timeDifference < timeMain){
      SetCharging(MPPT_Vout); //Set the charging voltage to MPPT_val (13.0 V default)

      PV_current = measureAverage(PV_CURRENT_PIN, 10, 0, 1); //No voltage div, hence R1=0 & R2=1, 1mV = 1 mA
      PV_voltage = measureAverage(PV_VOLT_PIN, 20, R3, R4);

      Power_new = PV_current * PV_voltage;

      if(Power_new > Power_old){
        MPPT_change = -MPPT_change;
      }
      
      MPPT_Vout =+ MPPT_change;
      Power_old = Power_new;
      delay(MPPT_delay_t);
      currentTime = millis();
      timeDifference = currentTime - tempTime;
    }

    /* ------------------------------------------------------------ */
    /* Enter float-storage charging mode with MPPT algorithm off*/
    while(Bulk == 0 && timeDifference < timeMain){
      SetCharging(1370); //Set the charging voltage to 13.7 Volts
      currentTime = millis();
      timeDifference = currentTime - tempTime;
    }

    /* --------------------------------------------------------------------------- */
    /*Measure the average of n = 20 readings from analog pins and generate reports */
    BAT_voltage = measureAverage(BAT_VOLT_PIN, 20, R5, R6);
    PV_voltage = measureAverage(PV_VOLT_PIN, 20, R3, R4);
    PV_current = measureAverage(PV_CURRENT_PIN, 10, 0, 1); //No voltage div, hence R1=0 & R2=1, 1mV = 1 mA
    
    generateReport(BAT_voltage, PV_voltage, PV_current, Bulk, 1); //Send report over bluetooth (bt = 1)
    generateReport(BAT_voltage, PV_voltage, PV_current, Bulk, 0); //Send report over USB serial port
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
/* Function that change the charging voltage (LM2576 Vout) to a set value  */
void SetCharging(unsigned long Voltage_mV){
  unsigned long V_ref_mV = ((((R8 + R9)*(long)1235)/R8) - ((R9 * Voltage_mV)/R8));
  unsigned long V_ref = map(V_ref_mV, 0, Vin_mV, 0, 1024); //Map value of Reference voltage in mV to 10bit value (0-1024)
  analogWrite(PWM_PIN, V_ref); //Set reference voltage on PWM pin
  return;
}


/* -------------------------------------------------------------------------------------------------------------- */
/* Function that send a report on the current status of the charger through bluetooth/USB serial port */
void generateReport(int BAT_voltage, int PV_voltage, int PV_current, bool Bulk, bool bt){
  int RX_PIN = 0, TX_PIN = 1; //Define serial port pins for standard Arduino USB Serial port (by default)
  if(bt == true){ //Define serial port pins for bluetooth module if bt == true
    RX_PIN = BT_RX_PIN;
    TX_PIN = BT_TX_PIN;
  }

  SoftwareSerial serialPort(RX_PIN, TX_PIN); //Define Software serial port
  serialPort.begin(9600); //Serial port configuration (baud rate: 9600)

  serialPort.print("---------------------------------------------------------------------------------\n");
  serialPort.print("MPPT Solar Charger with bluetooth communication\nReport on current status #");
  serialPort.print(n/2);
  serialPort.print("\n\n# PV panel illuminated:\t\t");
  serialPort.print(PV_voltage > 10000); //Print 1 if PV panel is illuminated by the sunlight (PV_V > 10.0V)
  serialPort.print(" (Illuminated/Shaded)\n# Current charging mode:\t");
  if(Bulk == 1){
    serialPort.print("Bulk mode (MPPT algorithm on)\n# Battery voltage:\t\t");
  }else if(Bulk == 0){
    serialPort.print("Float mode (MPPT algorithm off)\n# Battery voltage:\t\t");
  }
  serialPort.print(BAT_voltage / (float) 1000.0, 3);
  serialPort.print(" V \n# Photovoltaic panel voltage:\t");
  serialPort.print(PV_voltage / (float) 1000.0, 3);
  serialPort.print(" V \n# Photovoltaic panel current:\t");
  serialPort.print(PV_current / (float) 1000.0, 3);
  serialPort.print(" A \n# Power delivered by PV panel:\t");
  serialPort.print(((PV_current * (long) PV_voltage) / 1000000.0), 3);
  serialPort.print(" W \n# Battery SOC (state of charge)\t");
  serialPort.print((BAT_voltage - 11820)/13);
  serialPort.print(" % of full capacity \n\n");
  if(bt == false){ //Additional new lines for bluetooth module if bt == true
    serialPort.print("\n\n\n\n\n");
  }
  n++; //Increment current report number
  return;
}
