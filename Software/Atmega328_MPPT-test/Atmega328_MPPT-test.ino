/*  Name: MPPT Solar Charger with Bluetooth communication | MPPT algorithm (without Bt connection) test code
 *  Author: Karol Wojsław
 *  Date: 04/04/2021 (last release)
 *  Description: For test purposes only
 *  
 *  Atention: Code inconsistent with the current version (look up to Atmega328_battery_charger-test)
 */


#include <SoftwareSerial.h>

/*Define time intervals for different loops */
#define timeMain 60000UL //Time between main loop iterations (60000 ms = 1 minute)

/* MPPT algorithm parameters definition */
#define MPPT_delay_t 1000 //Time between iterations of MPPT algorithm (1000 ms = 1 second)

/*Define pin allocations on Arduino board */
#define BAT_VOLT_PIN    A0
#define PV_VOLT_PIN     A1
#define PV_CURRENT_PIN  A2
#define PWM_PIN         3

/*Fill in values below after callibration for improved precision of measurements and charger performance */
#define Vin_mV 5000 //Atmega328 supply voltage in mV
#define R3  4700 
#define R4  1000
#define R5  4700
#define R6  1000
#define R8  12000
#define R9  9000


void setup() {
  //Input-output pin mode definition
  pinMode(BAT_VOLT_PIN, INPUT);
  pinMode(PV_VOLT_PIN, INPUT);
  pinMode(PV_CURRENT_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);

  //Output state change to default on startup
  SetCharging(1300); //Set the charging voltage to 13.0 Volts by default
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
    delay(500);
    if(measureAverage(BAT_VOLT_PIN, 20, R5, R6) < 1300){
      Bulk = true; //Set Bulk to 1 if the Battery voltage is lower than 13.0V
    }else{
      Bulk = false; //Set Bulk to 0 if the Battery voltage is higher than 13.0V
    }
 
    /* ------------------------------------------------------------ */
    /* Enter bulk charging mode with MPPT algorithm on*/
    while(Bulk == 1 && timeDifference < timeMain){
      SetCharging(MPPT_Vout); //Set the charging voltage to MPPT_val (13.0 V default)

      PV_current = ((measureAverage(PV_CURRENT_PIN, 20, 1, 0) - 2500)*10);
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

      /* --------------------------------------------------------------------------- */
      /*Measure the average of n = 20 readings from analog pins and generate reports */
      BAT_voltage = measureAverage(BAT_VOLT_PIN, 20, R5, R6);
      PV_voltage = measureAverage(PV_VOLT_PIN, 20, R3, R4);
      PV_current = ((measureAverage(PV_CURRENT_PIN, 20, 1, 0) - 2500)*10); //No voltage div, hence R1=0 & R2=1, 2.5 V nominal, then 1mV = 10 mA
      
      generateReport(BAT_voltage, PV_voltage, PV_current, Bulk, 0); //Send report over USB serial port
    }

    /* ------------------------------------------------------------ */
    /* Enter float-storage charging mode with MPPT algorithm off*/
    while(Bulk == 0 && timeDifference < timeMain){
      SetCharging(1370); //Set the charging voltage to 13.7 Volts
      delay(1000);
      currentTime = millis();
      timeDifference = currentTime - tempTime;

      /* --------------------------------------------------------------------------- */
      /*Measure the average of n = 20 readings from analog pins and generate reports */
      BAT_voltage = measureAverage(BAT_VOLT_PIN, 20, R5, R6);
      PV_voltage = measureAverage(PV_VOLT_PIN, 20, R3, R4);
      PV_current = ((measureAverage(PV_CURRENT_PIN, 20, 1, 0) - 2500)*10); //No voltage div, hence R1=0 & R2=1, 2.5 V nominal, then 1mV = 10 mA
      
      generateReport(BAT_voltage, PV_voltage, PV_current, Bulk, 0); //Send report over USB serial port
    }
  }
}


/* -------------------------------------------------------------------------------------------------------------- */
/* Function that measure average of n readings from the input_pin and returns value in mV. Execution time:~5*n ms */
int measureAverage(int input_pin, int n, int V_div_R1, int V_div_R2){
  int readings_avr_mV, Vin_avr_mV, readings_sum = 0;
  for(int i = 0; i < n; i++){ //Take n measurements
    readings_sum += analogRead(input_pin); //Add a 10-bit (0-1024) value read to readings_sum
    delay(5); //Atmega328 takes ~0.1 ms to read an analog val. Wait 5 ms between readings for better performance 
  }
  readings_avr_mV = map((readings_sum / n), 0, 1024, 0, Vin_mV);
  Vin_avr_mV = (readings_avr_mV *(V_div_R1 + V_div_R2))/(V_div_R2);
  
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
  SoftwareSerial serialPort(0, 1); //Define Software serial port
  serialPort.begin(9600); //Serial port configuration (baud rate: 9600)

  serialPort.print("---------------------------------------------\n");
  serialPort.print("MPPT Solar Charger with Bt | Current status: \nPV panel status:: ");
  serialPort.print(PV_voltage > 1300); //Print 1 if PV panel is illuminated by the sunlight (PV_V > 13.0V)
  serialPort.print(",Illuminated (1) / Shaded/Night (0)\nCurrent charging mode:");
  if(Bulk == 1){
    serialPort.print("Bulk mode (MPPT algorithm on)\nBattery voltage: ");
  }else if(Bulk == 0){
    serialPort.print("Float mode (MPPT algorithm off)\nBattery voltage: ");
  }
  serialPort.print(BAT_voltage);
  serialPort.print(" mV \nPhotovoltaic panel voltage: ");
  serialPort.print(PV_voltage);
  serialPort.print(" mV \nPhotovoltaic panel current: ");
  serialPort.print(PV_current);
  serialPort.print(" mA \nPower delivered by PV panel: ");
  serialPort.print(PV_current * PV_voltage);
  serialPort.print(" mW \nBattery SOC (state of charge) ");
  serialPort.print((BAT_voltage - 11820)/13);
  serialPort.print(" % of full capacity \n\n\n");
  return;
}
