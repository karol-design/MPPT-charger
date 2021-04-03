#define time_a  600000 //Time 

void setup() {
  //...
}



void loop() {
  unsigned long currentTime = 0, tempTime = 0, timeDifference = 0; //Variables for time management in the main loop
  int BAT_voltage, PV_voltage, PV_current; //Values of measured voltages and currents in mV
  bool Day, Bulk;

  currentTime = millis();
  timeDifference = currentTime - tempTime;

  /* Main loop of the code set to regularly check if it's Day or Night and to send report over Bluetooth */
  while(1){
    currentTime = millis(); //Save current execution time
    timeDifference = currentTime - tempTime;
  
    digitalWrite(LM_ENABLE_PIN, HIGH); //Turn off LM2576
    PV_voltage = measureAverage(PV_VOLT_PIN, 20, R3, R4);


    /* ------------------------------------- */
    /* Enter Day Mode if PV_voltage > 12.0 V */
    if(PV_voltage > 1200){
      Day = true;
      
      while(BAT_voltage > 1300){ //Bulk charge with MPPT algorithm implemented
        SetCharging(1470); //Set the charging voltage to 14.7 Volts
      }

      //Float-storage charge without MPPT
      SetCharging(1370); //Set the charging voltage to 13.70 Volts
      delay(5000); //Wait for steady-state on RC filter (connected to PWM output)
      digitalWrite(LM_ENABLE_PIN, LOW); //Turn on LM2576 switching buck converter

    }

    /* ------------------------------------------------------------------------------------ */
    /* Enter Night Mode if PV_voltage < 12.0 V to save power ant prevent IC from any damage */
    else if(PV_voltage < 1200){
      Day = false;
      digitalWrite(LM_ENABLE_PIN, HIGH); //Turn off LM2576
    }

  delay(time_a);
  generateReport(BAT_voltage, PV_voltage, PV_current, 1); //Bluetooth (bt = 1)
  generateReport(BAT_voltage, PV_voltage, PV_current, 0); //Serial Port (bt = 0)

  }
}
