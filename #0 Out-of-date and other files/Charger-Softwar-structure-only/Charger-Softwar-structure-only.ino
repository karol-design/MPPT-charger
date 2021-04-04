#define timeMain  300000UL //Time between main loop iterations (300000 ms = 5 minutes)

void setup() {
  //...
}


void loop() {
  unsigned long currentTime, tempTime, timeDifference = 0; //Variables for time management in the main loop
  int BAT_voltage, PV_voltage, PV_current; //Values of measured voltages and currents in mV
  bool Bulk;

  /* Main loop of the code set to regularly check state of charge of the battery */
  while(true){
    currentTime = millis(); //Save current execution time
    tempTime = currentTime; //Set tempTime to currentTime at the beginning of the loop
    timeDifference = 0; //Set timeDifference to 0

    /* Check state of charge of the battery and decide if Bulk or Float charging mode should be executed */
    if(measureAverage(BAT_VOLT_PIN, 20, R5, R6) < 1300){
      Bulk == 1; //Set Bulk to 1 if the Battery voltage is lower than 13.0V
    }else{
      Bulk == 0;
    }
 
    /* --------------------------------- */
    /* Enter bulk charging mode */
    while(Bulk == 1 && timeDifference < timeMain){
      //SetCharging(1470); //Set the charging voltage to 14.7 Volts
      
      currentTime = millis();
      timeDifference = currentTime - tempTime;
    }

    /* --------------------------------- */
    /* Enter float-storage charging mode */
    while(Bulk == 0 && timeDifference < timeMain){
      //SetCharging(1370); //Set the charging voltage to 13.70 Volts

      currentTime = millis();
      timeDifference = currentTime - tempTime;
    }
  }
}
