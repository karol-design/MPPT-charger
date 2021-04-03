/* This is the code for the MPPT (Maximum Power Point Tracking) Charge Charge Controller with Bluetooth
 * Author: Karol Wojsław
 * Date: 27/03/2021
 * Version: 0.1 (Structure)
 */

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  V_bat = Measure_battery_voltage()
  V_pv = Measure_pv_voltage()

  if(V_bat > 13.0){ //Bulk charge with MPPT algorithm
    V_charging = 14.65 //Set charging voltage to 14.65 Volts
    
    while(true){ //Loop for MPPT algorithm
      I_pv = Measure_charging_current()

      if(I_pv < 200 mA){
        break; -->
      }
      
      P_new = I_pv * V_pv
      
      if(P_new > P_old)
        V_change = -V_change
     
      V_charging = V_charging + V_change
      P_old = P_new       
      delay(wait for steady state)
    }
    
  }
  
  //Float - storage charge
  V_charging = 13.7 //Set charging voltage to 13.7 Volts

  delay(some_time);
}
