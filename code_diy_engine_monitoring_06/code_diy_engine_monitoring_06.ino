/*************************************************************************
  * DIY_Engine_Monitoring and controls
  * Originally developed for: Chevrolet Performance EROD-LS3 6.2L powertrian to 2002 Tahoe chassis interface
  *
  * Written by Ken Adkison
  * From examples and libraries by the following:
  *
  * Revison History
  *
  * Renamed and pushed to GitHub now DIY_engine_monitoring_06 - Dec 14, 2024 
  * tahoe_mb_b_0_4.ino - pins reumbered
  * Major Revision - Revised PCB - All 5V Mega2560 based
  * tahoe_nex_07.ino - abandaning CAN - adding fan and AC speed controllers
  * taheo_nex_06_seeed.ino test version for seeed_can using MOSI
  * taheo_nex_06.ino - changed back to freematics
  * taheo_nex_05.ino - A/D works and displays on nextion, partly functional, no CAN
  * tahoe_nex_04.ino - Added previous looping structure to OBD2 PID data processsing 
  * tahoe_nex_03.ino 4/2/2024 - Restructured to loop through structures of A/D readings, calibrations, and display objects.  
  *   Reused loops shortens code providing a better structure for expanded desplay functionality later.
  * tahoe_nex_02.ino - First version with working Nextion display working from A/D data
  *
  *************************************************************************************************
      OBD-II_PIDs TEST CODE
      LOOVEE @ JUN24, 2017

      Query
      send id: 0x7df
        dta: 0x02, 0x01, PID_CODE, 0, 0, 0, 0, 0

      Response
      From id: 0x7E9 or 0x7EA or 0x7EB
        dta: len, 0x41, PID_CODE, byte0, byte1(option), byte2(option), byte3(option), byte4(option)

      https://en.wikipedia.org/wiki/OBD-II_PIDs

      Input a PID, then you will get reponse from vehicle, the input should be end with '\n'
  ***************************************************************************************************/

  #include <Arduino.h>
  // #include <Wire.h> // Master is MEGA 2560
  // #include <DS18B20.h>

  //#include "nextion_disp.h"  Not required at this time

  #include "tahoe.h"  // definitions and calibrations

    // #define DS18B20          // 1-wire temp sensors
    // #define DEBUG1        // AD RAW & limiter display - Comment this out if you don't need to see what happens in the serial Monitor
    // #define DEBUG2        // Nextion display command feed - Comment this out if you don't need to see
    // #define DEBUG3        // Mapped values - Comment this out if you don't need to see what happens in the serial Monitor
    // #define FAKE          // Faking data for bench testing without sensors
    // #define OUTPUT_TEST   // Driver outputs exercised for testing
    // #define WIRE
    #define TRACER

    #ifdef TRACER
      #include <ArduinoTrace.h>
    #endif

void sendCmd(String cmd)
  {
    nexSer.print(cmd);
    nexSer.write("\xFF\xFF\xFF");
    #ifdef DEBUG2
      if(cmd.length() > 0)
      {
        dbgSer.print("Sending command : ");
        dbgSer.println(cmd);
      } else
      {
        dbgSer.println("Empty command issued to clear the buffer"); 
      }
    #endif  
  }


void output_test()
  {
    while(1) {
      dbgSer.println("output.test");
      int i;
      for(i=fan_sp_lo; i <= fan_sp_hi; i++) {
        analogWrite(fan1_pin, i);
        sendCmd(nexOut[FAN1].text + String(i));
        DUMP(i);
        ad_in(I_FAN1, analogRead(i_fan1_pin));
        delay(500);
      }
      analogWrite(fan1_pin, 0);
      sendCmd(nexOut[FAN1].text + String(0));
      delay(250);

      for(i=fan_sp_lo; i <= fan_sp_hi; i++) {
        analogWrite(fan2_pin, i);
        sendCmd(nexOut[FAN2].text + String(i));
        ad_in(I_FAN2, analogRead(i_fan2_pin));
        delay(50);
      }
      analogWrite(fan2_pin, 0);
      sendCmd(nexOut[FAN2].text + String(0));
      delay(250);

      analogWrite(fan1_pin, fan_sp_hi);
      sendCmd(nexOut[FAN1].text + String(fan_sp_hi));
      delay(2000);
      ad_in(I_FAN1, analogRead(i_fan1_pin));
      analogWrite(fan1_pin, 0);
      sendCmd(nexOut[FAN1].text + String(0));
      delay(500);
      ad_in(I_FAN1, analogRead(i_fan1_pin));
      analogWrite(fan2_pin, fan_sp_hi);
      sendCmd(nexOut[FAN2].text + String(fan_sp_hi));
      delay(2000);
      ad_in(I_FAN2, analogRead(i_fan2_pin));
      analogWrite(fan2_pin, 0);
      sendCmd(nexOut[FAN2].text + String(0));
      delay(500);
      ad_in(I_FAN2, analogRead(i_fan2_pin));
      
      for(i = 0; i <= 254; i++) {
        analogWrite(pwm_ac_pin, i);
        sendCmd(nexOut[COMP].text + String(i));
        delay(50);
        ad_in(I_COMP, analogRead(i_comp_pin));
      }
      analogWrite(pwm_ac_pin, 0);
      sendCmd(nexOut[COMP].text + String(0));
      for(i=0; i <= 3; i++) {
        analogWrite(pwm_ac_pin, 254);
        sendCmd(nexOut[COMP].text + String(254));
        delay(250);
        ad_in(I_COMP, analogRead(i_comp_pin));
        analogWrite(pwm_ac_pin, 0);
        sendCmd(nexOut[COMP].text + String(0));
        delay(250);
        ad_in(I_COMP, analogRead(i_comp_pin));
      }


      for(i=0; i<=3; i++) {
        digitalWrite(evap_pin, 1);
          sendCmd(nexOut[EVAPS].bco + String(50712));
          sendCmd(nexOut[EVAPS].pco + String(0));
          delay(250);
        digitalWrite(evap_pin, 0);
          sendCmd(nexOut[EVAPS].bco + String(12678));
          sendCmd(nexOut[EVAPS].pco + String(65504));
          delay(250);
      }

      digitalWrite(fuel_en_pin, 1);
        sendCmd(nexOut[FUELEN].bco + String(50712));
        sendCmd(nexOut[FUELEN].pco + String(0));

      for(i=0; i<=254; i++) {
        analogWrite(pwm_fuel_pin, i);
        sendCmd(nexOut[FUEL].text + String(i));
        delay(50);
        ad_in(I_FUEL, analogRead(i_fuel_pin));
      }
      analogWrite(pwm_fuel_pin, 0);
        digitalWrite(fuel_en_pin, 0);
        sendCmd(nexOut[FUEL].text + String(0));
        sendCmd(nexOut[FUELEN].bco + String(12678));
        sendCmd(nexOut[FUELEN].pco + String(65504));
        ad_in(I_FUEL, analogRead(i_fuel_pin));

      for(i=0; i<=5; i++) {
        analogWrite(mil_pin, 254);
        sendCmd(nexOut[MILOUT].bco + String(63488));
        
        // DUMP(nexOut[MILOUT].bco + String(63488));
        Serial.println(nexOut[MILOUT].bco + String(63488));

        delay(500);
        analogWrite(mil_pin, 0);
        sendCmd(nexOut[MILOUT].bco + String(12678));
        delay(150);
      }


      for(i=0; i<=254; i++) {
        analogWrite(mphg_pin, i);
        sendCmd(nexOut[MPHG].text + String(i));
        delay(25);
      }
      analogWrite(mphg_pin, 0);
      sendCmd(nexOut[MPHG].text + String(0));

      for(i=0; i<=254; i++) {
        analogWrite(fuelg_pin, i);
        sendCmd(nexOut[FUELG].text + String(i));
        delay(25);
      }
      analogWrite(fuelg_pin, 0);
      sendCmd(nexOut[FUELG].text + String(0));

      for(i=0; i<=254; i++) {
        analogWrite(oilg_pin, i);
        sendCmd(nexOut[OILG].text + String(i));
        delay(25);
      }
      analogWrite(oilg_pin, 0);
      sendCmd(nexOut[OILG].text + String(0));

      for(i=0; i<=254; i++) {
        analogWrite(tempg_pin, i);
        sendCmd(nexOut[TEMPG].text + String(i));
        delay(25);
      }
      analogWrite(tempg_pin, 0);
      sendCmd(nexOut[TEMPG].text + String(0));

      for(i=0; i<=254; i++) {
        analogWrite(voltg_pin, i);
        sendCmd(nexOut[VOLTG].text + String(i));
        delay(25);
      }
      analogWrite(voltg_pin, 0);
      sendCmd(nexOut[VOLTG].text + String(0));
    }
  } // output_test


void ad_in(uint16_t ad, uint16_t raw)  // Process A/D input
  {
    if(millis() >= timeTarget[ad]) {    // if timer has progressed pasted target
      timeTarget[ad] = millis() + refresh[ad];  // add the refresh delay to the timer to reduce cpu load
      // DUMP(curr_val[ad]);
      
      if(cal[ad].rawLo < cal[ad].rawHi)  {  // Normal
        if(raw < cal[ad].rawLo)  raw = cal[ad].rawLo;    // safety if all else fails !! Should set error
        if(raw > cal[ad].rawHi)  raw = cal[ad].rawHi;    // safety if all else fails
      } else if(cal[ad].rawLo > cal[ad].rawHi)  {        // Low and Hi can be reversed in some calibrations
        if(raw > cal[ad].rawLo)  raw = cal[ad].rawLo;    // safety if all else fails !! Should set error
        if(raw < cal[ad].rawHi)  raw = cal[ad].rawHi;    // safety if all else fails 
      }
      
      //DUMP(ad);
      //DUMP(raw);
      curr_val[ad] = map(raw, cal[ad].rawLo, cal[ad].rawHi, cal[ad].calLo, cal[ad].calHi);   // apply calibrations to A/D inputs
      //DUMP(curr_val[ad]);

      //if(curr_val[ad] != last_val[ad]) {    // if the results have not changed from the last value, don't bother updating

        if(damping[ad] != 0) {    // dampen gauge (generally fuel)from displaying slosh   *********** test **********
          curr_val[ad] = (unsigned long)((last_val[ad] * damping[ad]) + (curr_val[ad] * (1 - damping[ad])));  // New value updates old value at most by 20%
        }

        if(curr_val[ad] >= gauge[ad].warn_l && curr_val[ad] <= gauge[ad].warn_h) sendCmd(nexObj[ad].pco + String(26508)); // gauge in normal value display GREEN
        else if(curr_val[ad] >= gauge[ad].crit_l && curr_val[ad] <= gauge[ad].warn_l) sendCmd(nexObj[ad].pco + String(65504)); // Warning low YELLOW
        else if(curr_val[ad] >= gauge[ad].warn_h && curr_val[ad] <= gauge[ad].crit_h) sendCmd(nexObj[ad].pco + String(65504)); // warning high YELLOW
        else if(curr_val[ad] <= gauge[ad].crit_l || curr_val[ad] >= gauge[ad].crit_h) 
          {
          sendCmd(nexObj[ad].pco + String(63488)); // critical alarm RED
          //sendCmd(nexObj[MIL].bco + String(63488)); // critical alarm RED
          
          // sendCmd(nexObj[MIL].text + nexObj[ad].almtext);  // does not work ??
          // nexSer.print(nexObj[MIL].text);
          // nexSer.print("g0.txt=help me \xFF\xFF\xFF");
          // nexSer.print(nexObj[ad].almtext);
          // nexSer.print("help me");
          // nexSer.write("\xFF\xFF\xFF");

          //delay(500);  // Time to read text - poor coding - Alarm window clearing sould go in outer loop, set and clear master alarm flag
          }

        else sendCmd(nexObj[ad].pco + String(63488));
        sendCmd(nexObj[MIL].bco + String(12678)); // clear alarm restore background color ***** should check for addtional alarms with flag *****
        
        sendCmd(nexObj[ad].raw + String(raw));
        // TRACE();
        sendCmd(nexObj[ad].text + String(curr_val[ad]));  // update text readout
        sendCmd(nexObj[ad].gauge + String(map(curr_val[ad], gauge[ad].start, gauge[ad].stop, 0, 100)));  // update bar graph
        
        last_val[ad] = curr_val[ad];
      //}
    }
  }
  

void setup() {
  // dbgSer.setDebugOutput(false);  // see Wire master
  // Wire.setSDA(0);
  // Wire.setSCL(1);

  // Wire.begin();
  //   delay(100);  // Wire settling ?

  dbgSer.begin(38400);  // debug
  while(!dbgSer);
  nexSer.begin(38400);  // Nextion display feed - Some dropout with long cable at 115200
  while(!nexSer);
  sendCmd("");          // clear the buffer

  //  INPUTS
  pinMode(ac_req_pin,    INPUT_PULLUP);
  pinMode(ac_recirc_pin, INPUT_PULLUP);
  pinMode(oil_lvl_pin,   INPUT_PULLUP);
  pinMode(cool_lvl_pin,  INPUT_PULLUP);

  pinMode(rpm_pin, INPUT_PULLUP);
  pinMode(vss_pin, INPUT_PULLUP);

  // OUTPUTS
  pinMode(pwm_ac_pin,   OUTPUT);
  pinMode(pwm_fuel_pin, OUTPUT);
  pinMode(fan1_pin,     OUTPUT);
  pinMode(fan2_pin,     OUTPUT);
  pinMode(pwm_en_pin,   OUTPUT);

  pinMode(tempg_pin,    OUTPUT);
  pinMode(voltg_pin,    OUTPUT);
  pinMode(fuelg_pin,    OUTPUT);
  pinMode(oilg_pin,     OUTPUT);
  pinMode(mphg_pin,     OUTPUT);
  pinMode(mil_pin,      OUTPUT);

  pinMode(fuel_en_pin,  OUTPUT);
  pinMode(evap_pin,     OUTPUT);


  for(i=0; i<=24; i++)  // send zeros to all display objects  *** KEEP UP TO DATE
    {
    sendCmd(nexObj[i].raw  + String(0));
    sendCmd(nexObj[i].text + String(0));
    }

  #ifdef FAKE
    fake_raw[OIL_P]  = cal[OIL_P].rawLo;
    fake_raw[COOL_T] = cal[COOL_T].rawLo;
    fake_raw[OIL_T]  = cal[OIL_T].rawLo;
    fake_raw[FUEL_L] = cal[FUEL_L].rawLo;
    fake_raw[FUEL_P] = cal[FUEL_P].rawLo;
    fake_raw[EVAP]   = cal[EVAP].rawLo;
    fake_raw[AC_HP]  = cal[AC_HP].rawLo;
    fake_raw[THR]    = cal[THR].rawLo;
    fake_raw[MAP]    = cal[MAP].rawLo;
  #endif  // FAKE

  // digitalWrite(pwm_en_pin,  1);

  #ifdef DS18B20
    dbgSer.print("1-wire Devices: ");
    dbgSer.println(ds.getNumberOfDevices());
      while (ds.selectNext()) {
      switch (ds.getFamilyCode()) {
        case MODEL_DS18S20:
          dbgSer.println("Model: DS18S20/DS1820");
          break;
        case MODEL_DS1822:
          dbgSer.println("Model: DS1822");
          break;
        case MODEL_DS18B20:
          dbgSer.println("Model: DS18B20");
          break;
        default:
          dbgSer.println("Unrecognized Device");
          break;
      }

      uint8_t address[8];
      ds.getAddress(address);

      dbgSer.print("Address:");
      for (uint8_t i = 0; i < 8; i++) {
        dbgSer.print(" ");
        dbgSer.print(address[i]);
      }
      dbgSer.println();

      dbgSer.print("Resolution: ");
      dbgSer.println(ds.getResolution());

      dbgSer.print("Power Mode: ");
      if (ds.getPowerMode()) {
        dbgSer.println("External");
      } else {
        dbgSer.println("Parasite");
      }

      dbgSer.print("Temperature: ");
      dbgSer.print(ds.getTempC());
      dbgSer.print(" C / ");
      dbgSer.print(ds.getTempF());
      dbgSer.println(" F");
      dbgSer.println();
    }
  #endif  // DS18B20 WIRE
  

  #ifdef OUTPUT_TEST
   output_test();
  #endif

}  //  End Setup


void loop() {  // MAIN LOOP ******************************************************

  // rpm = pulseIn(rpm_pin, HIGH);  //  rpm = 1 / pulseIn(rpm_pin, HIGH);  VERY SLOW
  // vss = pulseIn(vss_pin, HIGH);  //  vss = ((1 / pulseIn(vss_pin, HIGH)) / 60) / 800;  // pulses per mile is some standaard number 800?
  #ifdef DEBUG1  // RPM & VSS
    dbgSer.print("rpm = ");
    dbgSer.print(rpm);
    dbgSer.print(" , vss = ");
    dbgSer.println(vss);
  #endif

  #ifdef FAKE  // fake A/D inputs and scan them through the full range
    ad_in(OIL_P,  fake_raw[OIL_P]);
    ad_in(COOL_T, fake_raw[COOL_T]);
    ad_in(OIL_T,  fake_raw[OIL_T]);
    ad_in(FUEL_L, fake_raw[FUEL_L]);
    ad_in(FUEL_P, fake_raw[FUEL_P]);
    ad_in(EVAP,   fake_raw[EVAP]);
    ad_in(AC_HP,  fake_raw[AC_HP]);
    ad_in(THR,    fake_raw[THR]);
    ad_in(MAP,    fake_raw[MAP]);

    fake_raw[OIL_P]  = fake_raw[OIL_P]  +5;
    fake_raw[COOL_T] = fake_raw[COOL_T] -1;
    fake_raw[OIL_T]  = fake_raw[OIL_T]  -1;
    fake_raw[FUEL_L] = fake_raw[FUEL_L] +1;
    fake_raw[FUEL_P] = fake_raw[FUEL_P] +5;
    fake_raw[EVAP]   = fake_raw[EVAP]   +1;
    fake_raw[AC_HP]  = fake_raw[AC_HP]  +5;
    fake_raw[THR]    = fake_raw[THR]    +5;
    fake_raw[MAP]    = fake_raw[MAP]    +5;
  #endif  // FAKE data
  
    
    ad_in(OIL_P,  analogRead(oil_p_pin));
    ad_in(COOL_T, analogRead(cool_t_pin));
    ad_in(OIL_T,  analogRead(oil_t_pin));
    ad_in(FUEL_L, analogRead(fuel_l_pin));
    ad_in(FUEL_P, analogRead(fuel_p_pin));
    ad_in(EVAP,   analogRead(evap_pin));
    ad_in(AC_HP,  analogRead(ac_hp_pin));
    ad_in(AC_HP,  analogRead(ac_hp_pin));
    ad_in(THR,    analogRead(thr_pin));
    ad_in(MAP,    analogRead(map_pin));

  
    // ad_in(I_FUEL, analogRead(i_fuel_pin));
    // ad_in(I_COMP, analogRead(i_comp_pin));
    // ad_in(I_FAN1, analogRead(i_fan1_pin));
    // ad_in(I_FAN2, analogRead(i_fan2_pin));

    // ad_in(THR,    analogRead(thr_pin));
    // ad_in(MAP,    analogRead(map_pin));
    

  if(digitalRead(oil_lvl_pin) == 1) sendCmd("oilLvl.bco=" + String(63488));  // low oil
  else sendCmd("oilLvl.bco=" + String(26508));   // has oil

  if(digitalRead(cool_lvl_pin) == 1) sendCmd("coolLvl.bco=" + String(63488));  // low coolant
  else sendCmd("coolLvl.bco=" + String(26508));  
  
  if(nexSer.available() > 0)  // read data strings from Nextion Display, should be:  if(nexSer.available())   ?
  {
    char Received = nexSer.read();
    if (Received =='s')  {  // slider 1
      int num = nexSer.parseInt();
    }
  }
  
  /*ds.select(0);

  sendCmd(nexObj[15].text + String(int(ds.getTempF())));
  // ds.selectNext();
  ds.select(1);
  sendCmd(nexObj[16].text + String(int(ds.getTempF())));
  //   ds.selectNext();


  #ifdef WIRE
    dbgSer.print("1-wire Devices: ");
    dbgSer.println(ds.getNumberOfDevices());
    air_temp = ds.getTempC();
  #endif  // WIRE
  */

  if(curr_val[COOL_T] < fan_coolant_low)  {  // engine_t  ***** This does not yet accound for A/C needs ******
    fan1 = 0;
    fan2 = 0;
    // dbgSer.println("fan1 -a");
  }
  else if(curr_val[COOL_T] > fan_coolant_high)  {
    fan1 = fan_sp_hi;
    fan2 = fan_sp_hi;
    dbgSer.print("fan1 -b");
  }
  else  {
    fan1 = map(curr_val[COOL_T], fan_coolant_low, fan_coolant_high, fan_sp_lo, fan_sp_hi);
    fan2 = map(curr_val[COOL_T], fan_coolant_low+20, fan_coolant_high, fan_sp_lo, fan_sp_hi);  // +20 allows one fan only under light heat loads
    dbgSer.print("fan1 -c");
  }

  /*
  #ifdef DEBUG3
    // a/c CVC compressor - Deslug startup by starting with no flow then adding inital flow slowly to get oil and liquid moving prior to adding flow
    dbgSer.print("ac_req= ");
    dbgSer.print(digitalRead(ac_req_pin));  // 1 = key off, key on ac off = 0, ac on = 0, fan off = 1, fan on = 0

    dbgSer.print(", ac_recirc= ");
    dbgSer.println(digitalRead(ac_recirc_pin));  // 1 = key off, key on & ac off = 0, ac & recirc = 1
  #endif

  if(digitalRead(ac_req_pin) = 0)  {
    if(fan1 < 150) { 
      fan1 = 150;
    }
    if(fan2 < 150) { 
      fan2 = 150;
    }
    if(curr_val[5] < 250)  {  // cool_t
      if (ac_comp < 50) {
        ac_comp = 50;
        analogWrite(pwm_ac_pin, ac_comp);
        delay(200);
      }
      else analogWrite(pwm_ac_pin, ac_comp);
    }
    sendCmd(nexOut[2].text + String(ac_comp));
  }
  */
  // DUMP(fan1);
  //analogWrite(fan1_pin, fan1);
  //analogWrite(fan2_pin, fan2);

  sendCmd(nexOut[0].text + String(fan1));
  sendCmd(nexOut[1].text + String(fan2));
  
}  // loop