/*************************************************************************
  * DIY_Engine_Monitoring and controls
  * Originally developed for: Chevrolet Performance EROD-LS3 6.2L powertrian to 2002 Tahoe chassis interface
  *
  * Written by Ken Adkison
  * From examples and libraries by the following:
  *
  * Revison History
  *
  * ToDo List
  *   Add Screen dimming
  *
  * Remaned and pushed to GitHub now DIY_engine_monitoring_06 - Dec 14, 2024 
  * tahoe_mb_b_0_4.ino - pins reumbered
  * Major Revision - Revised PCB - All 5V Mega2560 based
  * tahoe_nex_07.ino - abandaning CAN - adding fan and AC speed controllers
  * taheo_nex_06_seeed.ino test version for seeed_can using MOSI
  * taheo_nex_06.ino - changed back to freematics
  * taheo_nex_05.ino - A/D works and displays on nextion, partly functional, no CAN
  * tahoe_nex_04.ino - Added previous looping structure to OBD2 PID data processsing 
  * tahoe_nex_03.ino 4/2/2024 - Restructured to loop through structures of A/D readings, calibrations, and display objects.  
  *   Reused loops shortens code providing a better structure for expanded desplay fuctionality later.
  * tahoe_nex_02.ino - First version with working Nextion display working from A/D data
  *
  /*************************************************************************************************
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
  #include <DS18B20.h>

  //#include "nextion_disp.h"  Not required at this time

  #include "tahoe.h"  // definitions and calibrations

  #define DS18B20         // 1-wire temp sensors
  // #define DEBUG1        // AD RAW & limiter display - Comment this out if you don't need to see what happens in the serial Monitor
  // #define DEBUG2        // Nextion display command feed - Comment this out if you don't need to see
  // #define DEBUG3        // Mapped values - Comment this out if you don't need to see what happens in the serial Monitor
  // #define FAKE
  // #define OUTPUT_TEST
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
      for(i=75; i<=254; i++) {
        analogWrite(fan1_pin, i);
        sendCmd(nexOut[0].text + String(i));
        delay(50);
      }
      analogWrite(fan1_pin, 0);
      sendCmd(nexOut[0].text + String(0));
      delay(250);

      for(i=75; i<=254; i++) {
        analogWrite(fan2_pin, i);
        sendCmd(nexOut[1].text + String(i));
        delay(50);
      }
      analogWrite(fan2_pin, 0);
      sendCmd(nexOut[1].text + String(0));
      delay(250);

      analogWrite(fan1_pin, 254);
      sendCmd(nexOut[0].text + String(254));
      delay(2000);
      analogWrite(fan1_pin, 0);
      sendCmd(nexOut[0].text + String(0));
      delay(500);
      analogWrite(fan2_pin, 254);
      sendCmd(nexOut[1].text + String(254));
      delay(2000);
      analogWrite(fan2_pin, 0);
      sendCmd(nexOut[1].text + String(0));
      delay(500);

      for(i=0; i<=254; i++) {
        analogWrite(pwm_ac_pin, i);
        sendCmd(nexOut[2].text + String(i));
        delay(50);
      }
      analogWrite(pwm_ac_pin, 0);
      sendCmd(nexOut[2].text + String(0));
      for(i=0; i<=3; i++) {
        analogWrite(pwm_ac_pin, 254);
        sendCmd(nexOut[2].text + String(254));
        delay(250);
        analogWrite(pwm_ac_pin, 0);
        sendCmd(nexOut[2].text + String(0));
        delay(250);
      }


      for(i=0; i<=3; i++) {
        digitalWrite(evap_pin, 1);
          sendCmd(nexOut[10].bco + String(50712));
          sendCmd(nexOut[10].pco + String(0));
          delay(250);
        digitalWrite(evap_pin, 0);
          sendCmd(nexOut[10].bco + String(12678));
          sendCmd(nexOut[10].pco + String(65504));
          delay(250);
      }

      digitalWrite(fuel_en_pin, 1);
        sendCmd(nexOut[9].bco + String(50712));
        sendCmd(nexOut[9].pco + String(0));

      for(i=0; i<=254; i++) {
        analogWrite(pwm_fuel_pin, i);
        sendCmd(nexOut[3].text + String(i));
        delay(50);
      }
      analogWrite(pwm_fuel_pin, 0);
        digitalWrite(fuel_en_pin, 0);
        sendCmd(nexOut[3].text + String(0));
        sendCmd(nexOut[9].bco + String(12678));
        sendCmd(nexOut[9].pco + String(65504));

      for(i=0; i<=5; i++) {
        analogWrite(mil_pin, 254);
        sendCmd(nexOut[11].bco + String(63488));
        
        DUMP(nexOut[11].bco + String(63488));
        Serial.println(nexOut[11].bco + String(63488));

        delay(500);
        analogWrite(mil_pin, 0);
        sendCmd(nexOut[11].bco + String(12678));
        delay(150);
      }


      for(i=0; i<=254; i++) {
        analogWrite(mphg_pin, i);
        sendCmd(nexOut[4].text + String(i));
        delay(25);
      }
      analogWrite(mphg_pin, 0);
      sendCmd(nexOut[4].text + String(0));

      for(i=0; i<=254; i++) {
        analogWrite(fuelg_pin, i);
        sendCmd(nexOut[5].text + String(i));
        delay(25);
      }
      analogWrite(fuelg_pin, 0);
      sendCmd(nexOut[5].text + String(0));

      for(i=0; i<=254; i++) {
        analogWrite(oilg_pin, i);
        sendCmd(nexOut[6].text + String(i));
        delay(25);
      }
      analogWrite(oilg_pin, 0);
      sendCmd(nexOut[6].text + String(0));

      for(i=0; i<=254; i++) {
        analogWrite(tempg_pin, i);
        sendCmd(nexOut[7].text + String(i));
        delay(25);
      }
      analogWrite(tempg_pin, 0);
      sendCmd(nexOut[7].text + String(0));

      for(i=0; i<=254; i++) {
        analogWrite(voltg_pin, i);
        sendCmd(nexOut[8].text + String(i));
        delay(25);
      }
      analogWrite(voltg_pin, 0);
      sendCmd(nexOut[8].text + String(0));
    }
  }


void ad_in(uint16_t ad, uint16_t raw)  // Process A/D input
  {
    if(millis() >= timeTarget[ad]) {    // if timer has progressed pasted target
      timeTarget[ad] = millis() + refresh[ad];  // add the refresh delay to the timer to reduce cpu load
      // DUMP(curr_val[ad]);
      
      /*if(cal[ad].rawLo < cal[ad].rawHi)  {  // Normal
        if(raw < cal[ad].rawLo)  raw = cal[ad].rawLo;    // safety if all else fails !! Should set error
        if(raw > cal[ad].rawHi)  raw = cal[ad].rawHi;    // safety if all else fails
      } else if(cal[ad].rawLo > cal[ad].rawHi)  {  // Low and Ho can be reversed in some calibrations
        if(raw > cal[ad].rawLo)  raw = cal[ad].rawLo;    // safety if all else fails !! Should set error
        if(raw < cal[ad].rawHi)  raw = cal[ad].rawHi;    // safety if all else fails 
      }
      */
      DUMP(ad);
      DUMP(raw);
      curr_val[ad] = map(raw, cal[ad].rawLo, cal[ad].rawHi, cal[ad].calLo, cal[ad].calHi);   // apply calibrations to A/D inputs
      DUMP(curr_val[ad]);

      if(curr_val[ad] != last_val[ad]) {    // if the results have not changed from the last value, don't bother updating

        if(damping[ad] != 0) {    // dampen fuel gauge from displaying slosh   *********** test **********
          curr_val[ad] = (unsigned long)((last_val[ad] * damping[ad]) + (curr_val[ad] * (1 - damping[ad])));  // New value updates old value at most by 20%
          // DUMP(curr_val[6]);
        }

        if(curr_val[ad] >= gauge[ad].warn_l && curr_val[ad] <= gauge[ad].warn_h) sendCmd(nexObj[ad].pco + String(26508)); // gauge in normal value display GREEN
        else if(curr_val[ad] >= gauge[ad].crit_l && curr_val[ad] <= gauge[ad].warn_l) sendCmd(nexObj[ad].pco + String(65504)); // Warning low YELLOW
        else if(curr_val[ad] >= gauge[ad].warn_h && curr_val[ad] <= gauge[ad].crit_h) sendCmd(nexObj[ad].pco + String(65504)); // warning high YELLOW
        else if(curr_val[ad] <= gauge[ad].crit_l || curr_val[ad] >= gauge[ad].crit_h) sendCmd(nexObj[ad].pco + String(63488)); // critical alarm RED
        else sendCmd(nexObj[ad].pco + String(63488));
        
        sendCmd(nexObj[ad].raw + String(raw));
        // TRACE();
        sendCmd(nexObj[ad].text + String(curr_val[ad]));  // update text readout
        sendCmd(nexObj[ad].gauge + String(map(curr_val[ad], gauge[ad].start, gauge[ad].stop, 0, 100)));  // update bar graph
        
        last_val[ad] = curr_val[ad];
      }
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
  sendCmd(""); // clear the buffer

  //  INPUTS
  pinMode(ac_req_pin, INPUT_PULLUP);
  pinMode(ac_recirc_pin, INPUT_PULLUP);
  pinMode(oil_lvl_pin, INPUT_PULLUP);
  pinMode(cool_lvl_pin, INPUT_PULLUP);

  pinMode(rpm_pin, INPUT_PULLUP);
  pinMode(vss_pin, INPUT_PULLUP);

  // OUTPUTS
  pinMode(pwm_ac_pin, OUTPUT);
  pinMode(pwm_fuel_pin, OUTPUT);
  pinMode(fan1_pin, OUTPUT);
  pinMode(fan2_pin, OUTPUT);
  pinMode(pwm_en_pin, OUTPUT);

  pinMode(tempg_pin, OUTPUT);
  pinMode(voltg_pin, OUTPUT);
  pinMode(fuelg_pin, OUTPUT);
  pinMode(oilg_pin, OUTPUT);
  pinMode(mphg_pin, OUTPUT);
  pinMode(mil_pin, OUTPUT);

  pinMode(fuel_en_pin, OUTPUT);
  pinMode(evap_pin, OUTPUT);


  for(i=0; i<=23; i++)  // send zeros to all display objects  *** KEEP UP TO DATE
    {
    sendCmd(nexObj[i].raw + String(0));
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

  //  we have all values
  #ifdef DEBUG1 
    // AD_print_all();
    dbgSer.print("rpm = ");
    dbgSer.print(rpm);
    dbgSer.print(" , vss = ");
    dbgSer.println(vss);
  #endif

  #ifdef FAKE
      ad_in(OIL_P,  fake_raw[OIL_P]);
      dbgSer.println(nexObj[OIL_P].text + String(curr_val[OIL_P]));
      ad_in(COOL_T, fake_raw[COOL_T]);
      dbgSer.println(nexObj[COOL_T].text + String(curr_val[COOL_T]));
      ad_in(OIL_T,  fake_raw[OIL_T]);
      ad_in(FUEL_L, fake_raw[FUEL_L]);
      ad_in(FUEL_P, fake_raw[FUEL_P]);
      ad_in(EVAP,   fake_raw[EVAP]);
      ad_in(AC_HP,  fake_raw[AC_HP]);
      ad_in(THR,    fake_raw[THR]);
      ad_in(MAP,    fake_raw[MAP]);

      fake_raw[OIL_P] = fake_raw[OIL_P] +5;
      fake_raw[COOL_T] = fake_raw[COOL_T] -5;
      fake_raw[OIL_T] = fake_raw[OIL_T] -5;
      fake_raw[FUEL_L] = fake_raw[FUEL_L] +5;
      fake_raw[FUEL_P] = fake_raw[FUEL_P] +5;
      fake_raw[EVAP] = fake_raw[EVAP] +5;
      fake_raw[AC_HP] = fake_raw[AC_HP] +5;
      fake_raw[THR] = fake_raw[THR] +5;
      fake_raw[MAP] = fake_raw[MAP] +5;
  #endif  // FAKE data
  
    ad_in(OIL_P,  analogRead(oil_p_pin));
    ad_in(COOL_T, analogRead(cool_t_pin));
    ad_in(OIL_T,  analogRead(oil_t_pin));
    ad_in(FUEL_L, analogRead(fuel_l_pin));
    ad_in(FUEL_P, analogRead(fuel_p_pin));
    ad_in(EVAP,   analogRead(evap_pin));
    ad_in(AC_HP,  analogRead(ac_hp_pin));
  
    ad_in(I_FUEL, analogRead(i_fuel));
    ad_in(I_COMP, analogRead(i_comp));
    ad_in(I_FAN1, analogRead(i_fan1));
    ad_in(I_FAN2, analogRead(i_fan2));

    ad_in(THR,    analogRead(thr_pin));
    ad_in(MAP,    analogRead(map_pin));

  
  if(nexSer.available() > 0)  // read data strings from Nextion Display
  {
    char Received = dbgSer.read();
    if (Received =='s')  {  // slider 1
      int num = nexSer.parseInt();
    }
  }

  ds.select(0);

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

  if(curr_val[COOL_T] < 170)  {  // engine_t
    fan1 = 0;
    fan2 = 0;
  }
  else if(curr_val[COOL_T] > 225)  {
    fan1 = 254;
    fan2 = 254;
  }
  else  {
    fan1 = map(curr_val[COOL_T], 180, 220, 80, 254);
    fan2 = map(curr_val[COOL_T], 180, 220, 80, 254);
  }

  // a/c CVC compressor - Deslug startup by starting with no flow then adding inital flow slowly to get oil and liquid moving prior to adding flow
  if(ac_req)  {
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

  analogWrite(fan1_pin, fan1);
  analogWrite(fan2_pin, fan2);

  // The following seems like a safey - redundant
  if(curr_val[5] > 200)  {  // engine_t
    fan1 = 254;
    fan2 = 254;
    analogWrite(fan1_pin, fan1);
    analogWrite(fan2_pin, fan2);
  }
  sendCmd(nexOut[0].text + String(fan1));
  sendCmd(nexOut[1].text + String(fan2));
}  // loop