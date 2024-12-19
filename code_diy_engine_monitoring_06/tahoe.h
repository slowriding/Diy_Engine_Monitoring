#include "pins_arduino.h"
#include <Arduino.h>
// #include <Wire.h> // Master is MEGA 2560

#define nexSer Serial1 // Controlling the Nextion HMI using serial1 (pin18 of the Arduino Mega) to prevent interfering with code upload
#define dbgSer Serial // Debug using default serial over USB towards Arduino serial Monitor
DS18B20 ds(36);

// int idx = 0;
// uint32_t last = 0, now = 0;  // for speed test
uint32_t i = 0;   // wire buffer

// int fake=0;    // fake data sequencer

// Variables used in logic control

// OBD2 decoded data from Freematics
uint16_t volts;
uint16_t load;
uint16_t manfp;
uint16_t engine_temp;
uint16_t rpm;
uint16_t vss;
uint16_t adv;
uint16_t air_temp;
uint16_t thr_pos;
uint16_t evap_press;

// ANALOG INPUTS              // Nextion fields
const int i_fuel       = A0;  // n73
const int i_comp       = A1;  // n72
const int i_fan1       = A2;  // n70
const int i_fan2       = A3;  // n71
const int oil_t_pin    = A4;  // n5:x5
const int cool_t_pin   = A5;  // n3:x3
const int fuel_l_pin   = A6;  // n4:x4
const int evap_pin     = A7;  // n7:x7
const int fuel_p_pin   = A8;  // n6:x6
const int ac_hp_pin    = A9;   // n10:x10
const int oil_p_pin    = A10;  // n2:x2
const int thr_pin      = A11;  // n8:x8
const int map_pin      = A12;  // n9:x9
const int vss_pin      = A14;  // n0:raw x0:mph
const int rpm_pin      = A15;  // n1:raw x1:rpm

const int ad_pins[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};

// DIGITAL INPUTS
const int ac_req_pin   = 34;
const int ac_recirc_pin= 33;
const int oil_lvl_pin  = 28;
const int cool_lvl_pin = 32;

// const int one_wire_pin = A13;

// OUTPUTS
const int pwm_ac_pin   = 2;
const int pwm_fuel_pin = 3;
const int fan1_pin     = 4;
const int fan2_pin     = 5;

const int tempg_pin    = 6;
const int voltg_pin    = 7;
const int fuelg_pin    = 8;
const int oilg_pin     = 9;
const int fuel_en_pin  = 10;
const int mphg_pin     = 11;
const int evap_sol_pin = 13;  
const int mil_pin      = 14;
const int pwm_en_pin   = 23;

// Global variables
// static char buff[100]; // for CAN msgs via wire  //  ??? not used
bool ac_req   = false;
bool cool_lvl = false;
bool oil_lvl  = false;
bool fuel_en  = false;
bool evap     = false;


uint8_t rpm_l    = 0;
uint8_t mph_l    = 0;
uint8_t fan1     = 0;
uint8_t fan2     = 0;
uint8_t ac_pwm   = 0;
uint8_t fuel_pwm = 0;
uint8_t mil      = 255;
uint8_t voltg    = 0;
uint8_t oilg     = 0;
uint8_t tempg    = 0;
uint8_t fuelg    = 0;


float airTemp       = 0;
float condenserTemp = 0;

uint32_t timTarget = 0;
uint16_t interval  = 200;
uint32_t readVal   = 0;
int32_t  dispVal   = 0;
int32_t  lastVal   = 0;
int      ac_comp   = 0;

/*
// ----- OBD
#define map_calrawLo 0
#define map_calrawHi 65,535
#define map_calvalLo 0
#define map_calvalHi 255
*/

#define ac_fan_on_cond_t 185
#define ac_fan_off_vcc 35
#define ac_fan_on_vcc 35
#define ac_delay 1         // secs
#define ac_min_on_time 50  // secs
#define ac_p_en 219.9      // f
#define ac_p_dis 180.1     // f

#define fan_low_sp = 30
#define fan_hi_sp = 100

#define fan_coolant_low 185
#define fan_coolant_high 210
#define fan_sp_lo 75
#define fan_sp_hi 255
#define fan_delay 2000

#define ac_dis_rpm 4700
#define ac_en_rpm  4500
#define ac_dis_tps 90
#define ac_en_tps  30

#define I_FUEL 0   // Index to arrays
#define I_COMP 1
#define I_FAN1 2
#define I_FAN2 3

#define OIL_T  4
#define COOL_T 5
#define FUEL_L 6

#define EVAP   7
#define FUEL_P 8
#define AC_HP  9
#define OIL_P  10
#define THR    11
#define MAP    12


struct calVal {
  uint16_t rawLo;   // lowest A/D RAW sensor reading
  uint16_t rawHi;   // highest A/D RAW sensor reading
  uint16_t calLo;   // lowest gauge displayed value
  uint16_t calHi;   // highest gauge displayed value
};

calVal cal[13] = {          // used to map sensor raw values to real units
  {0,1023,0,10},  // i_fuel  A0  uncal'd
  {0,1023,0,10},  // i_comp  A1  uncal'd
  {0,1023,0,10},  // i_fan1  A2  uncal'd
  {0,1023,0,10},  // i_fan2  A3  uncal'd
  {13230, 57,    0, 212},   // A4-oil_t, VERIFY CAL - (12760 measured cold - connected?)
  {13230, 57,    0, 212},   // A5-cool_t, R1 & R3 330Ω , 5600-56Ω, -2 - 212F, a/d 970-552 - calculated - (13334 measured cold - connected?)
                            // VDO coolant sensor 325-001 marked - VDO germany - D - 6-24 V - max 120C - 801/2/1 - 6 97
  {102,  308,  0, 30},    // A6-fuel_l, CAL-e-e 12/24, R1 & R3 330Ω, 41-250Ω E-F, 0-30 gal, a/d 542-653 - calculated CAL'D should be good - (13338 measured full - connected?)
  {7760,  23600, 0, 7},     // A7-evap_p, PN# 09430128 BX22298 Holds @ 10 in. HO2  0psi=1.457V 7760,  6.5 4.53v" 23600  cal'd E-E
                            // {7080,  24140, 0, 15},    // 7-evap_p, PN# 12209219 4125A Holds @ 10 in. HO2  0psi=1.37V 7330,  7" 4.60v" 23660  cal'd e-e
  {2240,  22550, 0, 80},    // A8-fuel_p,  0psi 2240,  80psi=22550 4.29v @ 80psi-sensor  CAL'D end-end
                            // {2240,  22550, 0, 80},   // 6-fuel_p, 0psi 0.6v 2440, 100psi=3.59v 15950, 150psi-sensor  CAL'D end-end
  {2700,  23865, 0, 150},   // A9-ac_hp - (0-400psi min - not connected)
  {2650,  24220, 0, 100},   // A10-oil_p. oil_p = (((oil_p2 / 204.6) * 32) - 16) - (65526 measured idle - connected?)
                            // {2630,  25165, 0, 100},   // 10-oil_p1 = (((oil_p1 / 204.6) * 32) - 16);  E-ROD
  {296,  840, 0, 100},   // A11-thr, 0.5 – 4.5 volt signal ranging from 0 – 100 % - CAL'D
  {296,     840,  0, 100},   // A12-MAP
};

int fake_raw[15]        = {0,0,0,0,0,  0,0,0,0,0, 0,0,0,0,0};
int curr_val[15]        = {0,0,0,0,0,  0,0,0,0,0, 0,0,0,0,0}; // integer equivalent  -- Should this be type int? ---------------?
int last_val[15]        = {0,0,0,0,0,  0,0,0,0,0, 0,0,0,0,0}; // prior displayed point used to smooth movement of pointer to next location
float damping[15]       = {.2,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0}; // Dampening factor set max change allowed per update (20% = 0.2)
uint16_t refresh[15]    = {0,0,0,0,0,  0,0,0,0,0, 0,0,0,0,0}; // milliseconds between updates
uint16_t timeTarget[15] = {0,0,0,0,0,  0,0,0,0,0, 0,0,0,0,0}; // Rolling target for next refresh


struct settings { 
  int valLT;    // Historical Low
  int valHT;    // Historical High

  int start;    // lowest value shown on scale 
  int crit_l;   // critical alarm point for input - triggers master warning flag
  int warn_l;   // warning value to draw attention - will set gauge color change with upcoming update
  int norm_l;   // start of normal running range
  
  int norm_h;   // end of normal running range
  int warn_h;   // high side warning
  int crit_h;   // High alarm point
  int stop;     // highest displayed scale point
};

settings gauge[15] = {
  {0,0, 0,0,0,0,        0,0,0,0},      // A0 i_fuel
  {0,0, 0,0,0,0,        0,0,0,0},      // A1 i_comp
  {0,0, 0,0,0,0,        0,0,0,0},      // A2 i_fan1
  {0,0, 0,0,0,0,        0,0,0,0},      // A3 i_fan2

  {0,0, 0,90,100,150,   200,230,250,280}, // A4 oil_tG   // maybe reading 
  {0,0, 0,140,160,185,  210,230,250,280}, // A5 cool_tG  // connected, reading something
  {0,0, 0,5,10,10,      30,30,30,30},     // A6 fuel_lG  // working
  {0,0, 0,0,0,0,        20,20,20,25},     // A7 evap_G   // ? maybe working
  {0,0, 0,50,55,58,     62,65,65,80},     // A8 fuel_pG  // working
  
  {0,0, 0,0,0,0,        0,0,200,200},     // A9 ac_hpG  // spasing
  {0,0, 0,15,25,30,     55,70,80,100},    // A10 oil_pG  // connected
  {0,0, 0,0,0,0,        0,90,90,100},     // A11 thrG
  {0,0, 0,0,0,0,        0,0,0,0},         // A12 map
  {0,0, 0,0,0,0,        0,0,0,0},         // A13 vss
  {0,0, 0,0,0,0,        0,0,0,0},         // A14 rpm
};


struct nexGauge {   // LCD Touchscreen text strings used to transfer data  // SHOULD BE raw, val, gauge, pco
  char astext[15];  // Human readable name - used for debug
  char raw[15];     // raw A/D input for debugging
  char text[15];    // gauge value in uint
  char gauge[15];   // Bar Graphs and such
  char pco[15];     // forground color
};


nexGauge nexObj[23] = {
  // Nom,     raw,         text         gauge        color       
  {"i_fuel",  "n43.val=", "n73.val=",  "",  "j73.pco="},   // A0-i_fuel,   nexObg[0]  
  {"i_comp",  "n42.val=", "n72.val=",  "",  "j72.pco="},   // A1-i_comp,   nexObg[1]  
  {"i_fan1",  "n40.val=", "n70.val=",  "",  "j70.pco="},   // A2-i_fan1,   nexObg[2]  
  {"i_fan2",  "n41.val=", "n71.val=",  "",  "j71.pco="},   // A3-i_fan2,   nexObg[3]  
  {"oil_t",   "n4.val=",  "n14.val=",  "j14.pco=",  ""},   // A4-oil_t,    nexObg[4]  

  {"cool_t", "n3.val=",   "n13.val=",  "j13.val=",  "j13.pco="},   // A5-cool_t,   nexObg[5]  
  {"fuel_l", "n5.val=",   "n15.val=",  "j15.pco=",  ""},   // A6-fuel_lG,  nexObg[6]
  {"evap",   "n7.val=",   "n17.val=",  "j17.pco=",  ""},   // A7-evap,     nexObg[7]  
  {"fuel_p", "n6.val=",   "n16.val=",  "j16.pco=",  ""},   // A8-fuel_p,   nexObg[8]  
  {"ac_hp",  "n20.val=",  "n30.val=",  "j30.pco=",  ""},   // A9-ac_hp,    nexObg[9]  

  {"oil_p",  "n2.val=",   "n12.val=",  "j12.val=",  "j12.pco="},   // A10-oil_p,   nexObg[10]  
  {"thr",    "n8.val=",   "n18.val=",  "j18.pco=",  ""},           // A11-thr,     nexObg[11]  
  {"map",    "n9.val=",   "n19.val=",  "j19.pco=",  ""},           // A12-map,     nexObg[12]  
  {"mph",    "n0.val=",   "n10.val=",  "j10.val=",  "j10.pco="},   // 13 A14-mph,     nexObg[13]
  {"rpm",    "n1.val=",   "n11.val=",  "j11.val=",  "j11.pco="},   // 14 A15-rpm,     nexObg[14]

  {"out_t",  "",          "n32.val=",  "",          "n32.pco="},    // 15- air_temp
  {"cond_t", "",          "n31.val=",  "",          "n31.pco="},    // 16- AC-condenser_temp
  {"evap",   "",          "t15.val=",  "",          "t15.pco="},    // 17- Fuel vapor evaprative valve closed
  {"mil",    "",          "t20.val=",  "",          "t20.pco="},    // 18- Engine Light
  {"fuel_en","",          "t13.val=",  "",          "t13.pco="},    // 19- Fuel pump speed control active
  {"cool_lvl","",         "coollvl.val=","",        "coollvl.pco="},// 20- Coolant Low   **** add oil lvl low
  {"acreq","",            "acreq.val=","",          "acreq.pco="},  // 21- A/C Requested}
  {"recirc","",           "crecirc.val=","",        "recirc.pco="}, // 22- A/C Recirc selected}
  // {"loadG",   "j11.val=", "",          "n11.val=",  "j11.pco="},   // 16- 4- load
  // {"advG",    "j12.val=", "",          "n12.val=",  "j12.pco="},   // 17- 5- adv
  // {"voltsG",  "j13.val=", "",          "n13.val=",  "j13.pco="},   // 18- 6- volts
};

struct nexOutputs {   // LCD Touchscreen text strings used to transfer data  // SHOULD BE raw, val, gauge, pco
  char astext[15];  // Human readable name - used for debug
  char raw[15];     // raw A/D input for debugging
  char text[15];    // gauge value in uint
  char bco[15];   // Bar Graphs and such
  char pco[15];     // forground color
};

nexOutputs nexOut[12] = {
  // Nom,       current,     text         background   forground       
  {"PWM_FAN1",  "n70.val=",  "n40.val=",  "",          ""},          // D4-fan1,     nexOut[0]  
  {"PWM_FAN2",  "n71.val=",  "n41.val=",  "",          ""},          // D5-fan2,     nexOut[1]  
  {"PWM_AC",    "n72.val=",  "n42.val=",  "",          ""},          // D2-pwm_ac,   nexOut[2]  
  {"PWM_FUEL",  "n73.val=",  "n43.val=",  "",          ""},          // D3-pwm_fuel, nexOut[3]
  {"MPH_G",     "",          "n44.val=",  "",          ""},          // D11-mph_g,   nexOut[4]
  {"FUEL_G",    "",          "n45.val=",  "",          ""},          // D8-mph_g,    nexOut[5]
  {"OIL_G",     "",          "n46.val=",  "",          ""},          // D9-mph_g,    nexOut[6]
  {"TEMP_G",    "",          "n47.val=",  "",          ""},          // D6-mph_g,    nexOut[7]
  {"VOLT_G",    "",          "n48.val=",  "",          ""},          // D7-mph_g,    nexOut[8]
  {"FUEL_EN",   "",          "",          "t13.bco=",   "t13.pco="},   // D10-mph_g,   nexOut[9]
  {"EVAP_S",    "",          "",          "t15.bco=",   "t15.pco="},   // D13-mph_g,   nexOut[10]
  {"MIL",       "",          "",          "t20.bco=",   "t20.pco="},   // D14-mph_g,   nexOut[11]
};