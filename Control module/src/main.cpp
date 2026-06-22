
//SW version: 1.0
//Functions: 
//Programmer/s: Davide Cagnoni
//Last edit: 12-06-2026

//___________________________________________________________________________Code description _____________________________________________________________________________________________________________________________________________________________
/*
This code is desiged to manage the two axes of an equatorial astronomical mount and a variety of gears and tools connected
to the telescope/camera installed on the mount.
This is a configurable system ables to manage different devices installed on the mount, it is configured
at code level an the entire system is operated throuht the touch display.

The device provide the user as output with:

 >> 3X Stepper motor controller
 >> 4x H bridge 
 >> 4X interrupt pins (for instance, optical rotary encoder inputs)
 >> 4X GPIO pins TTL
 >> 4X Analog inputs pin (0-5V)
 
The following code is configured to manage:
 >> A steppper motor on the declination axes
 >> A DC motor on the right ascension axes (H bridge)
 >> A DC motor for the focuser (H bridge)
 >> An optical encoder to read the right ascension rotation rate
*/

//_______________________________________________________________________Libraries inclusion_______________________________________________________________________________________________
#include <Arduino.h>
#define nexSerial Serial2 // Nextion display on serial port 2, change the serial also in the library file Nextion >> NexConfig.h >> #define nexSerial (row 37)
#include "ArduPID.h"
#include "AccelStepper.h"
#include <Nextion.h>      // Include the nextion library (the official one) https://github.com/itead/ITEADLIB_Arduino_Nextion
#include <H_bridge.h>

//______________________________________________________________________Variables and constant declaration________________________________________________________________________________________

// Debug variables
const long DEBUG_SP_BR = 115200;     // Boud rate for the debug serial port
const bool DEBUG_OUTPUT = HIGH;       // If set to HIGH the program will provide as serial output the debug data, the telemetry wont work correctly due to timing issues in the code execution
long int debug_timer = 0;

// Nextion GUI parameters
byte GUI_page = 0;

//_______________________________________________________________User defined parameters________________________________________
// mount_page parameters
uint32_t manual_ctr_butt_state = 0;
uint32_t DEC_pos_reset_butt_state = 0;
uint32_t DEC_pos_plus_butt_state = 0;
uint32_t DEC_pos_minus_butt_state = 0;
uint32_t RA_lunar_rate_sel_butt_state = 0;
uint32_t RA_solar_rate_sel_butt_state = 0;
uint32_t RA_sideral_rate_sel_butt_state = 0;
uint32_t RA_emisphere_sel_butt_state = 0;
uint32_t RA_selected_rate = 0;    
uint32_t RA_pos_plus_butt_state = 0;
uint32_t RA_pos_minus_butt_state = 0;

// RA axes parameters
int RA_enc_steps = 0; // Steps run by the RA encoder between two subsequent executions of the RA_axes function 
unsigned int RA_pos_time = 0;  // location in time [sec] of the RA axes (from 0 to 65535), no negative value are allowed
long int RA_last_time = 0;

// PID  parameters
double RA_input = 0;
double RA_output = 0;
double RA_setpoint = 0;
double RA_Kp = 1;
double RA_Ki = 0;
double RA_Kd = 0;

// Stepper motors parameters
const byte stepper_accel = 12;
const byte stepper_maxSpd = 32;

// DEC axes parameters
int DEC_pos_deg_100 = 0; // location in degree*100 of the DEC axes

// optic_page parameters
// These variable store the optic_page buttons status
uint32_t FCR_1_butt_state = 0;
uint32_t FCR_2_butt_state = 0;
long int fcr_last_time = 0;
int focus_speed = 0;

// General purpose parameters
unsigned long int loop_last_time = 0;
const int  LOOP_TIME = 50;        // Time between two subsequent executions of the entiree control loop in [ms]

//___________________________________________________________________________Pins numbers declaration_______________________________________________________________________________________________________________________

// Encoders pins
const byte ENC_1_IN_PIN = 2;  // Encoder 1 pin
const byte ENC_2_IN_PIN = 3;  // Encoder 2 pin
const byte ENC_3_IN_PIN = 18;  // Encoder 3 pin

// H bridges pins
const byte H_1_VRS_PIN = 22;  // H bridge verse pin  
const byte H_1_PWM_PIN= 4;    // H bridge PWM out pin
const byte H_2_VRS_PIN = 23;  // H bridge verse pin  
const byte H_2_PWM_PIN= 5;    // H bridge PWM out pin
const byte H_3_VRS_PIN = 24;  // H bridge verse pin  
const byte H_3_PWM_PIN= 6;    // H bridge PWM out pin
const byte H_4_VRS_PIN = 31;  // H bridge verse pin  
const byte H_4_PWM_PIN= 7;    // H bridge PWM out pin

// Stepper motors drivers pins
const byte STP_1_DIR_PIN = 25;
const byte STP_1_STP_PIN = 26;
const byte STP_2_DIR_PIN = 27;
const byte STP_2_STP_PIN = 28;
const byte STP_3_DIR_PIN = 29;
const byte STP_3_STP_PIN = 30;

// GPIO pins
const byte GPIO_0_PIN = 32;
const byte GPIO_1_PIN = 33;
const byte GPIO_2_PIN = 34;
const byte GPIO_3_PIN = 35;

// Analog input pins
const byte AnalogIN_0_PIN = A0;
const byte AnalogIN_1_PIN = A1;
const byte AnalogIN_2_PIN = A2;
const byte AnalogIN_3_PIN = A3;

//___________________________________________________________________Display object definitions_____________________________________________
/*
NexButton PMT = NexButton(9, 2, "PMT");    //Button PMT added  NexButton is the type of object in this case a simple button, "button name" = NexButton(page, ID, object name)  PMT is the pause button in the "Motor test" page
This code is not able to send to arduino a page number above 09, Arduino can't recognize it, so I use the numbers from 00 to 09, the page number on Arduino can be different from the respective page number on the Nextion display 
Every page is seen as a "separated communication channel"
*/

// Pages added as touch events, Page is the type of object in this case a page, 
// "page name" = NexPage(page, ID, object name) page is the number of page on the Nextion GUI
NexPage start_page = NexPage(0, 0, "start_page");   
NexPage mount_page = NexPage(1, 0, "mount_page"); 
NexPage optic_page = NexPage(2, 0, "optic_page");          
NexPage settings_page = NexPage(3, 0, "settings_page");  
NexPage nav_page = NexPage(4, 0, "nav_page");  

// Buttons object added as touch events, the buttons here declared ara grouped by the page they belonged to.
// Not all the buttons on the Nextion GUI require task on the microcontroller, if they do, they are labeled as
// "button name" = NexButton(page, ID, object name) 

// "mount_page" buttons
NexButton DEC_pos_reset_butt = NexButton(1, 7, "p1b0");   // Button used to reset to zero the position of the DEC axes
NexButton DEC_pos_plus_butt = NexButton(1, 10, "p1b1");   // Button used to rotate the DEC axes toward positive angles
NexButton DEC_pos_minus_butt = NexButton(1, 12, "p1b4");  // Button used to rotate the DEC axes toward negative angles

NexDSButton RA_lunar_rate_sel_butt = NexDSButton(1, 3, "p1bt0");    // Button used to select the "Lunar" rotation rate on the RA axes
NexDSButton RA_solar_rate_sel_butt = NexDSButton(1, 4, "p1bt1");    // Button used to select the "Solar" rotation rate on the RA axes
NexDSButton RA_sideral_rate_sel_butt = NexDSButton(1, 5, "p1bt2");  // Button used to select the "Sideral" rotation rate on the RA axes

NexButton RA_pos_plus_butt = NexButton(1, 13, "p1b3");                // Button used to rotate the RA axes forward in time
NexButton RA_pos_minus_butt = NexButton(1, 11, "p1b2");               // Button used to rotate the RA axes backward in time
NexDSButton manual_ctr_butt = NexDSButton(1, 9, "p1bt3");             // Two state button used as a start/stop button and to switch to the manual control of the mount

// "optic_page" buttons
// Buttons used to rotate the optic focuser motor
NexButton FCR_1_butt = NexButton(2, 4, "p2b0");    
NexButton FCR_2_butt = NexButton(2, 3, "p2b1");    

// "settings_page" buttons
NexDSButton RA_emisphere_sel_butt = NexDSButton(3, 5, "p3bt0");     // Two state button used to select the RA rotation verse

// These objects listed are the objects that will send a "Touch Event" to the microcontroller
NexTouch *nex_objects_list[] =   
 {
 // Pages added as touch events
 &start_page,   
 &mount_page,     
 &optic_page,           
 &settings_page,  
 &nav_page,
 
 // "mount_page" buttons
 &DEC_pos_reset_butt,
 &DEC_pos_plus_butt,
 &DEC_pos_minus_butt,
 &RA_lunar_rate_sel_butt,
 &RA_solar_rate_sel_butt,
 &RA_sideral_rate_sel_butt,
 &manual_ctr_butt,
 &RA_pos_plus_butt,
 &RA_pos_minus_butt,
 
 // "optic_page" buttons
 &FCR_1_butt,
 &FCR_2_butt,

 // "settings_page" buttons
 &RA_emisphere_sel_butt,
 NULL 
 };
 
//_________________________________________________________________________Functions declaration_____________________________________________________________________________

//_________________________________________________________________________User defined functions declaration_____________________________________________________________________________

// Nextion pages functions, every time a page is touched, the related funciton is called to update the GUI_page variable
void start_page_TouchEvent(void *ptr);
void mount_page_TouchEvent(void *ptr);
void optic_page_TouchEvent(void *ptr);
void settings_page_TouchEvent(void *ptr);
void nav_page_TouchEvent(void *ptr);  

// Mount aces control
void dec_axes_fcn(bool debug_out, uint32_t manual_ctr, uint32_t pos_reset, uint32_t pos_plus, uint32_t pos_minus);
void right_asc_axes_fcn(bool debug_out, uint32_t manual_ctr, byte emisphere, byte rate_selected, uint32_t pos_plus, uint32_t pos_minus);

// Focuser control
void focuser_fcn(bool debug_out, uint32_t fcr_plus, uint32_t fcr_minus);

// Nextion touch events functions
// "mount_main_page" buttons
void DEC_pos_reset_butt_PushEvent(void *ptr);
void DEC_pos_reset_butt_PopEvent(void *ptr);

void DEC_pos_plus_butt_PushEvent(void *ptr);
void DEC_pos_plus_butt_PopEvent(void *ptr);

void DEC_pos_minus_butt_PushEvent(void *ptr);
void DEC_pos_minus_butt_PopEvent(void *ptr);

void RA_rate_set_TouchEvent(void *ptr);

void manual_ctr_butt_TouchEvent(void *ptr);

void RA_pos_plus_butt_PushEvent(void *ptr);
void RA_pos_plus_butt_PopEvent(void *ptr);

void RA_pos_minus_butt_PushEvent(void *ptr);
void RA_pos_minus_butt_PopEvent(void *ptr);

// "optic_page" buttons
void FCR_1_butt_PushEvent(void *ptr); 
void FCR_1_butt_PopEvent(void *ptr); 

void FCR_2_butt_PushEvent(void *ptr); 
void FCR_2_butt_PopEvent(void *ptr);  

// "settings_page" buttons
void RA_emisphere_sel_butt_TouchEvent(void *ptr);

// Encoder 1 steps counter function
void enc_1_stp_counter(); 

// Nextion GUI debug
void Nextion_debug(int cycle_time);  // Time between a debug data package and the next one

//__________________________________________________________________User defined object___________________________ 
ArduPID RA_PID;

//_________________________________________________________________________Embdded objects inizialization_____________________________________________________________________-

//Nextion myNextion(&Serial2 , 115200); // Nextion GUI serial port 

// Stepper motors drivers
AccelStepper DEC_axes_motor(AccelStepper::DRIVER, STP_1_STP_PIN, STP_1_DIR_PIN);

/* Available commands for the stepper drivers:

stepper_#.setAcceleration(acceleration); // Define the motor acceleration
stepper_#.setMaxSpeed(speed);            // Define the maximum rotating speed

stepper_#.move(steps);              // Move of "steps" steps (+/-)
stepper_#.moveTo(steps);     // Move to the position "steps", no code will be executed until this operation ends

stepper_#setSpeed()   // Used to set a rotation speed negletting the acceleration
stepper_#.runSpeed(); // Rotate the motor at the speed specified in setSpeed()
 
stepper_#.currentPosition();   // Return the stepper current position in steps
stepper.setCurrentPosition(0); // To set the current motor position to the value specified 
 
stepper_#.enableOutputs();   // The driver is managing the motor
stepper_#.disableOutputs();  // The driver is turned off, the motor could be moved freely
*/

// H bridge object definition for the RA_axes motor and the Focuser motor
H_bridge RA_axes_motor(H_1_VRS_PIN, H_1_PWM_PIN);
H_bridge focuser_motor(H_2_VRS_PIN, H_2_PWM_PIN);

//________________________________________________________________________________Setup and Loop__________________________________________________________________________________________________________________________________________________

void setup() 
 {
 // Setting the debug output port if debug is requested
 if(DEBUG_OUTPUT == HIGH)
  {
  Serial.begin(DEBUG_SP_BR); 
  delay(500);
  Serial.println("Debug out active"); 
  }

 Serial2.begin(115200); // Nextion display serial port

 // GPIO pins setup
 pinMode(GPIO_0_PIN, OUTPUT);
 pinMode(GPIO_1_PIN, OUTPUT);
 pinMode(GPIO_2_PIN, OUTPUT);
 pinMode(GPIO_3_PIN, OUTPUT);
  
 // Encoder 1 interrupt pin set 
 attachInterrupt(digitalPinToInterrupt(ENC_1_IN_PIN), enc_1_stp_counter, FALLING); 

 // RA control function PID
 RA_PID.begin(&RA_input, &RA_output, &RA_setpoint, RA_Kp, RA_Ki, RA_Kd);
 RA_PID.setOutputLimits(0, 255);
 RA_PID.setBias(255.0 / 2.0);
 RA_PID.setWindUpLimits(-10, 10); // Groth bounds for the integral term to prevent integral wind-up
 RA_PID.start();
 
 // RA_axes and Focuser object init
 RA_axes_motor.begin();
 focuser_motor.begin();

 // Stepper motors set
 DEC_axes_motor.setAcceleration(stepper_accel);
 DEC_axes_motor.setMaxSpeed(stepper_maxSpd);

 // Functions called from the touch event on the Nextion GUI
 //PMT.attachPush(PMTPushCallback);  // When the object is pushed this function is called
 //PMT.attachPop(PMTPopCallback);    // When the object is released this function is called
 
 // Pages events
 mount_page.attachPush(mount_page_TouchEvent);       //Page press event every time the GUI enter on the page "mount_page"
 optic_page.attachPush(optic_page_TouchEvent);             //Page press event every time the GUI enter on the page "optic_page"
 settings_page.attachPush(settings_page_TouchEvent);       //Page press event every time the GUI enter on the page "settings_page"
 nav_page.attachPush(nav_page_TouchEvent);   //Page press event every time the GUI enter on the page "nav_page"

 // "mount_page" objects, each button call its "_TouchEvevent" function every time is pressed or pushed 
 DEC_pos_reset_butt.attachPush(DEC_pos_reset_butt_PushEvent, &DEC_pos_reset_butt);
 DEC_pos_reset_butt.attachPop(DEC_pos_reset_butt_PopEvent, &DEC_pos_reset_butt);

 DEC_pos_plus_butt.attachPush(DEC_pos_plus_butt_PushEvent, &DEC_pos_plus_butt);
 DEC_pos_plus_butt.attachPop(DEC_pos_plus_butt_PopEvent, &DEC_pos_plus_butt);
 
 DEC_pos_minus_butt.attachPush(DEC_pos_minus_butt_PushEvent, &DEC_pos_minus_butt);
 DEC_pos_minus_butt.attachPop(DEC_pos_minus_butt_PopEvent, &DEC_pos_minus_butt);
 
 // The button used to set the RA rate call the same function "RA_rate_set_TouchEvent" to get the selected rate
 RA_lunar_rate_sel_butt.attachPop(RA_rate_set_TouchEvent, &RA_lunar_rate_sel_butt);
 RA_solar_rate_sel_butt.attachPop(RA_rate_set_TouchEvent, &RA_solar_rate_sel_butt);
 RA_sideral_rate_sel_butt.attachPop(RA_rate_set_TouchEvent, &RA_sideral_rate_sel_butt); 
 
 manual_ctr_butt.attachPop(manual_ctr_butt_TouchEvent, &manual_ctr_butt); 

 RA_pos_plus_butt.attachPush(RA_pos_plus_butt_PushEvent, &RA_pos_plus_butt);
 RA_pos_plus_butt.attachPop(RA_pos_plus_butt_PopEvent, &RA_pos_plus_butt); 

 RA_pos_minus_butt.attachPush(RA_pos_minus_butt_PushEvent, &RA_pos_minus_butt);
 RA_pos_minus_butt.attachPop(RA_pos_minus_butt_PopEvent, &RA_pos_minus_butt);

 // "optic_page" objects
 FCR_1_butt.attachPush(FCR_1_butt_PushEvent, &FCR_1_butt);
 FCR_1_butt.attachPop(FCR_1_butt_PopEvent, &FCR_1_butt);
 
 FCR_2_butt.attachPush(FCR_2_butt_PushEvent, &FCR_2_butt);
 FCR_2_butt.attachPop(FCR_2_butt_PopEvent, &FCR_2_butt);

 // "settings_page" buttons
 RA_emisphere_sel_butt.attachPush(RA_emisphere_sel_butt_TouchEvent, &RA_emisphere_sel_butt);
 }

void loop() 
 {
 nexLoop(nex_objects_list); // Check for any touch event occoured on the Nextion GUI

 if((millis() - loop_last_time) > LOOP_TIME)
  {    
  detachInterrupt(ENC_1_IN_PIN);
  //___________________________________________________Write loop code here______________________________________________
  
  //Nextion_debug(1000);  // Nextion input debug function called

  focuser_fcn(0, FCR_1_butt_state, FCR_2_butt_state);
  right_asc_axes_fcn(1, manual_ctr_butt_state, RA_emisphere_sel_butt_state, RA_selected_rate, RA_pos_plus_butt_state, RA_pos_minus_butt_state);
  //dec_axes_fcn(0, manual_ctr_butt_state, DEC_pos_reset_butt_state, DEC_pos_plus_butt_state, DEC_pos_minus_butt_state);

  //__________________________________________________________Stop loop code here_______________________________________________________
  loop_last_time = millis(); 
  attachInterrupt(digitalPinToInterrupt(ENC_1_IN_PIN), enc_1_stp_counter, FALLING);
  }
 } 
 
//_________________________________________________________________________Functions_________________________________________________________________________

//_____________________________________________________User defined functions____________________________________________________________________________

void Nextion_debug(int cycle_time) 
 {
 if((millis() - debug_timer) > cycle_time)
  {
  Serial.print("Active page on the GUI: ");
  Serial.println(GUI_page);
  Serial.println();
  Serial.println();
  Serial.println("====== mount page buttons state: ======");
  Serial.println();
  Serial.print("DEC_pos_reset_butt_state: ");
  Serial.println(DEC_pos_reset_butt_state); 
  Serial.print("DEC_pos_plus_butt_state: ");
  Serial.println(DEC_pos_plus_butt_state);
  Serial.print("DEC_pos_minus_butt_state: ");
  Serial.println(DEC_pos_minus_butt_state); 
  Serial.println();
  Serial.print("RA_lunar_rate_sel_butt_state: ");
  Serial.println(RA_lunar_rate_sel_butt_state); 
  Serial.print("RA_solar_rate_sel_butt_state: ");
  Serial.println(RA_solar_rate_sel_butt_state); 
  Serial.print("RA_sideral_rate_sel_butt_state: ");
  Serial.println(RA_sideral_rate_sel_butt_state); 
  Serial.print("RA_selected_rate: ");
  Serial.println(RA_selected_rate); 
  Serial.print("RA_pos_plus_butt_state: ");
  Serial.println(RA_pos_plus_butt_state);
  Serial.print("RA_pos_minus_butt_state: ");
  Serial.println(RA_pos_minus_butt_state); 
  Serial.println();
  Serial.print("manual_ctr_butt_state: ");
  Serial.println(manual_ctr_butt_state);
  Serial.println();
  Serial.println();
   
  Serial.println("====== optic page buttons state: ======");
  Serial.println();
  Serial.print("FCR_1_butt_state: ");
  Serial.println(FCR_1_butt_state); 
  Serial.print("FCR_2_butt_state: ");
  Serial.println(FCR_2_butt_state); 
  Serial.println();
  Serial.println();
   
  Serial.println("====== settings page buttons state: ======");
  Serial.println();
  Serial.print("RA_emisphere_sel_butt_state: ");
  Serial.println(RA_emisphere_sel_butt_state); 
  Serial.println();
  Serial.println("--------------------------- END ---------------------------");
  debug_timer = millis();
  }
 }

void dec_axes_fcn(bool debug_out, uint32_t manual_ctr, uint32_t pos_reset, uint32_t pos_plus, uint32_t pos_minus)
 {
 const int DEC_steps2deg_100_conv_fac = 34; // Conversion factor from steps to degree*100 for the DEC axes
 int DEC_manual_rate = 30;                  // DEC axes manual control rotation rate (stp/sec)
 char DEC_pos_deg_100_char;

 // DEC axes control
 if(manual_ctr == 1)
  {
  DEC_pos_deg_100 = DEC_axes_motor.currentPosition()*DEC_steps2deg_100_conv_fac; // Updating DEC axes position in [degree*100]
  DEC_axes_motor.enableOutputs();

  if(pos_plus == 1 && pos_plus == 0)
   {
   DEC_axes_motor.setSpeed(DEC_manual_rate);
   DEC_axes_motor.runSpeed();
   }

  if(pos_minus == 1 && pos_minus == 0)
   {
   DEC_axes_motor.setSpeed(-DEC_manual_rate); 
   DEC_axes_motor.runSpeed();
   }

  if((pos_plus && pos_minus) == 1 || (pos_plus || pos_minus) == 0)
   {
   DEC_axes_motor.setSpeed(0);
   DEC_axes_motor.runSpeed();
   }
  }
 else
  {
  DEC_axes_motor.disableOutputs();
  }
 
 // DEC axes debug messages
 if(pos_reset == 1)
  DEC_axes_motor.setCurrentPosition(0); // Reseting DEC axes position

 // DEC axes position sending to the Nextion GUI, data sent only if "mount_page" is ctive on the GUI (GUI_page = 1)
 if(GUI_page == 1)
  {
  char DEC_pos_deg_100_char = DEC_pos_deg_100; // Data need to be sent as "char" variables
  Serial2.print("DEC_pos.val=");               // The data "label" prior to the actual data value needs to be the same used on teh Nextion GUI           
  Serial2.print(DEC_pos_deg_100_char); 
  // These three lines act as "data string terminator"
  Serial2.write(0xff);                  
  Serial2.write(0xff);
  Serial2.write(0xff);
  }

 // DEC axes debug messages
 if(debug_out == HIGH)
  {
  Serial.print("mount_page_fcn.DEC_pos_deg_100_char [degree*100]:");
  Serial.println(DEC_pos_deg_100_char);
  } 
 }
 
void right_asc_axes_fcn(bool debug_out, uint32_t manual_ctr, byte emisphere, byte rate_selected, uint32_t pos_plus, uint32_t pos_minus)
 {
 // RA axes parameters 
 uint16_t RA_curr_rate_StpS = 0; // Rotation rate in encoder steps per second
 uint16_t RA_curr_rate_arcSec = 0;
 const int RA_steps2arc_conv_fac = 45; // Conversion factor from steps to [arcseconds] for RA axes

 const int PWM_VAL_INC = 8;   // A higher value allows a slower speed increase, see formula in the next few rows
 const int MAX_PWM_VAL = 255; // Maximum micro controller analogWrite value
 const int STR_PWM_VAL = 20;  // Starting analogWrite value
 const int MAX_RA_VAL = 200; // Maximum analogWrite value applicable to the right ascension axes motor

 int RA_rate_error = 0;
 int RA_speed = 0;


 int out;
 // Rotation rates
 byte lunar_rate = 1000;
 byte solar_rate = 2000;
 byte sideral_rate = 3000;
 byte RA_manual_rate = 40;

 RA_curr_rate_StpS = 1000*RA_enc_steps / LOOP_TIME ;      // RA rotation rate in steps/ms
 RA_curr_rate_arcSec = RA_curr_rate_StpS * RA_steps2arc_conv_fac  ;  // RA rotation rate in arcseconds/s
 RA_input = RA_curr_rate_arcSec;

 if(debug_out == HIGH)
  {
  Serial.print("RA_curr_rate_StpS = ");
  Serial.println(RA_curr_rate_StpS);
  Serial.print("RA_enc_steps = ");
  Serial.println(RA_enc_steps);
  }

 RA_enc_steps = 0; // Encoder steps counter reset
 

 // RA axes rotation rate control
 if(manual_ctr == 0)
  {
  switch(rate_selected)
   {
   case 0:
    out = 0;
    RA_setpoint = 0;
    RA_PID.compute(); 
    RA_axes_motor.stop();
   break;
  
   case 1:
   out = 50;
    RA_setpoint = lunar_rate; 
    RA_PID.compute(); 
   break;
  
   case 2:
   out = 100;
    RA_setpoint = solar_rate;
    RA_PID.compute(); 
   break;
  
   case 3:
   out = 150;
    RA_setpoint = sideral_rate;
    RA_PID.compute(); 
   break;
  
   default:
   break;
   }

  if(emisphere == 0)
   RA_axes_motor.run(HIGH, MAX_PWM_VAL - out); 
 
  if(emisphere == 1)
   RA_axes_motor.run(LOW, out);  
  }
 else  // RA axes manual control
  {
  if((pos_plus || pos_minus) == 1 && (pos_plus && pos_minus) == 0)
   {  
   RA_speed = STR_PWM_VAL + (millis() - RA_last_time)/PWM_VAL_INC;
   if(RA_speed > MAX_RA_VAL)
   RA_speed = MAX_RA_VAL;
   }
  else 
   {
   RA_last_time = millis(); 
   RA_speed = 0;  
   }

  // The two RA_axes_motor.run are have differents values for the PWM due to hardware constraints in the H-bridge module
  if(pos_plus == 1 && pos_minus == 0)
   RA_axes_motor.run(HIGH, MAX_PWM_VAL - RA_speed); 
 
  if(pos_minus == 1 && pos_plus == 0)
   RA_axes_motor.run(LOW, RA_speed);  

  if((pos_plus && pos_minus) == 1 || (pos_plus || pos_minus) == 0)
   {
   RA_axes_motor.stop();
   RA_speed = 0;
   }  
  } 
 } 

// Focuser control
void focuser_fcn(bool debug_out, uint32_t fcr_plus, uint32_t fcr_minus)
 {
 bool focus_direction = LOW;
 const int PWM_VAL_INC = 8;   // A higher value allows a slower speed increase, see formula in the next few rows
 const int MAX_PWM_VAL = 255; // Maximum micro controller analogWrite value
 const int STR_PWM_VAL = 20;  // TStarting analogWrite value
 const int MAX_FCR_VAL = 200; // Maximum analogWrite value applicable to the focuser motor
 
 if((fcr_plus || fcr_minus) == 1 && (fcr_plus && fcr_minus) == 0)
  {  
  focus_speed = STR_PWM_VAL + (millis() - fcr_last_time) / PWM_VAL_INC;
  if(focus_speed > MAX_FCR_VAL)
   focus_speed = MAX_FCR_VAL;
  }
 else 
  {
  fcr_last_time = millis(); 
  focus_speed = 0;  
  }
  
 // The two focuser.run are have differents values for the PWM due to hardware constraints in the H-bridge module
 if(fcr_plus == 1 && fcr_minus == 0)
  focuser_motor.run(HIGH, MAX_PWM_VAL-focus_speed); 

 if(fcr_minus == 1 && fcr_plus == 0)
  focuser_motor.run(LOW, focus_speed); 

 if((fcr_plus && fcr_minus) == 1 || (fcr_plus || fcr_minus) == 0)
  {
  focuser_motor.stop();
  focus_speed = 0;
  }  
    
  
 if(debug_out == HIGH)
  {
  Serial.print("optic_page_fcn.focus_speed = ");
  Serial.println(focus_speed);
  Serial.print("optic_page_fcn.focus_direction = ");
  Serial.println(focus_direction);
  }
 }

// Encoder 1 step counter
void enc_1_stp_counter()
 {
 RA_enc_steps++;
 }
 
//__________________________________________Nextion GUI callback events_____________________________________________

//__________________________________________Pages_____________________________________________
void start_page_TouchEvent(void *ptr)  
 {
 GUI_page = 0;
 }  

void mount_page_TouchEvent(void *ptr)  
 {
 GUI_page = 1;
 }  
 
void optic_page_TouchEvent(void *ptr)  
 {
 GUI_page = 2;
 } 
 
void settings_page_TouchEvent(void *ptr)  
 {
 GUI_page = 3;
 } 

void nav_page_TouchEvent(void *ptr)  
 {
 GUI_page = 4;
 } 
 
//__________________________________________Objects in "mount_page"_____________________________________________

void DEC_pos_reset_butt_PushEvent(void *ptr)  
 {
 DEC_pos_reset_butt_state = 1;
 DEC_axes_motor.setCurrentPosition(0); 
 }  

 void DEC_pos_reset_butt_PopEvent(void *ptr)  
 {
 DEC_pos_reset_butt_state = 0;
 } 

void DEC_pos_plus_butt_PushEvent(void *ptr)  
 {
 DEC_pos_plus_butt_state = 1;
 } 
void DEC_pos_plus_butt_PopEvent(void *ptr)  
 {
 DEC_pos_plus_butt_state = 0;
 } 
 
void DEC_pos_minus_butt_PushEvent(void *ptr)  
 {
 DEC_pos_minus_butt_state  = 1;
 } 
void DEC_pos_minus_butt_PopEvent(void *ptr)  
 {
 DEC_pos_minus_butt_state  = 0;
 }
 
void manual_ctr_butt_TouchEvent(void *ptr)  
 {
 manual_ctr_butt.getValue(&manual_ctr_butt_state);
 }
 
void RA_pos_plus_butt_PushEvent(void *ptr)  
 {
 RA_pos_plus_butt_state = 1;
 }
void RA_pos_plus_butt_PopEvent(void *ptr)  
 {
 RA_pos_plus_butt_state = 0;
 } 
 
void RA_pos_minus_butt_PushEvent(void *ptr)  
 {
 RA_pos_minus_butt_state = 1;
 }
void RA_pos_minus_butt_PopEvent(void *ptr)  
 {
 RA_pos_minus_butt_state = 0;
 }
  
void RA_pos_reset_butt_TouchEvent(void *ptr)  
 {
 RA_enc_steps = 0; // Resetting RA axes position
 }

void RA_rate_set_TouchEvent(void *ptr)  
 {
 // Each time this function is called, the state of all the three rate button is registered, and then the correct rate is set
 RA_lunar_rate_sel_butt.getValue(&RA_lunar_rate_sel_butt_state);
 RA_solar_rate_sel_butt.getValue(&RA_solar_rate_sel_butt_state);
 RA_sideral_rate_sel_butt.getValue(&RA_sideral_rate_sel_butt_state);
 
 if(RA_lunar_rate_sel_butt_state || RA_solar_rate_sel_butt_state || RA_sideral_rate_sel_butt_state == 0)
  RA_selected_rate = 0; // Any rate selected

 if(RA_lunar_rate_sel_butt_state == 1 && (RA_solar_rate_sel_butt_state && RA_sideral_rate_sel_butt_state) == 0)
  RA_selected_rate = 1; // Lunar rate selected
 
 if(RA_solar_rate_sel_butt_state == 1 && (RA_lunar_rate_sel_butt_state && RA_sideral_rate_sel_butt_state) == 0)
  RA_selected_rate = 2; // Solar rate selected
 
 if(RA_sideral_rate_sel_butt_state == 1 && (RA_lunar_rate_sel_butt_state && RA_solar_rate_sel_butt_state) == 0)
  RA_selected_rate = 3; // Sideral rate selected
 }  
 
//__________________________________________Objects in "optic_page"_____________________________________________

void FCR_1_butt_PushEvent(void *ptr)  
 {
 FCR_1_butt_state = 1;
 } 
void FCR_1_butt_PopEvent(void *ptr)  
 {
 FCR_1_butt_state = 0;
 }  
 
void FCR_2_butt_PushEvent(void *ptr)  
 {
 FCR_2_butt_state = 1; 
 } 
void FCR_2_butt_PopEvent(void *ptr)  
 {
 FCR_2_butt_state = 0; 
 } 

//_________________________________________________________________Objects in "settings_page"_____________________________________________

void RA_emisphere_sel_butt_TouchEvent(void *ptr)  
 {
 RA_emisphere_sel_butt.getValue(&RA_emisphere_sel_butt_state);
 }

