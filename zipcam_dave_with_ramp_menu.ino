/*********************
 *
 *  Charlotte firmware, beta
 *   prepared for movie theater shot
 *   2013 November
 *  (c) David Koch  All rights reserved
 *    code by Dounan Hu, some tweaks by Pete Weisz
 *
 **********************/

/*  changes
 *    20131125-    began tracking changes
 *                  removed wheel menu (underlying math still there)
 *                  added exposure delay, but it isn't really right... quick fix
 *                  added exposure delay menu
 *                  cleaned up code for legibility (no more cascading if's)
 *                  changed runtime config paramters to uint16_t
 *                     also changed this in encoder(), DisplayTime(), DisplayDist()
 *                  prevented division by zero in recompute()
 *                  made parameter-editing step-size code more compact
 *                  flipped function of up/down buttons (up now goes to previous menu)
 *    20140129     removed some confusing vestigial stuff: wheel, units().
 *    20140930		adding the following
 *						bulb ramping (on/off)
 *						shutter spd initial (ms)
 *						shutter spd final (ms)
 *						ramp start time (h/m/s/ms)
 *						ramp end time (h/m/s/ms)
 *
 *   todo:
 *          figure out consequences of adding the exposure delay, and fix them
 *          change n decimal places for distance
 *          add scale factor for real distance units (hard code to wheel size?)
 *          delay after start, before motor starts (just once)
 *          reversing the motor?
 *          eeprom save settings
 *          sanity limits to user-configurable parameters (since some combination
 *              of settings cannot be executed)
 */

// some useful macros -PW
#define ARRAY_N_ELEMENTS(array)  ( sizeof(array) / sizeof(array[0]) )
#define PREVENT_DIV_ZERO(integer)  ( integer ? integer : 1 )

// include the library code:
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <AFMotor.h>
#include <AccelStepper.h>
#include <Stepper.h>

#include <math.h>

// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// These #defines make it easy to set the backlight color
#define LCD_OFF 0x0
#define LCD_ON 0x1
//#define RED 0x1
//#define YELLOW 0x3
//#define GREEN 0x2
//#define TEAL 0x6
//#define BLUE 0x4
//#define VIOLET 0x5
//#define WHITE 0x7

const int steps_per_revolution = 200; //steps per revoluntion = 360 degrees / step degree
const int shutterPin = 4;

const int MENU_DELAY = 100;
const float WHEEL_SIZE = 10;
const float WIRE_SIZE = 1;
const float RAIL_SIZE = 5;

// default values for config parameters that start as non-zero
const uint16_t DEFAULT_EXPOSURE_DELAY_MS = 100;      // -pw 20131125

// Arduino pins the encoder is attached to. Attach the center to ground.
#define ROTARY_PIN1 14
#define ROTARY_PIN2 15


// change this to the number of steps on your motor
#define STEPS 200
// define stepper motor pins
#define MOT_A1 10
#define MOT_A2 11
#define MOT_B1 8
#define MOT_B2 7
// inches moved in a single step
#define STEP_INTERVAL_LIMIT 0.01305
Stepper stepper(STEPS, MOT_A1, MOT_A2, MOT_B1, MOT_B2);
#define MOTOR_ENABLE_PIN 9
#define MOTOR_PWM_A 5
#define MOTOR_PWM_B 6
// 41% duty cycle PWM on pin 9
// 5.16V / 12.6V = ~41%
// Range 12.6-10.8V, 41-48%, 105-122
// 50 =  .48A
// 75 = .71A
// 68 ~ .638
// 122 ~1.24 @ 12.6V
//int pwmTorque = 122;
#define MOT_TORQUE 122

AF_Stepper motor(200, 1);


void setup() {
  // Debugging output
  Serial.begin(9600);

  //enable motor pin
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN, LOW);

  analogWrite(MOTOR_PWM_A, MOT_TORQUE);
  analogWrite(MOTOR_PWM_B, MOT_TORQUE);
  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Charlotte Dolly");
  lcd.setCursor(0, 1);
  lcd.print("Controller");
  lcd.setBacklight(LCD_ON);
  lcd.cursor();
  pinMode(shutterPin, OUTPUT);

  //rotary encoder
  rotary_init();

  delay(2000);
  lcd.clear();




}

boolean run = false;

uint8_t i = 0;
uint8_t button_count = 0;

/*
 *    menu system setup ... redid this -PW
 */

enum MenuItems {
  TOTAL_TIME,
  TOTAL_DIST,
  EXPOSURES,
  EXPOSURE_DELAY,
  SPEEDY,
  SHUTTER_DIST,
  SHUTTER_INTERVAL,
  HOLD_MOTOR_ENABLE,
  BULB_RAMP_ENABLE,
  SHUTTER_SPD_INITIAL,
  SHUTTER_SPD_FINAL,
  RAMP_START_TIME,
  RAMP_END_TIME,
  START
};

char* Menu[] = {
  "Run Time",
  "Run Distance",
  "Exposures",
  "Exp Delay (ms)",
  "Speed",
  "Shutter Distance",
  "Shutter Interval",
  "Hold Motor Btwn. Shots",
  "Bulb Ramp On/Off",
  "Shutter Spd Initial",
  "Shutter Spd Final",
  "Ramp Start Time",
  "Ramp End Time",
  "START MOTOR"
};

#define N_MENUS   ARRAY_N_ELEMENTS(Menu)

uint16_t  menu_increments[N_MENUS][6] = {
  {1, 60, 3600},              // tot_time
  {1, 12},                    // tot_dist
  {1, 10, 100, 1000, 10000},  // exposures
  {10, 100, 1000, 10000},     // exposure_delay
  {1},                        // speed
  {1},                        // shutter distance
  {1},                        // shutter interval
  {1},                        // hold motor enable
  {1},                        // bulb ramp enable
  {1, 10, 100, 1000},         // shutter spd initial
  {1, 10, 100, 1000},         // shutter spd final
  {1, 60, 3600},              // ramp start time
  {1, 60, 3600},              // ramp end time
  {1}                         // start motor
};

uint8_t  n_menu_increments[N_MENUS] =   {
  3,                       // tot_time
  2,                       // tot_dist
  5,                       // exposures
  4,                       // exposure_delay
  1,                       // speed
  1,                       // shutter distance
  1,                       // shutter interval
  1,                       // hold motor enable
  1,                       // bulb ramp enable
  4,                       // shutter spd initial
  4,                       // shutter spd final
  3,                       // ramp start time
  3,                       // ramp end time
  1                        // start motor
};

uint8_t menu = 0;
uint8_t menu_increment_selection[N_MENUS];

/***********   commented this out   -PW 20131125
* uint8_t calib_menu = 0;
* enum CalibMenuItems {WIRE, WHEEL, RAILS};
* CalibMenu[calib_menu_tot] = {"Wire", "Wheel", "Rails"};
***********************************/

// user configurable runtime parameters
uint16_t tot_time = 60;        // 5 minutes default
uint16_t tot_dist = 13; // 13 inches ~ 1000 steps / 5 * 360 degrees
uint16_t exposures = 10;
uint16_t exposure_delay = DEFAULT_EXPOSURE_DELAY_MS;
uint16_t hold_motor_enable = 0;
uint16_t bulb_ramp_enable = 0;
uint16_t shutter_spd_initial = 0; // 1/20 sec default
uint16_t shutter_spd_final = 34;  // 60 seconds default
uint16_t ramp_start_time = 120; // 2 minutes default
uint16_t ramp_end_time = 240;   // 4 minutes default

// runtime vars
uint16_t shut_spd_current = 50;

// automatically calculated runtime parameters
uint16_t motor_speed = 1;
float shutter_dist = 1.0;
uint16_t shutter_interval = 1;
float run_dist = 1.0;
uint16_t exposures_remaining = 1;
uint16_t ramp_time_delta = ramp_end_time - ramp_start_time;
uint16_t exp_ramp_start = 1.0 * ramp_start_time / tot_time * exposures;
uint16_t exp_ramp_end = 1.0 * ramp_end_time / tot_time * exposures;
float ramp_pos = 0.0; // time position in ramping goes from 0.0 to 1.0;

//temp values
uint16_t tempInt = 1;
float tempFloat = 1.0;

unsigned long last_exposure = 0;

char rotary_result;
boolean clear_lcd =  true;

void encoder(uint16_t &count, int chunk) {
  // NOTE! changed &count from int to uint16_t !!!
  while (true) {
    rotary_result = rotary_process();
    if (rotary_result) {
      rotary_result == 0x40 ? count += chunk : count -= chunk;
      //      if (count%5==0) {
      //          clearValue();
      displayValue();
      //      }
      clear_lcd = true;
    }
    if (clear_lcd) {
      clearValue();
      displayValue();
      clear_lcd = false;
    }
    if (lcd.readButtons()) {
      break;
    }
  }
}

const char ttable[7][4] = {
  {0x0, 0x2, 0x4,  0x0}, {0x3, 0x0, 0x1, 0x40},
  {0x3, 0x2, 0x0,  0x0}, {0x3, 0x2, 0x1,  0x0},
  {0x6, 0x0, 0x4,  0x0}, {0x6, 0x5, 0x0, 0x80},
  {0x6, 0x5, 0x4,  0x0},
};

volatile char rotary_state = 0;

/* Call this once in setup(). */
void rotary_init() {
  pinMode(ROTARY_PIN1, INPUT);
  pinMode(ROTARY_PIN2, INPUT);
  digitalWrite(ROTARY_PIN1, HIGH);
  digitalWrite(ROTARY_PIN2, HIGH);
}

/* Read input pins and process for events. Call this either from a
 * loop or an interrupt (eg pin change or timer).
 *
 * Returns 0 on no event, otherwise 0x80 or 0x40 depending on the direction.
 */
char rotary_process() {
  char pinstate = (digitalRead(ROTARY_PIN2) << 1) | digitalRead(ROTARY_PIN1);
  rotary_state = ttable[rotary_state & 0xf][pinstate];
  return (rotary_state & 0xc0);
}

// cleaned up
void recompute() { 
  //recalculate configuration values
  switch (menu) {
    case TOTAL_TIME:
      if (tot_time < 1) {
        tot_time = 1;
      }
      motor_speed = (tot_dist * 1.0 / PREVENT_DIV_ZERO(tot_time)) + 1.0;
      shutter_interval = (tot_time * 1.0 / PREVENT_DIV_ZERO(exposures));
      if (bulb_ramp_enable >= 1) {
        if (ramp_end_time > tot_time) {
          ramp_end_time = tot_time;
        }
        if (ramp_start_time >= ramp_end_time) {
          ramp_start_time = ramp_end_time - 1;
        }
        ramp_time_delta = ramp_end_time - ramp_start_time;
        Serial.print("Ramp Time Delta: ");
        Serial.println(ramp_time_delta);
        exp_ramp_start = 1.0 * ramp_start_time / tot_time * exposures;
        exp_ramp_end   = 1.0 * ramp_end_time   / tot_time * exposures;
        Serial.print("# Exposure Ramp Start: ");
        Serial.println(exp_ramp_start);
        Serial.print("# Exposure Ramp End: ");
        Serial.println(exp_ramp_end);
      }
      break;
    case TOTAL_DIST:
      motor_speed = (tot_dist * 1.0 / PREVENT_DIV_ZERO(tot_time)) + 1.0;
      shutter_dist = (tot_dist * 1.0 / PREVENT_DIV_ZERO(exposures));
      if (shutter_dist < STEP_INTERVAL_LIMIT) {
        tot_dist = ceil(STEP_INTERVAL_LIMIT * exposures);
        shutter_dist = (tot_dist * 1.0 / PREVENT_DIV_ZERO(exposures));
      }
      break;
    case EXPOSURES:
      shutter_interval = (tot_time * 1.0 / PREVENT_DIV_ZERO(exposures));
      shutter_dist = (tot_dist * 1.0 / PREVENT_DIV_ZERO(exposures));
      if (shutter_dist < STEP_INTERVAL_LIMIT) {
        exposures = floor(tot_dist/STEP_INTERVAL_LIMIT);
        shutter_dist = (tot_dist * 1.0 / PREVENT_DIV_ZERO(exposures));
      }
      break;
    case SPEEDY:
      //    tot_time = tot_dist / motor_speed;
      //    exposures = tot_time / shutter_interval;
      break;
    case SHUTTER_DIST:
      //    exposures = tot_dist / shutter_dist;
      //    shutter_interval = shutter_dist / motor_speed;
      break;
    case SHUTTER_INTERVAL:
      //    exposures = tot_time / shutter_interval;
      //    shutter_dist = shutter_interval * motor_speed;
      break;
    case HOLD_MOTOR_ENABLE:
      if (hold_motor_enable > 1) {
        hold_motor_enable = 0;
      }
      break;
    case BULB_RAMP_ENABLE:
      if (bulb_ramp_enable > 1) {
        bulb_ramp_enable = 0;
      }
      break;
    case SHUTTER_SPD_INITIAL:
      if (shutter_spd_initial > 34) {
        shutter_spd_initial = 34;
      }
      break;
    case SHUTTER_SPD_FINAL:
      if (shutter_spd_final > 34) {
        shutter_spd_final = 34;
      }
      break;
    case RAMP_START_TIME:
      if (ramp_start_time >= ramp_end_time) {
        ramp_start_time = ramp_end_time - 1;
      }
      ramp_time_delta = ramp_end_time - ramp_start_time;
      Serial.print("Ramp Time Delta: ");
      Serial.println(ramp_time_delta);
      exp_ramp_start = 1.0 * ramp_start_time / tot_time * exposures;
      exp_ramp_end   = 1.0 * ramp_end_time   / tot_time * exposures;
      Serial.print("# Exposure Ramp Start: ");
      Serial.println(exp_ramp_start);
      Serial.print("# Exposure Ramp End: ");
      Serial.println(exp_ramp_end);
      break;
    case RAMP_END_TIME:
      if (ramp_end_time < 1) {
        ramp_end_time = 1;
      }
      if (ramp_end_time > tot_time) {
        ramp_end_time = tot_time;
      }
      if (ramp_start_time >= ramp_end_time) {
        ramp_start_time = ramp_end_time - 1;
      }
      ramp_time_delta = ramp_end_time - ramp_start_time;
      Serial.print("Ramp Time Delta: ");
      Serial.println(ramp_time_delta);
      exp_ramp_start = 1.0 * ramp_start_time / tot_time * exposures;
      exp_ramp_end   = 1.0 * ramp_end_time   / tot_time * exposures;
      Serial.print("# Exposure Ramp Start: ");
      Serial.println(exp_ramp_start);
      Serial.print("# Exposure Ramp End: ");
      Serial.println(exp_ramp_end);
      break;
    default:
      // nada
      break;
  }  
  // end of switch(menu)
}

void setup_run() {
  run_dist = tot_dist;
  int rpm = ceil(motor_speed * 60);
  //motor.setSpeed(rpm);  //  rpm
  //stepper.setSpeed(rpm);
  stepper.setSpeed(60);

  exposures_remaining = exposures;
}

int inchesToSteps(uint16_t inches) {
  // 76.62835249 steps per inch
  int steps = ceil(76.62835249 * inches);
  Serial.print("Took: ");
  Serial.println(steps);
  return steps;
}

int floatInchesToSteps(float inches) {
  // 76.62835249 steps per inch
  // 0.01305 inches per step
  int steps = ceil(76.62835249 * inches);
  Serial.print("Took: ");
  Serial.println(steps);
  return steps;
}

// redid for bulb ramping 20140930
void shutter() {
  if (bulb_ramp_enable) {
    if ( exposures_remaining > (exposures - exp_ramp_start) ) {
      shut_spd_current = getShutterSpd(shutter_spd_initial);
    } else if ( exposures_remaining < (exposures - exp_ramp_end) ) {
      shut_spd_current = getShutterSpd(shutter_spd_final);
    } else {
      ramp_pos = 1.0 * (exposures - exposures_remaining - exp_ramp_start) / (exp_ramp_end - exp_ramp_start);
      Serial.print("t: ");
      Serial.println(ramp_pos);
      shut_spd_current = getShutterSpd(shutter_spd_initial);
      shut_spd_current += (getShutterSpd(shutter_spd_final)-getShutterSpd(shutter_spd_initial))/(1.0+exp(-16.0*(ramp_pos-0.5)));
    }
    Serial.print("shutter spd: ");
    Serial.println(shut_spd_current);
  }
  digitalWrite(shutterPin, HIGH);
  delay(bulb_ramp_enable ? shut_spd_current : 50);
  digitalWrite(shutterPin, LOW);
}

void stop() {
  motor.release();
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  menu = TOTAL_DIST;
  recompute();
  run = false;

  lcd.setBacklight(LCD_ON);
}

void start() {
  setup_run();
  run = true;

  lcd.setBacklight(LCD_OFF);
}

void run_motor() {
  if (run_dist >= shutter_dist) {

    if ((millis() - last_exposure) >= ((unsigned long)shutter_interval) * 1000) {
      expose();
      last_exposure = millis();
      delay(exposure_delay);    // -PW 20131125
      digitalWrite(MOTOR_ENABLE_PIN, HIGH);
      //motor.step(floatInchesToSteps(shutter_dist), FORWARD, INTERLEAVE);
      stepper.step(floatInchesToSteps(shutter_dist));
      if (!hold_motor_enable) {
        digitalWrite(MOTOR_ENABLE_PIN, LOW);
      }
      run_dist -= shutter_dist;
      Serial.print("dist left:");
      Serial.println(run_dist);
      Serial.print("time left:");
      Serial.println(last_exposure - millis());
    } else {
      Serial.print("waiting");
      Serial.println(last_exposure - millis());
      readButtons();
    }

    if (run_dist <= 0) {
      stop();
    }

  } else {

    digitalWrite(MOTOR_ENABLE_PIN, HIGH);
    //motor.step(floatInchesToSteps(run_dist), FORWARD, INTERLEAVE);
    stepper.step(floatInchesToSteps(run_dist));
    if (!hold_motor_enable) {
      digitalWrite(MOTOR_ENABLE_PIN, LOW);
    }
    run_dist = 0;
    if (exposures_remaining > 0) {
      expose();
      delay(exposure_delay);    // -PW 20131125
    }
    stop();

  }

}

void expose() {
  shutter();
  exposures_remaining--;
  Serial.print("exposures left: ");
  Serial.println(exposures_remaining);
  Serial.print("exposures #: ");
  Serial.println(exposures-exposures_remaining);
}

void displayMenu() {
  lcd.setCursor(0, 0);
  lcd.print(Menu[menu]);
}

void clearValue() {
  lcd.clear();
  displayMenu();
  //lcd.setCursor(0, 1);
  //lcd.print('               ');
}

String displayTime(uint16_t seconds) {
  uint16_t hour = seconds / 3600;
  uint16_t minute = seconds % 3600 / 60;
  uint16_t second = seconds % 3600 % 60;
  String time_str = String("h:");
  if (hour < 10) { time_str += 0; }
  time_str += hour + String(" m:");
  if (minute < 10) { time_str += 0; }
  time_str += minute + String(" s:");
  if (second < 10) { time_str += 0; }
  time_str += second;
  return time_str;
}

String displayDist(uint16_t inches) {
  uint16_t feet = inches / 12;
  inches = inches % 12;
  String dist_str = "";
  if (feet < 10) { dist_str += 0; }
  dist_str += feet + String(" ft. ");
  if (inches < 10) { dist_str += 0; }
  dist_str += inches + String(" in.");
  return dist_str;
  //return inches + String(" inches");
}

String displayOnOff(uint16_t value) {
  //return (value % 2) ? String("On ") : String("Off ");
  return (value >= 1) ? String("On ") : String("Off ");
}

String displayShutterSpd(uint16_t value) {
  char* shutterSpdStrings[] = {
    "1/20", "1/15", "1/13", "1/10", "1/8", "1/6", "1/5", "1/4", 
    "0.3", "0.4", "0.5", "0.6", "0.8", 
    "1.0", "1.3", "1.6", "2.0", "2.5", "3.2", 
    "4", "5", "6", "8", "10", "13", 
    "15", "20", "25", "30", 
    "35", "40", "45", "50", "55", "60" 
  };
  return shutterSpdStrings[value];
}

uint16_t getShutterSpd(uint16_t value) {
  int shutterSpds[] = { 
    50, 67, 77, 100, 125, 167, 200, 250, 
    300, 400, 500, 600, 800, 
    1000, 1300, 1600, 2000, 2500, 3200, 
    4000, 5000, 6000, 8000, 10000, 13000, 
    15000, 20000, 25000, 30000, 
    35000, 40000, 45000, 50000, 55000, 60000 
  };
  return shutterSpds[value];
}



void displayValue() {
  lcd.setCursor(0, 1);
  //  made into switch statement for legibility    -PW 20131125
  switch (menu) {
    case TOTAL_TIME:
      lcd.print(displayTime(tot_time));
      break;
    case TOTAL_DIST:
      lcd.print(displayDist(tot_dist));
      break;
    case EXPOSURES:
      lcd.print(exposures);
      break;
    case EXPOSURE_DELAY:
      lcd.print(exposure_delay);
      break;
    case SPEEDY:
      lcd.print(motor_speed, 5);
      break;
    case SHUTTER_DIST:
      // lcd.print(displayDist(shutter_dist));
      lcd.print(shutter_dist,5);
      lcd.print(" inches");
      break;
    case SHUTTER_INTERVAL:
      lcd.print(displayTime(shutter_interval));
      break;
    case HOLD_MOTOR_ENABLE:
      lcd.print(displayOnOff(hold_motor_enable));
      break;
    case BULB_RAMP_ENABLE:
      lcd.print(displayOnOff(bulb_ramp_enable));
      break;
    case SHUTTER_SPD_INITIAL:
      lcd.print(displayShutterSpd(shutter_spd_initial)+" s "+getShutterSpd(shutter_spd_initial)+" ms");
      break;
    case SHUTTER_SPD_FINAL:
      lcd.print(displayShutterSpd(shutter_spd_final)+" s "+getShutterSpd(shutter_spd_final)+" ms");
      break;
    case RAMP_START_TIME:
      lcd.print(displayTime(ramp_start_time));
      break;
    case RAMP_END_TIME:
      lcd.print(displayTime(ramp_end_time));
      break;
    case START:
      lcd.print("Select To Start");
  }   
  // end of switch(menu)
}


void readRotary() {

  lcd.setCursor(0, 1);
  displayValue();

  uint16_t increment = menu_increments[menu][menu_increment_selection[menu]];

  #define READ_ROT_CASE(label,var)		\
  case label:						\
  encoder(var, increment);	\
  break;						\
  // end of define

  switch (menu) {
      READ_ROT_CASE(TOTAL_TIME,          tot_time)
      READ_ROT_CASE(TOTAL_DIST,          tot_dist)
      READ_ROT_CASE(EXPOSURES,           exposures)
      READ_ROT_CASE(EXPOSURE_DELAY,      exposure_delay)
      READ_ROT_CASE(HOLD_MOTOR_ENABLE,   hold_motor_enable)
      READ_ROT_CASE(BULB_RAMP_ENABLE,    bulb_ramp_enable)
      READ_ROT_CASE(SHUTTER_SPD_INITIAL, shutter_spd_initial)
      READ_ROT_CASE(SHUTTER_SPD_FINAL,   shutter_spd_final)
      READ_ROT_CASE(RAMP_START_TIME,     ramp_start_time)
      READ_ROT_CASE(RAMP_END_TIME,       ramp_end_time)
  }
  recompute();

}

void displayLCD() {
  menu = button_count % N_MENUS;      /// this belongs somewhere else, maybe?
  displayMenu();
  readRotary();
}

void loop() {
  if (!run) {         // LCD configure mode
    displayLCD();
  } else {            // Motor mode
    run_motor();
    // lcd.print("Running");
  }

  readButtons();      // always read buttons once per cycle
}

void readButtons() {
  uint8_t buttons = lcd.readButtons();

  if (!buttons) {
    return;
  }

  if (run) {
    if (buttons & BUTTON_SELECT) {
      stop();
      delay(1000);
    }
    return;
  }

  uint16_t increment = menu_increments[menu][menu_increment_selection[menu]];

  if (buttons & BUTTON_UP) {
    button_count--;
  }

  if (buttons & BUTTON_DOWN) {
    button_count++;
  }

#define READ_BTN_CASE(label,var,op) \
case label:							\
  var op increment; 				\
  break;							\
// end of define

#define READ_BTN_CASE_DEC(label,var) 	READ_BTN_CASE(label,var,-=)
#define READ_BTN_CASE_INC(label,var) 	READ_BTN_CASE(label,var,+=)

  if (buttons & BUTTON_LEFT) {
    switch (menu) {
        READ_BTN_CASE_DEC(TOTAL_TIME, tot_time)
        READ_BTN_CASE_DEC(TOTAL_DIST, tot_dist)
        READ_BTN_CASE_DEC(EXPOSURES, exposures)
        READ_BTN_CASE_DEC(EXPOSURE_DELAY, exposure_delay)
        READ_BTN_CASE_DEC(HOLD_MOTOR_ENABLE, hold_motor_enable)
        READ_BTN_CASE_DEC(BULB_RAMP_ENABLE, bulb_ramp_enable)
        READ_BTN_CASE_DEC(SHUTTER_SPD_INITIAL, shutter_spd_initial)
        READ_BTN_CASE_DEC(SHUTTER_SPD_FINAL, shutter_spd_final)
        READ_BTN_CASE_DEC(RAMP_START_TIME, ramp_start_time)
        READ_BTN_CASE_DEC(RAMP_END_TIME, ramp_end_time)
    }
    recompute();
  }

  if (buttons & BUTTON_RIGHT) {
    switch (menu) {
        READ_BTN_CASE_INC(TOTAL_TIME, tot_time)
        READ_BTN_CASE_INC(TOTAL_DIST, tot_dist)
        READ_BTN_CASE_INC(EXPOSURES, exposures)
        READ_BTN_CASE_INC(EXPOSURE_DELAY, exposure_delay)
        READ_BTN_CASE_INC(HOLD_MOTOR_ENABLE, hold_motor_enable)
        READ_BTN_CASE_INC(BULB_RAMP_ENABLE, bulb_ramp_enable)
        READ_BTN_CASE_INC(SHUTTER_SPD_INITIAL, shutter_spd_initial)
        READ_BTN_CASE_INC(SHUTTER_SPD_FINAL, shutter_spd_final)
        READ_BTN_CASE_INC(RAMP_START_TIME, ramp_start_time)
        READ_BTN_CASE_INC(RAMP_END_TIME, ramp_end_time)
    }
    recompute();
  }

  if (buttons & BUTTON_SELECT) {
    menu_increment_selection[menu] ++;
    menu_increment_selection[menu] %= n_menu_increments[menu];
    if (menu == START) {
      start();
    }
  }

  //mop up before relooping
  //delay(50);

  lcd.clear();

  delay(MENU_DELAY);
}


