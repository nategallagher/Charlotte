/*********************
 *
 *  Charlotte firmware, 2.0
 *   2016 January
 *  (c) David Koch  All rights reserved
 *    code by Nate Gallagher, Dounan Hu, Pete Weisz
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
 *    20140930    adding the following
 *            bulb ramping (on/off)
 *            shutter spd initial (ms)
 *            shutter spd final (ms)
 *            ramp start time (h/m/s/ms)
 *            ramp end time (h/m/s/ms)
 *
 *   2016 v2.0 updates by Nate Gallagher:
 *          using AccelStepper library with A4988 motor driver from Pololu.com
 *          bulb ramping with S-curve acceleration and realistic exposure time settings
 *          move ramping with linear acceleration for smooth transition and beginning and end of run
 *             -based on arbitrary move #, total distance, and ramp distance/%
 *          eeprom save settings whenever motor is run, loads last saved setting on reset
 *          reversing the motor
 *          adjustable speed/acceleration per move settings
 *          hold motor delay setting to save on power while maintaining stability
 *          between shot delay setting for use with cheaper cameras that dont fire shutter right away
 *          delay after start, before motor starts (just once)
 *          reworked menu to be more intuitive. rearranged code for better understandability
 *          removed Rotary encoder function and using simple buttons instead
 *          figured out consequences of adding the exposure delay, and fix them
 *          change n decimal places for distance in feet/inches
 *          add scale factor for real distance units (hard coded to wheel size)
 *          sanity limits to user-configurable parameters (since some combination
 *              of settings cannot be executed)
 *
 */

// some useful macros -PW
#define ARRAY_N_ELEMENTS(array)  ( sizeof(array) / sizeof(array[0]) )
#define PREVENT_DIV_ZERO(integer)  ( integer ? integer : 1 )

// include the library code:
#include <Wire.h>
#include <math.h>
#include <EEPROM.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <AccelStepper.h>


// The shield uses the I2C SCL and SDA pins.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// LCD Backlight
#define LCD_OFF 0x0
#define LCD_ON 0x1

#define SHUTTER_PIN 4

// default values for config parameters that start as non-zero
#define WHEEL_STEP_INTERVAL_LIMIT 0.01305 // inches moved in a single step
#define WIRE_STEP_INTERVAL_LIMIT 0.012962963
#define STEPPER_STEPS_PER_REVOLUTION 200      // 1.8 deg / step * 360 deg / revolution
#define MOTOR_MAX_SPEED_INITIAL 7
#define ACCELERATION_INITIAL 35
#define DEFAULT_HOLD_MOTOR_DELAY_MS 300

// define A4988 digital pins
#define MOT_DIRECTION 12
#define MOT_STEP 11
#define MOT_SLEEP 10
#define MOT_RESET 9
#define MOT_MS3 8
#define MOT_MS2 7
#define MOT_MS1 6
#define MOT_ENABLE_PIN 5

// initialize accelstepper for a two wire board, MOT_STEP = step, MOT_DIRECTION = dir
//AccelStepper stepper(1, 11, 12);  //eleven is step pin, twelve direction
//AccelStepper stepper(1,MOT_STEP,MOT_DIRECTION);
AccelStepper stepper(AccelStepper::DRIVER ,MOT_STEP,MOT_DIRECTION);

// need to keep track of position for acceleration to work
int32_t currentPos = 0; // can go clockwise positive or counter-CW negative
uint8_t stepMultiplier = 1;

boolean run = false;
int8_t button_count = 0;

enum MenuItems {
  SHUTTER_INTERVAL,     //  0
  EXPOSURES,            //  1
  TOTAL_DIST,           //  2
  SPEEDY,               //  3
  ACCEL_SETTER,         //  4
  HOLD_MOTOR_DELAY,     //  5
  BETWEEN_SHOTS_DELAY,  //  6
  BULB_RAMP_ENABLE,     //  7
  SHUTTER_SPD_INITIAL,  //  8
  SHUTTER_SPD_FINAL,    //  9
  RAMP_START_TIME,      // 10
  RAMP_END_TIME,        // 11
  MOVE_RAMP_DISTANCE,   // 12
  MOTOR_DIRECTION,      // 13
  SHUTTER_DIST,         // 14
  TOTAL_TIME,           // 15
  START                 // 16
};

char* Menu[] = {
  "Shot Interval",
  "Exposures",
  "Run Distance",
  "Speed revs/sec",
  "Accel revs/sec^2",
  "Hold Motor Delay",
  "Btwn Shots Delay",
  "Bulb Ramp?",
  "Shutter Initial",
  "Shutter Final",
  "Ramp Start Time",
  "Ramp End Time",
  "MoveRampDistance",
  "Motor Direction",
  "Shot Distance",
  "Run Time",
  "START MOTOR"
};

#define N_MENUS   ARRAY_N_ELEMENTS(Menu)

uint16_t  menu_increments[N_MENUS][6] = {
  {1, 60, 3600},              // shutter interval seconds
  {1, 10, 100},               // exposures
  {1, 12},                    // tot_dist    inches
  {1},                        // max_speed  revs/sec
  {1},                        // accel_set  revs/sec^2
  {250},                      // hold_motor_delay milliseconds
  {250},                      // btwn_shots_delay milliseconds
  {1},                        // bulb ramp enable off/on
  {1},                        // shutter spd initial array number
  {1},                        // shutter spd final array number
  {1, 60, 3600},              // ramp start time seconds
  {1, 60, 3600},              // ramp end time seconds
  {1, 12},                    // ramp_dist    inches
  {2},                        // motor direction clockwise/counter-cw
  {1},                        // shutter distance inches & steps
  {1},                        // tot_time hours:minutes:seconds computed
  {1}                         // start motor
};

uint8_t  n_menu_increments[N_MENUS] =   {
  3,                       // shutter interval
  3,                       // exposures
  2,                       // tot_dist
  1,                       // max_speed
  1,                       // accel_set
  1,                       // hold_motor_delay
  1,                       // btwn_shots_delay
  1,                       // bulb ramp enable
  1,                       // shutter spd initial
  1,                       // shutter spd final
  3,                       // ramp start time
  3,                       // ramp end time
  2,                       // ramp_dist
  1,                       // motor direction
  1,                       // shutter distance
  1,                       // tot_time
  1                        // start motor
};

int8_t menu = 0;
uint8_t menu_increment_selection[N_MENUS];

/***********   commented this out   -PW 20131125
* uint8_t calib_menu = 0;
* enum CalibMenuItems {WIRE, WHEEL, RAILS};
* CalibMenu[calib_menu_tot] = {"Wire", "Wheel", "Rails"};
***********************************/

// user configurable runtime parameters
uint16_t shutter_interval = 5;  // seconds
uint16_t exposures = 300;
uint16_t exposures_remaining = exposures;
uint16_t tot_time = shutter_interval * (exposures-1);        // seconds
uint16_t tot_dist = 30; // "inches"
uint16_t ramp_dist = 15; // "inches"
uint16_t max_speed = MOTOR_MAX_SPEED_INITIAL;
uint16_t accel_set = ACCELERATION_INITIAL;
int16_t hold_motor_delay = DEFAULT_HOLD_MOTOR_DELAY_MS;
int8_t hold_motor_enable = 0;
uint16_t btwn_shots_delay = 1000;
uint16_t after_shot_delay = btwn_shots_delay;
uint16_t camera_move_delay = btwn_shots_delay;
int8_t bulb_ramp_enable = 0;
uint8_t shutter_spd_initial = 0; // 1/20 sec default
uint8_t shutter_spd_final = 13;  // 1.0 seconds default
uint16_t ramp_start_time = floor( (float)tot_time/4 );    // seconds 1/4 of total time
uint16_t ramp_end_time = ceil( (float)tot_time*3/4 );     // seconds 3/4 of total time
int8_t move_ramp_enable = 0;
int8_t motor_direction = 1;

// minimum shutter_interval is 2 between shot delays + the hold motor delay, if any
uint16_t shutter_interval_min = ceil((float)((btwn_shots_delay*2) + hold_motor_delay)/1000.0);

// runtime vars
uint16_t shut_spd_current = 50;
unsigned long last_exposure = 0;
unsigned long first_exposure = 0;
uint16_t currentTimeSecs = 0;

// automatically calculated runtime parameters
uint32_t tot_dist_steps = ceil(tot_dist / WIRE_STEP_INTERVAL_LIMIT);
uint32_t dist_steps_remaining = tot_dist_steps;
uint32_t shutter_dist_steps = round((float)tot_dist_steps/(float)(exposures-1));
uint32_t move_dist_current = shutter_dist_steps;

// EEPROM Store value functions
uint8_t byte_one = 0;
uint8_t byte_two = 0;
uint8_t eeprom_address = 0;

void store_bytes (uint16_t value) {
  byte_two = value & 0xFF;
  byte_one = ((value >> 8) & 0xFF);
  Serial.print("value: ");
  Serial.println(value);
  Serial.print("byte_one: ");
  Serial.println(byte_one);
  Serial.print("byte_two: ");
  Serial.println(byte_two);
  EEPROM.update(eeprom_address, byte_one);
  eeprom_address++;
  EEPROM.update(eeprom_address, byte_two);
  eeprom_address++;
}

void store_EEPROM_values() {
  eeprom_address = 0;
  // uint16_t shutter_interval    // 2 bytes
  store_bytes(shutter_interval);
  // uint16_t exposures           // 2 bytes
  store_bytes(exposures);
  // uint16_t tot_dist            // 2 bytes
  store_bytes(tot_dist);
  // uint16_t max_speed           // 2 bytes
  store_bytes(max_speed);
  // uint16_t accel_set           // 2 bytes
  store_bytes(accel_set);
  // int16_t  hold_motor_delay    // 2 bytes
  store_bytes(hold_motor_delay);
  // uint16_t btwn_shots_delay    // 2 bytes
  store_bytes(btwn_shots_delay);
  // int8_t   bulb_ramp_enable    // 1 byte
  store_bytes(bulb_ramp_enable);
  // uint8_t  shutter_spd_initial // 1 byte
  store_bytes(shutter_spd_initial);
  // uint8_t  shutter_spd_final   // 1 byte
  store_bytes(shutter_spd_final);
  // uint16_t ramp_start_time     // 2 bytes
  store_bytes(ramp_start_time);
  // uint16_t ramp_end_time       // 2 bytes
  store_bytes(ramp_end_time);
  // uint16_t ramp_dist            // 2 bytes
  store_bytes(ramp_dist);
  // int8_t   motor_direction     // 1 byte
  store_bytes(motor_direction);
}

uint16_t restore_bytes() {
  byte_one = EEPROM.read(eeprom_address);
  eeprom_address++;
  byte_two = EEPROM.read(eeprom_address);
  eeprom_address++;
  return ( (byte_two & 0xFFFFFF) + ((byte_one << 8) & 0xFFFFFFFF) );
}

void restore_EEPROM_values() {
  eeprom_address = 0;
  uint8_t  temp_uint8_t  = 0;
  uint16_t temp_uint16_t = 0;
  int8_t   temp_int8_t   = 0;
  int16_t  temp_int16_t  = 0;

  // uint16_t shutter_interval    // 2 bytes
  temp_uint16_t = restore_bytes();
  if (temp_uint16_t > 0) {
    shutter_interval = temp_uint16_t;
  }
  // uint16_t exposures           // 2 bytes
  temp_uint16_t = restore_bytes();
  if (temp_uint16_t > 1) {
    exposures = temp_uint16_t;
    exposures_remaining = exposures;
  }
  // recalculate total time
  tot_time = shutter_interval * (exposures-1);        // seconds
  // uint16_t tot_dist            // 2 bytes
  temp_uint16_t = restore_bytes();
  if (temp_uint16_t > 0) {
    tot_dist = temp_uint16_t;
  }
  // uint16_t max_speed           // 2 bytes
  temp_uint16_t = restore_bytes();
  if ((temp_uint16_t >= 1) && (temp_uint16_t <= 7)) {
    max_speed = temp_uint16_t;
  }
  // uint16_t accel_set           // 2 bytes
  temp_uint16_t = restore_bytes();
  if ((temp_uint16_t > 0) && (temp_uint16_t <= 50)) {
    accel_set = temp_uint16_t;
  }
  // int16_t  hold_motor_delay    // 2 bytes
  temp_int16_t = restore_bytes();
  if ((temp_int16_t >= 0) && (temp_int16_t <= 5050)) {
    hold_motor_delay = temp_int16_t;
    // set hold_motor_enable ON if maxed out on delay
    if ( hold_motor_delay >= ((50*floor(shutter_interval*1000/2/50)) + 50) ) {
      hold_motor_enable = 1;
    } else if ( hold_motor_delay >= 5050 ) {
      hold_motor_enable = 1;
    } else {
      hold_motor_enable = 0;
    }
  }
  // uint16_t btwn_shots_delay    // 2 bytes
  temp_uint16_t = restore_bytes();
  if ((temp_uint16_t >= 500) && (temp_uint16_t <= 5000)) {
    btwn_shots_delay = temp_uint16_t;
    after_shot_delay = btwn_shots_delay;
    camera_move_delay = btwn_shots_delay;
  }
  // int8_t   bulb_ramp_enable    // 1 byte
  temp_int8_t = restore_bytes();
  if ((temp_int8_t >= 0) && (temp_int8_t <= 1)) {
    bulb_ramp_enable = temp_int8_t;
  }
  // uint8_t  shutter_spd_initial // 1 byte
  temp_uint8_t = restore_bytes();
  if ((temp_uint8_t >= 0) && (temp_uint8_t <= 34)) {
    shutter_spd_initial = temp_uint8_t;
  }
  // uint8_t  shutter_spd_final   // 1 byte
  temp_uint8_t = restore_bytes();
  if ((temp_uint8_t >= 0) && (temp_uint8_t <= 34)) {
    shutter_spd_final = temp_uint8_t;
  }
  // uint16_t ramp_start_time     // 2 bytes
  temp_uint16_t = restore_bytes();
  if ((temp_uint16_t >= 0) && (temp_uint16_t <= tot_time)) {
    ramp_start_time = temp_uint16_t;
  }
  // uint16_t ramp_end_time       // 2 bytes
  temp_uint16_t = restore_bytes();
  if ((temp_uint16_t >= ramp_start_time) && (temp_uint16_t <= tot_time)) {
    ramp_end_time = temp_uint16_t;
  }
  // uint16_t ramp_dist            // 2 bytes
  temp_uint16_t = restore_bytes();
  // check that ramp distance is not more than half total distance
  if (temp_uint16_t <= floor(tot_dist/2.0)) {
    ramp_dist = temp_uint16_t;
    // set move_ramp_enable ON if ramp_dist > 0
    if ( ramp_dist > 0 ) {
      move_ramp_enable = 1;
    } else {
      move_ramp_enable = 0;
    }
  } else {  // disable move ramp
    ramp_dist = 0;
    move_ramp_enable = 0;
  }
  // int8_t   motor_direction     // 1 byte
  temp_int8_t = restore_bytes();
  if ((temp_int8_t == -1) || (temp_int8_t == 1)) {
    motor_direction = temp_int8_t;
  }
  // recalculate
  shutter_interval_min = ceil((float)((btwn_shots_delay*2) + hold_motor_delay)/1000.0);
  tot_dist_steps = ceil(tot_dist / WIRE_STEP_INTERVAL_LIMIT);
  dist_steps_remaining = tot_dist_steps;
  shutter_dist_steps = round((float)tot_dist_steps/(float)(exposures-1));
  move_dist_current = shutter_dist_steps;
}

void recompute() { 
  //recalculate configuration values
  switch (menu) {
    case SHUTTER_INTERVAL:
      if (shutter_interval < 1) {
        shutter_interval = 1;
      } else if ( ( (long)shutter_interval * (long)(exposures-1) ) > 65535 ) {
        shutter_interval = floor(65535.0/(float)(exposures-1));
      }
      tot_time = shutter_interval * (exposures-1);
      if (bulb_ramp_enable >= 1) {
        if (ramp_end_time > tot_time) {
          ramp_end_time = tot_time;
        }
        if (ramp_start_time >= ramp_end_time) {
          ramp_start_time = ceil((float)ramp_end_time/2.0);
        }
      }
      break;
    case EXPOSURES:
      if ( exposures > (tot_dist_steps + 1) ) {
        exposures = 2;
      } else if (exposures < 2) {
        exposures = tot_dist_steps + 1;
      }
      if ( ( (long)shutter_interval * (long)(exposures-1) ) > 65535 ) {
        exposures = floor(65535.0/shutter_interval) + 1;
      }
      tot_time = shutter_interval * (exposures-1);
      shutter_dist_steps = round((float)tot_dist_steps/(float)(exposures-1));
      if (bulb_ramp_enable >= 1) {
        if (ramp_end_time > tot_time) {
          ramp_end_time = tot_time;
        }
        if (ramp_start_time >= ramp_end_time) {
          ramp_start_time = ramp_end_time - 1;
        }
      }
      break;
    case TOTAL_DIST:
      tot_dist_steps = ceil(tot_dist / WIRE_STEP_INTERVAL_LIMIT);
      shutter_dist_steps = round((float)tot_dist_steps/(float)(exposures-1));
      if (shutter_dist_steps < 1) {
        tot_dist = ceil(WIRE_STEP_INTERVAL_LIMIT * (exposures-1));
        tot_dist_steps = ceil(tot_dist / WIRE_STEP_INTERVAL_LIMIT);
        shutter_dist_steps = round((float)tot_dist_steps/(float)(exposures-1));
      }
      if (ramp_dist > floor(tot_dist/2.0)) {
        ramp_dist = floor(tot_dist/2.0);
      }
      break;
    case SPEEDY:
      if (max_speed > 7) {
        max_speed = 1;
      } else if (max_speed < 1) {
        max_speed = 7;    // 7*200 = 1400steps/sec, faster max_speed causes slippage
      }
      break;
    case ACCEL_SETTER:
      if (accel_set > 50) {
        accel_set = 1;
      } else if (accel_set < 1) {
        accel_set = 50;    // 50*200 = 10,000steps/sec^2
      }
      break;
    case BETWEEN_SHOTS_DELAY:
      // milliseconds
      if ( btwn_shots_delay > 5000 ) {
        btwn_shots_delay = 250;
      } else if ( btwn_shots_delay <= 0 ) {
        btwn_shots_delay = 5000;
      }

      // minimum shutter_interval is 2 between shot delays + the hold motor delay, if any
      shutter_interval_min = btwn_shots_delay*2;  // milliseconds
      if (!hold_motor_enable) {
        shutter_interval_min += hold_motor_delay;
      }
      // convert milliseconds to seconds
      shutter_interval_min = ceil((float)(shutter_interval_min/1000.0));
      if (shutter_interval < shutter_interval_min) {
        shutter_interval = shutter_interval_min;
        if (shutter_interval < 1) {
          shutter_interval = 1;
        } else if ( ( (long)shutter_interval * (long)(exposures-1) ) > 65535 ) {
          shutter_interval = floor(65535.0/(float)(exposures-1));
        }
        tot_time = shutter_interval * (exposures-1);
        if (bulb_ramp_enable >= 1) {
          if (ramp_end_time > tot_time) {
            ramp_end_time = tot_time;
          }
          if (ramp_start_time >= ramp_end_time) {
            ramp_start_time = ceil((float)ramp_end_time/2.0);
          }
        }
      }
      break;
    case HOLD_MOTOR_DELAY:
      // max hold_motor_delay is half shutter_interval in milliseconds
      // ALSO can't be over 5000 milliseconds
      // 5050 == always on
      // 0 == always off
      if ( hold_motor_delay > 5050 ) {
        hold_motor_delay = 0;
      } else if ( hold_motor_delay > ((50*floor(shutter_interval*1000/2/50)) + 50) ) {
        hold_motor_delay = 0;
      } else if ( hold_motor_delay < 0 ) {
        if ( (floor(shutter_interval*1000/2)) > 5000 ) {
          hold_motor_delay = 5050;
        } else {
          hold_motor_delay = ((50*floor(shutter_interval*1000/2/50)) + 50);    //
        }
      }
      // set hold_motor_enable ON if maxed out on delay
      if ( hold_motor_delay >= ((50*floor(shutter_interval*1000/2/50)) + 50) ) {
        hold_motor_enable = 1;
      } else if ( hold_motor_delay >= 5050 ) {
        hold_motor_enable = 1;
      } else {
        hold_motor_enable = 0;
      }

      // minimum shutter_interval is 2 between shot delays + the hold motor delay, if any
      if (!hold_motor_enable) {
        shutter_interval_min = btwn_shots_delay*2;  // milliseconds
        shutter_interval_min += hold_motor_delay;
        // convert milliseconds to seconds
        shutter_interval_min = ceil((float)(shutter_interval_min/1000.0));
        if (shutter_interval < shutter_interval_min) {
          shutter_interval = shutter_interval_min;
          if (shutter_interval < 1) {
            shutter_interval = 1;
          } else if ( ( (long)shutter_interval * (long)(exposures-1) ) > 65535 ) {
            shutter_interval = floor(65535.0/(float)(exposures-1));
          }
          tot_time = shutter_interval * (exposures-1);
          if (bulb_ramp_enable >= 1) {
            if (ramp_end_time > tot_time) {
              ramp_end_time = tot_time;
            }
            if (ramp_start_time >= ramp_end_time) {
              ramp_start_time = ceil((float)ramp_end_time/2.0);
            }
          }
        }
      }
      break;
    case BULB_RAMP_ENABLE:
      if (bulb_ramp_enable > 1) {
        bulb_ramp_enable = 0;
      } else if (bulb_ramp_enable < 0) {
        bulb_ramp_enable = 1;
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
      break;
    case MOVE_RAMP_DISTANCE:
      if (ramp_dist == 0) {
        move_ramp_enable = 0;
      } else if (ramp_dist == 65535) {
        // overlap uint16_t
        ramp_dist = floor(tot_dist/2.0);
        move_ramp_enable = 1;
      } else if (ramp_dist > floor(tot_dist/2.0)) {
        ramp_dist = 0;
        move_ramp_enable = 0;
      }
      break;
    case MOTOR_DIRECTION:
      if (motor_direction > 1) {
        motor_direction = -1;
      } else if (motor_direction < -1) {
        motor_direction = 1;
      } else if (motor_direction == 0) {
        motor_direction = 1;
      }
      break;
    case SHUTTER_DIST:
      // calculated not setable
      break;
    case TOTAL_TIME:
      // calculated not setable
      break;
    default:
      // nada
      break;
  }  
  // end of switch(menu)
}

void setup_run() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("RUNNING");

  dist_steps_remaining = tot_dist_steps;
  exposures_remaining = exposures;
  move_dist_current = shutter_dist_steps;
  after_shot_delay = btwn_shots_delay;
  camera_move_delay = btwn_shots_delay;
  stepper.setMaxSpeed(max_speed*STEPPER_STEPS_PER_REVOLUTION*stepMultiplier);
  stepper.setAcceleration(accel_set*STEPPER_STEPS_PER_REVOLUTION*stepMultiplier);
  store_EEPROM_values();

  delay(2000);
  first_exposure = millis();
  last_exposure = first_exposure;
}

void shutter() {
  currentTimeSecs =  round( (float)(millis() - first_exposure ) / 1000.0 );
  if (bulb_ramp_enable) {

    // beginning of set
    if ( currentTimeSecs <= ramp_start_time ) {

      shut_spd_current = getShutterSpd(shutter_spd_initial);

    // end of set
    } else if ( currentTimeSecs >= ramp_end_time ) {

      shut_spd_current = getShutterSpd(shutter_spd_final);

    // bulb ramp "middle" portion of set
    } else {

      float ramp_pos = (float)(currentTimeSecs - ramp_start_time) / (float)(ramp_end_time - ramp_start_time);

      Serial.print("t: ");
      Serial.println(ramp_pos);

      shut_spd_current = getShutterSpd(shutter_spd_initial);
      int32_t shutter_spd_delta = ( getShutterSpd(shutter_spd_final) - getShutterSpd(shutter_spd_initial) );
      shut_spd_current += round( (float)shutter_spd_delta / ( 1.0+exp(-16.0*(ramp_pos-0.5)) ) );
      // f(x) = 1 / ( 1 + e^(-16*(x-0.5)) )

    }
    Serial.print("shutter spd: ");
    Serial.println(shut_spd_current);
  }
  if (exposures_remaining > 0) {
    digitalWrite(SHUTTER_PIN, HIGH);
    delay(bulb_ramp_enable ? shut_spd_current : 50);
    digitalWrite(SHUTTER_PIN, LOW);
  }
}

void stop() {
  digitalWrite(MOT_ENABLE_PIN, HIGH); // HIGH = Disable Motor
  menu = TOTAL_DIST;
  recompute();
  run = false;

  currentPos = 0;
  stepper.setCurrentPosition(currentPos); // reset home position
  lcd.clear();
  lcd.setBacklight(LCD_ON);
}

void start() {
  setup_run();
  run = true;
  //lcd.setBacklight(LCD_OFF);
}

void returnToStartPos() {
  delay(500);
  digitalWrite(MOT_ENABLE_PIN, LOW); // LOW = enable motor
  delayMicroseconds(50);
  currentPos = 0;
  stepper.runToNewPosition(currentPos); // return to home position 0
  digitalWrite(MOT_ENABLE_PIN, HIGH); // HIGH = disable motor
}

void expose() {
  shutter();
  exposures_remaining--;
  Serial.print("exposures left: ");
  Serial.println(exposures_remaining);
  Serial.print("exposures #: ");
  delay(after_shot_delay);
}

void move_camera() {
  uint16_t currentMove = exposures - exposures_remaining;
  uint32_t expected_position = 0;
  uint32_t last_position = 0;
  uint16_t moves_total = exposures - 1;

  if (move_ramp_enable) {

    float percent_ramp = (float)ramp_dist/(float)tot_dist;
    float V_max = (float)tot_dist_steps / (float)(moves_total-floor(percent_ramp*(float)moves_total));
    uint16_t moves_ramp = floor(percent_ramp*(float)moves_total);
    uint16_t moves_middle = moves_total - (2*moves_ramp);
    uint32_t dist_ramp_steps = round(V_max*(float)moves_ramp/2.0);
    uint32_t dist_middle_steps = tot_dist_steps - (2*dist_ramp_steps);
    uint16_t portionMove = currentMove;

    // beginning of ramp acceleration
    if ( currentMove <= moves_ramp ) {
      portionMove = currentMove;
      // p(t) = 1/2 a*t^2 = V_max * t^2 / 2 * moves_ramp
      expected_position = round(V_max*portionMove*portionMove/(2.0*moves_ramp));
    // middle steady speed section
    } else if ( currentMove <= (moves_ramp + moves_middle) ) {
      portionMove = currentMove - moves_ramp;
      // p(t) = V_max * t + offset from ramping
      expected_position = round(V_max*portionMove);
      expected_position += dist_ramp_steps;
    // deceleration finishing ramp
    } else {  // currentMove > (moves_ramp + moves_middle)
      portionMove = currentMove - (moves_ramp + moves_middle);
      // p(t) = V_max * t + 1/2 a*t^2 = V_max * t + V_max * t^2 / 2 * moves_ramp
      // p(t) = V_max * t * ( 1 - 0.5 * t / moves_ramp )
      expected_position = round( V_max*portionMove * ( 1.0 - (0.5*portionMove/moves_ramp) ) );
      // p(t) += offset from accel ramp and middle sections
      expected_position += dist_ramp_steps + dist_middle_steps;
    }

  } else { // move_ramp is NOT enabled
    float V_max = (float)tot_dist_steps / (float)moves_total;
    // p(t) = V_max * t
    expected_position = round(V_max*currentMove);
  }

  currentPos = expected_position * motor_direction * stepMultiplier;
  move_dist_current = expected_position - (stepper.currentPosition() * motor_direction / stepMultiplier);

  Serial.print("move step: ");
  Serial.println(move_dist_current);

  // calculate distance left
  dist_steps_remaining -= move_dist_current;

  digitalWrite(MOT_ENABLE_PIN, LOW); // LOW = enable motor
  delayMicroseconds(50);
  stepper.runToNewPosition(currentPos);
  
  if (!hold_motor_enable) {
    delay(hold_motor_delay); // hold motor temporarily to preserve position and avoid missed steps
    digitalWrite(MOT_ENABLE_PIN, HIGH); // HIGH = disable motor
  }
  delay(camera_move_delay);

  // display debug values
  lcd.clear();
  lcd.setCursor(0, 0);
  if (currentTimeSecs <= tot_time) { // display time remaining
    lcd.print(displayTimeBrief(tot_time-currentTimeSecs));
  } else {  // display time over calculated time
    lcd.print("-");
    lcd.print(displayTimeBrief(currentTimeSecs-tot_time));
  }
  lcd.print(" ");
  lcd.print(currentMove);
  lcd.print("/");
  lcd.print(exposures-1);

  lcd.setCursor(0, 1);
  lcd.print(displayDistBrief(round(dist_steps_remaining*WIRE_STEP_INTERVAL_LIMIT)));
  lcd.print(" ");
  lcd.print(move_dist_current*WIRE_STEP_INTERVAL_LIMIT,3);
  lcd.print("\"");

  //lcd.print(" ");
  //lcd.print(shut_spd_current);
  //lcd.print("ms");
}

void run_motor() {

  if ( ( millis() - last_exposure ) >= ( (unsigned long)(shutter_interval*1000) ) ) {
    if (exposures_remaining)  {

      // take shot
      last_exposure = millis();
      expose();

      if (exposures_remaining)  { // don't move camera if that was last shot
        // move camera position
        move_camera();
      }

    } else {  // ( exposures_remaining <= 0 )

      dist_steps_remaining = 0;
      returnToStartPos();
      lcd.clear();
      stop();    

    }

  } else {  // shutter interval hasn't passed yet
    readButtons();  // allows shot interrupt during interval
  }
} // end of function run_motor()

void displayMenu() {
  lcd.setCursor(0, 0);
  lcd.print(Menu[menu]);
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

String displayTimeBrief(uint16_t seconds) {
  uint16_t hour = seconds / 3600;
  uint16_t minute = seconds % 3600 / 60;
  uint16_t second = seconds % 3600 % 60;
  String time_str = "";
  if (hour > 0) { 
    time_str += hour + String("h");
    time_str += minute + String("m");
    time_str += second + String("s");
  } else if (minute > 0) {
    time_str += minute + String("m");
    time_str += second + String("s");
  } else {
    time_str += second + String("s");
  } 
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
}

String displayDistBrief(uint16_t inches) {
  uint16_t feet = inches / 12;
  inches = inches % 12;
  String dist_str = "";
  dist_str += feet + String("\'");
  dist_str += inches + String("\"");
  return dist_str;
}

String displayOnOff(int8_t value) {
  return (value >= 1) ? String("On ") : String("Off ");
}

String displayMotorDirection(int8_t value) {
  if (value == 1) {
    return String("Clockwise");
  } else if (value == -1) {
    return String("Counter-CW");
  } else {
    return String(value);
  }
}

String displayShutterSpd(uint8_t value) {
  char* shutterSpdStrings[] = {
    "1/20", "1/15", "1/13", "1/10", "1/8", "1/6", "1/5", "1/4",   // 0,1,2,3,4,5,6,7,
    "0.3", "0.4", "0.5", "0.6", "0.8",                            // 8,9,10,11,12,
    "1.0", "1.3", "1.6", "2.0", "2.5", "3.2",                     // 13,14,15,16,17,18,
    "4", "5", "6", "8", "10", "13",                               // 19,20,21,22,23,24,
    "15", "20", "25", "30",                                       // 25,26,27,28,
    "35", "40", "45", "50", "55", "60"                            // 29,30,31,32,33,34
  };
  return shutterSpdStrings[value];
}

uint16_t getShutterSpd(uint8_t value) {
  uint16_t shutterSpds[] = { 
    50, 67, 77, 100, 125, 167, 200, 250,      // 0,1,2,3,4,5,6,7,
    300, 400, 500, 600, 800,                  // 8,9,10,11,12,
    1000, 1300, 1600, 2000, 2500, 3200,       // 13,14,15,16,17,18,
    4000, 5000, 6000, 8000, 10000, 13000,     // 19,20,21,22,23,24,
    15000, 20000, 25000, 30000,               // 25,26,27,28,
    35000, 40000, 45000, 50000, 55000, 60000  // 29,30,31,32,33,34
  };
  return shutterSpds[value];
}

void displayValue() {
  lcd.setCursor(0, 1);
  //  made into switch statement for legibility    -PW 20131125
  switch (menu) {
    case SHUTTER_INTERVAL:
      lcd.print(displayTime(shutter_interval));
      break;
    case EXPOSURES:
      lcd.print(exposures);
      break;
    case TOTAL_DIST:
      lcd.print(displayDist(tot_dist));
      break;
    case SPEEDY:
      lcd.print(max_speed);
      break;
    case ACCEL_SETTER:
      lcd.print(accel_set);
      break;
    case HOLD_MOTOR_DELAY:
      if ( hold_motor_delay >= ((50*floor(shutter_interval*1000/2/50)) + 50) ) {
        lcd.print("ALWAYS ON");
      } else if ( hold_motor_delay >= 5050 ) {
        lcd.print("ALWAYS ON");
      } else if ( hold_motor_delay <= 0 ) {
        lcd.print("OFF");
      } else {
        lcd.print(hold_motor_delay);
        lcd.print(" ms");
      }
      break;
    case BETWEEN_SHOTS_DELAY:
      lcd.print(btwn_shots_delay);
      lcd.print(" ms");
      break;
    case BULB_RAMP_ENABLE:
      lcd.print(displayOnOff(bulb_ramp_enable));
      break;
    case SHUTTER_SPD_INITIAL:
      lcd.print(displayShutterSpd(shutter_spd_initial)+"s = "+getShutterSpd(shutter_spd_initial)+"ms");
      break;
    case SHUTTER_SPD_FINAL:
      lcd.print(displayShutterSpd(shutter_spd_final)+"s = "+getShutterSpd(shutter_spd_final)+"ms");
      break;
    case RAMP_START_TIME:
      lcd.print(displayTime(ramp_start_time));
      break;
    case RAMP_END_TIME:
      lcd.print(displayTime(ramp_end_time));
      break;
    case MOVE_RAMP_DISTANCE:
      if (ramp_dist == 0) {
        lcd.print("Disabled");
      } else {
        lcd.print(displayDist(ramp_dist));
      }
      break;
    case MOTOR_DIRECTION:
      lcd.print(displayMotorDirection(motor_direction));
      break;
    case SHUTTER_DIST:
      lcd.print(shutter_dist_steps*WIRE_STEP_INTERVAL_LIMIT,5);
      lcd.print("in ");
      lcd.print(shutter_dist_steps);
      break;
    case TOTAL_TIME:
      lcd.print(displayTime(tot_time));
      break;
    case START:
      lcd.print("Select To Start");
  }   
  // end of switch(menu)
}

void displayLCD() {
  //menu = button_count % N_MENUS;      /// this belongs somewhere else, maybe?
  menu = button_count;
  displayMenu();
  lcd.setCursor(0, 1);
  displayValue();
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
    if (bulb_ramp_enable == 0) {
      // skip over bulb ramp options if not enabled
      if (button_count == 12) {
        button_count = 7;
      } else {
        button_count--;
      }
    } else {
      button_count--;
    }
    // loop forward to end
    if (button_count < 0) {
      button_count = 16;
    }
  }

  if (buttons & BUTTON_DOWN) {
    if (bulb_ramp_enable == 0) {
      // skip over bulb ramp options if not enabled
      if (button_count == 7) {
        button_count = 12;
      } else {
        button_count++;
      }
    } else {
      button_count++;
    }
    // loop back to start
    if (button_count > 16) {
      button_count = 0;
    }
  }

  #define READ_BTN_CASE(label,var,op) \
  case label:             \
    var op increment;         \
    break;              \
  // end of define

  #define READ_BTN_CASE_DEC(label,var)  READ_BTN_CASE(label,var,-=)
  #define READ_BTN_CASE_INC(label,var)  READ_BTN_CASE(label,var,+=)

  if (buttons & BUTTON_LEFT) {
    switch (menu) {
        READ_BTN_CASE_DEC(SHUTTER_INTERVAL, shutter_interval)
        READ_BTN_CASE_DEC(EXPOSURES, exposures)
        READ_BTN_CASE_DEC(TOTAL_DIST, tot_dist)
        READ_BTN_CASE_DEC(SPEEDY, max_speed)
        READ_BTN_CASE_DEC(ACCEL_SETTER, accel_set)
        READ_BTN_CASE_DEC(HOLD_MOTOR_DELAY, hold_motor_delay)
        READ_BTN_CASE_DEC(BETWEEN_SHOTS_DELAY, btwn_shots_delay)
        READ_BTN_CASE_DEC(BULB_RAMP_ENABLE, bulb_ramp_enable)
        READ_BTN_CASE_DEC(SHUTTER_SPD_INITIAL, shutter_spd_initial)
        READ_BTN_CASE_DEC(SHUTTER_SPD_FINAL, shutter_spd_final)
        READ_BTN_CASE_DEC(RAMP_START_TIME, ramp_start_time)
        READ_BTN_CASE_DEC(RAMP_END_TIME, ramp_end_time)
        READ_BTN_CASE_DEC(MOVE_RAMP_DISTANCE, ramp_dist)
        READ_BTN_CASE_DEC(MOTOR_DIRECTION, motor_direction)
    }
    recompute();
  }

  if (buttons & BUTTON_RIGHT) {
    switch (menu) {
        READ_BTN_CASE_INC(SHUTTER_INTERVAL, shutter_interval)
        READ_BTN_CASE_INC(EXPOSURES, exposures)
        READ_BTN_CASE_INC(TOTAL_DIST, tot_dist)
        READ_BTN_CASE_INC(SPEEDY, max_speed)
        READ_BTN_CASE_INC(ACCEL_SETTER, accel_set)
        READ_BTN_CASE_INC(HOLD_MOTOR_DELAY, hold_motor_delay)
        READ_BTN_CASE_INC(BETWEEN_SHOTS_DELAY, btwn_shots_delay)
        READ_BTN_CASE_INC(BULB_RAMP_ENABLE, bulb_ramp_enable)
        READ_BTN_CASE_INC(SHUTTER_SPD_INITIAL, shutter_spd_initial)
        READ_BTN_CASE_INC(SHUTTER_SPD_FINAL, shutter_spd_final)
        READ_BTN_CASE_INC(RAMP_START_TIME, ramp_start_time)
        READ_BTN_CASE_INC(RAMP_END_TIME, ramp_end_time)
        READ_BTN_CASE_INC(MOVE_RAMP_DISTANCE, ramp_dist)
        READ_BTN_CASE_INC(MOTOR_DIRECTION, motor_direction)
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

  if (!run) {
    lcd.clear();
    delay(100); // Menu Delay whenever selection changes
  }
}

void setup() {
  // Debugging output
  Serial.begin(9600);
  Serial.println("Charlotte Dolly");
  Serial.println("Controller");


  // define control pins as output
  pinMode(MOT_DIRECTION, OUTPUT);
  pinMode(MOT_STEP, OUTPUT);
  pinMode(MOT_SLEEP, OUTPUT);
  pinMode(MOT_RESET, OUTPUT);
  pinMode(MOT_MS3, OUTPUT);
  pinMode(MOT_MS2, OUTPUT);
  pinMode(MOT_MS1, OUTPUT);
  pinMode(MOT_ENABLE_PIN, OUTPUT);
  
  // supply VCC 5 Volts to Motor control logic board through A1 pin
  pinMode(A1, OUTPUT);
  digitalWrite(A1, HIGH);

  //disable motor control at start, LOW = Enabled, HIGH = NOT Enabled
  digitalWrite(MOT_ENABLE_PIN, HIGH);

  // full step mode
  stepMultiplier = 1;
  digitalWrite(MOT_MS1, LOW);
  digitalWrite(MOT_MS2, LOW);
  digitalWrite(MOT_MS3, LOW);
  // half step mode
  stepMultiplier = 2;
  digitalWrite(MOT_MS1, HIGH);
  digitalWrite(MOT_MS2, LOW);
  digitalWrite(MOT_MS3, LOW);
  // quarter step mode
  stepMultiplier = 4;
  digitalWrite(MOT_MS1, LOW);
  digitalWrite(MOT_MS2, HIGH);
  digitalWrite(MOT_MS3, LOW);
  // eigth step mode
  //stepMultiplier = 8;
  //digitalWrite(MOT_MS1, HIGH);
  //digitalWrite(MOT_MS2, HIGH);
  //digitalWrite(MOT_MS3, LOW);
  // sixteenth step mode
  //stepMultiplier = 16;
  //digitalWrite(MOT_MS1, HIGH);
  //digitalWrite(MOT_MS2, HIGH);
  //digitalWrite(MOT_MS3, HIGH);

  // reset pin must be HIGH to enable control functions
  digitalWrite(MOT_RESET, HIGH);

  // SLEEP Pin: HIGH = Active Mode, LOW = Sleep Mode, 1us delay after coming out of sleep mode
  digitalWrite(MOT_SLEEP, HIGH);
  delayMicroseconds(50);

  // set other AccelStepper variables
  //stepper.setMinPulseWidth(20); // 20 microseconds
  stepper.setMaxSpeed(MOTOR_MAX_SPEED_INITIAL*STEPPER_STEPS_PER_REVOLUTION*stepMultiplier);
  stepper.setAcceleration(ACCELERATION_INITIAL*STEPPER_STEPS_PER_REVOLUTION*stepMultiplier);
  currentPos = 0;
  stepper.setCurrentPosition(currentPos);

  /***********   clear out EEPROM for first program upload
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }

  ***********************************/
  // read and restore previous set values from EEPROM
  restore_EEPROM_values();
  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Charlotte Dolly");
  lcd.setCursor(0, 1);
  lcd.print("Controller");
  lcd.setBacklight(LCD_ON);
  lcd.cursor();
  pinMode(SHUTTER_PIN, OUTPUT);

  delay(2000);
  lcd.clear();

}

void loop() {
  if (!run) {         // LCD configure mode
    displayLCD();
  } else {            // Motor mode
    run_motor();
  }
  readButtons();      // always read buttons once per cycle
}

