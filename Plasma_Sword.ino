
/*
   ========================
   | Arduino Plasma Sword |
   ========================

   Copyright ©2015 Dustin Westaby. All rights reserved.
   Version 1.0.0

   Description:
   This code is used to animate a single or double bladed light
   sword.  Libraries and modules were used from Adafruit, read source
   comments for specific usage.

   Special Thanks: Sean Bradley, Joshua Kane

   ---------------------
   |       USAGE       |
   ---------------------

   WHILE INACTIVE
   > Press button to activate
   > Hold button to enter color selection mode
    > Press to cycle modes
     > Hold to save selection

   WHILE ACTIVATED
   > Press button to extinguish
   > Hold button to enter animation selection mode
    > Press to cycle modes
     > Hold to save selection

   Possible Color Modes
   1. Energy
   2. Fire
   3. Water
   4. Rainbow

   Possible Animation Modes
   1. No extra animation
   2. Flicker on,  Comet Off
   3. Flicker off, Comet On
   4. Flicker on,  Comet On
   Note: Additional animations are ignored in Rainbow Color mode

*/

// ---------------------
// |     INCLUDES      |
// ---------------------
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>

#include <SoftwareSerial.h>
#include "Adafruit_Soundboard.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// ---------------------
// |      WIRING       |
// ---------------------

#define SFX_TX         3   // Sound Effects Soft Serial Transmit Wire
#define SFX_RX         5   // Sound Effects Soft Serial Receive Wire
#define SFX_RST        8   // Sound Effects Soft Serial Reset Wire (optional)
#define NEOPIXEL_PIN   6   // NeoPixel Serial Data
#define BUTTONPIN      4   // Digital Input - Button
#define LED_BUTTON_PIN 9   // Analog Output - Indicator LED
//      I2C SDA       A4   // Accelerometer Data
//      I2C SCL       A5   // Accelerometer Clock

// ---------------------
// |   MODULE SETUP    |
// ---------------------

//Accelerometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(117);
sensors_event_t event; //sensor raw data storage

//Software serial for sound board
SoftwareSerial ss = SoftwareSerial(SFX_TX, SFX_RX);
Adafruit_Soundboard sfx = Adafruit_Soundboard(&ss, NULL, SFX_RST);

//Setup for a standered strip of lights
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(49, NEOPIXEL_PIN);
Adafruit_NeoPixel strip  = Adafruit_NeoPixel(49, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ---------------------
// |     CONSTANTS      |
// ---------------------

//Rotation is identified before determines motion
enum axis_rotation {
  AXIS_X_POS = 0,
  AXIS_Y_POS,
  AXIS_Z_POS,
  AXIS_X_NEG,
  AXIS_Y_NEG,
  AXIS_Z_NEG,
};

//Pattern directions
#define FORWARD 0
#define REVERSE 1

//LED Button Heartbeat
#define HEARTBEAT_HIGH 254
#define HEARTBEAT_LOW   25

//Location of saved mode data
int eeprom_addr = 0;

//Color Modes Supported
#define MAX_COLOR_MODES 4
enum color_patters {
  ENERGY_COLOR = 0,
  FIRE_COLOR,
  WATER_COLOR,
  RAINBOW_COLOR,
};

#define MAX_SECONDARY_MODES 4
enum secondary_patterns {
  FLICKR_OFF_COMET_OFF = 0,
  FLICKR_ON_COMET_OFF,
  FLICKR_OFF_COMET_ON,
  FLICKR_ON_COMET_ON,
};

// ---------------------
// |     SOUNDMAP      |
// ---------------------

/*  List was generated using the Adafruit_Soundboard > menucommands example sketch.
    Type 'L' into the serial window
    Files are numbered in the order they were transfered to the sound board.
    Update soundmap data to coorespond to the numbered listed from the serial
    window, example output below.

0  name: POWERON WAV size: 44200
1  name: HUM     WAV size: 197030
2  name: POWEROFFWAV size: 36838
3  name: SWING2  WAV size: 27608
4  name: SWING3  WAV size: 27608
5  name: SWING4  WAV size: 27608
6  name: SWING5  WAV size: 27608
7  name: SWING6  WAV size: 27608
8  name: SWING7  WAV size: 27608
9  name: SWING8  WAV size: 27608
10 name: SWING1  WAV size: 27608
11 name: CLASH1  WAV size: 44200
12 name: CLASH2  WAV size: 44200
13 name: CLASH3  WAV size: 44200
14 name: CLASH4  WAV size: 44200
15 name: CLASH5  WAV size: 44200
16 name: CLASH6  WAV size: 44200
17 name: CLASH7  WAV size: 44200
18 name: CLASH8  WAV size: 44200
19 name: BOOT    WAV size: 33720
*/

#define SOUNDMAP_POWERON   0
#define SOUNDMAP_POWEROFF  2
#define SOUNDMAP_BOOT     19

#define MAX_NUM_SOUNDS 8
const uint8_t soundmap_swing[8] = {10,3,4,5,6,7,8,9};
const uint8_t soundmap_clash[8] = {11,12,13,14,15,16,17,18};

// ---------------------
// |      GLOBALS      |
// ---------------------

//color mixing controls
int primary_mix   = 20;
int secondary_mix = 100;
uint8_t mix_direction = FORWARD;

//brightness controls
uint8_t brightness_number = 200;
uint8_t brightness_direction = REVERSE;

//color wheel
int16_t color_index = 0;

//mode switches
uint8_t mode_primary_select   = 0;
uint8_t mode_secondary_select = 0;
uint8_t mode_comet1_select    = 0;
uint8_t mode_flicker2_select  = 0;

// ---------------------
// |    PROTOTYPES     |
// ---------------------

//default arguments here allow function to be called without parameters
void power_down_sequence(int sound = 1);
void rainbowCycle(int speed_sel = 100, int color_direction = FORWARD);

// ---------------------
// |     FUNCTIONS     |
// ---------------------

void setup()
{
   // ----------------
   // LED STRIP MODULE
   // ----------------

   strip.begin();
   strip.show(); // Initialize all pixels to 'off'

   // ----------------
   //  DIGTAL INPUTS
   // ----------------

   pinMode(BUTTONPIN, INPUT_PULLUP);

   // ----------------
   //  SERIAL HEADER
   // ----------------

   Serial.begin(115200);
   Serial.println("Halo Sword v1.0");
   Serial.println("Dustin Westaby");
   Serial.println("April 2015\n");

   // ----------------
   //  ACCELEROMETER
   // ----------------

   if(!accel.begin())
   {
      Serial.println("ADXL345 Not found");
   }

   /* Set the range */
   accel.setRange(ADXL345_RANGE_2_G);

   // ----------------
   //    SOUNDBOARD
   // ----------------

   ss.begin(9600); // softwareserial at 9600 baud

   //sfx.reset(); //opted not to reset to speed up boot time

   // ----------------
   //  MODE SELECTION
   // ----------------

   //retrieve mode from eeprom
   mode_primary_select   = EEPROM.read(eeprom_addr);
   mode_secondary_select = EEPROM.read(eeprom_addr + 1);

   //validate data from eeprom
   if(mode_primary_select >= MAX_COLOR_MODES)
   {
      mode_primary_select = 0;
   }
   if(mode_secondary_select > MAX_SECONDARY_MODES)
   {
      mode_secondary_select = 0;
   }

   //set global mode switches
   set_secondary_switches();

   // ----------------
   // PREPARE FOR MAIN
   // ----------------

   //play boot sound effect
   sfx.playTrack((uint8_t)SOUNDMAP_BOOT);
   delay(500);
}

void loop()
{
  unsigned long event_save_ms;

   // ----------------
   //    POWER OFF
   // ----------------

  //stay off for at least 0.5 seconds before allowing power up (syncs sounds)
  delay(500);

  //Wait For Power On (returns when button pressed)
  // this function also handles color mode selection
  wait_for_button_press_to_on();

   // ----------------
   //    POWER ON
   // ----------------

  //enter main run loop, button LED is off
  analogWrite(LED_BUTTON_PIN, 0);

  //power up light sequence (with sound effect)
  power_up_sequence();

  //stay on for at least 1 second before allowing power down or motion (syncs sounds)
  event_save_ms = millis();
  while( millis() - event_save_ms < 1000)
  {
    animate_selected_mode();
  }

  //Wait For Power Off (returns when button pressed)
  // this function also handles:
  // - animation mode selection
  // - animation of selected mode
  // - swing detection for sound effects
  wait_for_button_press_to_off();

  //power down light sequence (with sound effect)
  power_down_sequence();

}

// ----------------------
// | BUTTON SUBROUTINES |
// ----------------------

void wait_for_button_press_to_off()
{
  //this function performs the active animation and secondary mode selection

  unsigned long button_timer_last_pressed_off;
  uint8_t mode_secondary_selected;

  //wait for button press to extinguish from powered on
  while(digitalRead(BUTTONPIN))
  {
    animate_selected_mode();
  }
  button_timer_last_pressed_off = millis();
  delay(50);

  //wait for button release
  while(!digitalRead(BUTTONPIN))
  {
    if(millis() - button_timer_last_pressed_off > 2000) //enter ON config mode
    {
      //indicate button held by blinking LED
      //blinking = ok to let go of button
      analogWrite(LED_BUTTON_PIN, 0);
      delay(100);
      analogWrite(LED_BUTTON_PIN, 255);
      delay(100);
    }
  }

  //button was held down, enter mode select
  if((millis() - button_timer_last_pressed_off) > 2000)
  {
    mode_secondary_selected = 0;

    //wait for button to be held down again (save mode)
    while(mode_secondary_selected == 0)
    {
      //wait for button press
      while(digitalRead(BUTTONPIN))
      {
        animate_selected_mode();
      }
      button_timer_last_pressed_off = millis();
      delay(50);

      //wait for button release
      while(!digitalRead(BUTTONPIN))
      {
        if(millis() - button_timer_last_pressed_off > 2000)
        {
          //indicate button held by blinking LED
          //blinking = ok to let go of button
          analogWrite(LED_BUTTON_PIN, 0);
          delay(100);
          analogWrite(LED_BUTTON_PIN, 255);
          delay(100);
        }
      }

      //check for button held down
      if((millis() - button_timer_last_pressed_off) > 2000)
      {
        //escape from mode select
        mode_secondary_selected = 1;

        //save mode to eeprom
        EEPROM.write(eeprom_addr + 1, mode_secondary_select);
      }
      else
      {
        //normal button press, change modes
        mode_secondary_select++;
        if (mode_secondary_select >= MAX_COLOR_MODES)
        {
          mode_secondary_select = 0;
        }
        set_secondary_switches();
      }
    }
  } //end mode select
} //end wait for OFF button press

void set_secondary_switches()
{
  //common code for setting secondary modes

  switch(mode_secondary_select)
  {
    case FLICKR_OFF_COMET_OFF:
      mode_comet1_select   = 0;
      mode_flicker2_select = 0;
      break;
    case FLICKR_ON_COMET_OFF:
      mode_comet1_select   = 0;
      mode_flicker2_select = 1;
      break;
    case FLICKR_OFF_COMET_ON:
      mode_comet1_select   = 1;
      mode_flicker2_select = 0;
      break;
    case FLICKR_ON_COMET_ON:
      mode_comet1_select   = 1;
      mode_flicker2_select = 1;
      break;
    default:
      mode_comet1_select   = 0;
      mode_flicker2_select = 0;
      break;
  }
}

void wait_for_button_press_to_on()
{
  unsigned long button_timer_last_pressed_on;
  int mode_primary_selected = 0;

  //wait for button press to activate from powered off
  analogWrite(LED_BUTTON_PIN, 255);
  while(digitalRead(BUTTONPIN))
  {
    fade_power_led();
  }
  button_timer_last_pressed_on = millis();
  delay(50);

  //wait for button release
  while(!digitalRead(BUTTONPIN))
  {
    if(millis() - button_timer_last_pressed_on > 2000) //enter OFF config mode
    {
      //indicate button held by blinking LED
      //blinking = ok to let go of button
      analogWrite(LED_BUTTON_PIN, 0);
      delay(100);
      analogWrite(LED_BUTTON_PIN, 255);
      delay(100);
    }
  }

  //button was held down, enter mode select
  if((millis() - button_timer_last_pressed_on) > 2000)
  {
    mode_primary_selected = 0;

    //wait for button to be held down again (save mode)
    while(mode_primary_selected == 0)
    {
      //wait for button press
      while(digitalRead(BUTTONPIN))
      {
        if (mode_primary_select == RAINBOW_COLOR)
        {
          //demo rainbow mode (swing detection is only in run_rainbow_mode)
          rainbowCycle();
        }
        else
        {
          animate_selected_mode();
        }
      }
      button_timer_last_pressed_on = millis();
      delay(50);

      //wait for button release
      while(!digitalRead(BUTTONPIN))
      {
        if(millis() - button_timer_last_pressed_on > 2000)
        {
          //indicate button held by blinking LED
          //blinking = ok to let go of button
          analogWrite(LED_BUTTON_PIN, 0);
          delay(100);
          analogWrite(LED_BUTTON_PIN, 255);
          delay(100);
        }
      }

      //check for button held down
      if((millis() - button_timer_last_pressed_on) > 2000)
      {
        //escape from mode select
        mode_primary_selected = 1;

        //cycle power to lights (no sound)
        power_down_sequence(0);

        //save mode to eeprom
        EEPROM.write(eeprom_addr, mode_primary_select);
      }
      else
      {
        //normal button press, change modes
        mode_primary_select++;
        if (mode_primary_select >= MAX_COLOR_MODES)
        {
          mode_primary_select = 0;
        }
      }
    }
  } //end mode select
} //end wait for ON button press

// ------------------------
// | LIGHTING SUBROUTINES |
// ------------------------

void fade_power_led()
{
  static unsigned long lastUpdateFade;
  static int16_t on_time = 0;
  static int led_fade_direction = REVERSE;

  //this function will fade the button's indicator LED with a heartbeat

  // time to update?
  if((millis() - lastUpdateFade) > 5)
  {
    lastUpdateFade = millis();

    if(led_fade_direction == REVERSE)
    {
      on_time-=1;
      if(on_time <= HEARTBEAT_LOW)
      {
        on_time=HEARTBEAT_LOW;
        led_fade_direction = FORWARD;
      }
    }
    else
    {
      on_time+=1;
      if(on_time >= HEARTBEAT_HIGH)
      {
        on_time=HEARTBEAT_HIGH;
        led_fade_direction = REVERSE;
      }
    }

    //set pwm on LED
    analogWrite(9, on_time);

  } //end update
}

void power_up_sequence()
{

  //play boot sound effect
  sfx.playTrack((uint8_t)SOUNDMAP_POWERON);

  //startup sequence
  for (uint16_t i = 0; (i < strip.numPixels()); i++)
  {

    switch (mode_primary_select)
    {
      case ENERGY_COLOR:
        strip.setPixelColor(i, strip.Color(11, 100, 222));
        break;
      case FIRE_COLOR:
        strip.setPixelColor(i, strip.Color(222, 100, 11));
        break;
      case WATER_COLOR:
        strip.setPixelColor(i, strip.Color(11, 222, 100));
        break;
      case RAINBOW_COLOR:
        color_wheel(((i * 256 / strip.numPixels())) & 255, i);
        break;
      default:
        strip.setPixelColor(i, strip.Color(11, 100, 222));
        break;
    }
    strip.show();
    delay(5);
  }
}

void power_down_sequence(int sound )
{
  //play powerdown sound effect
  if(sound ==1)
  {
    sfx.playTrack((uint8_t)SOUNDMAP_POWEROFF);
  }

  //turn off lights in sequence
  for (uint16_t i = strip.numPixels(); (i > 0); i--)
  {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
    strip.show();
    delay(8);
  }
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();

  //reset animation sequences
  primary_mix = 20;
  secondary_mix = 100;
  mix_direction = FORWARD;
  brightness_number = 200;
  brightness_direction = REVERSE;
  strip.setBrightness(brightness_number);
  color_index=0;
}

void update_brightness_fade()
{
  uint8_t motion_value = motion_filter();
  //this is a secondary fade animation.
  // it layers ontop of the color mixing

  if(brightness_direction == REVERSE)
  {
    brightness_number--;
    if(brightness_number < 80)
    {
      brightness_direction = FORWARD;
    }
  }
  else
  {
    brightness_number++;
    if(brightness_number > 200)
    {
      brightness_direction = REVERSE;
    }
  }

  if(mode_flicker2_select) //enable or disable flicker
  { //flicker enabled
    if((brightness_number + motion_value) > 200)
    {
      strip.setBrightness(200);
    }
    else if ((brightness_number + motion_value) < 80)
    {
      strip.setBrightness(80);
    }
    else
    {
      strip.setBrightness(brightness_number + motion_value);
    }
  }
  else
  { //flicker disabled
    strip.setBrightness(brightness_number);
  }
}

void animate_selected_mode()
{
  static unsigned long lastUpdate_foreground;
  static unsigned long lastUpdate_background;
  uint32_t background_color;
  uint32_t foreground_color;
  static uint8_t comet_location = 0;

  if(mode_primary_select == RAINBOW_COLOR)
  {
    //also handles swing detection
    run_rainbow_mode();
  }
  else //other color modes
  {
    //trigger sound effects
    check_motion_for_swings();

    // time to update?
    if((millis() - lastUpdate_foreground) > 8) //speed of foreground animation (comet)
    {
      lastUpdate_foreground = millis();

      if((millis() - lastUpdate_background) > 20) //speed of background animation (color mixing and brightness fade)
      {
        lastUpdate_background = millis();

        //LED brightness changes
        update_brightness_fade();

        //perform primary color mix calculations
        if(mix_direction == REVERSE)
        {
          primary_mix--;
          if(primary_mix < 20)
          {
            mix_direction = FORWARD;
          }
        }
        else
        {
          primary_mix++;
          if(primary_mix > 100)
          {
            mix_direction = REVERSE;
          }
        }

        //perform boundary check on secondary mix
        secondary_mix = 100 - primary_mix;
        if(secondary_mix <1)
        {
          secondary_mix=1;
        }
      } //end update background

      //lookup color pattern for Static Fade distribution
      switch (mode_primary_select)
      {                                  //RED,        GREEN,         BLUE
         case ENERGY_COLOR:
            background_color = strip.Color(primary_mix, secondary_mix, 222);
            foreground_color = strip.Color(secondary_mix, 222, 222);
            break;
         case FIRE_COLOR:
            background_color = strip.Color(255, primary_mix, 0);
            foreground_color = strip.Color(222, secondary_mix, 0);
            break;
         case WATER_COLOR:
            background_color = strip.Color(primary_mix, 222, secondary_mix);
            foreground_color = strip.Color(secondary_mix, 222, 222);
            break;
         case RAINBOW_COLOR: // NOT USED, Rainbow is handled elsewhere, see run_rainbow_mode
            Serial.println("Error: Rainbow"); // log error then proceed to use default animation
         default:
            background_color = strip.Color(primary_mix, secondary_mix, 222);
            foreground_color = strip.Color(20, 80, 222);
            break;
      }

      //reset background color
      for (uint16_t i = 0; (i < strip.numPixels()); i++)
      {
        strip.setPixelColor(i, background_color);
      }

      if(mode_comet1_select) //comet enabled?
      {
        //calculate foreground comet location
        comet_location++;
        if(comet_location > strip.numPixels())
        {
           comet_location = 0;
        }
        //comet is 2 pixels wide
        strip.setPixelColor(comet_location, foreground_color);
        strip.setPixelColor(comet_location-1, foreground_color);
      }

      //update lights
      strip.show();

    } //end handle update timer
  } //end other color modes
} // end animate_selected_mode

// -----------------------
// | RAINBOW SUBROUTINES |
// -----------------------

void run_rainbow_mode()
{
  unsigned long event_save_ms;
  uint8_t filter_value = motion_filter();
  int randNumber = random(MAX_NUM_SOUNDS);

  Serial.println(filter_value);

  //check for swings (higher sensitivity than normal)
  if(filter_value > 30)
  {
    sfx.playTrack((uint8_t)soundmap_swing[randNumber]);

    //run at high speed and reverse for 1 second
    event_save_ms = millis();
    while( millis() - event_save_ms < 1000)
    {
      rainbowCycle(0, REVERSE);
    }
  }
  else
  {
    //default speed
    rainbowCycle();
  }
}

void rainbowCycle(int speed_sel, int color_direction)
{
  uint16_t i;
  static unsigned long lastUpdateCycle;
  static unsigned long lastSpeedChange;
  static uint16_t randomspeed;

  //change speed after period of time
  if((millis() - lastSpeedChange) > 500)
  {
    lastSpeedChange = millis();
    randomspeed += random(1,5);
    if( randomspeed > 30)
    {
      randomspeed = random(1,4)*5;
    }
  }

  // time to update (does some speed alterations)
  if((millis() - lastUpdateCycle) > min(randomspeed,speed_sel))
  {
    lastUpdateCycle = millis();

    //increment index and handle rollover
    if(color_direction == FORWARD)
    {
      if(--color_index<=0)
      {
        color_index=(256*5); // 5 cycles of all colors on wheel
      }
    }
    else //REVERSE
    {
      if(++color_index>(256*5))
      {
        color_index=0;
      }
    }

    //color wheel calculations from position
    for(i=0; i< strip.numPixels(); i++)
    {
      color_wheel(((i * 256 / strip.numPixels()) + color_index) & 255, i);
    }
    strip.show();
  } //end update strip
} //end rainbowCycle

void color_wheel(byte ColorWheelPos, uint16_t i)
{
  if(ColorWheelPos < 85)
  {
    strip.setPixelColor(i, strip.Color(ColorWheelPos * 3, 255 - ColorWheelPos * 3, 0));
  }
  else if(ColorWheelPos < 170)
  {
    ColorWheelPos -= 85;
    strip.setPixelColor(i, strip.Color(255 - ColorWheelPos * 3, 0, ColorWheelPos * 3));
  }
  else
  {
    ColorWheelPos -= 170;
    strip.setPixelColor(i, strip.Color(0, ColorWheelPos * 3, 255 - ColorWheelPos * 3));
  }
}

// ----------------------
// | MOTION SUBROUTINES |
// ----------------------

int check_motion_for_swings()
{
  //This function performs the complex task to
  // determine if a swing or a melee motion was
  // detected.  It then will play a random sound
  // that coorosponds to that motion.

  int return_value = 0;
  static int active_axis=AXIS_X_POS;
  static int active_counter_x_pos = 0;
  static int active_counter_y_pos = 0;
  static int active_counter_z_pos = 0;
  static int active_counter_x_neg = 0;
  static int active_counter_y_neg = 0;
  static int active_counter_z_neg = 0;
  static long last_sound_timestamp = 0;
  int randNumber = random(MAX_NUM_SOUNDS);

  /* Get a new sensor event */
  accel.getEvent(&event);

  // ------------------------
  //  POSITIVE ROATION CHECK
  // ------------------------

  if (event.acceleration.x > 5)
  {
    active_counter_x_pos++;
    if (active_counter_x_pos > 50)
    {
      active_counter_x_pos=25;
      active_counter_y_pos=0;
      active_counter_z_pos=0;
      active_counter_x_neg = 0;
      active_counter_y_neg = 0;
      active_counter_z_neg = 0;
      active_axis=AXIS_X_POS;
    }
  }
  if (event.acceleration.y > 5)
  {
    active_counter_y_pos++;
    if (active_counter_y_pos > 50)
    {
      active_counter_y_pos=25;
      active_counter_x_pos=0;
      active_counter_z_pos=0;
      active_counter_x_neg = 0;
      active_counter_y_neg = 0;
      active_counter_z_neg = 0;
      active_axis=AXIS_Y_POS;
    }
  }
  if (event.acceleration.z > 5)
  {
    active_counter_z_pos++;
    if (active_counter_z_pos > 50)
    {
      active_counter_z_pos=25;
      active_counter_y_pos=0;
      active_counter_x_pos=0;
      active_counter_x_neg = 0;
      active_counter_y_neg = 0;
      active_counter_z_neg = 0;
      active_axis=AXIS_Z_POS;
    }
  }

  // ------------------------
  //  NEGATIVE ROATION CHECK
  // ------------------------

  if (event.acceleration.x < -5)
  {
    active_counter_x_neg++;
    if (active_counter_x_neg > 50)
    {
      active_counter_x_pos=0;
      active_counter_y_pos=0;
      active_counter_z_pos=0;
      active_counter_x_neg = 25;
      active_counter_y_neg = 0;
      active_counter_z_neg = 0;
      active_axis=AXIS_X_NEG;
    }
  }
  if (event.acceleration.y < -5)
  {
    active_counter_y_neg++;
    if (active_counter_y_neg > 50)
    {
      active_counter_y_pos=0;
      active_counter_x_pos=0;
      active_counter_z_pos=0;
      active_counter_x_neg = 0;
      active_counter_y_neg = 25;
      active_counter_z_neg = 0;
      active_axis=AXIS_Y_NEG;
    }
  }
  if (event.acceleration.z < -5)
  {
    active_counter_z_neg++;
    if (active_counter_z_neg > 50)
    {
      active_counter_z_pos=0;
      active_counter_y_pos=0;
      active_counter_x_pos=0;
      active_counter_x_neg = 0;
      active_counter_y_neg = 0;
      active_counter_z_neg = 25;
      active_axis=AXIS_Z_NEG;
    }
  }

  // ----------------------------
  //  DETERMINE MOTION DIRECTION
  // ----------------------------
  // Different motions activate different sounds

  if(millis() - last_sound_timestamp > 1005) //each sound byte is approx 1 second duration
  {
    //Y Positive Trigger
    if (active_axis==AXIS_Y_POS)
    {
      if((event.acceleration.y > 15) || (event.acceleration.y < 5))
      {
        return_value = 1;

        sfx.playTrack((uint8_t)soundmap_swing[randNumber]);
        last_sound_timestamp = millis();
      }
    }

    //Y Negative Trigger
    else if (active_axis==AXIS_Y_NEG)
    {
      if((event.acceleration.y < -15) || (event.acceleration.y > -5))
      {
        return_value = 1;

        sfx.playTrack((uint8_t)soundmap_swing[randNumber]);
        last_sound_timestamp = millis();
      }
    }

    //X Positive Trigger
    else if (active_axis==AXIS_X_POS)
    {
      if((event.acceleration.x > 15) || (event.acceleration.x < 5))
      {
        return_value = 1;

        sfx.playTrack((uint8_t)soundmap_swing[randNumber]);
        last_sound_timestamp = millis();
      }
    }

    //X Negative Trigger
    else if (active_axis==AXIS_X_NEG)
    {
      if((event.acceleration.x < -15) || (event.acceleration.x > -5))
      {
        return_value = 1;

        sfx.playTrack((uint8_t)soundmap_swing[randNumber]);
        last_sound_timestamp = millis();
      }
    }

    //Y Secondary Trigger
    if ( (active_axis==AXIS_Z_NEG) ||
         (active_axis==AXIS_Z_POS) ||
         (active_axis==AXIS_X_POS) ||
         (active_axis==AXIS_X_NEG) )
    {
      if((event.acceleration.y > 10) || (event.acceleration.y < -10))
      {
        return_value = 1;

        sfx.playTrack((uint8_t)soundmap_clash[randNumber]);
        last_sound_timestamp = millis();
      }
    }

    //X Secondary trigger
    if ( (active_axis==AXIS_Z_NEG) ||
         (active_axis==AXIS_Z_POS) ||
         (active_axis==AXIS_Y_POS) ||
         (active_axis==AXIS_Y_NEG) )
    {
      if((event.acceleration.x > 10) || (event.acceleration.x < -10))
      {
        return_value = 1;

        sfx.playTrack((uint8_t)soundmap_clash[randNumber]);
        last_sound_timestamp = millis();
      }
    }

  }// end sound timer

  return return_value;
}

uint8_t motion_filter()
{
  //used for motion based flicker light animation
  // and rainbow swing detection

  int16_t xsave = accel.getX();
  int16_t ysave = accel.getY();
  int16_t zsave = accel.getZ();
  int16_t value_save;

  value_save = (int16_t)sqrt(xsave * xsave +
                             ysave * ysave +
                             zsave * zsave);

  value_save = abs(value_save);
  value_save = value_save/5;

  return value_save;
}
