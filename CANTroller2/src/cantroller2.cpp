// Carpet CANTroller II  Source Code  - For Arduino Due with Adafruit 2.8inch Captouch TFT shield.
#include <SPI.h>  // SPI serial bus needed to talk to the LCD and the SD card
#include <Wire.h>  // Contains I2C serial bus, needed to talk to touchscreen chip
#include "Arduino.h"
#include <Adafruit_SleepyDog.h>  // Watchdog
#include <vector>
#include <string>
#include <iomanip>  // Formatting cout
using namespace std;

// Display.H

#ifdef CAP_TOUCH
    #include <Adafruit_FT6206.h>  // For interfacing with the cap touchscreen controller chip
    bool cap_touch = true;
#else
    #include <XPT2046_Touchscreen.h>
    bool cap_touch = false;

#endif

// #include <font_Arial.h> // from ILI9341_t3
// #include <SPI.h>

#include <Adafruit_ILI9341.h>
// #include <TFT_eSPI.h>

// #include <ILI9341_t3.h>  // A different tft library that came with the resistive touchscreen


// display related globals
#define BLK  0x0000
#define BLU  0x001f
#define MBLU 0x009f  // midnight blue. b/c true blue too dark to see over black
#define RBLU 0x043f  // royal blue
#define RED  0xf800
#define DRED 0xb000
#define GRN  0x07e0
#define CYN  0x07ff  // 00000 111 111 11111 
#define DCYN 0x0575  //
#define MGT  0xf81f
#define ORG  0xfca0
#define DORG 0xfa40  // Dark orange aka brown
#define YEL  0xffe0
#define LYEL 0xfff8
#define WHT  0xffff
#define DGRY 0x39c7  // very dark grey
#define GRY1 0x8410  // 10000 100 000 10000 = 84 10  dark grey
#define GRY2 0xc618  // 11000 110 000 11000 = C6 18  light grey
#define LGRY 0xd6ba  // very light grey
#define PNK  0xfcf3  // Pink is the best color
#define DPNK 0xfa8a  // We need all shades of pink
#define LPNK 0xfe18  // Especially light pink, the champagne of pinks

// LCD supports 18-bit color, but GFX library uses 16-bit color, organized (MSB) 5b-red, 6b-green, 5b-blue (LSB)
// Since the RGB don't line up with the nibble boundaries, it's tricky to quantify a color, here are some colors:
// Color picker websites: http://www.barth-dev.de/online/rgb565 , https://chrishewett.com/blog/true-rgb565-colour-picker/
// LCD is 2.8in diagonal, 240x320 pixels

#define disp_width_pix 320  // Horizontal resolution in pixels (held landscape)
#define disp_height_pix 240  // Vertical resolution in pixels (held landscape)
#define disp_lines 20  // Max lines of text displayable at line height = disp_line_height_pix
#define disp_fixed_lines 11  // Lines of static variables/values always displayed
#define disp_tuning_lines 8  // Lines of dynamic variables/values in dataset pages 
#define disp_line_height_pix 12  // Pixel height of each text line. Screen can fit 16x 15-pixel or 20x 12-pixel lines
#define disp_vshift_pix 2  // Unknown.  Note: At smallest text size, characters are 5x7 pix + pad on rt and bot for 6x8 pix.
#define disp_font_height 8
#define disp_font_width 6
#define disp_bargraph_width 40
#define disp_bargraph_squeeze 1
#define disp_maxlength 6  // How many characters fit between the ":" and the units string
#define touch_cell_v_pix 48  // When touchscreen gridded as buttons, height of each button
#define touch_cell_h_pix 53  // When touchscreen gridded as buttons, width of each button
#define touch_margin_h_pix 1  // On horizontal axis, we need an extra margin along both sides button sizes to fill the screen

// ctrl.h
// This trickery lets us keep the ISR code here in the same file as the encoder class
#define MAKE_ENCODER(NAME, a_pin, b_pin, sw_pin) \
Encoder NAME = Encoder((a_pin), (b_pin), (sw_pin)); \
void IRAM_ATTR NAME##_a_isr() { \
    if ((NAME)._bounce_danger != Encoder::ENC_A) { \
        if (!(NAME)._a_stable) { \
            (NAME)._spinrate_isr_us = (NAME)._spinspeedTimer.elapsed(); \
            (NAME)._spinspeedTimer.reset(); \
            (NAME)._delta += digitalRead(b_pin) ? -1 : 1; \
        } \
        (NAME)._bounce_danger = Encoder::ENC_A; \
    } \
}; \
void IRAM_ATTR NAME##_b_isr() { \
    if ((NAME)._bounce_danger != Encoder::ENC_B) { \
        (NAME)._a_stable = digitalRead(a_pin); \
        (NAME)._bounce_danger = Encoder::ENC_B; \
    } \
};

#define SETUP_ENCODER(NAME) \
(NAME)._a_isr = &NAME##_a_isr; \
(NAME)._b_isr = &NAME##_b_isr; \
(NAME).setup();

// globals.h
#include <SdFat.h>  // SD card & FAT filesystem library
#include <Servo.h>  // Makes PWM output to control motors (for rudimentary control of our gas and steering)
#include <Adafruit_NeoPixel.h>  // Plan to allow control of neopixel LED onboard the esp32
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Arduino.h"
#include <Preferences.h>
#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>
#include <string>
#include <iomanip>

// Here are the different runmodes documented
//
// ** Basic Mode **
// - Required: BasicMode switch On
// - Priority: 1 (Highest)
// The gas and brake don't do anything in Basic Mode. Just the steering works, so use the pedals.
// This mode is enabled with a toggle switch in the controller box.  When in Basic Mode, the only
// other valid mode is Shutdown Mode. Shutdown Mode may override Basic Mode.
// - Actions: Release and deactivate brake and gas actuators.  Steering PID keep active  
//
// ** Shutdown Mode **
// - Required: BasicMode switch Off & Ignition Off
// - Priority: 2
// This mode is active whenever the ignition is off.  In other words, whenever the
// little red pushbutton switch by the joystick is unclicked.  This happens before the
// ignition is pressed before driving, but it also may happen if the driver needs to
// panic and E-stop due to loss of control or any other reason.  The ignition will get cut
// independent of the controller, but we can help stop the car faster by applying the
// brakes. Once car is stopped, we release all actuators and then go idle.
// - Actions: 1. Release throttle. If car is moving AND BasicMode Off, apply brakes to stop car
// - Actions: 2: Release brakes and deactivate all actuators including steering
//
// ** Stall Mode **
// - Required: Engine stopped & BasicMode switch Off & Ignition On
// - Priority: 3
// This mode is active when the engine is not running.  If car is moving, then it presumably may
// coast to a stop.  The actuators are all enabled and work normally.  Starting the engine will 
// bring you into Hold Mode.  Shutdown Mode and Basic Mode both override Stall Mode. Note: This
// mode allows for driver to steer while being towed or pushed, or working on the car.
// - Actions: Enable all actuators
//
// ** Hold Mode **
// - Required: Engine running & JoyVert<=Center & BasicMode switch Off & Ignition On
// - Priority: 4
// This mode is entered from Stall Mode once engine is started, and also, whenever the car comes
// to a stop while driving around in Fly Mode.  This mode releases the throttle and will 
// continuously increase the brakes until the car is stopped, if it finds the car is moving. 
// Pushing up on the joystick from Hold mode releases the brakes & begins Fly Mode.
// Shutdown, Basic & Stall Modes override Hold Mode.
// # Actions: Close throttle, and Apply brake to stop car, continue to ensure it stays stopped.
//
// ** Fly Mode **
// - Required: (Car Moving OR JoyVert>Center) & In gear & Engine running & BasicMode Off & Ign On
// - Priority: 5
// This mode is for driving under manual control. In Fly Mode, vertical joystick positions
// result in a proportional level of gas or brake (AKA "Manual" control).  Fly Mode is
// only active when the car is moving - Once stopped or taken out of gear, we go back to Hold Mode.
// If the driver performs a special secret "cruise gesture" on the joystick, then go to Cruise Mode.
// Special cruise gesture might be: Pair of sudden full-throttle motions in rapid succession
// - Actions: Enable all actuators, Watch for gesture
//
// ** Cruise Mode **
// - Required: Car Moving & In gear & Engine running & BasicMode switch Off & Ignition On
// - Priority: 6 (Lowest)
// This mode is entered from Fly Mode by doing a special joystick gesture. In Cruise Mode,
// the brake is disabled, and the joystick vertical is different: If joyv at center, the
// throttle will actively maintain current car speed.  Up or down momentary joystick presses
// serve to adjust that target speed. A sharp, full-downward gesture will drop us back to 
// Fly Mode, promptly resulting in braking (if kept held down).
// - Actions: Release brake, Maintain car speed, Handle joyvert differently, Watch for gesture

// Defines for all the GPIO pins we're using
#define button_pin 0  // (button0 / strap to 1) - This is the left "Boot" button on the esp32 board
#define joy_horz_pin 1  // (adc) - Either analog left-right input (joystick)
#define joy_vert_pin 2  // (adc) - Either analog up-down input (joystick)
#define tft_dc_pin 3  // (strap X) - Output, Assert when sending data to display chip to indicate commands vs. screen data
#define battery_pin 4  // (adc) -  Analog input, mule battery voltage level, full scale is 15.638V
#define pot_wipe_pin 5  // (adc) - Analog in from 20k pot. Use 1% series R=22k to 5V on wipe=CW-0ohm side, and R=15k to gnd on wipe-CCW-0ohm side. Gives wipe range of 1.315V (CCW) to 3.070V (CW) with 80 uA draw.
#define brake_pos_pin 6  // (adc) - Analog input, tells us linear position of brake actuator. Blue is wired to ground, POS is wired to white.
#define pressure_pin 7  // (adc) - Analog input, tells us brake fluid pressure. Needs a R divider to scale max possible pressure (using foot) to 3.3V.
#ifdef CAP_TOUCH
    #define i2c_sda_pin 8  // (i2c0 sda / adc) - Hijack these pins for the touchscreen and micro-sd i2c bus
    #define i2c_scl_pin 9  // (i2c0 scl / adc) - Hijack these pins for the touchscreen and micro-sd i2c bus
#else
    #define touch_irq_pin 8  // (i2c0 scl / adc) - With resistive touchscreen this pin is freed up
    #define touch_cs_pin 9  // (i2c0 scl / adc) - Use as chip select for resistive touchscreen
#endif
#define tft_cs_pin 10  // (spi0 cs) -  Output, active low, Chip select allows ILI9341 display chip use of the SPI bus
#define tft_mosi_pin 11  // (spi0 mosi) - Used as spi interface data to sd card and tft screen
#define tft_sclk_pin 12  // (spi0 sclk) - Used as spi interface clock for sd card and tft screen
#define tft_miso_pin 13  // (spi0 miso) - Used as spi interface data from sd card and possibly (?) tft screen
#define steer_pwm_pin 14  // (pwm0) - Output, PWM signal positive pulse width sets steering motor speed from full left to full speed right, (50% is stopped). Jaguar asks for an added 150ohm series R when high is 3.3V
#define brake_pwm_pin 15  // (pwm1) - Output, PWM signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% is stopped) 
#define gas_pwm_pin 16  // (pwm1) - Output, PWM signal duty cycle controls throttle target. On Due this is the pin labeled DAC1 (where A13 is on Mega)
#define hotrc_ch1_horz_pin 17  // (pwm0 / tx1) - Hotrc Ch1 thumb joystick input.
#define hotrc_ch2_vert_pin 18  // (pwm0 / rx1) - Hotrc Ch2 bidirectional trigger input
#define onewire_pin 19  // (usb-otg) - Onewire bus for temperature sensor data
#define hotrc_ch3_ign_pin 20  // (usb-otg) - Ignition control, Hotrc Ch3 PWM toggle signal
#define hotrc_ch4_cruise_pin 21  // (pwm0) - Cruise control, Hotrc Ch4 PWM toggle signal
#define tach_pulse_pin 35  // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per engine rotation. (no pullup)
#define speedo_pulse_pin 36  // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per driven pulley rotation. Open collector sensors need pullup)
#define ignition_pin 37  // (spi-ram / oct-spi) - Output flips a relay to kill the car ignition, active high (no pullup)
#define syspower_pin 38  // (spi-ram / oct-spi) - Output, flips a relay to power all the tranducers
#define tft_rst_pin 39  // TFT Reset allows us to reboot the screen when it crashes
#define encoder_b_pin 40  // Int input, The B (aka DT) pin of the encoder. Both A and B complete a negative pulse in between detents. If B pulse goes low first, turn is CW. (needs pullup)
#define encoder_a_pin 41  // Int input, The A (aka CLK) pin of the encoder. Both A and B complete a negative pulse in between detents. If A pulse goes low first, turn is CCW. (needs pullup)
#define encoder_sw_pin 42  // Input, Encoder above, for the UI.  This is its pushbutton output, active low (needs pullup)
#define joy_ign_btn_pin 43  // (uart0 tx) - Joystick ignition button. Reserve for possible jaguar interface
#define joy_cruise_btn_pin 44  // (uart0 rx) - Joystick cruise button. Reserve for possible jaguar interface
#define starter_pin 45  // (strap to 0) - Input, active high when vehicle starter is engaged (needs pulldown)
#define basicmodesw_pin 46  // (strap X) - Input, asserted to tell us to run in basic mode, active low (needs pullup)
#define sdcard_cs_pin 47  // Output, chip select allows SD card controller chip use of the SPI bus, active low
#define neopixel_pin 48  // (rgb led) - Data line to onboard Neopixel WS281x

// #define ctrl_horz_ch1_pin 1  // (adc) - Either analog left-right input (joystick), or Hotrc Ch1 thumb joystick PWM signal.
// #define ctrl_vert_ch2_pin 2  // (adc) - Either analog up-down input (joystick), or Hotrc Ch2 bidirectional trigger signal.
// #define unused 17  // (pwm0 / tx1) - 
// #define unused 18  // (pwm0 / rx1) -  

#define tp_irq_pin -1  // Optional int input so touchpanel can interrupt us (need to modify shield board for this to work)
#define tft_ledk_pin -1  // (spi-ram / oct-spi) - Output, Optional PWM signal to control brightness of LCD backlight (needs modification to shield board to work)
#define encoder_pwr_pin -1
#define led_rx_pin -1  // Unused on esp32
#define led_tx_pin -1  // Unused on esp32
#define heartbeat_led_pin -1

#define adcbits 12
#define adcrange_adc 4095  // = 2^adcbits-1
#define adcmidscale_adc 2047  // = 2^(adcbits-1)-1

// Global settings
bool serial_debugging = true; 
bool timestamp_loop = false;  // Makes code write out timestamps throughout loop to serial port
bool take_temperatures = true;

// Persistent config storage
Preferences config;
    
// Readily available possibilities we could wire up if we want
//
// * Status LEDs (digital out)
// * Control of steering or brake motor coast vs. brake
// * CAN bus as a superior interface to brake and steering Jaguars (only on Due I think?)
// * Steering limit switches left and right, handle here instead of in Jaguar (digital in)
// * Engine temperature module overheat panic input (digital in)
// * Remote E-Stop panic inputs (digital in)
// * Serial interface to the lighting controller (if we can think of a reason)
// * Mule starter (digital out)
// * E-brake handle position (digital in)

// Globals -------------------
//
class Timer {  // 32 bit microsecond timer overflows after 71.5 minutes
  protected:
    volatile uint32_t start_us = 0;
    volatile uint32_t timeout_us = 0;
    volatile uint32_t remain_us;
    volatile bool enabled = true;
  public:
    Timer (void) { reset(); }
    Timer (uint32_t arg_timeout_us) { set (arg_timeout_us); }
    IRAM_ATTR void set (uint32_t arg_timeout_us) { timeout_us = arg_timeout_us; reset(); }
    IRAM_ATTR void reset (void) { start_us = esp_timer_get_time(); remain_us = timeout_us; }
    IRAM_ATTR void pause (void) { remain_us = remain(); enabled = false; }
    IRAM_ATTR void resume (void) { start_us = esp_timer_get_time() - remain_us; enabled = true; }
    IRAM_ATTR bool expired (void) { return (enabled) ? (esp_timer_get_time() >= start_us + timeout_us): false; }
    IRAM_ATTR uint32_t elapsed (void) { return (enabled) ? (esp_timer_get_time() - start_us) : (timeout_us - remain_us); }
    IRAM_ATTR uint32_t remain (void) { return (enabled) ? ((start_us + timeout_us) - esp_timer_get_time()) : remain_us; }
    IRAM_ATTR uint32_t get_timeout (void) { return timeout_us; }
    IRAM_ATTR bool get_enabled (void) { return enabled; }
};

double convert_units (double from_units, double convert_factor, bool invert, double in_offset = 0.0, double out_offset = 0.0) {
    if (!invert) return out_offset + convert_factor * (from_units - in_offset);
    if (from_units - in_offset) return out_offset + convert_factor / (from_units - in_offset);
    printf ("convert_units refused to divide by zero: %lf, %lf, %d, %lf, %lf", from_units, convert_factor, invert, in_offset, out_offset);
    return -1;
}

// run state globals
enum runmodes { BASIC, SHUTDOWN, STALL, HOLD, FLY, CRUISE, CAL };
int32_t runmode = SHUTDOWN;
int32_t oldmode = BASIC;  // So we can tell when the mode has just changed. start as different to trigger_mode start algo
int32_t gesture_progress = 0;  // How many steps of the Cruise Mode gesture have you completed successfully (from Fly Mode)
bool shutdown_complete = false;  // Shutdown mode has completed its work and can stop activity
bool we_just_switched_modes = true;  // For mode logic to set things up upon first entry into mode
bool park_the_motors = false;  // Indicates we should release the brake & gas so the pedals can be used manually without interference
bool calmode_request = false;
bool panic_stop = false;
bool cruise_gesturing = false;  // Is cruise mode enabled by gesturing?  Otherwise by press of cruise button
bool cruise_sw_held = false;
bool cruise_adjusting = false;
Timer gestureFlyTimer;  // Used to keep track of time for gesturing for going in and out of fly/cruise modes
Timer cruiseSwTimer;
Timer sleepInactivityTimer (10000000);  // After shutdown how long to wait before powering down to sleep
Timer stopcarTimer (7000000);  // Allows code to fail in a sensible way after a delay if nothing is happening
//  ---- tunable ----
uint32_t motor_park_timeout_us = 4000000;  // If we can't park the motors faster than this, then give up.
uint32_t gesture_flytimeout_us = 500000;  // Time allowed for joy mode-change gesture motions (Fly mode <==> Cruise mode) (in us)
uint32_t cruise_sw_timeout_us = 500000;  // how long do you have to hold down the cruise button to start cruise mode (in us)
Timer motorParkTimer(motor_park_timeout_us);

// calibration related
bool cal_joyvert_brkmotor = false;  // Allows direct control of brake motor using controller vert
bool cal_pot_gasservo = false;  // Allows direct control of gas servo using pot
bool cal_pot_gas_ready = false;  // To avoid immediately overturning gas pot, first pot must be turned to valid range
bool cal_set_hotrc_failsafe_ready = false;  

// generic values
//  ---- tunable ----
int32_t default_margin_adc = 12;  // Default margin of error for comparisons of adc values (ADC count 0-4095)

// pid related globals
//  ---- tunable ----
uint32_t steer_pid_period_ms = 185;  // (Not actually a pid) Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer steerPidTimer (steer_pid_period_ms*1000);  // not actually tunable, just needs value above
uint32_t brake_pid_period_ms = 185;  // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer brakePidTimer (brake_pid_period_ms*1000);  // not actually tunable, just needs value above
int32_t brake_spid_ctrl_dir = SPID::REV;  // 0 = fwd, 1 = rev. Because a higher value on the brake actuator pulsewidth causes a decrease in pressure value
double brake_spid_initial_kp = 2.18;  // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
double brake_spid_initial_ki_hz = 0.215;  // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
double brake_spid_initial_kd_s = 1.130;  // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
uint32_t cruise_pid_period_ms = 300;  // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer cruisePidTimer (cruise_pid_period_ms*1000);  // not actually tunable, just needs value above
double cruise_spid_initial_kp = 0.157;  // PID proportional coefficient (cruise) How many RPM for each unit of difference between measured and desired car speed  (unitless range 0-1)
double cruise_spid_initial_ki_hz = 0.035;  // PID integral frequency factor (cruise). How many more RPM for each unit time trying to reach desired car speed  (in 1/us (mhz), range 0-1)
double cruise_spid_initial_kd_s = 0.044;  // PID derivative time factor (cruise). How much to dampen sudden RPM changes due to P and I infuences (in us, range 0-1)
int32_t cruise_spid_ctrl_dir = SPID::FWD;  // 1 = fwd, 0 = rev.
uint32_t gas_pid_period_ms = 225;  // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer gasPidTimer (gas_pid_period_ms*1000);  // not actually tunable, just needs value above
double gas_spid_initial_kp = 0.245;  // PID proportional coefficient (gas) How much to open throttle for each unit of difference between measured and desired RPM  (unitless range 0-1)
double gas_spid_initial_ki_hz = 0.015;  // PID integral frequency factor (gas). How much more to open throttle for each unit time trying to reach desired RPM  (in 1/us (mhz), range 0-1)
double gas_spid_initial_kd_s = 0.022;  // PID derivative time factor (gas). How much to dampen sudden throttle changes due to P and I infuences (in us, range 0-1)
int32_t gas_spid_ctrl_dir = SPID::REV;  // 0 = fwd, 1 = rev.

// starter related
bool starter = LOW;
bool starter_last = LOW;
bool sim_starter = false;

// mule battery related
double battery_adc = adcmidscale_adc;
double battery_v = 10.0;
double battery_filt_v = 10.0;
//  ---- tunable ----
double battery_max_v = 16.0;  // The max vehicle voltage we can sense. Design resistor divider to match. Must exceed max V possible.
double battery_convert_v_per_adc = battery_max_v/adcrange_adc;
bool battery_convert_invert = false;
int32_t battery_convert_polarity = SPID::FWD;
double battery_ema_alpha = 0.01;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 

// potentiometer related
double pot_percent = 50;
double pot_filt_percent = pot_percent;
double pot_min_percent = 0;  //
double pot_max_percent = 100;  //
//  ---- tunable ----
double pot_min_adc = 0;  // TUNED 230603 - Used only in determining theconversion factor
double pot_max_adc = 4090;  // TUNED 230613 - adc max measured = ?, or 9x.? % of adc_range. Used only in determining theconversion factor
double pot_convert_percent_per_adc = (pot_max_percent - pot_min_percent)/(pot_max_adc - pot_min_adc);  // 100 % / (3996 adc - 0 adc) = 0.025 %/adc
bool pot_convert_invert = false;
double pot_convert_offset = -0.08;
int32_t pot_convert_polarity = SPID::FWD;
double pot_ema_alpha = 0.1;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 

// controller related
enum ctrls { HOTRC, JOY, SIM };  // Possible sources of gas, brake, steering commands
enum ctrl_axes { HORZ, VERT };
enum ctrl_thresh { MIN, DB, MAX };
enum ctrl_edge { BOT, TOP };
enum raw_filt { RAW, FILT };
bool joy_centered = false;
int32_t ctrl_db_adc[2][2];  // [HORZ/VERT] [BOT/TOP] - to store the top and bottom deadband values for each axis of selected controller
int32_t ctrl_pos_adc[2][2] = { { adcmidscale_adc, adcmidscale_adc }, { adcmidscale_adc, adcmidscale_adc} };  // [HORZ/VERT] [RAW/FILT]
volatile bool hotrc_vert_preread = 0;
volatile bool hotrc_ch3_sw, hotrc_ch4_sw, hotrc_ch3_sw_event, hotrc_ch4_sw_event, hotrc_ch3_sw_last, hotrc_ch4_sw_last;
volatile int32_t hotrc_horz_pulse_us = 1500;
volatile int32_t hotrc_vert_pulse_us = 1500;
Timer hotrcPulseTimer;  // OK to not be volatile?
// Merging these into Hotrc class
bool hotrc_radio_detected = false;
bool hotrc_radio_detected_last = hotrc_radio_detected;
bool hotrc_suppress_next_ch3_event = true;  // When powered up, the hotrc will trigger a Ch3 and Ch4 event we should ignore
bool hotrc_suppress_next_ch4_event = true;  // When powered up, the hotrc will trigger a Ch3 and Ch4 event we should ignore
//  ---- tunable ----
double hotrc_pulse_period_us = 1000000.0 / 50;
double ctrl_ema_alpha[2] = { 0.01, 0.1 };  // [HOTRC/JOY] alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
int32_t ctrl_lims_adc[2][2][3] = { { { 3, 375, 4092 }, { 3, 375, 4092 } }, { { 9, 200, 4085 }, { 9, 200, 4085 } }, }; // [HOTRC/JOY] [HORZ/VERT], [MIN/DEADBAND/MAX] values as ADC counts
bool ctrl = HOTRC;  // Use HotRC controller to drive instead of joystick?
// Limits of what pulsewidth the hotrc receiver puts out
// For some funky reason I was unable to initialize these in an array format !?!?!
// int32_t hotrc_pulse_lims_us[2][2];  // = { { 1009, 2003 }, { 1009, 2003 } };  // [HORZ/VERT] [MIN/MAX]  // These are the limits of hotrc vert and horz high pulse
int32_t hotrc_pulse_vert_min_us = 990;  // 1009;
int32_t hotrc_pulse_vert_max_us = 1990;  // 2003;
int32_t hotrc_pulse_horz_min_us = 990;  // 1009;
int32_t hotrc_pulse_horz_max_us = 1990;  // 2003;

// Maybe merging these into Hotrc class
int32_t hotrc_pos_failsafe_min_adc = 140;  // The failsafe setting in the hotrc must be set to a trigger level equal to max amount of trim upward from trigger released.
int32_t hotrc_pos_failsafe_max_adc = 320;
int32_t hotrc_pos_failsafe_pad_adc = 10;
uint32_t hotrc_panic_timeout = 1000000;  // how long to receive flameout-range signal from hotrc vertical before panic stopping
Timer hotrcPanicTimer(hotrc_panic_timeout);

// steering related
int32_t steer_pulse_safe_us = 0;
int32_t steer_pulse_out_us;  // pid loop output to send to the actuator (steering)
//  ---- tunable ----
int32_t steer_pulse_right_min_us = 500;  // Smallest pulsewidth acceptable to jaguar
int32_t steer_pulse_right_us = 800;  // Steering pulsewidth corresponding to full-speed right steering (in us)
int32_t steer_pulse_stop_us = 1500;  // Steering pulsewidth corresponding to zero steering motor movement (in us)
int32_t steer_pulse_left_us = 2200;  // Steering pulsewidth corresponding to full-speed left steering (in us)
int32_t steer_pulse_left_max_us = 2500;  // Longest pulsewidth acceptable to jaguar
int32_t steer_safe_percent = 72;  // Sterring is slower at high speed. How strong is this effect 

// brake pressure related
int32_t pressure_adc;
// AnalogSensor pressure (&pressure_adc, "Pressure:", "adc ", 658, 2100);
//  ---- tunable ----
int32_t pressure_min_adc = 658; // Sensor reading when brake fully released.  230430 measured 658 adc (0.554V) = no brakes
int32_t pressure_sensor_max_adc = adcrange_adc; // Sensor reading max, limited by adc Vmax. (ADC count 0-4095). 230430 measured 2080 adc (1.89V) is as hard as chris can push (wimp)
int32_t pressure_max_adc = 2080; // Sensor measured maximum reading. (ADC count 0-4095). 230430 measured 2080 adc (1.89V) is as hard as [wimp] chris can push
double pressure_convert_psi_per_adc = 1000.0 * (3.3 - 0.554) / ( (pressure_sensor_max_adc - pressure_min_adc) * (4.5 - 0.554) );  // 1000 psi * (adc_max v - v_min v) / ((4095 adc - 658 adc) * (v-max v - v-min v)) = 0.2 psi/adc 
bool pressure_convert_invert = false;
// int32_t pressure_convert_polarity = SPID::FWD;
double pressure_ema_alpha = 0.1;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
double pressure_margin_psi = 2.5;  // Margin of error when comparing brake pressure adc values (psi)
double pressure_min_psi = 0.0;  // TUNED 230602 - Brake pressure when brakes are effectively off. Sensor min = 0.5V, scaled by 3.3/4.5V is 0.36V of 3.3V (ADC count 0-4095). 
double pressure_max_psi = convert_units (pressure_max_adc - pressure_min_adc, pressure_convert_psi_per_adc, pressure_convert_invert);  // TUNED 230602 - Highest possible pressure achievable by the actuator 
double pressure_hold_initial_psi = 150;  // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
double pressure_hold_increment_psi = 15;  // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
double pressure_panic_initial_psi = 250;  // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
double pressure_panic_increment_psi = 25;  // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
// max pedal bent 1154
double pressure_psi = (pressure_min_psi+pressure_max_psi)/2;
double pressure_filt_psi = pressure_psi;  // Stores new setpoint to give to the pid loop (brake)

// brake actuator motor related
double brake_pulse_out_us;  // sets the pulse on-time of the brake control signal. about 1500us is stop, higher is fwd, lower is rev
//  ---- tunable ----
Timer brakeIntervalTimer (500000);  // How much time between increasing brake force during auto-stop if car still moving?
int32_t brake_increment_interval_us = 500000;  // How often to apply increment during auto-stopping (in us)
int32_t brake_pulse_retract_min_us = 500;  // Smallest pulsewidth acceptable to jaguar
int32_t brake_pulse_retract_us = 650;  // Brake pulsewidth corresponding to full-speed retraction of brake actuator (in us)
int32_t brake_pulse_stop_us = 1500;  // Brake pulsewidth corresponding to center point where motor movement stops (in us)
int32_t brake_pulse_extend_us = 2350;  // Brake pulsewidth corresponding to full-speed extension of brake actuator (in us)
int32_t brake_pulse_extend_max_us = 2500;  // Longest pulsewidth acceptable to jaguar
int32_t brake_pulse_margin_us = 40; // If pid pulse calculation exceeds pulse limit, how far beyond the limit is considered saturated 

// brake actuator position related
double brake_pos_in;
double brake_pos_filt_in;
//  ---- tunable ----
double brake_pos_convert_in_per_adc = 3.3 * 10000.0 / (5.0 * adcrange_adc * 557);  // 3.3 v * 10k ohm / (5 v * 4095 adc * 557 ohm/in) = 0.0029 in/adc = 2.89 m-in/adc 
bool brake_pos_convert_invert = false;
int32_t brake_pos_convert_polarity = SPID::FWD;
double brake_pos_ema_alpha = 0.25;
double brake_pos_abs_min_retract_in = 0.335;  // TUNED 230602 - Retract value corresponding with the absolute minimum retract actuator is capable of. ("in"sandths of an inch)
double brake_pos_nom_lim_retract_in = 0.506;  // Retract limit during nominal operation. Brake motor is prevented from pushing past this. (in)
double brake_pos_zeropoint_in = 3.179;  // TUNED 230602 - Brake position value corresponding to the point where fluid PSI hits zero (in)
double brake_pos_park_in = 4.234;  // TUNED 230602 - Best position to park the actuator out of the way so we can use the pedal (in)
double brake_pos_nom_lim_extend_in = 4.624;  // TUNED 230602 - Extend limit during nominal operation. Brake motor is prevented from pushing past this. (in)
double brake_pos_abs_max_extend_in = 8.300;  // TUNED 230602 - Extend value corresponding with the absolute max extension actuator is capable of. (in)
double brake_pos_margin_in = .029;  //
// int32_t brake_pos_abs_min_retract_adc = 116;  // TUNED 230602 - Retract value corresponding with the absolute minimum retract actuator is capable of. (ADC count 0-4095)
// int32_t brake_pos_nom_lim_retract_adc = 175;  // Retract limit during nominal operation. Brake motor is prevented from pushing past this. (ADC count 0-4095)
// int32_t brake_pos_zeropoint_adc = 1100;  // TUNED 230602 - Brake position value corresponding to the point where fluid PSI hits zero (ADC count 0-4095)
// int32_t brake_pos_park_adc = 1465;  // TUNED 230602 - Best position to park the actuator out of the way so we can use the pedal (ADC count 0-4095)
// int32_t brake_pos_nom_lim_extend_adc = 1600;  // TUNED 230602 - Extend limit during nominal operation. Brake motor is prevented from pushing past this. (ADC count 0-4095)
// int32_t brake_pos_abs_max_extend_adc = 2872;  // TUNED 230602 - Extend value corresponding with the absolute max extension actuator is capable of. (ADC count 0-4095)
// int32_t brake_pos_margin_adc = 10;  //    

// throttle servo related
int32_t gas_pulse_out_us = 1501;  // pid loop output to send to the actuator (gas)
int32_t gas_pulse_govern_us = 1502;  // Governor must scale the pulse range proportionally. This is given a value in the loop
//  ---- tunable ----
Timer gasServoTimer (500000);  // We expect the servo to find any new position within this time
int32_t gas_governor_percent = 95;  // Software governor will only allow this percent of full-open throttle (percent 0-100)
int32_t gas_pulse_cw_min_us = 1000;  // Servo cw limit pulsewidth. Servo: full ccw = 2500us, center = 1500us , full cw = 500us
int32_t gas_pulse_redline_us = 1400;  // Gas pulsewidth corresponding to full open throttle with 180-degree servo (in us)
int32_t gas_pulse_idle_us = 1800;  // Gas pulsewidth corresponding to fully closed throttle with 180-degree servo (in us)
int32_t gas_pulse_ccw_max_us = 2000;  // Servo ccw limit pulsewidth. Hotrc controller ch1/2 min(lt/br) = 1000us, center = 1500us, max(rt/th) = 2000us (with scaling knob at max).  ch4 off = 1000us, on = 2000us
int32_t gas_pulse_park_slack_us = 30;  // Gas pulsewidth beyond gas_pulse_idle_us where to park the servo out of the way so we can drive manually (in us)

// tachometer related
Timer tachPulseTimer;  // OK to not be volatile?
volatile int32_t tach_delta_us = 0;
volatile int32_t tach_buf_delta_us = 0;
volatile uint32_t tach_time_us;
double tach_rpm = 50.0;  // Current engine speed, raw value converted to rpm (in rpm)
double tach_filt_rpm = 50.0;  // Current engine speed, filtered (in rpm)
double tach_govern_rpm;  // Software engine governor creates an artificially reduced maximum for the engine speed. This is given a value in calc_governor()
//  ---- tunable ----
double tach_convert_rpm_per_rpus = 60.0 * 1000000.0;  // 1 rot/us * 60 sec/min * 1000000 us/sec = 60000000 rot/min
bool tach_convert_invert = true;
int32_t tach_convert_polarity = SPID::FWD;      
double tach_ema_alpha = 0.015;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
double tach_idle_rpm = 700.0;  // Min value for engine hz, corresponding to low idle (in rpm)
double tach_max_rpm = 6000.0;  // Max possible engine rotation speed
double tach_redline_rpm = 4000.0;  // Max value for tach_rpm, pedal to the metal (in rpm)
double tach_margin_rpm = 15.0;  // Margin of error for checking engine rpm (in rpm)
double tach_stop_thresh_rpm = 0.01;  // Below which the engine is considered stopped - this is redundant,
int32_t tach_stop_timeout_us = 400000;  // Time after last magnet pulse when we can assume the engine is stopped (in us)
int32_t tach_delta_abs_min_us = 6500;  // 6500 us corresponds to about 10000 rpm, which isn't possible. Use to reject retriggers

// carspeed/speedo related
double speedo_govern_mph;  // Governor must scale the top vehicle speed proportionally. This is given a value in the loop
double speedo_mph = 1.01;  // Current car speed, raw as sensed (in mph)
double speedo_filt_mph = 1.02;  // Current car speed, filtered (in mph)
Timer speedoPulseTimer;  // OK to not be volatile?
volatile int32_t speedo_delta_us = 0;
volatile int32_t speedo_buf_delta_us = 0;
volatile uint32_t speedo_time_us;
//  ---- tunable ----
double speedo_convert_mph_per_rpus = 1000000.0 * 3600.0 * 20 * 3.14159 / (19.85 * 12 * 5280);  // 1 rot/us * 1000000 us/sec * 3600 sec/hr * 1/19.85 gearing * 20*pi in/rot * 1/12 ft/in * 1/5280 mi/ft = 179757 mi/hr (mph)
// Mule gearing:  Total -19.845x (lo) ( Converter: -3.5x to -0.96x Tranny -3.75x (lo), -1.821x (hi), Final drive -5.4x )
bool speedo_convert_invert = true;
int32_t speedo_convert_polarity = SPID::FWD;      
double speedo_ema_alpha = 0.015;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
double speedo_idle_mph = 4.50;  // What is our steady state speed at engine idle? Pulley rotation frequency (in milli-mph)
double speedo_redline_mph = 15.0;  // What is our steady state speed at redline? Pulley rotation frequency (in milli-mph)
double speedo_max_mph = 25.0;  // What is max speed car can ever go
double speedo_stop_thresh_mph = 0.01;  // Below which the car is considered stopped
uint32_t speedo_stop_timeout_us = 400000;  // Time after last magnet pulse when we can assume the car is stopped (in us)
int32_t speedo_delta_abs_min_us = 4500;  // 4500 us corresponds to about 40 mph, which isn't possible. Use to reject retriggers
            
// neopixel and heartbeat related
uint8_t neo_wheelcounter = 0;
uint8_t neo_brightness_max = 21;
uint32_t neo_timeout = 150000;
Timer neoTimer (neo_timeout);
bool neo_heartbeat = (neopixel_pin >= 0);
uint8_t neo_brightness = neo_brightness_max;  // brightness during fadeouts
enum neo_colors { N_RED, N_GRN, N_BLU };
uint8_t neo_heartcolor[3] = { 0xff, 0xff, 0xff };
Timer heartbeatTimer (1000000);
int32_t heartbeat_state = 0;
int32_t heartbeat_level = 0;
int32_t heartbeat_ekg[4] = { 170000, 150000, 530000, 1100000 };
int32_t heartbeat_pulse = 255;

// diag/monitoring variables
Timer loopTimer (1000000);  // how long the previous main loop took to run (in us)
uint32_t loop_period_us = 10000;
double loop_freq_hz = 1;  // run loop real time frequency (in Hz)
volatile int32_t loop_int_count = 0;  // counts interrupts per loop
int32_t loopno = 1;
uint32_t looptimes_us[20];
bool loop_dirty[20];
int32_t loopindex = 0;
bool diag_ign_error_enabled = true;

// pushbutton related
enum sw_presses { NONE, SHORT, LONG };  // used by encoder sw and button algorithms
bool button_last = 0;
bool button_it = 0;
bool btn_press_timer_active = false;
bool btn_press_suppress_click = false;
bool btn_press_action = NONE;

// external signal related
bool ignition = LOW;
bool ignition_last = ignition;
bool ignition_output_enabled = false;  // disallows configuration of ignition pin as an output until hotrc detected
bool syspower = HIGH;
bool syspower_last = syspower;
bool basicmodesw = LOW;
bool cruise_sw = LOW;

// simulator related
bool simulating_last = false;
Timer simTimer;
int32_t sim_edit_delta = 0;
int32_t sim_edit_delta_touch = 0;
int32_t sim_edit_delta_encoder = 0;
//  ---- tunable ----
bool simulating = false;
bool sim_joy = false;
bool sim_tach = true;
bool sim_speedo = true;
bool sim_brkpos = false;
bool sim_basicsw = true;
bool sim_cruisesw = true;
bool sim_pressure = true;
bool sim_syspower = true;
bool pot_pressure = true;  // Use the pot to simulate the brake pressure

SdFat sd;  // SD card filesystem
#define approot "cantroller2020"
#define logfile "log.txt"
#define error(msg) sd.errorHalt(F(msg))  // Error messages stored in flash.
SdFile root;  // Directory file.
SdFile file;  // Use for file creation in folders.

SPID brakeSPID (&pressure_filt_psi, brake_spid_initial_kp, brake_spid_initial_ki_hz, brake_spid_initial_kd_s, brake_spid_ctrl_dir, brake_pid_period_ms);
SPID gasSPID (&tach_filt_rpm, gas_spid_initial_kp, gas_spid_initial_ki_hz, gas_spid_initial_kd_s, gas_spid_ctrl_dir, gas_pid_period_ms);
SPID cruiseSPID (&speedo_filt_mph, cruise_spid_initial_kp, cruise_spid_initial_ki_hz, cruise_spid_initial_kd_s, cruise_spid_ctrl_dir, cruise_pid_period_ms);

// Servo library lets us set pwm outputs given an on-time pulse width in us
static Servo steer_servo;
static Servo brake_servo;
static Servo gas_servo;
static Adafruit_NeoPixel neostrip(1, neopixel_pin, NEO_GRB + NEO_GRB + NEO_KHZ800);

// Temperature sensor related
double temp_min = -67.0;  // Minimum reading of sensor is -25 C = -67 F
double temp_max = 257.0;  // Maximum reading of sensor is 125 C = 257 F
double temp_room = 77.0;  // "Room" temperature is 25 C = 77 F
enum temp_sensors { AMBIENT, ENGINE, WHEEL_FL, WHEEL_FR, WHEEL_RL, WHEEL_RR };
Timer tempTimer (2000000);
enum temp_status { IDLE, CONVERT, DELAY };
int32_t temp_status = IDLE;
double temps[6];
int32_t temp_detected_device_ct = 0;
int32_t temperature_precision = 12;  // 9-12 bit resolution
OneWire onewire (onewire_pin);
DallasTemperature tempsensebus (&onewire);
DeviceAddress temp_temp_addr;
int32_t temp_current_index = 0;
DeviceAddress temp_addrs[6];

// Interrupt service routines
//
// The tach and speed use a hall sensor being triggered by a passing magnet once per pulley turn. These ISRs call mycros()
// on every pulse to know the time since the previous pulse. I tested this on the bench up to about 0.750 mph which is as 
// fast as I can move the magnet with my hand, and it works. Update: Janky bench test appeared to work up to 11000 rpm.
void IRAM_ATTR tach_isr (void) {  // The tach and speedo isrs get the period of the vehicle pulley rotations.
    tach_time_us = tachPulseTimer.elapsed();
    if (tach_time_us > tach_delta_abs_min_us) {  // ignore spurious triggers or bounces
        tachPulseTimer.reset();
        tach_delta_us = tach_time_us;
    }
}
void IRAM_ATTR speedo_isr (void) {  //  Handler can get the most recent rotation time at speedo_delta_us
    speedo_time_us = speedoPulseTimer.elapsed();
    if (speedo_time_us > speedo_delta_abs_min_us) {  // ignore spurious triggers or bounces
        speedoPulseTimer.reset();
        speedo_delta_us = speedo_time_us;    
    }
}

// Attempt to use MCPWM input capture pulse width timer unit to get precise hotrc readings
// int32_t hotrc_ch3_pulse_us, hotrc_ch4_pulse_us;
// uint32_t mcpwm_unit0_capture, mcpwm_unit1_capture, mcpwm_unit2_capture;
// uint32_t mcpwm_unit0_capture_last, mcpwm_unit1_capture_last, mcpwm_unit2_capture_last;
// int32_t hotrc_ch3_preread;
// void IRAM_ATTR hotrc_isr (void) {
//     mcpwm_unit0_capture = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);
//     mcpwm_unit1_capture = mcpwm_capture_signal_get_value(MCPWM_UNIT_1, MCPWM_SELECT_CAP0);
//     hotrc_horz_pulse_us = (int32_t)(mcpwm_unit0_capture - mcpwm_unit0_capture_last);
//     hotrc_vert_pulse_us = (int32_t)(mcpwm_unit1_capture - mcpwm_unit1_capture_last);
//     mcpwm_unit0_capture_last = mcpwm_unit0_capture;
//     mcpwm_unit1_capture_last = mcpwm_unit1_capture;
// }
// // // Separate attempt to use timers to measure pulses
// // void IRAM_ATTR hotrc_ch1_isr (void) {
// //     mcpwm_unit0_capture = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);
// //     hotrc_horz_pulse_us = (int32_t)(mcpwm_unit0_capture - mcpwm_unit0_capture_last);
// //     mcpwm_unit0_capture_last = mcpwm_unit0_capture;
// // }
// // void IRAM_ATTR hotrc_ch2_isr (void) {
// //     mcpwm_unit1_capture = mcpwm_capture_signal_get_value(MCPWM_UNIT_1, MCPWM_SELECT_CAP0);
// //     hotrc_vert_pulse_us = (int32_t)(mcpwm_unit1_capture - mcpwm_unit1_capture_last);
// //     mcpwm_unit1_capture_last = mcpwm_unit1_capture;
// // }

void IRAM_ATTR hotrc_vert_isr (void) {  // On falling edge, records high pulse width to determine ch2 steering slider position
    if (hotrc_vert_preread) hotrcPulseTimer.reset();
    else hotrc_vert_pulse_us = hotrcPulseTimer.elapsed();
    hotrc_vert_preread = !(digitalRead (hotrc_ch2_vert_pin));  // Read pin after timer operations to maximize clocking accuracy
}
void IRAM_ATTR hotrc_horz_isr (void) {  // On falling edge, records high pulse width to determine ch2 steering slider position
    hotrc_horz_pulse_us = hotrcPulseTimer.elapsed();
}
void IRAM_ATTR hotrc_ch3_isr (void) {  // On falling edge, records high pulse width to determine ch3 button toggle state
    hotrc_ch3_sw = (hotrcPulseTimer.elapsed() <= 1500);  // Ch3 switch true if short pulse, otherwise false
    if (hotrc_ch3_sw != hotrc_ch3_sw_last) hotrc_ch3_sw_event = true;  // So a handler routine can be signaled. Handler must reset this to false
    hotrc_ch3_sw_last = hotrc_ch3_sw;
}
void IRAM_ATTR hotrc_ch4_isr (void) {  // On falling edge, records high pulse width to determine ch4 button toggle state
    hotrc_ch4_sw = (hotrcPulseTimer.elapsed() <= 1500);  // Ch4 switch true if short pulse, otherwise false
    if (hotrc_ch4_sw != hotrc_ch4_sw_last) hotrc_ch4_sw_event = true;  // So a handler routine can be signaled. Handler must reset this to false
    hotrc_ch4_sw_last = hotrc_ch4_sw;
}

// Utility functions
#define arraysize(x) ((int32_t)(sizeof(x) / sizeof((x)[0])))  // A macro function to determine the length of string arrays
#define floor(amt, lim) ((amt <= lim) ? lim : amt)
#define ceiling(amt, lim) ((amt >= lim) ? lim : amt)
// #define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
// #define min(a, b) ( (a <= b) ? a : b)
// #define max(a, b) ( (a >= b) ? a : b)
#undef max
inline double max (double a, double b) { return (a > b) ? a : b; }
inline int32_t max (int32_t a, int32_t b) { return (a > b) ? a : b; }
#undef min
inline double min (double a, double b) { return (a < b) ? a : b; }
inline int32_t min (int32_t a, int32_t b) { return (a < b) ? a : b; }
#undef constrain
inline double constrain (double amt, double low, double high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline int32_t constrain (int32_t amt, int32_t low, int32_t high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline uint32_t constrain (uint32_t amt, uint32_t low, uint32_t high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
#undef map
inline double map (double x, double in_min, double in_max, double out_min, double out_max) {
    if (in_max - in_min) return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
    return out_max;  // Instead of dividing by zero, return the highest valid result
}
inline int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    if (in_max - in_min) return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
    return out_max;  // Instead of dividing by zero, return the highest valid result
}
bool rounding = true;
double dround (double val, int32_t digits) { return (rounding) ? (std::round(val * std::pow (10, digits)) / std::pow (10, digits)) : val; }

bool car_stopped (void) { return (speedo_filt_mph < speedo_stop_thresh_mph); }
bool engine_stopped (void) { return (tach_filt_rpm < tach_stop_thresh_rpm); }

uint32_t colorwheel (uint8_t WheelPos) {
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85) return neostrip.Color (255 - WheelPos * 3, 0, WheelPos * 3);
    if (WheelPos < 170) {
        WheelPos -= 85;
        return neostrip.Color (0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return neostrip.Color (WheelPos * 3, 255 - WheelPos * 3, 0);
}
void calc_deadbands (void) {
    ctrl_db_adc[VERT][BOT] = (adcrange_adc-ctrl_lims_adc[ctrl][VERT][DB])/2;  // Lower threshold of vert joy deadband (ADC count 0-4095)
    ctrl_db_adc[VERT][TOP] = (adcrange_adc+ctrl_lims_adc[ctrl][VERT][DB])/2;  // Upper threshold of vert joy deadband (ADC count 0-4095)
    ctrl_db_adc[HORZ][BOT] = (adcrange_adc-ctrl_lims_adc[ctrl][HORZ][DB])/2;  // Lower threshold of horz joy deadband (ADC count 0-4095)
    ctrl_db_adc[HORZ][TOP] = (adcrange_adc+ctrl_lims_adc[ctrl][HORZ][DB])/2;  // Upper threshold of horz joy deadband (ADC count 0-4095)
}
void calc_governor (void) {
    tach_govern_rpm = map ((double)gas_governor_percent, 0.0, 100.0, 0.0, tach_redline_rpm);  // Create an artificially reduced maximum for the engine speed
    gas_pulse_govern_us = map ((int32_t)(gas_governor_percent*(tach_redline_rpm-tach_idle_rpm)/tach_redline_rpm), 0, 100, gas_pulse_idle_us, gas_pulse_redline_us);  // Governor must scale the pulse range proportionally
    speedo_govern_mph = map ((double)gas_governor_percent, 0.0, 100.0, 0.0, speedo_redline_mph);  // Governor must scale the top vehicle speed proportionally
}
// Exponential Moving Average filter : Smooth out noise on inputs. 0 < alpha < 1 where lower = smoother and higher = more responsive
// Pass in a fresh raw value, address of filtered value, and alpha factor, filtered value will get updated
void ema_filt (double raw, double* filt, double alpha) {
    // if (!raw) *filt = 0.0; else
    *filt = alpha * raw + (1 - alpha) * (*filt);
}
void ema_filt (int32_t raw, double* filt, double alpha) {
    ema_filt ((double)raw, filt, alpha);
}
void ema_filt (int32_t raw, int32_t* filt, double alpha) {
    *filt = (int32_t)(alpha * (double)raw + (1 - alpha) * (double)(*filt));
}

void sd_init() {
    if (!sd.begin (sdcard_cs_pin, SD_SCK_MHZ (50))) sd.initErrorHalt();  // Initialize at highest supported speed that is not over 50 mhz. Go lower if errors.
    if (!root.open ("/")) error("open root failed");
    if (!sd.exists (approot)) { 
        if (sd.mkdir (approot)) Serial.println (F("Created approot directory\n"));  // cout << F("Created approot directory\n");
        else error("Create approot failed");
    }
    // Change volume working directory to Folder1.
    // if (sd.chdir(approot)) {
    //    cout << F("\nList of files in appdir:\n");
    //    char *apppath = (char*)malloc((arraysize(appdir)+2)*sizeof(char));
    //        sd.ls(strcat("/",approot, LS_R);
    // }
    // else {
    //     error("Chdir approot failed\n");
    // }    
    // if (!file.open(logfile, O_WRONLY | O_CREAT)) {
    //     error("Open logfile failed\n");
    // }
    // file.close();
    // Serial.println(F("Filesystem init finished\n"));  // cout << F("Filesystem init finished\n");
    // for (byte a = 10; a >= 1; a--) {
    //     char fileName[12];
    //     sprintf(fileName, "%d.txt", a);
    //     file = sd.open(fileName, FILE_WRITE); //create file
    // }
}

// int* x is c++ style, int *x is c style
bool adj_val (int32_t* variable, int32_t modify, int32_t low_limit, int32_t high_limit) {  // sets an int reference to new val constrained to given range
    int32_t oldval = *variable;
    if (*variable + modify < low_limit) *variable = low_limit;
    else if (*variable + modify > high_limit) *variable = high_limit;
    else *variable += modify;
    return (*variable != oldval);
}
bool adj_val (double* variable, int32_t modify, double low_limit, double high_limit) {  // sets an int reference to new val constrained to given range
    double oldval = *variable;
    if (*variable + modify < low_limit) *variable = low_limit;
    else if (*variable + modify > high_limit) *variable = high_limit;
    else *variable += modify; 
    return (*variable != oldval);
}
bool adj_val (double* variable, double modify, double low_limit, double high_limit) {  // sets an int reference to new val constrained to given range
    double oldval = *variable;
    if (*variable + modify < low_limit) *variable = low_limit;
    else if (*variable + modify > high_limit) *variable = high_limit;
    else *variable += modify; 
    return (*variable != oldval);
}

void adj_bool (bool* val, int32_t delta) { if (delta != 0) *val = (delta > 0); }  // sets a bool reference to 1 on 1 delta or 0 on -1 delta 

// pin operations that first check if pin exists for the current board
void set_pin (int32_t pin, int32_t mode) { if (pin >= 0) pinMode (pin, mode); }
void write_pin (int32_t pin, int32_t val) {  if (pin >= 0) digitalWrite (pin, val); }
int32_t read_pin (int32_t pin) { return (pin >= 0) ? digitalRead (pin) : -1; }

void enable_pids (int32_t en_brake, int32_t en_gas, int32_t en_cruise) {  // pass in 0 (disable), 1 (enable), or -1 (leave it alone) for each pid loop
    if (en_brake != -1) brakeSPID.set_enable ((bool)en_brake);
    if (en_gas != -1) gasSPID.set_enable ((bool)en_gas);
    if (en_cruise != -1) cruiseSPID.set_enable ((bool)en_cruise);
}

void syspower_set (bool val) {
    if (digitalRead (syspower_pin) != val) {
        write_pin (syspower_pin, val);
        // delay (val * 500);
    }
}

// spid.h
#include "math.h"  // Just using for signbit() function. Note signbit returns true for negative signed argument
#include <cmath>
// Here is the brake PID math
// Our target is the desired amount of the measured value. The error is what we must add to our current value to get there
// We make 3 terms P I and D which add to become our output which goes to the actuator.
// P term scales proportionally to error, however it can never reach the setpoint 
// I term steadily grows the longer the error is the same sign,
// D term counteracts fast changes from P (and I), serving to prevent overshooting target. 
// Error = Setpoint - ProcessValue
// Output  =  P + I + D  =  (K * Error) + (K / Tau_I) + (Error - LastError)

// PID tuning
// https://www.youtube.com/watch?v=VVOi2dbtxC0&ab_channel=EEVblog

class SPID {  // Soren's home-made pid loop
  public:
    #define RANGED 0  // assign to outCenterMode if pid output just spans a range, rather than deviating from a centerpoint 
    #define CENTERED 1  // assign to outCenterMode if pid output deviates from a centerpoint 
    #define ERROR_TERM 0  // What the proportional term is proportional_to
    #define SENSED_INPUT 1  // What the proportional term is proportional_to
    static const int32_t FWD = 1;  // Actuator_direction influences sensed value in the same direction
    static const int32_t REV = -1;  // Actuator_direction influences sensed value in the opposite direction
    bool clamp_integral = false;  // Squashes integral windup if pushing the wrong direction 
  private:
    double kp_coeff = 0.01, ki_coeff = 0.01, kd_coeff = 0.01, kp, ki_hz, kd_s;
    int32_t actuator_direction = FWD;
    bool rounding = false, open_loop = false, enabled = false, never_enabled = true;
    uint32_t sample_period_ms;
    double output, out_min, out_max, target, target_last, error, error_last, output_last, p_term, i_term, d_term, saturation_margin;
    double saturation_margin_percent = 10.0, near_target_error_thresh_ratio = 0.005;  // Fraction of the input range where if the error has not exceeded in either dir since last zero crossing, it is considered zero (to prevent endless microadjustments) 
    double myinput, mytarget, myoutput, input_last = 0, near_target_error_thresh = 0, near_target_lock = 0;
    double in_center = 2047, out_center = 2047;
    double* p_input; double* p_in_min; double* p_in_max; double* p_output = &output; double* p_out_min = &out_min; double* p_out_max = &out_max;
    bool out_center_mode = CENTERED, in_center_mode = CENTERED, proportional_to = ERROR_TERM, differential_of = ERROR_TERM, saturated = false, output_hold = false;
    // Sensor* in_device; ServoPWM* out_device;
  public:
    SPID (double* arg_input, double arg_kp, double arg_ki_hz, double arg_kd_s, int32_t arg_dir, uint32_t arg_period_ms) {  // , bool arg_in_center_mode, bool arg_out_center_mode
        p_input = arg_input;
        sample_period_ms = arg_period_ms;
        set_tunings(arg_kp, arg_ki_hz, arg_kd_s);
        set_actuator_direction(arg_dir);
    }
    SPID (double* arg_input, double* arg_output, double arg_kp, double arg_ki_hz, double arg_kd_s, int32_t arg_dir, uint32_t arg_period_ms) {  // , bool arg_in_center_mode, bool arg_out_center_mode
        p_output = arg_output;
        *p_out_min = *arg_output;  // This has to point somewhere for now until it gets set
        *p_out_max = *arg_output;  // This has to point somewhere for now until it gets set
        SPID (arg_input, arg_kp, arg_ki_hz, arg_kd_s, arg_dir, arg_period_ms);
    }
    // SPID(Sensor* arg_in_device, ServoPWM* arg_out_device, double arg_kp, double arg_ki_hz, double arg_kd_s, int32_t direction, uint32_t arg_sample_period_ms) {
    //     in_device = arg_in_device;
    //     p_input = &in_device->val_native;
    //     p_in_min = &in_device->min_native;  // This has to point somewhere for now until it gets set
    //     p_in_max = &in_device->max_native;  // This has to point somewhere for now until it gets set
    //     out_device = arg_out_device;
    //     p_output = &out_device->val_native;
    //     p_out_min = &out_device->min_native;  // This has to point somewhere for now until it gets set
    //     p_out_max = &out_device->max_native;  // This has to point somewhere for now until it gets set
    //     set_tunings(arg_kp, arg_ki_hz, arg_kd_s);
    //     set_actuator_direction(out_device.get_direction());
    //     // set_center_modes(arg_in_center_mode, arg_out_center_mode);
    //     sample_period_ms = arg_sample_period_ms;
    // }
    bool constrain_value (double* value, double min, double max, double margin = 0.0) {  // Constrains referred value to given range, returning 1 if value was beyond range + margins
        bool sat = false;
        if (*value < min + margin) {
            sat = true;
            if (*value < min) *value = min;
        }
        else if (*value > max - margin) {
            sat = true;
            if (*value > max) *value = max;
        }
        return sat;
    }
    double round (double val, int32_t digits = 4) { return (rounding) ? (std::round (val * std::pow (10, digits)) / std::pow (10, digits)) : val; }
    // Clamp. Returns 0 if saturated and signs of (value - center) and error are the same (or in case of reverse actuator, different).
    double clamp_it (double arg_value, double arg_center) {  // If output is overshooting in the same direction as the error and the integral term is contributing, reset i_term
        bool val_sign = signbit (arg_value - arg_center) ^ (actuator_direction < 0);  // reverse value sign if actuator direction is reversed
        if ((signbit (error) == val_sign) && saturated) return 0;  // Clamp value if it is saturated and going in the direction that would further widen the error
        return arg_value;  // otherwise return arg_value unmolested
    }
    double compute (void) {
        if (!enabled) return *p_output;
        myinput = *p_input;
        mytarget = target;
        error = mytarget - myinput;
        myoutput = *p_output;  // =out_center (?),  = output_last (?)
        
        printf ("in:%4.0lf tg:%4.0lf er:%4.0lf i1:%4.0lf", myinput, mytarget, error, i_term);
        
        i_term += ki_coeff * error;  // Update integral
        printf (" i2:%4.0lf", i_term);

        if (clamp_integral) i_term = clamp_it(i_term, 0.0);
        printf (" i3:%4.0lf", i_term);

        if (proportional_to == ERROR_TERM) {  // If proportional_to Error (default). Note in this mode kp_coeff is the same sign as the actuator direction
            // if (out_center_mode) saturated = constrain_value (&i_term, out_center-*p_out_min, *p_out_max-out_center);  // Constrain i_term before adding p_term. Todo: Pass margin into constrain() for saturation determination
            // else
            constrain_value (&i_term, *p_out_min-*p_out_max, *p_out_max-*p_out_min);  // Constrain i_term before adding p_term. Todo: Pass margin into constrain() for saturation determination
            
            printf (" i4:%4.0lf s1:%d", i_term, saturated);

            p_term = kp_coeff * error;
            printf (" p:%4.0lf", p_term);

            myoutput += i_term + p_term;
            // myoutput = i_term + p_term;
            printf (" o1:%4.0lf", myoutput);

        }
        else if (proportional_to == SENSED_INPUT) {  // If proportional_to Input. Note in this mode kp_coeff is the opposite sign of the actuator direction
            p_term = kp_coeff * (myinput - input_last);  // p_term is based on input change (with opposite sign) rather than distance from target
            myoutput += i_term + p_term;
            // myoutput = i_term + p_term;
            saturated = constrain_value (&myoutput, *p_out_min, *p_out_max, saturation_margin);  // Constrain combined i_term + p_term.  Todo: Pass margin into constrain() for saturation determination
        }
        d_term = kd_coeff * (differential_of = ERROR_TERM) ? (myinput - input_last) : (error - error_last);  // Note d_term is opposite sign to input change
        printf (" d:%4.0lf", d_term);

        myoutput += d_term;  // Include d_term in output
        printf (" o2:%4.0lf", myoutput);

        saturated = constrain_value (&myoutput, *p_out_min, *p_out_max, saturation_margin);  // Constrain output to range. Todo: Pass margin into constrain() for saturation determination
        printf (" o3:%4.0lf s2:%d\n", myoutput, saturated);

        input_last = myinput;  // store previously computed input
        error_last = error;
        output_last = myoutput;
        target_last = mytarget;
        // printf (" in=%-+9.4lf lst=%-+9.4lf err=%-+9.4lf tgt=%-+9.4lf", myinput, input_last, error, mytarget);
        // printf (" pt=%-+9.4lf it=%-+9.4lf dt=%-+9.4lf out=%-+9.4lf\n", p_term, i_term, d_term, output);
        *p_output = round (myoutput);
        return *p_output;
    }
    // Comments related to compute() function
    // printf (" in=%-+9.4lf err=%-+9.4lf errlast=%-+9.4lf ntet=%-+9.4lf kp_co=%-+9.4lf ki_co=%-+9.4lf kd_co=%-+9.4lf kds=%-+9.4lf", input, error, error_last, near_target_error_thresh, kp_coeff, ki_coeff, kd_coeff, kd_s);
    //
    // Add a layer of history here, with an additional condition that output only holds if change in error has been small for X time period
    // Also add code to hold output (below in this function) if output_hold = true
    // if (signbit(error_last) != signbit(error) && abs(error_last - error) < near_target_error_thresh) {
    //     near_target_lock = target;
    //     output_hold = true;
    // }
    // else if (abs (near_target_lock - myinput) > near_target_error_thresh) output_hold = false;
    //
    //
    // p_term = round (p_term);
    // i_term = round (i_term);
    // d_term = round (d_term);
    //
    // Add handling for CENTERED controller!!  (our brake)
    // if (out_center_mode == CENTERED) *p_output += out_center;
    //
    // printf (" ntl2=%-+9.4lf sat=%1d outh=%1d pterm=%-+9.4lf iterm=%-+9.4lf dterm=%-+9.4lf out=%-+9.4lf\n", near_target_lock, saturated, output_hold, p_term, i_term, d_term, output);
    //
    // printf ("uc output: %7.2lf, min: %7.2lf, max:%7.2lf, ", *p_output, *p_out_min, *p_out_max);
    // printf ("c output: %7.2lf, min: %7.2lf, max:%7.2lf\n", *p_output, *p_out_min, *p_out_max);
    void set_input (double arg_input) {
        *p_input = round (arg_input);
        constrain_value (p_input, *p_in_min, *p_in_max);
    }
    void set_target (double arg_target) {
        // printf ("SPID::set_target():  received arg_target=%-+9.4lf, *p_in_min=%-+9.4lf, *p_in_max=%-+9.4lf\n", arg_target, *p_in_min, *p_in_max);
        target = round (arg_target);
        constrain_value (&target, *p_in_min, *p_in_max);
        error = target - *p_input;
    }
    void set_tunings (double arg_kp, double arg_ki_hz, double arg_kd_s) {  // Set tuning parameters to negative values if higher actuator values causes lower sensor values and vice versa 
        if (arg_kp < 0 || arg_ki_hz < 0 || arg_kd_s < 0) {  // || ( arg_kp <= 0 && arg_ki_hz <= 0 && arg_kd_s <= 0 ) ) ) {
            printf ("Warning: SPID::set_tunings() ignored request to set tuning parameters to negative values.\n");
            return;
        }
        double sample_period_s = ((double)sample_period_ms)/1000;
        kp = arg_kp;  ki_hz = arg_ki_hz;  kd_s = arg_kd_s;
        kp_coeff = arg_kp * ((proportional_to == ERROR_TERM) ? 1 : -1);
        ki_coeff = arg_ki_hz * sample_period_s;
        if (sample_period_s) kd_coeff = arg_kd_s * ((differential_of == ERROR_TERM) ? -1 : 1) / sample_period_s;
        else {
            printf ("set_tunings refused to divide by zero to set kd_coeff\n");
            kd_coeff = 0;
        }
    }
    void set_actuator_direction (int32_t direction) {
        if (direction != actuator_direction) {
            kp_coeff = (0 - kp_coeff);
            ki_coeff = (0 - ki_coeff);
            kd_coeff = (0 - kd_coeff);
            actuator_direction = direction;
        }
    }
   void set_input_limits (double* p_arg_min, double* p_arg_max) {
        if (*p_arg_min >= *p_arg_max) {
            printf ("SPID ignored attempt to set input min %lf > max %lf.\n", *p_arg_min, *p_arg_max);
            return;
        }
        p_in_min = p_arg_min;  p_in_max = p_arg_max;
        constrain_value (p_input, *p_in_min, *p_in_max);
        near_target_error_thresh = (*p_in_max - *p_in_min) * near_target_error_thresh_ratio;
    }
    void set_near_target_thresh (double arg_near_target_thresh_percent) { near_target_error_thresh_ratio = arg_near_target_thresh_percent; }
    void set_output_limits (double* p_arg_min, double* p_arg_max) {
        if (*p_arg_min >= *p_arg_max) {
            printf ("SPID ignored request to set output min %lf > max %lf.\n", *p_arg_min, *p_arg_max);
            return;
        }
        p_out_min = p_arg_min;
        p_out_max = p_arg_max;
        if (!out_center) out_center = (*p_out_min + *p_out_max)/2;  // This is pointless, I believe
        if (saturation_margin_percent) saturation_margin = 100 * (*p_out_max - *p_out_min) / saturation_margin_percent;  // Default saturation margin is 5% of the output range 
        else printf ("SPID refused to divide by zero to set saturation margin\n");
        // printf ("Limit: min:%4lf max:%4lf cent:%lf\n", *p_out_min, *p_out_max, out_center);
        constrain_value (p_output, *p_out_min, *p_out_max);
    }
    void set_output_limits (double arg_min, double arg_max) {
        if (arg_min >= arg_max) {
            printf ("SPID ignored attempt to set output min %lf > max %lf.\n", arg_min, arg_max);
            return;
        }
        *p_out_min = arg_min;
        *p_out_max = arg_max;
        set_output_limits (p_out_min, p_out_max);     
    }
    void set_input_center (void) { in_center_mode = RANGED; }  // Call w/o arguments to set input to RANGED mode
    void set_input_center (double arg_in_center) {  // Sets input to CENTERED (centerpoint) mode and takes value of center point. 
        if (arg_in_center < *p_in_min || arg_in_center > *p_in_max) {
            printf ("SPID ignored request to set input centerpoint outside range.\n");
            return;
        }
        in_center_mode = CENTERED;
        in_center = arg_in_center;
    }
    void set_output_center (void) {  // Call w/o arguments to set output to RANGED mode
        // out_center = (*p_out_max - *p_out_min)/2;
        out_center = *p_out_min;
        out_center_mode = RANGED;
    }
    void set_output_center (double arg_out_center) {  // Sets output to CENTERED (centerpoint) mode and takes value of center point. 
        // printf ("Set_Cent: min (%ld): %4lf max (%ld): %4lf cent:%lf\n", p_out_min, *p_out_min, p_out_max, *p_out_max, arg_out_center);
        if (arg_out_center < *p_out_min || arg_out_center > *p_out_max) {
            printf ("SPID ignored request to set output centerpoint outside range.\n");
            return;
        }
        out_center = arg_out_center;
        out_center_mode = CENTERED;
    }
    void set_pd_modes (bool arg_prop_to, bool arg_diff_of) {  // For each specify ERROR_TERM or SENSED_INPUT
        if (proportional_to != arg_prop_to) {
            kp_coeff = (0 - kp_coeff);
            proportional_to = arg_prop_to;
        }
        if (differential_of != arg_diff_of) {
            kd_coeff = (0 - kd_coeff);
            differential_of = arg_diff_of;
        }
    }
    void set_sample_period (uint32_t arg_period_ms) {
        if (sample_period_ms) {
            double ratio = (double)arg_period_ms / (double)sample_period_ms;
            ki_coeff *= ratio;
            kd_coeff /= ratio;
            sample_period_ms = arg_period_ms;
        }
        else printf ("SPID refusing to divide by zero setting sample period\n");
    }
    void set_open_loop (bool arg_open_loop) { open_loop = arg_open_loop; }
    void set_enable (bool arg_enable) {
        if (arg_enable && never_enabled) {
            if (out_center_mode) *p_output = out_center;
            else *p_output = (*p_out_max - *p_out_min)/2;
            output_last = *p_output;
            never_enabled = false;
        }
        enabled = arg_enable;
    }
    void set_saturation_margin_percent (double arg_margin_percent) { 
        if (arg_margin_percent) {
            saturation_margin_percent = arg_margin_percent;
            saturation_margin = 100 * (*p_out_max - *p_out_min) / saturation_margin_percent;
        }
        else printf ("SPID: You can't set zero saturation margin\n");
    }
    double get_saturation_margin_percent (void) { return saturation_margin_percent; }
    double get_saturation_margin (void) { return saturation_margin; }
    double get_kp_coeff (void) { return kp_coeff; }
    double get_ki_coeff (void) { return ki_coeff; }
    double get_kd_coeff (void) { return kd_coeff; }
    double get_kp (void) { return kp; }
    double get_ki_hz (void) { return ki_hz; }
    double get_kd_s (void) { return kd_s; }
    bool get_out_center_mode (void) { return out_center_mode; }
    bool get_open_loop (void) { return open_loop; }
    bool get_in_center_mode (void) { return in_center_mode; }
    bool get_p_mode (void) { return proportional_to; }
    bool get_d_mode (void) { return differential_of; }
    bool get_actuator_direction (void) { return actuator_direction; }
    double get_near_target_error_thresh_ratio (void) { return near_target_error_thresh_ratio; }
    double get_near_target_error_thresh (void) { return near_target_error_thresh; }
    bool get_out_center (void) { return out_center; }
    bool get_in_center (void) { return in_center; }
    bool get_saturated (void) { return saturated; }
    double get_p_term (void) { return p_term; }  // ((p_term >= 0.001) ? p_term : 0); }
    double get_i_term (void) { return i_term; }  // ((i_term >= 0.001) ? i_term : 0); }
    double get_d_term (void) { return d_term; }  // ((d_term >= 0.001) ? d_term : 0); }
    double get_error (void) { return error; }
    double get_target (void) { return target; }
    double get_output (void) { return *p_output; }
    double get_input (void) { return *p_input; }
    bool get_enabled (void) { return enabled; }
    int32_t get_sample_period (void) { return sample_period_ms; }
};

// Instantiate PID loops
//
// Steering:  Motor direction and velocity are controlled with PWM, proportional to joystick horizontal direction and magnitude
//   Setpoint Value: Proportional to Joystick Horz ADC value.  0V = Full Left, 2.5V = Stop, 5V = Full Right
//   Measured Value: We have no feedback, other than the joystick current horizontal position
//   Actuator Output Value: PWM square wave sent to Jaguar, w/ high pulse width of: ~2.2ms = Full Left, 1.5ms = Stop, ~800us = Full Right
//   Limits: Reed switch limit signals for left and right may be handled by us, or by the jaguar controller
//   Setpoint scaling: Kp/Ki/Kd values should decrease appropriately as a function of vehicle speed (safety) 
//
//   Notes: The steering has no feedback sensing, other than two digital limit switches at the ends of travel.  
//   So just consider the error to be the difference between the joystick position and the last output value.
//
// Brakes:  Motor direction & velocity are controlled with PWM until brake pressure matches pressure setpoint
//   Setpoint Value: * Default: Pressure setpoint proportional to Joystick Vert distance from center when below center.
//       * In Hold Mode: Brake adjusts automatically to keep car stopped, as long as joystick below center
//       * In Cruise Mode: Brake is kept released 
//   Measured Value: Analog voltage from brake fluid pressure sensor. 0-3.3V proportional to 0-1000psi
//   Actuator Output Value: PWM signal to Brake Jaguar unit.
//       0% duty = Full speed extend (less brake), 50% = Stop, 100% = Full speed Retract (more brake)
//   Position: Analog 0-3.3V proportional to the travel length of the actuator (not used as feedback)
//
// Gas:  Servo angle is adjusted with PWM until engine rpm matches rpm target setpoint
//   Setpoint Value: * Default: RPM Setpoint proportional to Joystick Vert distance from center when above center.
//       * In Cruise Mode: Upward or downward joy vert motions modify vehicle speed setpoint
//                          Gas pid setppoints are output from cruise pid
//   Measured Value: * Default: Engine speed determined from tach pulses
//   Actuator Output Value: PWM signal to throttle servo
//       0% duty = Fully close throttle.  This will idle.  100% duty = Fully open throttle.
//
// Cruise:
//   Setpoint Value: * Default: Set to the current vehicle speed when mode is entered.
//       * In Cruise Mode: Upward or downward joy vert motions suspend loop and accelerate or decelerate,
//                         upon return to center loop resumes with new speed target set to vehicle speed when released.
//   Measured Value: * Vehicle speed determined from pulley sensor pulses
//   Actuator Output Value: Cruise PID output values become setpoint values for the Gas PID above
//       0% duty = Car stopped.  100% duty = Car max speed.
//
// One way to tune a PID loop:
// 1) Set Kp = 0, Ki = 0, and Kd = 0
// 2) Increase Kp until output starts to oscillate
// 3) Increase Ki until oscillation stops
// 4) If improved response time is needed, increase Kd slightly and go back to step 2
//
// Ziegler-Nichols method:
// 1) Set Kp = 0, Ki = 0, and Kd = 0
// 2) Increase Kp until output starts to oscillate.
// 3) Record Kc = critical value of Kp, and Pc = period of oscillations
// 4) Set Kp=0.6*Kc and Ki=1.2*Kc/Pc and Kd=Kc*Pc/13.33  (Or for P only:  Kp=Kc/2)  (Or for PI:  Kp=0.45*Kc and Ki=0.54*Kc/Pc)

// If output constrain prevents us from exceeding the limits of the actuator. But we need to know two things as we constrain, to preevent "Windup", a condition where the I term exploded due to an extended error when maybe the motor couldn't meet the target. Once back to normal, don't want I term wound up.
// So improve this Clamp to check for A. Is it saturating? I.e. was constrain necessary or not? and B. Is the sign (polarity) of the output the same as that of the error?  If both are true, we have Integrator Windup.  So when we detect this, we can temporarily "Clamp" the I-term to 0 until we are "recovered".
// Recovered can be either of these conditions:  1. We are no longer saturated (constrain is doing nothing)m or, 2. The error changes sign. (then reconnect I term.)
// When determining saturation or not, add a margin.

std::vector<string> loop_names(20);

void loop_savetime (uint32_t timesarray[], int32_t &index, vector<string> &names, bool dirty[], string loopname) {  // (int32_t timesarray[], int32_t index) {
    if (dirty[index]) {
        names[index] = loopname;  // names[index], name);
        dirty[index] = false;
    }
    timesarray[index] = esp_timer_get_time();
    index++;
}

Hotrc hotrc (&ctrl_pos_adc[VERT][FILT], hotrc_pos_failsafe_min_adc, hotrc_pos_failsafe_max_adc, hotrc_pos_failsafe_pad_adc);
    
Display screen(tft_cs_pin, tft_dc_pin);
    
// Encoder encoder(encoder_a_pin, encoder_b_pin, encoder_sw_pin);
MAKE_ENCODER(encoder, encoder_a_pin, encoder_b_pin, encoder_sw_pin);

void setup() {
    set_pin (heartbeat_led_pin, OUTPUT);
    set_pin (encoder_a_pin, INPUT_PULLUP);
    set_pin (encoder_b_pin, INPUT_PULLUP);
    set_pin (encoder_sw_pin, INPUT_PULLUP);  // The esp32 pullup is too weak. Use resistor
    set_pin (brake_pwm_pin, OUTPUT);
    set_pin (steer_pwm_pin, OUTPUT);
    set_pin (tft_dc_pin, OUTPUT);
    set_pin (gas_pwm_pin, OUTPUT);
    // set_pin (ignition_pin, OUTPUT);  // drives relay to turn on/off car. Active high
    set_pin (basicmodesw_pin, INPUT_PULLUP);
    set_pin (tach_pulse_pin, INPUT_PULLUP);
    set_pin (speedo_pulse_pin, INPUT_PULLUP);
    set_pin (joy_horz_pin, INPUT);
    set_pin (joy_vert_pin, INPUT);
    set_pin (pressure_pin, INPUT);
    set_pin (brake_pos_pin, INPUT);
    set_pin (battery_pin, INPUT);
    set_pin (hotrc_ch1_horz_pin, INPUT);
    set_pin (hotrc_ch2_vert_pin, INPUT);
    set_pin (neopixel_pin, OUTPUT);
    set_pin (sdcard_cs_pin, OUTPUT);
    set_pin (tft_cs_pin, OUTPUT);
    set_pin (pot_wipe_pin, INPUT);
    set_pin (button_pin, INPUT_PULLUP);    
    set_pin (starter_pin, INPUT_PULLDOWN);
    set_pin (tp_irq_pin, INPUT_PULLUP);
    set_pin (led_rx_pin, OUTPUT);
    set_pin (encoder_pwr_pin, OUTPUT);
    set_pin (tft_rst_pin, OUTPUT);
    set_pin (hotrc_ch3_ign_pin, INPUT);
    set_pin (hotrc_ch4_cruise_pin, INPUT);
    set_pin (joy_ign_btn_pin, INPUT_PULLDOWN);
    set_pin (joy_cruise_btn_pin, INPUT_PULLUP);
        
    // write_pin (ignition_pin, ignition);
    write_pin (tft_cs_pin, HIGH);   // Prevent bus contention
    write_pin (sdcard_cs_pin, HIGH);   // Prevent bus contention
    write_pin (tft_dc_pin, LOW);
    write_pin (led_rx_pin, LOW);  // Light up
    write_pin (encoder_pwr_pin, HIGH);
    write_pin (tft_rst_pin, HIGH);
    
    // This bit is here as a way of autdetecting soren's breadboard, since his LCD is wired upside-down.
    // Soren put a strong external pulldown on the pin, so it'll read low for autodetection. 
    set_pin (syspower_pin, INPUT);  // Using weak ESP pullup to ensure this doesn't turn on syspower on the car
    flip_the_screen = !(read_pin (syspower_pin));  // Will cause the LCD to be upside down
    // Then set the put as an output as normal.
    set_pin (syspower_pin, OUTPUT);
    write_pin (syspower_pin, syspower);

    analogReadResolution (adcbits);  // Set Arduino Due to 12-bit resolution (default is same as Mega=10bit)
    Serial.begin (115200);  // Open serial port
    // printf("Serial port open\n");  // This works on Due but not ESP32
    
    for (int32_t x=0; x<arraysize(loop_dirty); x++) loop_dirty[x] = true;
    
    if (display_enabled) {
        delay (500); // This is needed to allow the screen board enough time after a cold boot before we start trying to talk to it.
        config.begin("FlyByWire", false);
        dataset_page = config.getUInt("dpage", PG_RUN);
        dataset_page_last = config.getUInt("dpage", PG_TEMP);
        screen.init();
    }
    neostrip.begin();  // start datastream
    neostrip.show();  // Turn off the pixel
    neostrip.setBrightness (neo_brightness_max);  // It truly is incredibly bright
    // 230417 removing sdcard init b/c boot is hanging here unless I insert this one precious SD card
    // Serial.print(F("Initializing filesystem...  "));  // SD card is pretty straightforward, a single call. 
    // if (! sd.begin(usd_cs_pin, SD_SCK_MHZ(25))) {   // ESP32 requires 25 mhz limit
    //     Serial.println(F("SD begin() failed"));
    //     for(;;); // Fatal error, do not continue
    // }
    // sd_init();
    // Serial.println(F("Filesystem started"));

    SETUP_ENCODER(encoder);

    // Configure MCPWM GPIOs
    //
    // The four channel inputs can be connected to any available GPIO pins on the ESP32, but the specific MCPWM unit and
    // output channels are used to configure the MCPWM GPIOs and input capture. Here's the mapping used in the code:
    // INPUT_PIN_1 is connected to MCPWM0A (MCPWM unit 0, output channel A).
    // INPUT_PIN_2 is connected to MCPWM1A (MCPWM unit 0, output channel B).
    // INPUT_PIN_3 is connected to MCPWM2A (MCPWM unit 0, output channel C).
    // INPUT_PIN_4 is connected to MCPWM3A (MCPWM unit 0, output channel D).
    // You can change these mappings according to your specific pin assignments. Ensure that the MCPWM GPIOs you select are compatible with input capture functionality.
    // Attempt to use MCPWM input capture pulse width timer unit to get precise hotrc readings
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/mcpwm.html#capture
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/kconfig.html#mcpwm-configuration
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/kconfig.html#mcpwm-configuration
    // // Configure MCPWM GPIOs
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, hotrc_ch1_horz_pin);
    // mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, hotrc_ch2_vert_pin);
    // // Configure MCPWM units 0 and 1
    // mcpwm_config_t pwm_config;
    // pwm_config.frequency = 0;  // Set frequency to 0 for input mode
    // pwm_config.cmpr_a = 0;  // Set duty cycle to 0 for input mode
    // pwm_config.counter_mode = MCPWM_UP_COUNTER;
    // pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    // mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    // mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);
    // // mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);
    // // mcpwm_capture_enable(MCPWM_UNIT_1, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);
    // // mcpwm_capture_set_cb(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, hotrc_ch1_isr, NULL);
    // // mcpwm_capture_set_cb(MCPWM_UNIT_1, MCPWM_SELECT_CAP0, hotrc_ch2_isr, NULL);
    // Ch3 and Ch4 interrupts work with slower timers
    // attachInterrupt (digitalPinToInterrupt(hotrc_ch3_ign_pin), hotrc_ch3_isr, CHANGE);
    // attachInterrupt (digitalPinToInterrupt(hotrc_ch4_cruise_pin), hotrc_ch4_isr, FALLING);

    // Set up our interrupts
    Serial.print (F("Pulse timers and interrupts ... "));
    attachInterrupt (digitalPinToInterrupt(tach_pulse_pin), tach_isr, RISING);
    attachInterrupt (digitalPinToInterrupt(speedo_pulse_pin), speedo_isr, RISING);
    if (ctrl == HOTRC) {
        attachInterrupt (digitalPinToInterrupt (hotrc_ch2_vert_pin), hotrc_vert_isr, CHANGE);
        attachInterrupt (digitalPinToInterrupt (hotrc_ch1_horz_pin), hotrc_horz_isr, FALLING);
        attachInterrupt (digitalPinToInterrupt (hotrc_ch3_ign_pin), hotrc_ch3_isr, FALLING);
        attachInterrupt (digitalPinToInterrupt (hotrc_ch4_cruise_pin), hotrc_ch4_isr, FALLING);
    }
    Serial.println (F("set up and enabled\n"));

    calc_deadbands();
    calc_governor();

    // Set up the soren pid loops
    brakeSPID.set_input_limits (&pressure_min_psi, &pressure_max_psi);  // Make sure pressure target is in range
    brakeSPID.set_output_limits ((double)brake_pulse_retract_us, (double)brake_pulse_extend_us);
    
    printf ("CANT: min:%4ld max:%4ld cent:%ld\n", brake_pulse_retract_us, brake_pulse_extend_us, brake_pulse_stop_us);

    brakeSPID.set_output_center ((double)brake_pulse_stop_us);  // Sets actuator centerpoint and puts pid loop in output centerpoint mode. Becasue actuator value is defined as a deviation from a centerpoint
    
    gasSPID.set_input_limits (&tach_idle_rpm, &tach_govern_rpm);
    gasSPID.set_output_limits ((double)gas_pulse_govern_us, (double)gas_pulse_idle_us);
    cruiseSPID.set_input_limits (&speedo_idle_mph, &speedo_govern_mph);
    cruiseSPID.set_output_limits (tach_idle_rpm, tach_govern_rpm);
    
    gasSPID.set_open_loop(1);  // Added temporarily to debug brake pid

    steer_servo.attach (steer_pwm_pin);
    brake_servo.attach (brake_pwm_pin);
    gas_servo.attach (gas_pwm_pin);

    neo_heartbeat = (neopixel_pin >= 0);
    neostrip.begin();
    neostrip.show(); // Initialize all pixels to 'off'
    neostrip.setBrightness (neo_brightness_max);

    tempsensebus.setWaitForConversion (true);  // Whether to block during conversion process
    tempsensebus.setCheckForConversion (true);  // Do not listen to device for conversion result, instead we will wait the worst-case period
    tempsensebus.begin();
    temp_detected_device_ct = tempsensebus.getDeviceCount();
    printf ("Temp sensors: Detected %d devices.\nParasitic power is: ", temp_detected_device_ct);  // , DEC);
    printf ((tempsensebus.isParasitePowerMode()) ? "On\n" : "Off\n");
    // for (int32_t x = 0; x < arraysize(temp_addrs); x++) {
    for (int32_t x = 0; x < temp_detected_device_ct; x++) {
        if (tempsensebus.getAddress (temp_temp_addr, x)) printf ("Found sensor device: index %d, addr %d\n", x, temp_temp_addr);  // temp_addrs[x]
        else printf ("Found ghost device : index %d, addr unknown\n", x);  // printAddress (temp_addrs[x]);
        tempsensebus.setResolution (temp_temp_addr, temperature_precision);  // temp_addrs[x]
    }
    
    // xTaskCreatePinnedToCore ( codeForTask1, "Task_1", 1000, NULL, 1, &Task1, 0);
    // if (ctrl == HOTRC) {  // Look for evidence of a normal (not failsafe) hotrc signal. If it's not yet powered on, we will ignore its spurious poweron ignition event
    //     int32_t temp = hotrc_vert_pulse_us;
    //     hotrc_radio_detected = ((ctrl_lims_adc[HOTRC][VERT][MIN] <= temp && temp < hotrc_pos_failsafe_min_us) || (hotrc_pos_failsafe_max_us < temp && temp <= ctrl_lims_adc[HOTRC][VERT][MAX]));
    //     for (int32_t x = 0; x < 4; x++) {
    //         delay (20);
    //         if (!((ctrl_lims_adc[HOTRC][VERT][MIN] < temp && temp < hotrc_pos_failsafe_min_us) || (hotrc_pos_failsafe_max_us < temp && temp < ctrl_lims_adc[HOTRC][VERT][MAX]))
    //             || (hotrcPulseTimer.elapsed() > (int32_t)(hotrc_pulse_period_us*2.5))) hotrc_radio_detected = false;
    //     }
    //     printf ("HotRC radio signal: %setected\n", (!hotrc_radio_detected) ? "Not d" : "D");
    // }
    
    // pressure.set_convert ( 1000.0 * 3.3 / (adcrange_adc * (4.5 - 0.55)), 0.0, false);
    // pressure.set_names ("pressure", "adc ", "psi ");
    // pressure.set_limits (129.0, 452.0);
    
    int32_t watchdog_time_ms = Watchdog.enable(2500);  // Start 2.5 sec watchdog
    printf ("Watchdog enabled. Timer set to %ld ms.\n", watchdog_time_ms);
    hotrcPanicTimer.reset();
    loopTimer.reset();  // start timer to measure the first loop
    Serial.println (F("Setup done"));
}

// Main loop.  Each time through we do these eight steps:
//
// 0) Beginning-of-the-loop nonsense
// 1) Gather new telemetry and filter the signals
// 2) Check if our current runmode has been overridden by certain specific conditions
// 3) Read joystick horizontal and determine new steering setpoint
// 4) Do actions based on which runmode we are in (including gas & brake setpoint), and possibly change runmode 
// 5) Step the PID loops and update the actuation outputs
// 6) Service the user interface
// 7) Log to SD card
// 8) Do the control loop bookkeeping at the end of each loop
//   
void loop() {
    loopindex = 0;  // reset at top of loop
    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "top");
    // cout << "(top)) spd:" << speedo_filt_mph << " tach:" << tach_filt_rpm;
    
    // Update inputs.  Fresh sensor data, and filtering.
    //

    // ESP32 "boot" button.  This is not working (?)
    button_last = button_it;
    if (!read_pin (button_pin)) {
        if (!button_it) {  // If press just occurred
            dispResetButtonTimer.reset();  // Looks like someone just pushed the esp32 "boot" button
            btn_press_timer_active = true;  // flag to indicate timing for a possible long press
        }
        else if (btn_press_timer_active && dispResetButtonTimer.expired()) {
            btn_press_action = LONG;  // Set flag to handle the long press event. Note, routine handling press should clear this
            btn_press_timer_active = false;  // Clear timer active flag
            btn_press_suppress_click = true;  // Prevents the switch release after a long press from causing a short press
        }
        button_it = true;  // Store press is in effect
    }
    else {  // if button is not being pressed
        btn_press_action = NONE;  // Any button action handling needs to happen in the same loop or is lost
        if (button_it && !btn_press_suppress_click) btn_press_action = SHORT;  // if the button was just released, a short press occurred, which must be handled
        btn_press_timer_active = false;  // Clear timer active flag
        button_it = false;  // Store press is not in effect
        btn_press_suppress_click = false;  // End click suppression
    }
    // if (btn_press_action != NONE) 
    // printf ("it:%d ac:%ld lst:%d ta:%d sc:%d el:%ld\n", button_it, btn_press_action, button_last, btn_press_timer_active, btn_press_suppress_click, dispResetButtonTimer.elapsed());
    
    // External digital signals - takes 11 us to read
    if (!simulating || !sim_basicsw) basicmodesw = !digitalRead (basicmodesw_pin);   // 1-value because electrical signal is active low
    if (ctrl == JOY && (!simulating || !sim_cruisesw)) cruise_sw = digitalRead (joy_cruise_btn_pin);

    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "pre");

    // Temperature sensors
    if (take_temperatures && tempTimer.expired()) {
        if (temp_status == IDLE) {
            if (++temp_current_index >= 2) temp_current_index -= 2;  // replace 1 with arraysize(temps)
            tempsensebus.setWaitForConversion (false);  // makes it async
            tempsensebus.requestTemperatures();
            tempsensebus.setWaitForConversion (true);
            tempTimer.set(750000 / (1 << (12 - temperature_precision)));  // Give some time before reading temp
            temp_status = CONVERT;
        }
        else if (temp_status == CONVERT) {
            temps[temp_current_index] = tempsensebus.getTempFByIndex(temp_current_index);
            tempTimer.set(1500000);
            temp_status = DELAY;
        }
        else if (temp_status == DELAY) {
            // printf ("temps[%ld] = %lf F\n", temp_current_index, temps[temp_current_index]);
            tempTimer.set(60000);
            temp_status = IDLE;
        }
    }
    // double temps[temp_detected_device_ct];
    // uint32_t timecheck;
    // if (take_temperatures && tempTimer.expired()) {
    //     cout << endl << "loop# " << loopno << " stat0:" << temp_status;
    //     if (temp_status == IDLE) {
    //         wait_one_loop = true;
    //         if (++temp_current_index >= 2) temp_current_index -= 2;  // replace 1 with arraysize(temps)
    //         timecheck = micros();
    //         // tempsensebus.requestTemperaturesByIndex (temp_current_index);
    //         tempsensebus.setWaitForConversion (false);  // Do not block during conversion process
    //         tempsensebus.requestTemperatures();
    //         tempsensebus.setWaitForConversion (true);  // Do not listen to device for conversion result, instead we will wait the worst-case period
    //         cout << " my0:" << micros()-timecheck;
    //         //tempTimer.set (tempsensebus.millisToWaitForConversion (temperature_precision)*1000);
    //         tempTimer.set (800000);         
    //         temp_status = CONVERT;
    //     }
    //     else if (temp_status == CONVERT) {
    //         wait_one_loop = true;
    //         timecheck = micros();
    //         temps[temp_current_index] = tempsensebus.getTempFByIndex(temp_current_index);
    //         cout << " my1:" << micros()-timecheck;
    //         tempTimer.set(1500000);
    //         temp_status = DELAY;
    //     }
    //     else if (temp_status == DELAY) {
    //         //printf ("\n loop:%d temps[%ld] = %lf F\n", loopno, temp_current_index, temps[temp_current_index]);
    //         tempTimer.set(60000);
    //         if (++temp_current_index >= temp_detected_device_ct) temp_current_index -= temp_detected_device_ct;
    //         temp_status = IDLE;
    //     }
    //     cout << " stat1:" << temp_status << " id:"  << temp_current_index << " tmp:" << ((temp_status == IDLE) ? temps[temp_current_index] : -1) << endl;
    // }
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "pst");

    encoder.update();  // Read encoder input signals

    // Potentiometer - takes 400 us to read & convert (?!)
    pot_percent = convert_units ((double)analogRead (pot_wipe_pin), pot_convert_percent_per_adc, pot_convert_invert, 0.0, pot_convert_offset);  // Potentiometer
    ema_filt (pot_percent, &pot_filt_percent, pot_ema_alpha);
    
    // Voltage of vehicle battery - takes 70 us to read, convert, and filter
    battery_v = convert_units ((double)analogRead (battery_pin), battery_convert_v_per_adc, battery_convert_invert);
    ema_filt (battery_v, &battery_filt_v, battery_ema_alpha);  // Apply EMA filter

       // Brake position - takes 70 us to read, convert, and filter
    if (!simulating || !sim_brkpos) {
        brake_pos_in = convert_units ((double)analogRead (brake_pos_pin), brake_pos_convert_in_per_adc, brake_pos_convert_invert);
        ema_filt (brake_pos_in, &brake_pos_filt_in, brake_pos_ema_alpha);
    }
    else brake_pos_filt_in = (brake_pos_nom_lim_retract_in + brake_pos_zeropoint_in)/2;  // To keep brake position in legal range during simulation
    
    if (!simulating || !sim_starter) starter = read_pin (starter_pin);

    // Tach - takes 22 us to read when no activity
    if (!simulating || !sim_tach) {
        if (tach_delta_us) {  // If a valid rotation has happened since last time, delta will have a value
            tach_buf_delta_us = tach_delta_us;  // Copy delta value (in case another interrupt happens during handling)
            tach_delta_us = 0;  // Indicates to isr we processed this value
            tach_rpm = convert_units ((double)(tach_buf_delta_us), tach_convert_rpm_per_rpus, tach_convert_invert);
            ema_filt (tach_rpm, &tach_filt_rpm, tach_ema_alpha);  // Sensor EMA filter
        }
        else if (!engine_stopped() && tachPulseTimer.elapsed() >= tach_stop_timeout_us) {  // If time between pulses is long enough an engine can't run that slow
            tach_rpm = 0.0;  // If timeout since last magnet is exceeded
            tach_filt_rpm = 0.0;
        }        
    }
    
    // Speedo - takes 14 us to read when no activity
    if (!simulating || !sim_speedo) { 
        if (speedo_delta_us) {  // If a valid rotation has happened since last time, delta will have a value
            speedo_buf_delta_us = speedo_delta_us;  // Copy delta value (in case another interrupt happens during handling)
            speedo_delta_us = 0;  // Indicates to isr we processed this value
            speedo_mph = convert_units ((double)(speedo_buf_delta_us), speedo_convert_mph_per_rpus, speedo_convert_invert);  // Update car speed value  
            ema_filt (speedo_mph, &speedo_filt_mph, speedo_ema_alpha);  // Sensor EMA filter
        }
        else if (!car_stopped() && speedoPulseTimer.elapsed() >= speedo_stop_timeout_us) {  // If time between pulses is long enough an engine can't run that slow
            speedo_mph = 0.0;
            speedo_filt_mph = 0.0;
        }
    }

    // Brake pressure - takes 72 us to read
    if (simulating && pot_pressure) pressure_filt_psi = map (pot_filt_percent, 0.0, 100.0, pressure_min_psi, pressure_max_psi);
    else if (!simulating && !sim_pressure) {
        pressure_adc = analogRead (pressure_pin);
        pressure_psi = convert_units ((double)pressure_adc, pressure_convert_psi_per_adc, pressure_convert_invert, (double)pressure_min_adc);
        ema_filt (pressure_psi, &pressure_filt_psi, pressure_ema_alpha);  // Sensor EMA filter
        // pressure.set_raw (analogRead (pressure_pin));
        // ema_filt (pressure.get_val(), &pressure_filt_psi, pressure_ema_alpha);  // Sensor EMA filter
    }
    
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "inp");  //

    // Controller handling
    //
    // Read horz and vert inputs, determine steering pwm output -  - takes 40 us to read. Then, takes 13 us to handle
    if (!simulating || !sim_joy) {  // Handle HotRC button generated events and detect potential loss of radio signal - takes 15 us to handle
        if (ctrl == HOTRC) {
            ctrl_pos_adc[VERT][RAW] = map (hotrc_vert_pulse_us, hotrc_pulse_vert_max_us, hotrc_pulse_vert_min_us, ctrl_lims_adc[ctrl][VERT][MAX], ctrl_lims_adc[ctrl][VERT][MIN]);
            ctrl_pos_adc[HORZ][RAW] = map (hotrc_horz_pulse_us, hotrc_pulse_horz_max_us, hotrc_pulse_horz_min_us, ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);
        }
        else {
            ctrl_pos_adc[VERT][RAW] = analogRead (joy_vert_pin);  // Read joy vertical
            ctrl_pos_adc[HORZ][RAW] = analogRead (joy_horz_pin);  // Read joy horizontal
        }
        ctrl_pos_adc[VERT][RAW] = constrain (ctrl_pos_adc[VERT][RAW], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
        ctrl_pos_adc[HORZ][RAW] = constrain (ctrl_pos_adc[HORZ][RAW], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);   
        if (ctrl_pos_adc[VERT][RAW] > ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][RAW] < ctrl_db_adc[VERT][TOP]) ctrl_pos_adc[VERT][FILT] = adcmidscale_adc;  // if joy vert is in the deadband, set joy_vert_filt to center value
        else ema_filt (ctrl_pos_adc[VERT][RAW], &ctrl_pos_adc[VERT][FILT], ctrl_ema_alpha[ctrl]);  // otherwise do ema filter to determine joy_vert_filt
        if (ctrl_pos_adc[HORZ][RAW] > ctrl_db_adc[HORZ][BOT] && ctrl_pos_adc[HORZ][RAW] < ctrl_db_adc[HORZ][TOP]) ctrl_pos_adc[HORZ][FILT] = adcmidscale_adc;  // if joy horz is in the deadband, set joy_horz_filt to center value
        else ema_filt (ctrl_pos_adc[HORZ][RAW], &ctrl_pos_adc[HORZ][FILT], ctrl_ema_alpha[ctrl]);  // otherwise do ema filter to determine joy_horz_filt
    }
    if (runmode != SHUTDOWN || !shutdown_complete) { // Unless fully shut down at the moment, set the steering output
        if (ctrl_pos_adc[HORZ][FILT] >= ctrl_db_adc[HORZ][TOP]) {
            steer_pulse_safe_us = steer_pulse_stop_us + (int32_t)((double)(steer_pulse_right_us - steer_pulse_stop_us) * (1 - ((double)steer_safe_percent * speedo_filt_mph / ((double)speedo_redline_mph * 100) )));
            steer_pulse_out_us = map (ctrl_pos_adc[HORZ][FILT], ctrl_db_adc[HORZ][TOP], ctrl_lims_adc[ctrl][HORZ][MAX], steer_pulse_stop_us, steer_pulse_safe_us);  // Figure out the steering setpoint if joy to the right of deadband
        }
        else if (ctrl_pos_adc[HORZ][FILT] <= ctrl_db_adc[HORZ][BOT]) {
            steer_pulse_safe_us = steer_pulse_stop_us - (int32_t)((double)(steer_pulse_stop_us - steer_pulse_left_us) * (1 - ((double)steer_safe_percent * speedo_filt_mph / ((double)speedo_redline_mph * 100) )));
            steer_pulse_out_us = map (ctrl_pos_adc[HORZ][FILT], ctrl_db_adc[HORZ][BOT], ctrl_lims_adc[ctrl][HORZ][MIN], steer_pulse_stop_us, steer_pulse_safe_us);  // Figure out the steering setpoint if joy to the left of deadband
        }
        else steer_pulse_out_us = steer_pulse_stop_us;  // Stop the steering motor if inside the deadband
    }
    if (ctrl == JOY) ignition = read_pin (joy_ign_btn_pin);
    else if (ctrl == HOTRC) {
        if (hotrc_ch3_sw_event) {  // Turn on/off the vehicle ignition
            if (hotrc_suppress_next_ch3_event) hotrc_suppress_next_ch3_event = false;
            else ignition = !ignition;
            hotrc_ch3_sw_event = false;
        }
        hotrc.calc();  // Detect loss of radio reception and panic stop
        if (ctrl_pos_adc[VERT][FILT] > hotrc.get_failsafe_min() && ctrl_pos_adc[VERT][FILT] < hotrc.get_failsafe_max()) {
            if (hotrc_radio_detected && hotrcPanicTimer.expired()) {
                hotrc_radio_detected = false;
                hotrc_suppress_next_ch3_event = true;  // reject spurious ch3 switch event upon next hotrc poweron
                hotrc_suppress_next_ch4_event = true;  // reject spurious ch4 switch event upon next hotrc poweron
            }
        }
        else {
            hotrcPanicTimer.reset();
            hotrc_radio_detected = true;
            if (!ignition_output_enabled) {
                set_pin (ignition_pin, OUTPUT);  // do NOT plug in the joystick when using the hotrc to avoid ign contention
                ignition_output_enabled = true;
            }
        }
    }
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "joy");  //
    
    // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in shutdown mode with no activity
    //
    // printf("mode: %d, panic: %d, vpos: %4ld, min: %4ld, max: %4ld, elaps: %6ld", runmode, panic_stop, ctrl_pos_adc[VERT][FILT], hotrc_pos_failsafe_min_us, hotrc_pos_failsafe_max_us, hotrcPanicTimer.elapsed());
    if (basicmodesw) runmode = BASIC;  // if basicmode switch on --> Basic Mode
    else if (runmode != CAL && (panic_stop || !ignition)) runmode = SHUTDOWN;
    else if (runmode != CAL && (starter || engine_stopped())) runmode = STALL;  // otherwise if engine not running --> Stall Mode
    
    if (runmode == BASIC) {  // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        if (we_just_switched_modes) {  // Upon entering basic mode, the brake and gas actuators need to be parked out of the way so the pedals can be used.
            // syspower = HIGH;  // Power up devices if not already
            enable_pids (0, 0, 0);
            gasServoTimer.reset();  // Ensure we give the servo enough time to move to position
            gasSPID.set_enable (false);
            cruiseSPID.set_enable (false);
            motorParkTimer.reset();  // Set a timer to timebox this effort
            park_the_motors = true;  // Flags the motor parking to happen
        }
        if (!engine_stopped() && !basicmodesw) runmode = HOLD;  // If we turned off the basic mode switch with engine running, go to Hold mode. If engine is not running, we'll end up in Stall Mode automatically
    }
    else if (runmode == SHUTDOWN) { // In shutdown mode we stop the car if it's moving then park the motors.
        if (we_just_switched_modes) {  // If basic switch is off, we need to stop the car and release brakes and gas before shutting down                
            enable_pids (1, 1, 0);
            gasSPID.set_target (tach_idle_rpm);  //  Release the throttle 
            shutdown_complete = false;
            shutdown_color = LPNK;
            disp_runmode_dirty = true;
            calmode_request = false;
            park_the_motors = false;
            if (!car_stopped()) {
                if (panic_stop && brakeSPID.get_target() < pressure_panic_initial_psi) brakeSPID.set_target (pressure_panic_initial_psi);
                else if (!panic_stop && brakeSPID.get_target() < pressure_hold_initial_psi) brakeSPID.set_target (pressure_hold_initial_psi);
                brakeIntervalTimer.reset();
                stopcarTimer.reset();
            }
        }
        if (ignition && !panic_stop) {
            // syspower = HIGH;  // Power up devices if not already
            runmode = STALL;
        }
        if (!shutdown_complete) {  // If we haven't yet stopped the car and then released the brakes and gas all the way
            if (car_stopped() || stopcarTimer.expired()) {  // If car has stopped, or timeout expires, then release the brake
                if (shutdown_color == LPNK) {  // On first time through here
                    enable_pids (0, 0, 0);
                    brakeSPID.set_enable (false);
                    gasSPID.set_enable (false);
                    park_the_motors = true;  // Flags the motor parking to happen, only once
                    gasServoTimer.reset();  // Ensure we give the servo enough time to move to position
                    motorParkTimer.reset();  // Set a timer to timebox this effort
                    shutdown_color = DPNK;
                    disp_runmode_dirty = true;
                }
                else if (!park_the_motors) {  // When done parking the motors we can finish shutting down
                    shutdown_complete = true;
                    shutdown_color = colorcard[SHUTDOWN];
                    disp_runmode_dirty = true;
                    sleepInactivityTimer.reset();
                }
            }
            else if (brakeIntervalTimer.expired()) {
                brakeSPID.set_target (brakeSPID.get_target() + (panic_stop) ? pressure_panic_increment_psi : pressure_hold_increment_psi);  // Slowly add more brakes until car stops
                brakeIntervalTimer.reset();  
            }
        }
        else if (calmode_request) {  // if fully shut down and cal mode requested
            // syspower = HIGH;  // Power up devices if not already
            runmode = CAL;
        }
        else if (sleepInactivityTimer.expired()) {
            // syspower = LOW;  // Power down devices
            // go to sleep?    
        }
    }
    else if (runmode == STALL) {  // In stall mode, the gas doesn't have feedback, so runs open loop, and brake pressure target proportional to joystick
        if (we_just_switched_modes) enable_pids (0, 0, 0);
        if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][BOT]) brakeSPID.set_target (pressure_min_psi);  // If in deadband or being pushed up, no pressure target
        else brakeSPID.set_target (map ((double)ctrl_pos_adc[VERT][FILT], (double)ctrl_db_adc[VERT][BOT], (double)ctrl_lims_adc[ctrl][VERT][MIN], pressure_min_psi, pressure_max_psi));  // Scale joystick value to pressure adc setpoint
        if (!starter && !engine_stopped()) runmode = HOLD;  // Enter Hold Mode if we started the car
        // Throttle behavior is handled in pid section
    }
    else if (runmode == HOLD) {
        if (we_just_switched_modes) {  // Release throttle and push brake upon entering hold mode
            enable_pids (1, 1, 0);
            gasSPID.set_target (tach_idle_rpm);  // Let off gas (if gas using PID mode)
            if (car_stopped()) brakeSPID.set_target (pressure_filt_psi + pressure_hold_increment_psi); // If the car is already stopped then just add a touch more pressure and then hold it.
            else if (brakeSPID.get_target() < pressure_hold_initial_psi) brakeSPID.set_target (pressure_hold_initial_psi);  //  These hippies need us to stop the car for them
            brakeIntervalTimer.reset();
            stopcarTimer.reset();
            joy_centered = false;  // Fly mode will be locked until the joystick first is put at or below center
        }
        if (brakeIntervalTimer.expired() && !stopcarTimer.expired()) {  // Each interval the car is still moving, push harder
            if (!car_stopped()) brakeSPID.set_target (brakeSPID.get_target() + pressure_hold_increment_psi);
            brakeIntervalTimer.reset();
        }
        if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) joy_centered = true; // Mark joystick at or below center, now pushing up will go to fly mode
        else if (joy_centered && (ctrl == JOY || hotrc_radio_detected)) runmode = FLY; // Enter Fly Mode upon joystick movement from center to above center
    }
    else if (runmode == FLY) {
        if (we_just_switched_modes) {
            enable_pids (1, 1, 1);
            gesture_progress = 0;
            gestureFlyTimer.set (gesture_flytimeout_us); // Initialize gesture timer to already-expired value
            cruise_sw_held = false;
            cruiseSwTimer.reset();
        }
        if (car_stopped() && ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT]) runmode = HOLD;  // Go to Hold Mode if we have braked to a stop  // && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][BOT]
        else if (ctrl == HOTRC && !(simulating && sim_joy) && !hotrc_radio_detected) runmode = HOLD;  // Radio must be good to fly. This should already be handled elsewhere but another check can't hurt
        else {  // Update the gas and brake targets based on joystick position, for the PIDs to drive
            if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP])  {  // If we are trying to accelerate
                gasSPID.set_target (map ((double)ctrl_pos_adc[VERT][FILT], (double)ctrl_db_adc[VERT][TOP], (double)ctrl_lims_adc[ctrl][VERT][MAX], tach_idle_rpm, tach_govern_rpm));
            }
            else gasSPID.set_target (tach_idle_rpm);  // Let off gas (if gas using PID mode)
            if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT])  {  // If we are trying to brake, scale joystick value to determine pressure adc setpoint
                brakeSPID.set_target (map ((double)ctrl_pos_adc[VERT][FILT], (double)ctrl_db_adc[VERT][BOT], (double)ctrl_lims_adc[ctrl][VERT][MIN], pressure_min_psi, pressure_max_psi));
            }
            else brakeSPID.set_target (pressure_min_psi);  // Default when joystick not pressed   
        }
        // Cruise mode can be entered by pressing a controller button, or by holding the brake on full for a half second. Which epends on the cruise_gesturing flag.
        // The gesture involves pushing the joystick from the center to the top, then to the bottom, then back to center, quickly enough.
        if (ctrl == JOY) {
            if (cruise_gesturing) {  // If we are configured to use joystick gestures to go to cruise mode, the gesture is 
                if (!gesture_progress && ctrl_pos_adc[VERT][FILT] >= ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][TOP])  { // Re-zero gesture timer for potential new gesture whenever joystick at center
                    gestureFlyTimer.reset();
                }
                if (gestureFlyTimer.expired()) gesture_progress = 0; // If gesture timeout has expired, cancel any in-progress gesture
                else {  // Otherwise check for successful gesture motions
                    if (!gesture_progress && ctrl_pos_adc[VERT][FILT] >= ctrl_lims_adc[ctrl][VERT][MAX] - default_margin_adc)  { // If joystick quickly pushed to top, step 1 of gesture is successful
                        gesture_progress++;
                        gestureFlyTimer.reset();
                    }
                    else if (gesture_progress == 1 && ctrl_pos_adc[VERT][FILT] <= ctrl_lims_adc[ctrl][VERT][MIN] + default_margin_adc)  { // If joystick then quickly pushed to bottom, step 2 succeeds
                        gesture_progress++;
                        gestureFlyTimer.reset();
                    }
                    else if (gesture_progress == 2 && ctrl_pos_adc[VERT][FILT] >= ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][TOP]) { // If joystick then quickly returned to center, go to Cruise mode
                        runmode = CRUISE;
                    }        
                }
            }
            if (!cruise_sw) {  // If button not currently pressed
                if (cruise_sw_held && cruiseSwTimer.expired()) runmode = CRUISE;  // After a long press of sufficient length, upon release enter Cruise mode
                cruise_sw_held = false;  // Cancel button held state
            }
            else if (!cruise_sw_held) {  // If the button just now got pressed
                cruiseSwTimer.reset(); // Start hold time timer
                cruise_sw_held = true;  // Get into button held state
            }
        }
        else if (ctrl == HOTRC && hotrc_ch4_sw_event) {
            if (hotrc_suppress_next_ch4_event) hotrc_suppress_next_ch4_event = false;
            else runmode == CRUISE;
            hotrc_ch4_sw_event = false;    
        }
    }
    else if (runmode == CRUISE) {
        if (we_just_switched_modes) {  // Upon first entering cruise mode, initialize things
            enable_pids (1, 1, 1);
            cruiseSPID.set_target (speedo_filt_mph);
            brakeSPID.set_target (pressure_min_psi);  // Let off the brake and keep it there till out of Cruise mode
            gestureFlyTimer.reset();  // reset gesture timer
            cruise_sw_held = false;
            cruise_adjusting = false;
        }
        if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP]) {  // When joystick vert above center, increase the throttle target proportional to how far off center
            cruise_adjusting = true;  // Suspend pid loop control of gas
            gasSPID.set_target (map ((double)ctrl_pos_adc[VERT][FILT], (double)ctrl_db_adc[VERT][TOP], (double)ctrl_lims_adc[ctrl][VERT][MAX], tach_filt_rpm, tach_govern_rpm));
        }
        else if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT]) {  // When joystick vert below center, decrease the throttle target proportional to how far off center
            cruise_adjusting = true;  // Suspend pid loop control of gas
            gasSPID.set_target (map ((double)ctrl_pos_adc[VERT][FILT], (double)ctrl_lims_adc[ctrl][VERT][MIN], (double)ctrl_db_adc[VERT][BOT], tach_idle_rpm, tach_filt_rpm));
        }
        else cruise_adjusting = false;  // if joystick at center
        if (cruise_adjusting) cruiseSPID.set_target (speedo_filt_mph);
        
        // This old gesture trigger drops to Fly mode if joystick moved quickly from center to bottom
        // if (ctrl_pos_adc[VERT][FILT] <= ctrl_lims_adc[ctrl][VERT][MIN]+default_margin_adc && abs(mycros() - gesture_timer_us) < gesture_flytimeout_us)  runmode = FLY;  // If joystick quickly pushed to bottom 
        // printf ("hotvpuls=%ld, hothpuls=%ld, joyvfilt=%ld, joyvmin+marg=%ld, timer=%ld\n", hotrc_vert_pulse_us, hotrc_horz_pulse_us, ctrl_pos_adc[VERT][RAW], ctrl_lims_adc[ctrl][VERT][MIN] + default_margin_adc, gesture_timer_us);
        if (ctrl == JOY) {
            if (ctrl_pos_adc[VERT][FILT] > ctrl_lims_adc[ctrl][VERT][MIN] + default_margin_adc) gestureFlyTimer.reset();  // Keep resetting timer if joystick not at bottom
            else if (gestureFlyTimer.expired()) runmode = FLY;  // New gesture to drop to fly mode is hold the brake all the way down for 500 ms
            if (cruise_sw) cruise_sw_held = true;  // Pushing cruise button sets up return to fly mode
            else if (cruise_sw_held) {  // Release of button drops us back to fly mode
                cruise_sw_held = false;
                runmode = FLY;
            }
        }
        else if (ctrl == HOTRC && hotrc_ch4_sw_event) {
            if (hotrc_suppress_next_ch4_event) hotrc_suppress_next_ch4_event = false;
            else runmode == FLY;
            hotrc_ch4_sw_event = false;
        }
        if (car_stopped()) {  // In case we slam into a brick wall, get out of cruise mode
            if (serial_debugging) Serial.println (F("Error: Car stopped in cruise mode"));  // , speedo_filt_mph, neutral
            runmode = HOLD;  // Back to Hold Mode  
        }
    }
    else if (runmode == CAL) {
        if (we_just_switched_modes) {  // If basic switch is off, we need to stop the car and release brakes and gas before shutting down                
            enable_pids (0, 0, 0);
            calmode_request = false;
            cal_pot_gas_ready = false;
            cal_pot_gasservo = false;
            cal_joyvert_brkmotor = false;
            cal_set_hotrc_failsafe_ready = false;
        }
        else if (calmode_request) runmode = SHUTDOWN;
        if (!cal_pot_gas_ready) {
            double temp = map (pot_filt_percent, pot_min_percent, pot_max_percent, (double)gas_pulse_ccw_max_us, (double)gas_pulse_cw_min_us);
            if (temp <= (double)gas_pulse_idle_us && temp >= (double)gas_pulse_redline_us) cal_pot_gas_ready = true;
        }
        if (btn_press_action == SHORT) {
            hotrc.set_failsafe();
            hotrc.print();  // Also perhaps write values to flash
            std::cout << "\nHotrc failsafe range set!  Min: " << hotrc.get_failsafe_min() << "adc, Max: " << hotrc.get_failsafe_max() << " adc, including " << hotrc.get_pad() << " adc pad both ways" << std::endl;
            btn_press_action = NONE;
        }
    }
    else { // Obviously this should never happen
        if (serial_debugging) Serial.println (F("Error: Invalid runmode entered"));
        runmode = SHUTDOWN;
    }
    if (runmode != SHUTDOWN && runmode != BASIC) park_the_motors = false;
    if (runmode != oldmode) disp_runmode_dirty = true;  // runmode should not be changed after this point in loop
    we_just_switched_modes = (runmode != oldmode);
    
    // cout << "rm:" << runmode << " om:" << oldmode << "vert:" << ctrl_pos_adc[VERT][FILT] << " up?" << (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) << " jc?" << joy_centered << "\n";
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "mod");  //

    // Update motor outputs - takes 185 us to handle every 30ms when the pid timer expires, otherwise 5 us
    //
    
    // Steering - Update motor output
    if (steerPidTimer.expired()) {
        steerPidTimer.reset();
        steer_pulse_out_us = constrain (steer_pulse_out_us, steer_pulse_right_us, steer_pulse_left_us);  // Don't be out of range
        steer_servo.writeMicroseconds (steer_pulse_out_us);   // Write steering value to jaguar servo interface
    }
    // Brakes - Update motor output
    if (brakePidTimer.expired()) {
        brakePidTimer.reset();
        if (runmode == CAL && cal_joyvert_brkmotor) {
            if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP]) brake_pulse_out_us = (double)map (ctrl_pos_adc[VERT][FILT], ctrl_db_adc[VERT][TOP], ctrl_lims_adc[ctrl][VERT][MAX], brake_pulse_stop_us, brake_pulse_extend_us);
            else if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT]) brake_pulse_out_us = (double)map (ctrl_pos_adc[VERT][FILT], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_db_adc[VERT][BOT], brake_pulse_retract_us, brake_pulse_stop_us);
            else brake_pulse_out_us = (double)brake_pulse_stop_us;
        }
        else if (park_the_motors) {
            if (brake_pos_filt_in + brake_pos_margin_in <= brake_pos_park_in) brake_pulse_out_us =  // If brake is retracted from park point, extend toward park point, slowing as we approach
                (double)map ((int32_t)brake_pos_filt_in, (int32_t)brake_pos_park_in, (int32_t)brake_pos_nom_lim_retract_in, brake_pulse_stop_us, brake_pulse_extend_us);
            else if (brake_pos_filt_in - brake_pos_margin_in >= brake_pos_park_in) brake_pulse_out_us =  // If brake is extended from park point, retract toward park point, slowing as we approach
                (double)map ((int32_t)brake_pos_filt_in, (int32_t)brake_pos_park_in, (int32_t)brake_pos_nom_lim_extend_in, brake_pulse_stop_us, brake_pulse_retract_us);
        }
        else if (runmode != BASIC) brake_pulse_out_us = brakeSPID.compute();  // Otherwise the pid control is active
        if (runmode != BASIC || park_the_motors) {
            if (runmode == CAL && cal_joyvert_brkmotor)  // In Cal mode constrain the motor to its entire range, instead of to the calibrated limits
                brake_pulse_out_us = constrain (brake_pulse_out_us, (double)brake_pulse_retract_min_us, (double)brake_pulse_extend_max_us);  // Send to the actuator. Refuse to exceed range    
            else {  // Prevent any movement of motor which would exceed position limits. Improve this by having the motor actively go back toward position range if position is beyond either limit
                if ( ((brake_pos_filt_in + brake_pos_margin_in <= brake_pos_nom_lim_retract_in) && ((int32_t)brake_pulse_out_us < brake_pulse_stop_us)) ||  // If the motor is at or past its position limit in the retract direction, and we're intending to retract more ...
                    ((brake_pos_filt_in - brake_pos_margin_in >= brake_pos_nom_lim_extend_in) && ((int32_t)brake_pulse_out_us > brake_pulse_stop_us)) )  // ... or same thing in the extend direction ...
                    brake_pulse_out_us = brake_pulse_stop_us;  // ... then stop the motor
                brake_pulse_out_us = constrain (brake_pulse_out_us, (double)brake_pulse_retract_us, (double)brake_pulse_extend_us);  // Send to the actuator. Refuse to exceed range    
            } 
            brake_servo.writeMicroseconds ((int32_t)brake_pulse_out_us);  // Write result to jaguar servo interface
        }
    }
    // Cruise - Update gas target. Controls gas rpm target to keep speed equal to cruise mph target, except during cruise target adjustment, gas target is determined in cruise mode logic.
    if (cruisePidTimer.expired() && runmode == CRUISE && !cruise_adjusting) {
        cruisePidTimer.reset();
        gasSPID.set_target (cruiseSPID.compute());  // 
    }
    // Gas - Update servo output. Determine gas actuator output from rpm target.  PID loop is effective in Fly or Cruise mode.
    if (gasPidTimer.expired()) {
        gasPidTimer.reset();
        if (park_the_motors) gas_pulse_out_us = gas_pulse_idle_us + gas_pulse_park_slack_us;
        else if (runmode == STALL) {  // Stall mode runs the gas servo directly proportional to joystick. This is truly open loop
            if (starter) gas_pulse_out_us = gas_pulse_govern_us;
            else if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) gas_pulse_out_us = gas_pulse_idle_us;  // If in deadband or being pushed down, we want idle
            else gas_pulse_out_us = map (ctrl_pos_adc[VERT][FILT], ctrl_db_adc[VERT][TOP], ctrl_lims_adc[ctrl][VERT][MAX], gas_pulse_idle_us, gas_pulse_govern_us);  // Actuators still respond and everything, even tho engine is turned off
        }
        else if (runmode != BASIC) {
            if (runmode == CAL && cal_pot_gas_ready && cal_pot_gasservo) 
                gas_pulse_out_us = (int32_t)(map (pot_filt_percent, pot_min_percent, pot_max_percent, (double)gas_pulse_ccw_max_us, (double)gas_pulse_cw_min_us));
            else if (gasSPID.get_open_loop())  // This isn't really open loop, more like simple proportional control, with output set proportional to target 
                gas_pulse_out_us = (int32_t)(map (gasSPID.get_target(), tach_idle_rpm, tach_govern_rpm, (double)gas_pulse_idle_us, (double)gas_pulse_govern_us)); // scale gas rpm target onto gas pulsewidth target (unless already set in stall mode logic)
            else gas_pulse_out_us = (int32_t)(gasSPID.compute());  // Do proper pid math to determine gas_pulse_out_us from engine rpm error
            // printf ("Gas PID   rm= %+-4ld target=%-+9.4lf", runmode, (double)gasSPID.get_target());
            // printf (" output = %-+9.4lf,  %+-4ld\n", gasSPID.get_output(), gas_pulse_out_us);
        }
        if (runmode != BASIC || park_the_motors) {
            if (runmode == CAL && cal_pot_gas_ready && cal_pot_gasservo)
                gas_pulse_out_us = constrain (gas_pulse_out_us, gas_pulse_cw_min_us, gas_pulse_ccw_max_us);
            else gas_pulse_out_us = constrain (gas_pulse_out_us, gas_pulse_govern_us, gas_pulse_idle_us);
            // printf (" output = %-+9.4lf,  %+-4ld\n", gasSPID.get_output(), gas_pulse_out_us);
            gas_servo.writeMicroseconds (gas_pulse_out_us);  // Write result to servo
        }
    }
    if (park_the_motors) {  //  When parking motors, IF the timeout expires OR the brake and gas motors are both close enough to the park position THEN stop trying to park the motors
        bool brake_parked = (abs(brake_pos_filt_in - brake_pos_park_in) <= brake_pos_margin_in);
        bool gas_parked = ((gas_pulse_out_us == gas_pulse_idle_us + gas_pulse_park_slack_us) && gasServoTimer.expired());
        if ((brake_parked && gas_parked) || motorParkTimer.expired()) park_the_motors = false;
    }

    // Auto-Diagnostic  :   Check for worrisome oddities and dubious circumstances. Report any suspicious findings
    //
    // This section should become a real time self-diagnostic system, to look for anything that doesn't seem right and display an
    // informed trouble code. Much like the engine computer in cars nowadays, which keep track of any detectable failures for you to
    // retreive with an OBD tool. Eventually this should include functions allowing us to detect things like:
    //  1. A sensor or actuator is unplugged, movement blocked, missing magnetic pulses, etc.
    //  2. Air in the brake lines.
    //  3. Axle/brake drum may be going bad (increased engine RPM needed to achieve certain speedo)  (beware going up hill may look the same).
    //  4. E-brake has been left on (much the same symptoms as above? (beware going up hill may look the same) 
    //  5. Battery isn't charging, or just running low.
    //  6. Carburetor not behaving (or air filter is clogged). (See above about engine deiseling - we can detect this!)
    //  7. After increasing braking, the actuator position changes in the opposite direction, or vise versa.
    //  8. Changing an actuator is not having the expected effect.
    //  9. A tunable value suspected to be out of tune.
    //  10. Check regularly for ridiculous shit. Compare our variable values against a predetermined list of conditions which shouldn't be possible or are at least very suspect. For example:
    //     A) Sensor reading is out of range, or has changed faster than it ever should.
    //     B) Stopping the car on entering hold/shutdown mode is taking longer than it ever should.
    //     C) Mule seems to be accelerating like a Tesla.
    //     D) Car is accelerating yet engine is at idle.
    //  11. The control system has nonsensical values in its variables.
    //
    if (!ignition && !engine_stopped()) {
        if (diag_ign_error_enabled) { // See if the engine is turning despite the ignition being off
            Serial.println (F("Detected engine rotation in the absense of ignition signal"));  // , tach_filt_rpm, ignition
            diag_ign_error_enabled = false;  // Prevents endless error reporting the same error
        }
    }
    else diag_ign_error_enabled = true;
    // I don't think we really need to panic about this, but it does seem curious. Actually this will probably occur like when we're sliding
    // into camp after a ride, and kill the engine before we stop the car. For a fraction of a second the engine would keep turning anyway.
    // Or for that matter whenever the carb is out of tune and making the engine diesel after we kill the ign.

    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "pid");  //
        
    // Touchscreen handling - takes 800 us to handle every 20ms when the touch timer expires, otherwise 20 us (includes touch timer + encoder handling w/o activity)
    //
    int32_t touch_x, touch_y, trow, tcol;
    // if (screen.ts.touched() == 1 ) { // Take actions if one touch is detected. This panel can read up to two simultaneous touchpoints
    if (ts.touched()) { // Take actions if one touch is detected. This panel can read up to two simultaneous touchpoints
        touch_accel = 1 << touch_accel_exponent;  // determine value editing rate
        // TS_Point touchpoint = screen.ts.getPoint();  // Retreive a point
        TS_Point touchpoint = ts.getPoint();  // Retreive a point
        #ifdef CAP_TOUCH
            touchpoint.x = map (touchpoint.x, 0, disp_height_pix, disp_height_pix, 0);  // Rotate touch coordinates to match tft coordinates
            touchpoint.y = map (touchpoint.y, 0, disp_width_pix, disp_width_pix, 0);  // Rotate touch coordinates to match tft coordinates
            touch_y = disp_height_pix-touchpoint.x; // touch point y coordinate in pixels, from origin at top left corner
            touch_x = touchpoint.y; // touch point x coordinate in pixels, from origin at top left corner
        #else
            touch_x = constrain (touchpoint.x, 355, 3968);
            touch_y = constrain (touchpoint.y, 230, 3930);
            
            touch_x = map (touch_x, 355, 3968, 0, disp_width_pix);
            touch_y = map (touch_y, 230, 3930, 0, disp_height_pix);
            if (!flip_the_screen) {
                touch_x = disp_width_pix - touch_x;
                touch_y = disp_height_pix - touch_y;
            }
        #endif
        std::cout << "Touch: ptx:" << touchpoint.x << ", pty:" << touchpoint.y << ", ptz:" << touchpoint.z << " x:" << touch_x << ", y:" << touch_y << std::endl;
        // std::cout << "Touch: ptx:" << touchpoint.x << ", pty:" << touchpoint.y << " x:" << touch_x << ", y:" << touch_y << ", z:" << touchpoint.z << std::endl;
        
        trow = constrain((touch_y + touch_fudge)/touch_cell_v_pix, 0, 4);  // The -8 seems to be needed or the vertical touch seems off (?)
        tcol = (touch_x-touch_margin_h_pix)/touch_cell_h_pix;
        // Take appropriate touchscreen actions depending how we're being touched
        if (tcol==0 && trow==0 && !touch_now_touched) {
            if (++dataset_page >= arraysize(pagecard)) dataset_page -= arraysize(pagecard);  // Displayed dataset page can also be changed outside of simulator
        }
        else if (tcol==0 && trow==1) {  // Long touch to enter/exit editing mode, if in editing mode, press to change selection of item to edit
            if (tuning_ctrl == OFF) {
                selected_value = 0;  // if entering select mode from off mode, select first variable
                if (touch_longpress_valid && touchHoldTimer.expired()) {
                    tuning_ctrl = SELECT;
                    touch_longpress_valid = false;
                }
            }
            else if (tuning_ctrl == EDIT && !touch_now_touched) {
                tuning_ctrl = SELECT;  // drop back to select mode
                selected_value++;  // and move to next selection
            }
            else if (tuning_ctrl == SELECT) {
                if (!touch_now_touched) {
                    if (++selected_value >= arraysize (dataset_page_names[dataset_page])) selected_value -= arraysize (dataset_page_names[dataset_page]);
                }
                else if (touch_longpress_valid && touchHoldTimer.expired()) {
                    tuning_ctrl = OFF;
                    touch_longpress_valid = false;
                }
            }
        }
        else if (tcol==0 && trow==2) {  // Pressed the increase value button, for real time tuning of variables
            if (tuning_ctrl == SELECT) tuning_ctrl = EDIT;  // If just entering edit mode, don't change value yet
            else if (tuning_ctrl == EDIT) sim_edit_delta_touch = touch_accel;  // If in edit mode, increase value
        }
        else if (tcol==0 && trow==3) {  // Pressed the decrease value button, for real time tuning of variables
            if (tuning_ctrl == SELECT) tuning_ctrl = EDIT;  // If just entering edit mode, don't change value yet
            else if (tuning_ctrl == EDIT) sim_edit_delta_touch = -touch_accel;  // If in edit mode, decrease value
        }
        else if (tcol==0 && trow==4) {  // && trow == 0 . Pressed the simulation mode toggle. Needs long press
            if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.get_timeout()) {
                simulating = !simulating;
                touch_longpress_valid = false;
            }
        }
        else if (tcol==2 && trow==0 && (runmode == CAL || (runmode == SHUTDOWN && shutdown_complete))) {
            if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.get_timeout()) {
                calmode_request = true;
                touch_longpress_valid = false;
            }  // Pressed the basic mode toggle button. Toggle value, only once per touch
        }
        else if (simulating) {
            if (tcol==3 && trow==0 && sim_basicsw && !touch_now_touched) basicmodesw = !basicmodesw;  // Pressed the basic mode toggle button. Toggle value, only once per touch
            else if (tcol==3 && trow==1 && sim_pressure) adj_val (&pressure_filt_psi, (double)touch_accel, pressure_min_psi, pressure_max_psi);   // (+= 25) Pressed the increase brake pressure button
            else if (tcol==3 && trow==2 && sim_pressure) adj_val (&pressure_filt_psi, (double)-touch_accel, pressure_min_psi, pressure_max_psi); // (-= 25) Pressed the decrease brake pressure button
            else if (tcol==3 && trow==4 && sim_joy) adj_val (&ctrl_pos_adc[HORZ][FILT], -touch_accel, ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);  // (-= 25) Pressed the joystick left button
            else if (tcol==4 && trow==0 && !touch_now_touched) ignition = !ignition; // Pressed the ignition switch toggle button. Toggle value, only once per touch
            else if (tcol==4 && trow==1 && sim_tach) adj_val (&tach_filt_rpm, (double)touch_accel, 0.0, tach_redline_rpm);  // (+= 25) Pressed the increase engine rpm button
            else if (tcol==4 && trow==2 && sim_tach) adj_val (&tach_filt_rpm, (double)(-touch_accel), 0.0, tach_redline_rpm);  // (-= 25) Pressed the decrease engine rpm button
            else if (tcol==4 && trow==3 && sim_joy) adj_val (&ctrl_pos_adc[VERT][FILT], touch_accel, ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);  // (+= 25) Pressed the joystick up button
            else if (tcol==4 && trow==4 && sim_joy) adj_val (&ctrl_pos_adc[VERT][FILT], -touch_accel, ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);  // (-= 25) Pressed the joystick down button
            else if (tcol==5 && trow==0 && sim_syspower) {  // You need to enable syspower simulation, be in sim mode and then long-press the syspower button to toggle it. (Hard to do by accident)/
                if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.get_timeout()) {
                    syspower = !syspower;
                    touch_longpress_valid = false;
                }
            }
            else if (tcol==5 && trow==1 && sim_speedo) adj_val (&speedo_filt_mph, 0.005*(double)touch_accel, 0.0, speedo_redline_mph);  // (+= 50) // Pressed the increase vehicle speed button
            else if (tcol==5 && trow==2 && sim_speedo) adj_val (&speedo_filt_mph, -0.005*(double)touch_accel, 0.0, speedo_redline_mph);  // (-= 50) Pressed the decrease vehicle speed button
            else if (tcol==5 && trow==4 && sim_joy) adj_val (&ctrl_pos_adc[HORZ][FILT], touch_accel, ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);  // (+= 25) Pressed the joystick right button                           
        }
        if (touch_accel_exponent < touch_accel_exponent_max && (touchHoldTimer.elapsed() > (touch_accel_exponent + 1) * touchAccelTimer.get_timeout())) touch_accel_exponent++; // If timer is > the shift time * exponent, and not already maxed, double the edit speed by incrementing the exponent
        touch_now_touched = true;
    }  // (if screen.ts reads a touch)
    else {  // If not being touched, put momentarily-set simulated button values back to default values
        if (simulating) cruise_sw = false;  // // Makes this button effectively momentary
        sim_edit_delta_touch = 0;  // Stop changing value
        touch_now_touched = false;  // remember last touch state
        touch_accel_exponent = 0;
        touchHoldTimer.reset();
        touch_longpress_valid = true;
    }
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "tch");  //

    // Encoder handling
    //
    uint32_t encoder_sw_action = encoder.handleSwitchAction();
    if (encoder_sw_action != Encoder::NONE) {  // First deal with any unhandled switch press events
        if (encoder_sw_action == Encoder::SHORT)  {  // if short press
            if (tuning_ctrl == EDIT) tuning_ctrl = SELECT;  // If we were editing a value drop back to select mode
            else if (tuning_ctrl == SELECT) tuning_ctrl = EDIT;  // If we were selecting a variable start editing its value
            else ;  // Unless tuning, short press now does nothing. I envision it should switch desktops from our current analysis interface to a different runtime display 
        }
        else tuning_ctrl = (tuning_ctrl == OFF) ? SELECT : OFF;  // Long press starts/stops tuning
    }
    if (tuning_ctrl == EDIT) sim_edit_delta_encoder = encoder.handleTuning();
    else if (tuning_ctrl == SELECT) selected_value += encoder.handleSelection();  // If overflow constrain will fix in general handler below
    else if (tuning_ctrl == OFF) dataset_page += encoder.handleSelection();  // If overflow tconstrain will fix in general below

    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "enc");  //

    // Tuning : implement effects of changes made by encoder or touchscreen to simulating, dataset_page, selected_value, or tuning_ctrl
    //
    sim_edit_delta += sim_edit_delta_encoder + sim_edit_delta_touch;  // Allow edits using the encoder or touchscreen
    sim_edit_delta_touch = 0;
    sim_edit_delta_encoder = 0;
    if (tuning_ctrl != tuning_ctrl_last || dataset_page != dataset_page_last || selected_value != selected_value_last || sim_edit_delta) tuningCtrlTimer.reset();  // If just switched tuning mode or any tuning activity, reset the timer
    else if (tuning_ctrl != OFF && tuningCtrlTimer.expired()) tuning_ctrl = OFF;  // If the timer expired, go to OFF and redraw the tuning corner
    dataset_page = constrain (dataset_page, 0, arraysize(pagecard)-1);  // select next or prev only 1 at a time, avoiding over/underflows, and without giving any int negative value
    if (dataset_page != dataset_page_last) {
        if (tuning_ctrl == EDIT) tuning_ctrl = SELECT;  // If page is flipped during edit, drop back to select mode
        disp_dataset_page_dirty = true;  // Redraw the fixed text in the tuning corner of the screen with data from the new dataset page
    }
    if (tuning_ctrl == SELECT) {
        selected_value = constrain (selected_value, tuning_first_editable_line[dataset_page], disp_tuning_lines-1);  // Skip unchangeable values for all PID modes
        if (selected_value != selected_value_last) disp_selected_val_dirty = true;
    }
    if (tuning_ctrl != tuning_ctrl_last || disp_dataset_page_dirty) disp_selected_val_dirty = true;
    bool adj;
    adj = false;
    if (tuning_ctrl == EDIT && sim_edit_delta != 0) {  // Change tunable values when editing
        if (dataset_page == PG_RUN) {
            if (selected_value == 3) adj_bool (&sim_brkpos, sim_edit_delta);
            else if (selected_value == 4) adj_bool (&sim_joy, sim_edit_delta);
            else if (selected_value == 5) adj_bool (&sim_pressure, sim_edit_delta);
            else if (selected_value == 6) adj_bool (&sim_tach, sim_edit_delta);
            else if (selected_value == 7) adj_bool (&sim_speedo, sim_edit_delta);
        }
        else if (dataset_page == PG_JOY) {
            if (selected_value == 2) adj = adj_val (&ctrl_lims_adc[ctrl][HORZ][MIN], sim_edit_delta, 0, adcmidscale_adc - ctrl_lims_adc[ctrl][HORZ][DB] / 2 - 1);
            else if (selected_value == 3) adj = adj_val (&ctrl_lims_adc[ctrl][HORZ][MAX], sim_edit_delta, adcmidscale_adc + ctrl_lims_adc[ctrl][HORZ][DB] / 2 + 1, adcrange_adc);
            else if (selected_value == 4) adj = adj_val (&ctrl_lims_adc[ctrl][HORZ][DB], sim_edit_delta, 0, (adcmidscale_adc - ctrl_lims_adc[ctrl][HORZ][MIN] > ctrl_lims_adc[ctrl][HORZ][MAX] - adcmidscale_adc) ? 2*(ctrl_lims_adc[ctrl][HORZ][MAX] - adcmidscale_adc) : 2*(adcmidscale_adc - ctrl_lims_adc[ctrl][HORZ][MIN]));
            else if (selected_value == 5) adj = adj_val (&ctrl_lims_adc[ctrl][VERT][MIN], sim_edit_delta, 0, adcmidscale_adc - ctrl_lims_adc[ctrl][VERT][DB] / 2 - 1);
            else if (selected_value == 6) adj = adj_val (&ctrl_lims_adc[ctrl][VERT][MAX], sim_edit_delta, adcmidscale_adc + ctrl_lims_adc[ctrl][VERT][DB] / 2 + 1, adcrange_adc);
            else if (selected_value == 7) adj = adj_val (&ctrl_lims_adc[ctrl][VERT][DB], sim_edit_delta, 0, (adcmidscale_adc - ctrl_lims_adc[ctrl][VERT][MIN] > ctrl_lims_adc[ctrl][VERT][MAX] - adcmidscale_adc) ? 2*(ctrl_lims_adc[ctrl][VERT][MAX] - adcmidscale_adc) : 2*(adcmidscale_adc - ctrl_lims_adc[ctrl][VERT][MIN]));
            if (adj) calc_deadbands();  // update derived variables relevant to changes made
        }
        else if (dataset_page == PG_CAR) {
            if (selected_value == 0) {
                adj = adj_val (&gas_governor_percent, sim_edit_delta, 0, 100);
                if (adj) calc_governor();  // update derived variables relevant to changes made
            }
            else if (selected_value == 1) adj_val (&tach_idle_rpm, 0.01*sim_edit_delta, 0, tach_redline_rpm - 1);
            else if (selected_value == 2) adj_val (&tach_redline_rpm, 0.01*sim_edit_delta, tach_idle_rpm, 8000.0);
            else if (selected_value == 3) adj_val (&speedo_idle_mph, 0.01*sim_edit_delta, 0, speedo_redline_mph - 1);
            else if (selected_value == 4) adj_val (&speedo_redline_mph, 0.01*sim_edit_delta, speedo_idle_mph, 30.0);
            else if (selected_value == 5) gasSPID.set_open_loop (sim_edit_delta > 0);
            else if (selected_value == 6 && runmode == CAL) adj_bool (&cal_joyvert_brkmotor, sim_edit_delta);
            else if (selected_value == 7 && runmode == CAL) adj_bool (&cal_pot_gasservo, (sim_edit_delta < 0 || cal_pot_gas_ready) ? sim_edit_delta : -1);
      }
        else if (dataset_page == PG_PWMS) {
            if (selected_value == 0) adj_val (&steer_pulse_left_us, sim_edit_delta, steer_pulse_stop_us + 1, steer_pulse_left_max_us);
            else if (selected_value == 1) adj_val (&steer_pulse_stop_us, sim_edit_delta, steer_pulse_right_us + 1, steer_pulse_left_us - 1);
            else if (selected_value == 2) adj_val (&steer_pulse_right_us, sim_edit_delta, steer_pulse_right_min_us, steer_pulse_stop_us - 1);
            else if (selected_value == 3) adj_val (&brake_pulse_extend_us, sim_edit_delta, brake_pulse_stop_us + 1, brake_pulse_extend_max_us);
            else if (selected_value == 4) adj_val (&brake_pulse_stop_us, sim_edit_delta, brake_pulse_retract_us + 1, brake_pulse_extend_us - 1);
            else if (selected_value == 5) adj_val (&brake_pulse_retract_us, sim_edit_delta, brake_pulse_retract_min_us, brake_pulse_stop_us -1);
            else if (selected_value == 6) adj_val (&gas_pulse_idle_us, sim_edit_delta, gas_pulse_redline_us + 1, gas_pulse_ccw_max_us - gas_pulse_park_slack_us);
            else if (selected_value == 7) adj_val (&gas_pulse_redline_us, sim_edit_delta, gas_pulse_cw_min_us, gas_pulse_idle_us - 1);
        }
        else if (dataset_page == PG_BPID) {
            if (selected_value == 5) brakeSPID.set_tunings (brakeSPID.get_kp()+0.001*(double)sim_edit_delta, brakeSPID.get_ki_hz(), brakeSPID.get_kd_s());
            else if (selected_value == 6) brakeSPID.set_tunings (brakeSPID.get_kp(), brakeSPID.get_ki_hz()+0.001*(double)sim_edit_delta, brakeSPID.get_kd_s());
            else if (selected_value == 7) brakeSPID.set_tunings (brakeSPID.get_kp(), brakeSPID.get_ki_hz(), brakeSPID.get_kd_s()+0.001*(double)sim_edit_delta);
        }
        else if (dataset_page == PG_GPID) {
            if (selected_value == 5) gasSPID.set_tunings (gasSPID.get_kp()+0.001*(double)sim_edit_delta, gasSPID.get_ki_hz(), gasSPID.get_kd_s());
            else if (selected_value == 6) gasSPID.set_tunings (gasSPID.get_kp(), gasSPID.get_ki_hz()+0.001*(double)sim_edit_delta, gasSPID.get_kd_s());
            else if (selected_value == 7) gasSPID.set_tunings (gasSPID.get_kp(), gasSPID.get_ki_hz(), gasSPID.get_kd_s()+0.001*(double)sim_edit_delta);
        }
        else if (dataset_page == PG_CPID) {
            if (selected_value == 5) cruiseSPID.set_tunings (cruiseSPID.get_kp()+0.001*(double)sim_edit_delta, cruiseSPID.get_ki_hz(), cruiseSPID.get_kd_s());
            else if (selected_value == 6) cruiseSPID.set_tunings (cruiseSPID.get_kp(), cruiseSPID.get_ki_hz()+0.001*(double)sim_edit_delta, cruiseSPID.get_kd_s());
            else if (selected_value == 7) cruiseSPID.set_tunings (cruiseSPID.get_kp(), cruiseSPID.get_ki_hz(), cruiseSPID.get_kd_s()+0.001*(double)sim_edit_delta);
        }
        else if (dataset_page == PG_TEMP) {        
            if (selected_value == 4) adj_val (&pressure_adc, sim_edit_delta, pressure_min_adc, pressure_max_adc);
            else if (selected_value == 5) adj_val (&hotrc_pos_failsafe_min_adc, sim_edit_delta, ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
            else if (selected_value == 6) adj_val (&hotrc_pos_failsafe_max_adc, sim_edit_delta, ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
            else if (selected_value == 7) adj_val (&brake_pos_zeropoint_in, 0.001*sim_edit_delta, brake_pos_nom_lim_retract_in, brake_pos_nom_lim_extend_in);
        }
        sim_edit_delta = 0;
    }
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "tun");  //
    
    // Ignition & Panic stop logic and Update output signals
    if (!car_stopped()) {
        if (ctrl == HOTRC && !(simulating && sim_joy) && !hotrc_radio_detected && hotrc_radio_detected_last) panic_stop = true;  // panic_stop could also have been initiated by the user button
        else if (!ignition && ignition_last) panic_stop = true;
    }
    else if (panic_stop) panic_stop = false;  // Cancel panic if car is stopped
    if (ctrl != JOY) {  // When using joystick, ignition is controlled with button, not the code
        hotrc_radio_detected_last = hotrc_radio_detected;
        if (panic_stop) ignition = LOW;  // Kill car if panicking
        if ((ignition != ignition_last) && ignition_output_enabled) {  // Whenever ignition state changes, assuming we're allowed to write to the pin
            write_pin (ignition_pin, !ignition);  // Turn car off or on (ign output is active low), ensuring to never turn on the ignition while panicking
            ignition_last = ignition;  // Make sure this goes after the last comparison
        }
    }
    if (syspower != syspower_last) {
        syspower_set (syspower);
        syspower_last = syspower;
    }
    if (btn_press_action == LONG) {
        screen.tft_reset();
        btn_press_action = NONE;
    }
    if (!screen.get_reset_finished()) screen.tft_reset();  // If resetting tft, keep calling tft_reset until complete
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "ext");  //

    if (neopixel_pin >= 0) {  // Heartbeat led algorithm
        if (neo_heartbeat) {
            neo_heartcolor[N_RED] = (((runmode == SHUTDOWN) ? shutdown_color : colorcard[runmode]) & 0xf800) >> 8;
            neo_heartcolor[N_GRN] = (((runmode == SHUTDOWN) ? shutdown_color : colorcard[runmode]) & 0x7e0) >> 3;
            neo_heartcolor[N_BLU] = (((runmode == SHUTDOWN) ? shutdown_color : colorcard[runmode]) & 0x1f) << 3;
            int32_t neocolor = neostrip.Color (neo_heartcolor[N_BLU], neo_heartcolor[N_RED], neo_heartcolor[N_GRN]);
            if (heartbeatTimer.expired()) {
                heartbeat_pulse = !heartbeat_pulse;
                if (heartbeat_pulse) neo_brightness = neo_brightness_max;
                else neoTimer.reset();
                if (++heartbeat_state >= arraysize (heartbeat_ekg)) heartbeat_state -= arraysize (heartbeat_ekg);
                heartbeatTimer.set (heartbeat_ekg[heartbeat_state]);
            }
            else if (!heartbeat_pulse && neo_brightness) {
                neo_brightness = (int8_t)((double)neo_brightness_max * (1 - (double)neoTimer.elapsed() / (double)neo_timeout));
                if (neoTimer.expired() || neo_brightness < 1) neo_brightness = 0;
            }
            int32_t neocolor_last, neobright_last;
            if (neocolor != neocolor_last || neo_brightness != neobright_last) {
                neostrip.setPixelColor (0, neocolor);
                neostrip.setBrightness (neo_brightness);
                neostrip.show();
                neocolor_last = neocolor;
                neobright_last = neo_brightness;
            }
        }
        else if (neoTimer.expired()) {  // Rainbow fade
            neoTimer.reset();
            neostrip.setPixelColor (0, colorwheel(++neo_wheelcounter));
            neostrip.show();
        }
    }
    else if (heartbeat_led_pin >= 0) {  // Just make a heartbeat on the native board led
        heartbeat_pulse = !heartbeat_pulse;
        if (++heartbeat_state >= arraysize (heartbeat_ekg)) heartbeat_state -= arraysize (heartbeat_ekg);
        heartbeatTimer.set (heartbeat_ekg[heartbeat_state]);
        write_pin (heartbeat_led_pin, heartbeat_pulse);
    }
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "hrt");
    
    // Display updates
    if (display_enabled) screen.update();
    else {
        if (dataset_page_last != dataset_page) config.putUInt ("dpage", dataset_page);
        dataset_page_last = dataset_page;
        selected_value_last = selected_value;
        simulating_last = simulating;
        oldmode = runmode;
    }
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "dis");

    // Kick watchdogs
    Watchdog.reset();  // Kick the watchdog to keep us alive
    // if (display_enabled) screen.watchdog();
 
    // Do the control loop bookkeeping at the end of each loop
    //
    loop_period_us = loopTimer.elapsed();  // us since beginning of this loop
    loopTimer.reset();
    loop_freq_hz = 1000000.0 / ((loop_period_us) ? (double)loop_period_us : 1);  // Prevent potential divide by zero
    loopno++;  // I like to count how many loops
    if (timestamp_loop) {
        // loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "end");  //
        printf ("\rRM:%ld Lp#%ld us:%5ld ", runmode, loopno, loop_period_us);
        for (int32_t x=1; x<loopindex; x++) std::cout << ", " << std::setw(3) << loop_names[x] << x << ": " << std::setw(4) << looptimes_us[x]-looptimes_us[x-1];
        if (loop_period_us > 25000) printf ("\n");
    }
    loop_int_count = 0;
}

class Hotrc {
  protected:
    int32_t* val;
    int32_t avg, min_index, max_index, failsafe_min, failsafe_max;
    int32_t depth = 250, index = 1, padding = 7, calc_count = 0;
    int32_t history[250];  // It will not accept history[depth] - wtf
    uint32_t sum;
    bool detect_ready = false;
  public:
    Hotrc (int32_t* arg_val, int32_t arg_failsafe_min, int32_t arg_failsafe_max, int32_t arg_padding) {
        val = arg_val;
        for (int32_t x = 0; x < depth; x++) history[x] = *val;
        avg = *val;
        sum = avg * depth;
        min_index = 0;
        max_index = 0;
        failsafe_min = arg_failsafe_min;
        failsafe_max = arg_failsafe_max;
        if (arg_padding != -1) padding = arg_padding;
    }
    void set_failsafe (int32_t arg_failsafe_min, int32_t arg_failsafe_max) {
        failsafe_min = arg_failsafe_min - padding;
        failsafe_max = arg_failsafe_max + padding;
    }
    void set_failsafe (void) {
        failsafe_min = history[min_index] - padding;
        failsafe_max = history[max_index] + padding;
    }
    void set_pad (int32_t arg_pad) { padding = arg_pad; } 
    int32_t calc (void) {
        int32_t nextindex = (index+1) % depth;
        sum += *val - history[nextindex];
        avg = sum/depth;
        int32_t save_min = history[min_index];
        int32_t save_max = history[max_index];
        history[nextindex] = *val;
        if (*val <= save_min) min_index = nextindex;
        else if (min_index == nextindex) for (int32_t x = 0; x < depth; x++) if (history[x] < history[min_index]) min_index = x;
        if (*val >= save_max) max_index = nextindex;
        else if (max_index == nextindex) for (int32_t x = 0; x < depth; x++) if (history[x] > history[max_index]) max_index = x;
        if (!detect_ready) if (++calc_count >= depth) detect_ready = true;
        index = nextindex;
        return avg;
    }
    void print (void) { std::cout << "Hotrc:" << history[index] << " avg:" << avg << " min[" << min_index << "]:" << history[min_index] << " max[" << max_index << "]:" << history[max_index] << std::endl; }
    bool connection_lost (void) { return (detect_ready && (history[min_index] < failsafe_min || history[max_index] > failsafe_max)); }
    int32_t get_min (void) { return history[min_index]-padding; }
    int32_t get_max (void) { return history[max_index]+padding; }
    int32_t get_pad (void) { return padding; }
    int32_t get_avg (void) { return avg; }
    int32_t get_failsafe_min (void) { return failsafe_min; }
    int32_t get_failsafe_max (void) { return failsafe_max; }
};



// Display.H
bool flip_the_screen = false;

// string* pagecard = new string[8];  // How we might allocate on the heap instead of in the stack
// string* modecard = new string[7];

char pagecard[8][5] = { "Run ", "Joy ", "Car ", "PWMs", "Bpid", "Gpid", "Cpid", "Temp" };
char modecard[7][7] = { "Basic", "Shutdn", "Stall", "Hold", "Fly", "Cruise", "Cal" };
int32_t colorcard[arraysize(modecard)] = { MGT, RED, ORG, YEL, GRN, CYN, MBLU };
enum dataset_pages { PG_RUN, PG_JOY, PG_CAR, PG_PWMS, PG_BPID, PG_GPID, PG_CPID, PG_TEMP };

char telemetry[disp_fixed_lines][9] = {  
    "   Speed",
    "    Tach",
    "Brk Pres",   
    "Joy Horz",
    "Joy Vert",
    "CruisTgt",
    " Brk Tgt",
    " Gas Tgt",
    " Brk PWM",
    " Gas PWM",
    "SteerPWM",
};
char dataset_page_names[arraysize(pagecard)][disp_tuning_lines][9] = {
    {   " Battery",  // PG_RUN
        " Brk Pos",
        "     Pot",
        "SimBkPos",
        " Sim Joy",
        "Sim Pres",
        "Sim Tach",
        " Sim Spd", },
    {   "Horz Raw",  // PG_JOY
        "Vert Raw",
        "Horz Min",
        "Horz Max",
        " Horz DZ",
        "Vert Min",
        "Vert Max",
        " Vert DZ", },
    {   "Governor",  // PG_CAR
        "Eng Idle",
        "Eng RedL",
        "Spd Idle",
        "Spd RedL",
        "GasOpnLp",
        " Cal Brk",
        " Cal Gas", },
    {   "Str Left",  // PG_PWMS
        "Str Stop",
        "Str Rght",
        "Brk Extd",
        "Brk Stop",
        "Brk Retr",
        "Gas Idle",
        "Gas RedL", },
    {   "Pres Err",  // PG_BPID
        "  P Term",
        "  I Term",
        "  D Term",
        " PID Out",
        "  Kp (P)",
        "  Ki (I)",
        "  Kd (D)", },
    {   " Eng Err",  // PG_GPID
        "  P Term",
        "  I Term",
        "  D Term",
        " PID Out",
        "  Kp (P)",
        "  Ki (I)",
        "  Kd (D)" },
    {   " Spd Err",  // PG_CPID
        "  P Term",
        "  I Term",
        "  D Term",
        " PID Out",
        "  Kp (P)",
        "  Ki (I)",
        "  Kd (D)", },
    {   " Tmp Amb",  // PG_TEMP
        " Tmp Eng",
        "Tmp WhRL",  // "Tmp WhFL",
        "Tmp WhRR",  // "Tmp WhFR",
        "Pres ADC",
        "RadioMin",
        "RadioMax",
        "BrkZeroP", },
};
int32_t tuning_first_editable_line[disp_tuning_lines] = { 3, 2, 0, 0, 5, 5, 5, 4 };  // first value in each dataset page that's editable. All values after this must also be editable
char units[disp_fixed_lines][5] = { "mph ", "rpm ", "psi ", "adc ", "adc ", "mph ", "psi ", "rpm ", "\xe5s  ", "\xe5s  ", "\xe5s  " };
char tuneunits[arraysize(pagecard)][disp_tuning_lines][5] = {
    { "V   ", "in  ", "%   ", "    ", "    ", "    ", "    ", "    " },  // PG_RUN
    { "adc ", "adc ", "adc ", "adc ", "adc ", "adc ", "adc ", "adc " },  // PG_JOY
    { "%   ", "rpm ", "rpm ", "mph ", "mph ", "    ", "    ", "    " },  // PG_CAR
    { "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  " },  // PG_PWMS
    { "psi ", "psi ", "psi ", "psi ", "psi ", "    ", "Hz  ", "sec " },  // PG_BPID
    { "mph ", "mph ", "mph ", "mph ", "mph ", "    ", "Hz  ", "sec " },  // PG_GPID
    { "rpm ", "rpm ", "rpm ", "rpm ", "rpm ", "    ", "Hz  ", "sec " },  // PG_CPID
    { "\x09""F  ", "\x09""F  ", "\x09""F  ", "\x09""F  ", "adc ", "\xe5s  ", "\xe5s  ", "in  " },  // PG_TEMP
    // { "\x09 F ", "\x09 F ", "\x09 F ", "\x09 F ", "\x09 F ", "\x09 F ", "    ", "    " },  // PG_TEMP
};
char simgrid[4][3][5] = {
    { "prs\x18", "rpm\x18", "car\x18" },
    { "prs\x19", "rpm\x19", "car\x19" },
    { "    ", " \x1e  ", "    " },
    { " \x11  ", " \x1f  ", "  \x10 " },  // Font special characters map:  https://learn.adafruit.com/assets/103682
};
char side_menu_buttons[5][4] = { "PAG", "SEL", "\x18  ", "\x19  ", "SIM" };  // Pad shorter names with spaces on the right
char top_menu_buttons[4][6] = { " CAL ", "BASIC", " IGN ", "POWER" };  // Pad shorter names with spaces to center
char disp_values[disp_lines][disp_maxlength+1];  // Holds previously drawn value strings for each line
bool disp_polarities[disp_lines];  // Holds sign of previously drawn values
bool display_enabled = true;  // Should we run 325x slower in order to get bombarded with tiny numbers?  Probably.
bool disp_bool_values[6];
bool disp_selected_val_dirty = true;
bool disp_dataset_page_dirty = true;
bool disp_sidemenu_dirty = true;
bool disp_runmode_dirty = true;
int32_t disp_needles[disp_lines];
int32_t disp_targets[disp_lines];
int32_t disp_age_quanta[disp_lines];
Timer dispAgeTimer[disp_lines];  // int32_t disp_age_timer_us[disp_lines];
Timer dispRefreshTimer (100000);  // Don't refresh screen faster than this (16667us = 60fps, 33333us = 30fps, 66666us = 15fps)
Timer dispResetButtonTimer (500000);  // How long to press esp32 "boot" button before screen will reset and redraw
uint32_t tft_watchdog_timeout = 100000;

// tuning-ui related globals
enum tuning_ctrl_states { OFF, SELECT, EDIT };
int32_t tuning_ctrl = OFF;
int32_t tuning_ctrl_last = OFF;
int32_t dataset_page = PG_RUN;  // Which of the six 8-value dataset pages is currently displayed, and available to edit
int32_t dataset_page_last = PG_TEMP;
int32_t selected_value = 0;  // In the real time tuning UI, which of the editable values (0-7) is selected. -1 for none 
int32_t selected_value_last = 0;
//  ---- tunable ----
Timer tuningCtrlTimer (25000000);  // This times out edit mode after a a long period of inactivity

// touchscreen related
bool touch_now_touched = false;  // Is a touch event in progress
bool touch_longpress_valid = true;
int32_t touch_accel_exponent = 0;  // Will edit values by +/- 2^touch_accel_exponent per touch_period interval
int32_t touch_accel = 1 << touch_accel_exponent;  // Touch acceleration level, which increases the longer you hold. Each edit update chages value by this
int32_t touch_fudge = 0;  // -8
//  ---- tunable ----
int32_t touch_accel_exponent_max = 8;  // Never edit values faster than this. 2^8 = 256 change in value per update
Timer touchPollTimer (35000);  // Timer for regular touchscreen sampling
Timer touchHoldTimer (1000000);  // For timing touch long presses
Timer touchAccelTimer (850000);  // Touch hold time per left shift (doubling) of touch_accel

// run state globals
int32_t shutdown_color = colorcard[SHUTDOWN];

#ifdef CAP_TOUCH
    Adafruit_FT6206 ts;  // 2.8in cap touch panel on tft lcd
#else
    XPT2046_Touchscreen ts (touch_cs_pin, touch_irq_pin);  // 3.2in resistive touch panel on tft lcd
#endif

class Display {
    private:
        // Adafruit_ILI9341 _tft (tft_cs_pin, tft_dc_pin, tft_rst_pin); // LCD screen
        Adafruit_ILI9341 _tft; // LCD screen
        // TFT_eSPI _tft; // LCD screen

        // ILI9341_t3 _tft;
        Timer _tftResetTimer;
        Timer _tftDelayTimer;
        int32_t _timing_tft_reset;
        bool _procrastinate = false, reset_finished = false;
        bool _disp_redraw_all = true;
    public:

        Display (int8_t cs_pin, int8_t dc_pin) : _tft(cs_pin, dc_pin), _tftResetTimer(100000), _tftDelayTimer(3000000), _timing_tft_reset(0){}

        void init() {
            printf ("Init LCD... ");
            yield();
            _tft.begin();
            _tft.setRotation( (flip_the_screen) ? 3 : 1);  // 0: Portrait, USB Top-Rt, 1: Landscape, usb=Bot-Rt, 2: Portrait, USB=Bot-Lt, 3: Landscape, USB=Top-Lt
            printf ("TFT Setrotation: %d\n", flip_the_screen ? 3 : 1);

            for (int32_t lineno=0; lineno <= disp_fixed_lines; lineno++)  {
                disp_age_quanta[lineno] = -1;
                memset (disp_values[lineno], 0, strlen (disp_values[lineno]));
                disp_polarities[lineno] = 1;
            }
            for (int32_t row=0; row<arraysize (disp_bool_values); row++) disp_bool_values[row] = 1;
            for (int32_t row=0; row<arraysize (disp_needles); row++) disp_needles[row] = -5;  // Otherwise the very first needle draw will blackout a needle shape at x=0. Do this offscreen
            for (int32_t row=0; row<arraysize (disp_targets); row++) disp_targets[row] = -5;  // Otherwise the very first target draw will blackout a target shape at x=0. Do this offscreen
            yield();
            _tft.fillScreen (BLK);  // Black out the whole screen
            yield();
            draw_touchgrid (false);
            yield();
            draw_fixed (dataset_page, dataset_page_last, false);
            yield();
            _disp_redraw_all = true;
            printf ("Success.\nTouchscreen initialization... ");
            ts.begin();
            // if (!ts.begin(40)) printf ("Couldn't start touchscreen controller");  // pass in 'sensitivity' coefficient
            printf ("Touchscreen started\n");
        }
        bool tft_reset() {  // call to begin a tft reset, and continue to call every loop until returns true (or get_reset_finished() returns true), then stop
            if (reset_finished) {
                reset_finished = false;
                _timing_tft_reset = 1;
             }
            if (_timing_tft_reset == 1) {
                write_pin (tft_rst_pin, LOW);
                _timing_tft_reset = 2;
            }
            else if (_timing_tft_reset == 2 && _tftResetTimer.expired()) {
                write_pin (tft_rst_pin, HIGH);
                init();
                _timing_tft_reset = 0;
                reset_finished = true;
            }
            return reset_finished;
        }
        void watchdog() {  // Call in every loop to perform a reset upon detection of blocked loops and 
            if (loop_period_us > tft_watchdog_timeout && _timing_tft_reset == 0) _timing_tft_reset = 1;
            if (_timing_tft_reset == 0 || !_tftDelayTimer.expired()) _tftDelayTimer.reset();
            else tft_reset();
        }
        bool get_reset_finished() { return reset_finished; }

        // Functions to write to the screen efficiently
        //
        void draw_bargraph_base (int32_t corner_x, int32_t corner_y, int32_t width) {  // draws a horizontal bargraph scale.  124, y, 40
            _tft.drawFastHLine (corner_x+disp_bargraph_squeeze, corner_y, width-disp_bargraph_squeeze*2, GRY1);
            for (int32_t offset=0; offset<=2; offset++) _tft.drawFastVLine ((corner_x+disp_bargraph_squeeze)+offset*(width/2 - disp_bargraph_squeeze), corner_y-1, 3, WHT);
        }
        void draw_needle_shape (int32_t pos_x, int32_t pos_y, int32_t color) {  // draws a cute little pointy needle
            _tft.drawFastVLine (pos_x-1, pos_y, 2, color);
            _tft.drawFastVLine (pos_x, pos_y, 4, color);
            _tft.drawFastVLine (pos_x+1, pos_y, 2, color);
        }
        void draw_target_shape (int32_t pos_x, int32_t pos_y, int32_t t_color, int32_t r_color) {  // draws a cute little target symbol
            _tft.drawFastVLine (pos_x-1, pos_y+7, 2, t_color);
            _tft.drawFastVLine (pos_x, pos_y+5, 4, t_color);
            _tft.drawFastVLine (pos_x+1, pos_y+7, 2, t_color);
        }
        void draw_bargraph_needle (int32_t n_pos_x, int32_t old_n_pos_x, int32_t pos_y, int32_t n_color) {  // draws a cute little pointy needle
            draw_needle_shape (old_n_pos_x, pos_y, BLK);
            draw_needle_shape (n_pos_x, pos_y, n_color);
        }
        void draw_string (int32_t x_new, int32_t x_old, int32_t y, const char* text, const char* oldtext, int32_t color, int32_t bgcolor, bool forced=false) {  // Send in "" for oldtext if erase isn't needed
            int32_t oldlen = strlen(oldtext);
            int32_t newlen = strlen(text);
            _tft.setTextColor (bgcolor);  
            for (int32_t letter=0; letter < oldlen; letter++) {
                if (newlen - letter < 1) {
                    _tft.setCursor (x_old+disp_font_width*letter, y);
                    _tft.print (oldtext[letter]);
                }
                else if (oldtext[letter] != text[letter]) {
                    _tft.setCursor (x_old+disp_font_width*letter, y);
                    _tft.print (oldtext[letter]);
                }
            }
            _tft.setTextColor (color);  
            for (int32_t letter=0; letter < newlen; letter++) {
                if (oldlen - letter < 1) {
                    _tft.setCursor (x_new+disp_font_width*letter, y);
                    _tft.print (text[letter]);
                }
                else if (oldtext[letter] != text[letter] || forced) {
                    _tft.setCursor (x_new+disp_font_width*letter, y);
                    _tft.print (text[letter]);
                }
            }
        }
        void draw_mmph (int32_t x, int32_t y, int32_t color) {  // This is my cheesy pixel-drawn "mmph" compressed horizontally to 3-char width
            _tft.setTextColor (color);
            _tft.setCursor (x, y);
            _tft.print ("m");
            _tft.setCursor (x+4, y);
            _tft.print ("m");  // Overlapping 'mm' complete (x = 0-8)
            _tft.drawFastVLine (x+10, y+2, 6, color);
            _tft.drawPixel (x+11, y+2, color);
            _tft.drawPixel (x+11, y+6, color);
            _tft.drawFastVLine (x+12, y+3, 3, color);  // 'p' complete (x = 10-12)
            _tft.drawFastVLine (x+14, y, 7, color);
            _tft.drawPixel (x+15, y+2, color);
            _tft.drawFastVLine (x+16, y+3, 4, color);  // 'h' complete (x = 14-16)
        }
        void draw_thou (int32_t x, int32_t y, int32_t color) {  // This is my cheesy pixel-drawn "thou" compressed horizontally to 3-char width
            _tft.drawFastVLine (x+1, y+1, 5, color);
            _tft.drawFastHLine (x, y+2, 3, color);
            _tft.drawPixel (x+2, y+6, color);  // 't' complete (x = 0-2)
            _tft.drawFastVLine (x+4, y, 7, color);
            _tft.drawPixel (x+5, y+3, color);
            _tft.drawPixel (x+6, y+2, color);
            _tft.drawFastVLine (x+7, y+3, 4, color);  // 'h' complete (x = 4-7)
            _tft.drawFastVLine (x+9, y+3, 3, color);
            _tft.drawFastHLine (x+10, y+2, 2, color);
            _tft.drawFastHLine (x+10, y+6, 2, color);
            _tft.drawFastVLine (x+12, y+3, 3, color);  // 'o' complete (x = 9-12)
            _tft.drawFastVLine (x+14, y+2, 4, color);
            _tft.drawPixel (x+15, y+6, color);
            _tft.drawFastVLine (x+16, y+2, 5, color);  // 'u' complete (x = 14-16)
        }
        void draw_string_units (int32_t x, int32_t y, const char* text, const char* oldtext, int32_t color, int32_t bgcolor) {  // Send in "" for oldtext if erase isn't needed
            _tft.setCursor (x, y);
            _tft.setTextColor (bgcolor);
            _tft.print (oldtext);  // Erase the old content
            _tft.setCursor (x, y);
            _tft.setTextColor (color);
            _tft.print (text);  // Erase the old content
        }
        void draw_colons (int32_t x_pos, int32_t first, int32_t last, int32_t color) {
            for (int32_t lineno=first; lineno <= last; lineno++) {
                _tft.drawPixel (x_pos, lineno*disp_line_height_pix+3, color);  // Tiny microscopic colon dots
                _tft.drawPixel (x_pos, lineno*disp_line_height_pix+7, color);  // Tiny microscopic colon dots
                // _tft.fillRect (x_pos, (lineno+1)*disp_line_height_pix+3, 2, 2, color);  // Big goofy looking colon dots
                // _tft.fillRect (x_pos, (lineno+1)*disp_line_height_pix+7, 2, 2, color);  // Big goofy looking colon dots
            }
        }
        // draw_fixed displays 20 rows of text strings with variable names. and also a column of text indicating units, plus boolean names, all in grey.
        void draw_fixed (int32_t page, int32_t page_last, bool redraw_tuning_corner, bool forced=false) {  // set redraw_tuning_corner to true in order to just erase the tuning section and redraw
            yield();  // experiment
            _tft.setTextColor (GRY2);
            _tft.setTextSize (1);
            // if (redraw_tuning_corner) _tft.fillRect(10, 145, 154, 95, BLK); // _tft.fillRect(0,145,167,95,BLK);  // Erase old dataset page area - This line alone uses 15 ms
            int32_t y_pos;
            if (!redraw_tuning_corner) {
                for (int32_t lineno=0; lineno < disp_fixed_lines; lineno++)  {  // Step thru lines of fixed telemetry data
                    y_pos = (lineno+1)*disp_line_height_pix+disp_vshift_pix;
                    draw_string (12, 12, y_pos, telemetry[lineno], "", GRY2, BLK, forced);
                    draw_string_units (104, y_pos, units[lineno], "", GRY2, BLK);
                    draw_bargraph_base (124, y_pos+7, disp_bargraph_width);
                }
                // draw_colons(7+disp_font_width*arraysize(telemetry[0]), 1, disp_fixed_lines+disp_tuning_lines, GRY1);  // I can't decide if I like the colons or not
            }
            for (int32_t lineno=0; lineno < disp_tuning_lines; lineno++)  {  // Step thru lines of dataset page data
                yield();  // experiment
                draw_string(12, 12, (lineno+disp_fixed_lines+1)*disp_line_height_pix+disp_vshift_pix, dataset_page_names[page][lineno], dataset_page_names[page_last][lineno], GRY2, BLK, forced);
                draw_string_units(104, (lineno+disp_fixed_lines+1)*disp_line_height_pix+disp_vshift_pix, tuneunits[page][lineno], tuneunits[page_last][lineno], GRY2, BLK);
                if (redraw_tuning_corner) {
                    int32_t corner_y = (lineno+disp_fixed_lines+1)*disp_line_height_pix+disp_vshift_pix+7;  // lineno*disp_line_height_pix+disp_vshift_pix-1;
                    draw_bargraph_base (124, corner_y, disp_bargraph_width);
                    if (disp_needles[lineno] >= 0) draw_bargraph_needle (-1, disp_needles[lineno], corner_y-6, BLK);  // Let's draw a needle
                }
            }
        }
        void draw_hyphen (int32_t x_pos, int32_t y_pos, int32_t color) {  // Draw minus sign in front of negative numbers
            _tft.drawFastHLine (x_pos+2, y_pos+3, 3, color);
        }
        void draw_dynamic (int32_t lineno, char const* disp_string, int32_t value, int32_t lowlim, int32_t hilim, int32_t target=-1) {
            yield();  // experiment
            int32_t age_us = (int32_t)((double)(dispAgeTimer[lineno].elapsed()) / 2500000); // Divide by us per color gradient quantum
            int32_t x_base = 59;
            bool polarity = (value >= 0);  // polarity 0=negative, 1=positive
            if (strcmp(disp_values[lineno], disp_string) || _disp_redraw_all) {  // If value differs, Erase old value and write new
                int32_t y_pos = lineno*disp_line_height_pix+disp_vshift_pix;
                if (polarity != disp_polarities[lineno]) draw_hyphen (x_base, y_pos, (!polarity) ? GRN : BLK);
                draw_string (x_base+disp_font_width, x_base+disp_font_width, y_pos, disp_string, disp_values[lineno], GRN, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
                strcpy (disp_values[lineno], disp_string);
                disp_polarities[lineno] = polarity;
                dispAgeTimer[lineno].reset();
                disp_age_quanta[lineno] = 0;
            }  // to-do: Fix failure to freshen aged coloration of unchanged characters of changed values
            else if (age_us > disp_age_quanta[lineno] && age_us < 11)  {  // As readings age, redraw in new color. This may fail and redraw when the timer overflows? 
                int32_t color;
                if (age_us < 8) color = 0x1fe0 + age_us*0x2000;  // Base of green with red added as you age, until yellow is achieved
                else color = 0xffe0 - (age_us-8) * 0x100;  // Then lose green as you age further
                int32_t y_pos = (lineno)*disp_line_height_pix+disp_vshift_pix;
                if (!polarity) draw_hyphen (x_base, y_pos, color);
                draw_string (x_base+disp_font_width, x_base+disp_font_width, y_pos, disp_values[lineno], "", color, BLK);
                disp_age_quanta[lineno] = age_us;
            }
            yield();  // experiment
            if (lowlim < hilim) {  // Any value having a given range deserves a bargraph gauge with a needle
                int32_t corner_x = 124;    
                int32_t corner_y = lineno*disp_line_height_pix+disp_vshift_pix-1;
                int32_t n_pos = map (value, lowlim, hilim, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
                int32_t ncolor = (n_pos > disp_bargraph_width-disp_bargraph_squeeze || n_pos < disp_bargraph_squeeze) ? DORG : GRN;
                n_pos = corner_x + constrain (n_pos, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
                if (target != -1) {  // If target value is given, draw a target on the bargraph too
                    int32_t t_pos = map (target, lowlim, hilim, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
                    int32_t tcolor = (t_pos > disp_bargraph_width-disp_bargraph_squeeze || t_pos < disp_bargraph_squeeze) ? DORG : ( (t_pos != n_pos) ? YEL : GRN );
                    t_pos = corner_x + constrain (t_pos, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
                    if (t_pos != disp_targets[lineno] || (t_pos == n_pos)^(disp_needles[lineno] != disp_targets[lineno]) || _disp_redraw_all) {
                        draw_target_shape (disp_targets[lineno], corner_y, BLK, -1);  // Erase old target
                        _tft.drawFastHLine (disp_targets[lineno]-(disp_targets[lineno] != corner_x+disp_bargraph_squeeze), lineno*disp_line_height_pix+disp_vshift_pix+7, 2+(disp_targets[lineno] != corner_x+disp_bargraph_width-disp_bargraph_squeeze), GRY1);  // Patch bargraph line where old target got erased
                        for (int32_t offset=0; offset<=2; offset++) _tft.drawFastVLine ((corner_x+disp_bargraph_squeeze)+offset*(disp_bargraph_width/2 - disp_bargraph_squeeze), lineno*disp_line_height_pix+disp_vshift_pix+6, 3, WHT);  // Redraw bargraph graduations in case one got corrupted by target erasure
                        draw_target_shape (t_pos, corner_y, tcolor, -1);  // Draw the new target
                        disp_targets[lineno] = t_pos;  // Remember position of target
                    }
                }
                if (n_pos != disp_needles[lineno] || _disp_redraw_all) {
                    draw_bargraph_needle (n_pos, disp_needles[lineno], corner_y, ncolor);  // Let's draw a needle
                    disp_needles[lineno] = n_pos;  // Remember position of needle
                }
            }
            else if (disp_needles[lineno] >= 0) {  // If value having no range is drawn over one that did ...
                draw_bargraph_needle (-1, disp_needles[lineno], lineno*disp_line_height_pix+disp_vshift_pix-1, BLK);  // Erase the old needle
                disp_needles[lineno] = -1;  // Flag for no needle
            }
        }
        int32_t significant_place (double value) {  // Returns the decimal place of the most significant digit of a given float value, without relying on logarithm math
            int32_t place = 0;
            if (value >= 1) { // int32_t vallog = std::log10(value);  // Can be sped up
                place = 1;
                while (value >= 10) {
                    value /= 10;
                    place++;
                }
            }
            else if (value) {  // checking (value) rather than (value != 0.0) can help avoid precision errors caused by digital representation of floating numbers
                while (value < 1) {
                    value *= 10;
                    place--;
                }
            }
            return place;
        }
        std::string abs_itoa (int32_t value, int32_t maxlength) {  // returns an ascii string representation of a given integer value, using scientific notation if necessary to fit within given width constraint
            value = abs (value);  // This function disregards sign
            if (significant_place(value) <= maxlength) return std::to_string (value);  // If value is short enough, return it
            std::string result;
            int32_t magnitude = std::log10 (value);  // check how slow is log() function? Compare performance vs. multiple divides ( see abs_ftoa() )
            double scaledValue = value / std::pow (10, magnitude + 1 - maxlength);  // was (10, magnitude - 5);
            if (scaledValue >= 1.0 && scaledValue < 10.0) result = std::to_string (static_cast<int>(scaledValue));
            else result = std::to_string (scaledValue);
            if (magnitude >= maxlength) result += "e" + std::to_string (magnitude);
            return result;
        }
        std::string abs_ftoa (double value, int32_t maxlength, int32_t sigdig) {  // returns an ascii string representation of a given double value, formatted to efficiently fit withinthe given width constraint
            value = abs (value);  // This function disregards sign
            int32_t place = significant_place (value);  // Learn decimal place of the most significant digit in value
            if (place >= sigdig && place <= maxlength) {  // Then we want simple cast to an integer w/o decimal point (eg 123456, 12345, 1234)
                std::string result (std::to_string ((int32_t)value));
                return result;
            }
            if (place >= 0 && place < maxlength) {  // Then we want float formatted with enough nonzero digits after the decimal point for 4 significant digits (eg 123.4, 12.34, 1.234, 0)
                int32_t length = min (sigdig+1, maxlength);
                char buffer[length+1];
                std::snprintf (buffer, length + 1, "%.*g", length - 1, value);
                std::string result (buffer);  // copy buffer to result
                return result;
            }
            if (place >= 3-maxlength && place < maxlength) {  // Then we want decimal w/o initial '0' limited to 3 significant digits (eg .123, .0123, .00123)
                std::string result (std::to_string(value));
                size_t decimalPos = result.find ('.');  // Remove any digits to the left of the decimal point
                if (decimalPos != std::string::npos) result = result.substr (decimalPos);
                if (result.length() > sigdig) result.resize (sigdig+1);  // Limit the string length to the desired number of significant digits
                return result;
            }  // Otherwise we want scientific notation with precision removed as needed to respect maxlength (eg 1.23e4, 1.23e5, but using long e character not e for negative exponents
            char buffer[maxlength+1];  // Allocate buffer with the maximum required size
            int32_t sigdigless = sigdig - 1 - (place <= -10);  // was: if (place <= -10) return std::string ("~0");  // Ridiculously small values just indicate basically zero
            snprintf (buffer, sizeof (buffer), "%*.*f%*d", maxlength-sigdigless, sigdigless, value, maxlength-1, 0);
            std::string result (buffer);  // copy buffer to result
            if (result.find ("e+0") != std::string::npos) result.replace (result.find ("e+0"), 3, "e");  // Remove useless "+0" from exponent
            else if (result.find ("e-0") != std::string::npos) result.replace (result.find ("e-0"), 3, "\x88");  // For very small scientific notation values, replace the "e-0" with a phoenetic long e character, to indicate negative power  // if (result.find ("e-0") != std::string::npos) 
            else if (result.find ("e+") != std::string::npos) result.replace (result.find ("e+"), 3, "e");  // For ridiculously large values
            else if (result.find ("e-") != std::string::npos) result.replace (result.find ("e-"), 3, "\x88");  // For ridiculously small values
            return result;    
        }
        void draw_dynamic (int32_t lineno, int32_t value, int32_t lowlim, int32_t hilim, int32_t target=-1) {
            std::string val_string = abs_itoa (value, (int32_t)disp_maxlength);
            // std::cout << "Int: " << value << " -> " << val_string << ", " << ((value >= 0) ? 1 : -1) << std::endl;
            draw_dynamic (lineno, val_string.c_str(), value, lowlim, hilim, (int32_t)target);
        }
        void draw_dynamic (int32_t lineno, double value, double lowlim, double hilim, int32_t target=-1) {
            std::string val_string = abs_ftoa (value, (int32_t)disp_maxlength, 3);
            // std::cout << "Flt: " << value << " -> " << val_string << ", " << ((value >= 0) ? 1 : -1) << std::endl;
            draw_dynamic (lineno, val_string.c_str(), (int32_t)value, (int32_t)lowlim, (int32_t)hilim, target);
        }
        void draw_dynamic (int32_t lineno, double value, double lowlim, double hilim, double target) {
            draw_dynamic (lineno, value, lowlim, hilim, (int32_t)target);
        }
        void draw_runmode (int32_t runmode, int32_t oldmode, int32_t color_override=-1) {  // color_override = -1 uses default color
            yield();
            int32_t color = (color_override == -1) ? colorcard[runmode] : color_override;
            int32_t x_new = 8+6*(2+strlen (modecard[runmode]))-3;
            int32_t x_old = 8+6*(2+strlen (modecard[oldmode]))-3;
            draw_string (8+6, 8+6, disp_vshift_pix, modecard[oldmode], "", BLK, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
            draw_string (x_old, x_old, disp_vshift_pix, "Mode", "", BLK, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
            draw_string (8+6, 8+6, disp_vshift_pix, modecard[runmode], "", color, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
            draw_string (x_new, x_new, disp_vshift_pix, "Mode", "", color, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
        }
        void draw_dataset_page (int32_t page, int32_t page_last, bool forced=false) {
            draw_fixed (page, page_last, true, forced);  // Erase and redraw dynamic data corner of screen with names, units etc.
            // for (int32_t lineno=0; lineno<disp_lines; lineno++) draw_hyphen (59, lineno*disp_line_height_pix+disp_vshift_pix, BLK);
            yield();
            draw_string (83, 83, disp_vshift_pix, pagecard[page], pagecard[page_last], RBLU, BLK, forced); // +6*(arraysize(modecard[runmode])+4-namelen)/2
        }
        void draw_selected_name (int32_t tun_ctrl, int32_t tun_ctrl_last, int32_t selected_val, int32_t selected_last) {
            yield();
            if (selected_val != selected_last) draw_string (12, 12, 12+(selected_last+disp_fixed_lines)*disp_line_height_pix+disp_vshift_pix, dataset_page_names[dataset_page][selected_last], "", GRY2, BLK);
            draw_string (12, 12, 12+(selected_val+disp_fixed_lines)*disp_line_height_pix+disp_vshift_pix, dataset_page_names[dataset_page][selected_val], "", (tun_ctrl == EDIT) ? GRN : ((tun_ctrl == SELECT) ? YEL : GRY2), BLK);
        }
        void draw_bool (bool value, int32_t col) {  // Draws values of boolean data
            if ((disp_bool_values[col-2] != value) || _disp_redraw_all) {  // If value differs, Erase old value and write new
                int32_t x_mod = touch_margin_h_pix + touch_cell_h_pix*(col) + (touch_cell_h_pix>>1) - arraysize (top_menu_buttons[col-2]-1)*(disp_font_width>>1) - 2;
                draw_string (x_mod, x_mod, 0, top_menu_buttons[col-2], "", (value) ? GRN : LGRY, DGRY);
                disp_bool_values[col-2] = value;
            }
        }
        void draw_simbuttons (bool create) {  // draw grid of buttons to simulate sensors. If create is true it draws buttons, if false it erases them
            _tft.setTextColor (LYEL);
            for (int32_t row = 0; row < arraysize(simgrid); row++) {
                for (int32_t col = 0; col < arraysize(simgrid[row]); col++) {
                    yield();
                    int32_t cntr_x = touch_margin_h_pix + touch_cell_h_pix*(col+3) + (touch_cell_h_pix>>1) +2;
                    int32_t cntr_y = touch_cell_v_pix*(row+1) + (touch_cell_v_pix>>1);
                    if (strcmp (simgrid[row][col], "    " )) {
                        _tft.fillCircle (cntr_x, cntr_y, 19, create ? DGRY : BLK);
                        if (create) {
                            _tft.drawCircle (cntr_x, cntr_y, 19, LYEL);
                            int32_t x_mod = cntr_x-(arraysize (simgrid[row][col])-1)*(disp_font_width>>1);
                            draw_string (x_mod, x_mod, cntr_y-(disp_font_height>>1), simgrid[row][col], "", LYEL, DGRY);
                        }
                    }
                }     
            }
        }
        void draw_touchgrid (bool side_only) {  // draws edge buttons with names in 'em. If replace_names, just updates names
            int32_t namelen = 0;
            _tft.setTextColor (WHT);
            for (int32_t row = 0; row < arraysize (side_menu_buttons); row++) {  // Step thru all rows to draw buttons along the left edge
                yield();
                _tft.fillRoundRect (-9, touch_cell_v_pix*row+3, 18, touch_cell_v_pix-6, 8, DGRY);
                _tft.drawRoundRect (-9, touch_cell_v_pix*row+3, 18, touch_cell_v_pix-6, 8, LYEL);
                namelen = 0;
                for (uint32_t x = 0 ; x < arraysize (side_menu_buttons[row]) ; x++ ) {
                    if (side_menu_buttons[row][x] != ' ') namelen++; // Go thru each button name. Need to remove spaces padding the ends of button names shorter than 4 letters 
                }
                for (int32_t letter = 0; letter < namelen; letter++) {  // Going letter by letter thru each button name so we can write vertically 
                    yield();
                    _tft.setCursor (1, ( touch_cell_v_pix*row) + (touch_cell_v_pix/2) - (int32_t)(4.5*((double)namelen-1)) + (disp_font_height+1)*letter); // adjusts vertical offset depending how many letters in the button name and which letter we're on
                    _tft.println (side_menu_buttons[row][letter]);  // Writes each letter such that the whole name is centered vertically on the button
                }
            }
            if (!side_only) {
                for (int32_t col = 2; col <= 5; col++) {  // Step thru all cols to draw buttons across the top edge
                    yield();
                    _tft.fillRoundRect (touch_margin_h_pix + touch_cell_h_pix*(col) + 3, -9, touch_cell_h_pix-6, 18, 8, DGRY);
                    _tft.drawRoundRect (touch_margin_h_pix + touch_cell_h_pix*(col) + 3, -9, touch_cell_h_pix-6, 18, 8, LYEL);  // _tft.width()-9, 3, 18, (_tft.height()/5)-6, 8, LYEL);
                    // draw_bool (top_menu_buttons[btn], btn+3);
                }
            }
        }

        void update() {
            if (simulating != simulating_last || _disp_redraw_all) {
                draw_simbuttons(simulating);  // if we just entered simulator draw the simulator buttons, or if we just left erase them
                simulating_last = simulating;
                _procrastinate = true;  // Waits till next loop to draw changed values
            }
            if ((disp_dataset_page_dirty || _disp_redraw_all)) {
                static bool first = true;
                draw_dataset_page(dataset_page, dataset_page_last, first);
                first = false;
                disp_dataset_page_dirty = false;
                if (dataset_page_last != dataset_page) config.putUInt("dpage", dataset_page);
                dataset_page_last = dataset_page;
                _procrastinate = true;  // Waits till next loop to draw changed values
            }
            if ((disp_sidemenu_dirty || _disp_redraw_all)) {
                draw_touchgrid(true);
                disp_sidemenu_dirty = false;
                _procrastinate = true;  // Waits till next loop to draw changed values
            }
            if (disp_selected_val_dirty || _disp_redraw_all) {
                draw_selected_name(tuning_ctrl, tuning_ctrl_last, selected_value, selected_value_last);
                disp_selected_val_dirty = false;
                selected_value_last = selected_value;
                tuning_ctrl_last = tuning_ctrl; // Make sure this goes after the last comparison
            }
            if (disp_runmode_dirty || _disp_redraw_all) {
                draw_runmode(runmode, oldmode, (runmode == SHUTDOWN) ? shutdown_color : -1);
                disp_runmode_dirty = false;
                oldmode = runmode;  // remember what mode we're in for next time
            }
            if ((dispRefreshTimer.expired() && !_procrastinate) || _disp_redraw_all) {
                dispRefreshTimer.reset();
                double drange;
                draw_dynamic(1, speedo_filt_mph, 0.0, speedo_redline_mph, cruiseSPID.get_target());
                draw_dynamic(2, tach_filt_rpm, 0.0, tach_redline_rpm, gasSPID.get_target());
                draw_dynamic(3, pressure_filt_psi, pressure_min_psi, pressure_max_psi, brakeSPID.get_target());  // (brake_active_pid == S_PID) ? (int32_t)brakeSPID.get_target() : pressure_target_adc);
                draw_dynamic(4, ctrl_pos_adc[HORZ][FILT], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);
                draw_dynamic(5, ctrl_pos_adc[VERT][FILT], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
                draw_dynamic(6, cruiseSPID.get_target(), 0.0, speedo_govern_mph);
                draw_dynamic(7, brakeSPID.get_target(), pressure_min_psi, pressure_max_psi);
                draw_dynamic(8, gasSPID.get_target(), 0.0, tach_redline_rpm);
                draw_dynamic(9, (int32_t)brake_pulse_out_us, brake_pulse_retract_us, brake_pulse_extend_us);
                draw_dynamic(10, gas_pulse_out_us, gas_pulse_redline_us, gas_pulse_idle_us);
                draw_dynamic(11, steer_pulse_out_us, steer_pulse_right_us, steer_pulse_left_us);
                if (dataset_page == PG_RUN) {
                    draw_dynamic(12, battery_filt_v, 0.0, battery_max_v);
                    draw_dynamic(13, brake_pos_filt_in, brake_pos_nom_lim_retract_in, brake_pos_nom_lim_extend_in);
                    draw_dynamic(14, pot_filt_percent, pot_min_percent, pot_max_percent);
                    // draw_dynamic (14, brakeSPID.get_proportionality(), -1, -1);
                    draw_dynamic(15, sim_brkpos, -1, -1);
                    draw_dynamic(16, sim_joy, -1, -1);
                    draw_dynamic(17, sim_pressure, -1, -1);
                    draw_dynamic(18, sim_tach, -1, -1);
                    draw_dynamic(19, sim_speedo, -1, -1);
                }
                else if (dataset_page == PG_JOY) {
                    draw_dynamic(12, ctrl_pos_adc[HORZ][RAW], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);
                    draw_dynamic(13, ctrl_pos_adc[VERT][RAW], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
                    draw_dynamic(14, ctrl_lims_adc[ctrl][HORZ][MIN], 0, (adcrange_adc-ctrl_lims_adc[ctrl][HORZ][MAX])/2);
                    draw_dynamic(15, ctrl_lims_adc[ctrl][HORZ][MAX], (ctrl_lims_adc[ctrl][HORZ][MIN]-adcrange_adc)/2, adcrange_adc);
                    draw_dynamic(16, ctrl_lims_adc[ctrl][HORZ][DB], 0, (adcmidscale_adc - ctrl_lims_adc[ctrl][HORZ][MIN] > ctrl_lims_adc[ctrl][HORZ][MAX] - adcmidscale_adc) ? 2*(ctrl_lims_adc[ctrl][HORZ][MAX] - adcmidscale_adc) : 2*(adcmidscale_adc - ctrl_lims_adc[ctrl][HORZ][MIN]));
                    draw_dynamic(17, ctrl_lims_adc[ctrl][VERT][MIN], 0, (adcrange_adc-ctrl_lims_adc[ctrl][VERT][MAX])/2);
                    draw_dynamic(18, ctrl_lims_adc[ctrl][VERT][MAX], (ctrl_lims_adc[ctrl][VERT][MIN]-adcrange_adc)/2, adcrange_adc);
                    draw_dynamic(19, ctrl_lims_adc[ctrl][VERT][DB], 0, (adcmidscale_adc - ctrl_lims_adc[ctrl][VERT][MIN] > ctrl_lims_adc[ctrl][VERT][MAX] - adcmidscale_adc) ? 2*(ctrl_lims_adc[ctrl][VERT][MAX] - adcmidscale_adc) : 2*(adcmidscale_adc - ctrl_lims_adc[ctrl][VERT][MIN]));
                }
                else if (dataset_page == PG_CAR) {
                    draw_dynamic(12, gas_governor_percent, 0, 100);
                    draw_dynamic(13, tach_idle_rpm, 0.0, tach_redline_rpm);
                    draw_dynamic(14, tach_redline_rpm, 0.0, tach_max_rpm);
                    draw_dynamic(15, speedo_idle_mph, 0.0, speedo_redline_mph);
                    draw_dynamic(16, speedo_redline_mph, 0.0, speedo_max_mph);
                    draw_dynamic(17, gasSPID.get_open_loop(), -1, -1);
                    // draw_dynamic(17, ctrl, -1, -1);  // 0 if hotrc
                    draw_dynamic(18, cal_joyvert_brkmotor, -1, -1);
                    draw_dynamic(19, cal_pot_gasservo, -1, -1);
                }
                else if (dataset_page == PG_PWMS) {
                    draw_dynamic(12, steer_pulse_left_us, steer_pulse_stop_us, steer_pulse_left_max_us);
                    draw_dynamic(13, steer_pulse_stop_us, steer_pulse_left_us, steer_pulse_right_us);
                    draw_dynamic(14, steer_pulse_right_us, steer_pulse_right_min_us, steer_pulse_stop_us);
                    draw_dynamic(15, brake_pulse_extend_us, brake_pulse_stop_us, brake_pulse_extend_max_us);
                    draw_dynamic(16, brake_pulse_stop_us, brake_pulse_retract_us, brake_pulse_extend_us);
                    draw_dynamic(17, brake_pulse_retract_us, brake_pulse_retract_min_us, brake_pulse_stop_us);
                    draw_dynamic(18, gas_pulse_idle_us, gas_pulse_cw_min_us, gas_pulse_ccw_max_us);
                    draw_dynamic(19, gas_pulse_redline_us, gas_pulse_cw_min_us, gas_pulse_ccw_max_us);
                }
                else if (dataset_page == PG_BPID) {
                    drange = brake_pulse_extend_us-brake_pulse_retract_us;
                    draw_dynamic(12, brakeSPID.get_error(), pressure_min_psi-pressure_max_psi, pressure_max_psi-pressure_min_psi);
                    draw_dynamic(13, brakeSPID.get_p_term(), -drange, drange);
                    draw_dynamic(14, brakeSPID.get_i_term(), -drange, drange);
                    draw_dynamic(15, brakeSPID.get_d_term(), -drange, drange);
                    draw_dynamic(16, brakeSPID.get_output(), (double)brake_pulse_retract_us, (double)brake_pulse_extend_us);  // brake_spid_speedo_delta_adc, -range, range);
                    draw_dynamic(17, brakeSPID.get_kp(), 0.0, 2.0);
                    draw_dynamic(18, brakeSPID.get_ki_hz(), 0.0, 2.0);
                    draw_dynamic(19, brakeSPID.get_kd_s(), 0.0, 2.0);
                }
                else if (dataset_page == PG_GPID) {
                    drange = gas_pulse_idle_us-gas_pulse_govern_us;
                    draw_dynamic(12, gasSPID.get_error(), tach_idle_rpm-tach_govern_rpm, tach_govern_rpm-tach_idle_rpm);
                    draw_dynamic(13, gasSPID.get_p_term(), -drange, drange);
                    draw_dynamic(14, gasSPID.get_i_term(), -drange, drange);
                    draw_dynamic(15, gasSPID.get_d_term(), -drange, drange);
                    draw_dynamic(16, gasSPID.get_output(), (double)gas_pulse_idle_us, (double)gas_pulse_govern_us);  // gas_spid_speedo_delta_adc, -drange, drange);
                    draw_dynamic(17, gasSPID.get_kp(), 0.0, 2.0);
                    draw_dynamic(18, gasSPID.get_ki_hz(), 0.0, 2.0);
                    draw_dynamic(19, gasSPID.get_kd_s(), 0.0, 2.0);
                }
                else if (dataset_page == PG_CPID) {
                    drange = tach_govern_rpm-tach_idle_rpm;
                    draw_dynamic(12, cruiseSPID.get_error(), speedo_idle_mph-speedo_govern_mph, speedo_govern_mph-speedo_idle_mph);
                    draw_dynamic(13, cruiseSPID.get_p_term(), -drange, drange);
                    draw_dynamic(14, cruiseSPID.get_i_term(), -drange, drange);
                    draw_dynamic(15, cruiseSPID.get_d_term(), -drange, drange);
                    draw_dynamic(16, cruiseSPID.get_output(), tach_idle_rpm, tach_govern_rpm);  // cruise_spid_speedo_delta_adc, -drange, drange);
                    draw_dynamic(17, cruiseSPID.get_kp(), 0.0, 2.0);
                    draw_dynamic(18, cruiseSPID.get_ki_hz(), 0.0, 2.0);
                    draw_dynamic(19, cruiseSPID.get_kd_s(), 0.0, 2.0);
                }
                else if (dataset_page == PG_TEMP) {
                    draw_dynamic(12, temps[AMBIENT], temp_min, temp_max);
                    draw_dynamic(13, temps[ENGINE], temp_min, temp_max);
                    // draw_dynamic(14, temps[WHEEL_FL], temp_min, temp_max);
                    // draw_dynamic(15, temps[WHEEL_FR], temp_min, temp_max);
                    draw_dynamic(14, temps[WHEEL_RL], temp_min, temp_max);
                    draw_dynamic(15, temps[WHEEL_RR], temp_min, temp_max);
                    draw_dynamic(16, pressure_adc, pressure_min_adc, pressure_max_adc);
                    draw_dynamic(17, hotrc_pos_failsafe_min_adc, ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
                    draw_dynamic(18, hotrc_pos_failsafe_max_adc, ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
                    draw_dynamic(19, brake_pos_zeropoint_in, brake_pos_nom_lim_retract_in, brake_pos_nom_lim_extend_in);   
                }
                draw_bool((runmode == CAL), 2);
                draw_bool((runmode == BASIC), 3);
                draw_bool(ignition, 4);
                draw_bool(syspower, 5);
            }
            _procrastinate = false;
            _disp_redraw_all = false;
        }
};


// ctrl.H
class Encoder {
    private:
        // class vars
        //  ---- tunable ----
        static const uint32_t _spinrate_min_us = 2500;  // Will reject spins faster than this as an attempt to debounce behavior
        static const uint32_t _accel_thresh_us = 100000;  // Spins faster than this will be accelerated
        static const int32_t _accel_max = 50;  // Maximum acceleration factor
        static const uint32_t _longPressTime = 800000;

        // instance vars
        uint8_t _sw_pin;
        int32_t _state = 0;
        int32_t _sw_action = NONE;  // Flag for encoder handler to know an encoder switch action needs to be handled
        uint32_t _spinrate_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
        uint32_t _spinrate_last_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
        uint32_t _spinrate_old_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
        bool _sw = false;  // Remember whether switch is being pressed
        bool _timer_active = false;  // Flag to prevent re-handling long presses if the sw is just kept down
        bool _suppress_click = false;  // Flag to prevent a short click on switch release after successful long press
        //  ---- tunable ----
        Timer _longPressTimer;  // Used to time long button presses

    public:
        enum sw_presses { NONE, SHORT, LONG };
        enum _inputs { ENC_A, ENC_B };

        volatile uint32_t _spinrate_isr_us = 100000;  // Time elapsed between last two detents
        volatile bool _a_stable = true;  //  Stores the value of encoder A pin as read during B pin transition (where A is stable)
        volatile int32_t _bounce_danger = ENC_B;  // Which of the encoder A or B inputs is currently untrustworthy due to bouncing 
        volatile int32_t _delta = 0;  // Keeps track of un-handled rotary clicks of the encoder.  Positive for CW clicks, Negative for CCW. 

        uint8_t _a_pin;
        uint8_t _b_pin;
        void (*_a_isr)();
        void (*_b_isr)();
        Timer _spinspeedTimer;  // Used to figure out how fast we're spinning the knob.  OK to not be volatile?

        Encoder(uint8_t a, uint8_t b, uint8_t sw) : _a_pin(a), _b_pin(b), _sw_pin(sw), _longPressTimer(_longPressTime){}
        
        void setLongPressTimer(uint32_t t){
            _longPressTimer.set(t);
        }
    
        void setup() {
            attachInterrupt(digitalPinToInterrupt(_a_pin), _a_isr, CHANGE); \
            attachInterrupt(digitalPinToInterrupt(_b_pin), _b_isr, CHANGE);
        }

        void update() {
            // Encoder - takes 10 us to read when no encoder activity
            // Read and interpret encoder switch activity. Encoder rotation is handled in interrupt routine
            // Encoder handler routines should act whenever encoder_sw_action is SHORT or LONG, setting it back to
            // NONE once handled. When handling press, if encoder_long_clicked is nonzero then press is a long press
            if (!read_pin(_sw_pin)) {  // if encoder sw is being pressed (switch is active low)
                if (!_sw) {  // if the press just occurred
                    _longPressTimer.reset();  // start a press timer
                    _timer_active = true;  // flag to indicate timing for a possible long press
                }
                else if (_timer_active && _longPressTimer.expired()) {  // If press time exceeds long press threshold
                    _sw_action = LONG;  // Set flag to handle the long press event. Note, routine handling press should clear this
                    _timer_active = false;  // Keeps us from entering this logic again until after next sw release (to prevent repeated long presses)
                    _suppress_click = true;  // Prevents the switch release after a long press from causing a short press
                }
                _sw = true;  // Remember a press is in effect
            }
            else {  // if encoder sw is not being pressed
                if (_sw && !_suppress_click) _sw_action = SHORT;  // if the switch was just released, a short press occurred, which must be handled
                _timer_active = false;  // Allows detection of next long press event
                _sw = false;  // Remember press is not in effect
                _suppress_click = false;  // End click suppression
            }
        }

        uint32_t handleSwitchAction() {
            uint32_t ret = _sw_action;
            _sw_action = NONE;
            return ret;
        }

        uint32_t handleSelection() {
            uint32_t d = 0;
            if (_delta) {  // Now handle any new rotations
                if (_spinrate_isr_us >= _spinrate_min_us) {  // Reject clicks coming in too fast as bounces
                    _spinrate_old_us = _spinrate_last_us;  // Store last few spin times for filtering purposes ...
                    _spinrate_last_us = _spinrate_us;  // ...
                    _spinrate_us = constrain (_spinrate_isr_us, _spinrate_min_us, _accel_thresh_us);
                    d = constrain (_delta, -1, 1);  // Only change one at a time when selecting or turning pages
                }
                _delta = 0;  // Our responsibility to reset this flag after handling events
            }
            return d;
        }

        uint32_t handleTuning() {
            uint32_t d = 0;
            if (_delta) {  // Handle any new rotations
                if (_spinrate_isr_us >= _spinrate_min_us) {  // Reject clicks coming in too fast as bounces
                    _spinrate_old_us = _spinrate_last_us;  // Store last few spin times for filtering purposes ...
                    _spinrate_last_us = _spinrate_us;  // ...
                    _spinrate_us = constrain(_spinrate_isr_us, _spinrate_min_us, _accel_thresh_us);
                    int32_t _temp = (_spinrate_old_us > _spinrate_last_us) ? _spinrate_old_us : _spinrate_last_us;  // Find the slowest of the last 3 detents ...
                    if (_temp < _spinrate_us) _temp = _spinrate_us;
                    _temp = map (_temp, _spinrate_min_us, _accel_thresh_us, _accel_max, 1);  // if turning faster than 100ms/det, proportionally accelerate the effect of each detent by up to 50x. encoder_temp variable repurposed here to hold # of edits per detent turned
                    d = _delta * _temp;  // If a tunable value is being edited, turning the encoder changes the value
                }
                _delta = 0;  // Our responsibility to reset this flag after handling events
            }
            return d;
        }
};

