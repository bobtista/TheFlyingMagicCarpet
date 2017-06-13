/* CarpetLedLogic.ino
 *
 *   This is the main entry point for the carpet's LED system.
 *
 *   Author: Anders Linn
 *   Date: June 2017
 */

#include <DmxSimple.h>
#include <FastLED.h>
#include "CRBGW.h"
#include "LegacyRoutines.h"

#define NUM_DMX_LEDS 2
#define NUM_RGB_LEDS 30
#define NUM_RGBW_LEDS 30
#define NUM_THEORETICAL_RGBW_LEDS ( NUM_RGBW_LEDS + ( NUM_RGBW_LEDS / 3 ) )
#define DMX_DATA_PIN 3
#define RGB_DATA_PIN 7 // 6
#define RGBW_DATA_PIN 6 // 7

// rope demo arrays
CRGBW * rgbwleds;
CRGB * convertedrgbwleds;

void robeDemoSetup() {
   static CRGBW rgbw[NUM_RGBW_LEDS];
   static CRGB convrgbw[NUM_THEORETICAL_RGBW_LEDS];
   rgbwleds = rgbw;
   convertedrgbwleds = convrgbw;
   // this needs to be passed the raw led array, we'll handle to conversion elsewhere
   FastLED.addLeds<NEOPIXEL, RGBW_DATA_PIN>( convertedrgbwleds, NUM_THEORETICAL_RGBW_LEDS);
   for ( int i = 0; i < NUM_RGBW_LEDS; ++i ) {
     rgbwleds[i] = CRGB::Red;
   }
   FastLED.show();
}

void robeDemo() {
  static int loc = 0;
  static int end = NUM_RGBW_LEDS;
  static int flip = 0;
  static CRGB clr(rand()%255,rand()%255,rand()%255);
  static CRGB oldclr = CRGB::Red;
  if ( end == 0 ) {
    end = NUM_RGBW_LEDS;
    if ( flip == 0 ) {
      flip = -1;
    } else {
      flip = 0;
    }
    oldclr = clr;
    clr = CRGB(rand()%255,rand()%255,rand()%255);
    return;
  }
  for ( int i = 0; i < NUM_RGBW_LEDS; ++i ) {
    if ( i == loc ) {
      rgbwleds[i] = clr;
      break;
    } else {
      rgbwleds[i] = oldclr;
    }
  }
  ++loc;
  loc %= end;
  if ( loc == 0 ) {
    --end;
  }
  convertNeopixelRgbwArray( rgbwleds, convertedrgbwleds, NUM_RGBW_LEDS, NUM_THEORETICAL_RGBW_LEDS );
  FastLED.show(); 
  FastLED.delay( 50 );
}

void setup() {
   // dmxSetup();
   robeDemoSetup();
}

void loop() {
   // dmxLoop();
   robeDemo();
}
