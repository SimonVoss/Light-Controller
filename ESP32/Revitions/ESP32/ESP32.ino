

#include <FastLED.h>
#include "BluetoothSerial.h"


#define LED_PIN 2    
#define NUM_LEDS 30  
#define AUDIO_PIN 4  
#define UPDATES_PER_SECOND 150
#define BRIGHTNESS 255
CRGB leds[NUM_LEDS];

// init Class:
BluetoothSerial ESP_BT;


CRGBPalette16 currentPalette;
TBlendType currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;


// Parameters for Bluetooth interface
int incoming;
static int button = 0;
static int value = 0;


int s = 0;
int r = 152;
int g = 0;
int b = 10;


//Timer function to work
unsigned long previousMillis[2] = { 0 };
const long interval[3] = { 300, 100, 25 };

bool Strope_State = 0;
unsigned long currentMillis = 0;

void setup() {

  ESP_BT.begin("VossBoss_ESP32_Control");  //Name of your Bluetooth interface -> will show up on your phone


  currentPalette = RainbowColors_p;
  currentBlending = LINEARBLEND;

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
  Serial.begin(115200);
  pinMode(AUDIO_PIN, INPUT);
}
void loop() {
  unsigned long currentMillis = millis();
  Serial.print("Button :");
  Serial.println(button);


  // -- -- -- -- -- -- -- -- -- --Receive Bluetooth signal-- -- -- -- -- -- -- -- -- -- --
  if (ESP_BT.available()) {
    BT_Read();
  }

  // -- -- -- -- -- -- -- -- -- --Selecting mode to play-- -- -- -- -- -- -- -- -- -- --
  switch (button) {
    case 0:
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(0, 0, 0);
      }
      FastLED.show();
      break;

    case 1:
      running();
      break;

    case 2:
      Strope();
      break;

    case 3:
      ChangePalettePeriodically();
      static uint8_t startIndex = 0;
      startIndex = startIndex + 1; /* motion speed */
      FillLEDsFromPaletteColors(startIndex);
      FastLED.show();
      FastLED.delay(1000 / UPDATES_PER_SECOND);
      break;

    case 4:
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(0, 255, 0);
      }
      FastLED.show();
      break;
  }
}


void Strope() {
  unsigned long currentMillis = millis();
  //Turning LEDS on
  if (currentMillis - previousMillis[1] >= interval[1] && Strope_State == 0) {  //if time is out and the LEDS are currently off
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(255, 255, 255);
    }
    FastLED.show();
    Strope_State = 1;
    previousMillis[1] = currentMillis;
    Serial.println("On");
  }


  //Turning LEDS off

  if (currentMillis - previousMillis[2] >= interval[2] && Strope_State == 1) {  //if time is out and the LEDS are currently off
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show();
    Strope_State = 0;
    previousMillis[2] = currentMillis;
    Serial.println("Off");
  }
}

void BT_Read() {
  incoming = ESP_BT.read();  //Read what we receive
  // separate button ID from button value -> button ID is 10, 20, 30, etc, value is 1 or 0
  button = floor(incoming / 10);
  value = incoming % 10;
  if (value == 0) {
    button = 0;
  }
}


void running() {

  s = analogRead(AUDIO_PIN);
  s = s * 2;
  Serial.println(s);

  if (currentMillis - previousMillis[3] >= interval[3]) {  //if there have gone more than the interval the funktion is executed
    if ((s >= 450) && (s <= 550)) {
      leds[(NUM_LEDS / 2) - 1] = CRGB(0, 0, 255);
      leds[NUM_LEDS / 2] = CRGB(0, 0, 255);
    } else if ((s >= 400) && (s <= 450)) {
      leds[(NUM_LEDS / 2) - 1] = CRGB(153, 153, 0);
      leds[NUM_LEDS / 2] = CRGB(153, 153, 0);
    } else if ((s >= 350) && (s <= 400)) {
      leds[(NUM_LEDS / 2) - 1] = CRGB(255, 50, 255);
      leds[NUM_LEDS / 2] = CRGB(255, 50, 255);
    } else if ((s >= 300) && (s <= 350)) {
      leds[(NUM_LEDS / 2) - 1] = CRGB(10, 25, 217);
      leds[NUM_LEDS / 2] = CRGB(10, 25, 217);
    }

    else if ((s >= 276) && (s <= 300)) {
      leds[(NUM_LEDS / 2) - 1] = CRGB(50, 50, 150);
      leds[NUM_LEDS / 2] = CRGB(50, 50, 150);
    } else if ((s >= 250) && (s <= 275)) {
      leds[(NUM_LEDS / 2) - 1] = CRGB(230, 0, 10);
      leds[NUM_LEDS / 2] = CRGB(230, 0, 10);
    } else if ((s >= 235) && (s <= 250)) {
      leds[(NUM_LEDS / 2) - 1] = CRGB(0, 160, 0);
      leds[NUM_LEDS / 2] = CRGB(0, 160, 0);
    } else if ((s >= 200) && (s <= 230)) {
      leds[(NUM_LEDS / 2) - 1] = CRGB(1, 0, 1);
      leds[NUM_LEDS / 2] = CRGB(1, 0, 1);
    } else {
      leds[(NUM_LEDS / 2) - 1] = CRGB(r, s - 100, b);
      leds[NUM_LEDS / 2] = CRGB(r, s - 100, b);
    }
    for (int i = 0; i <= ((NUM_LEDS / 2) - 2); i++) {
      leds[i] = leds[i + 1];
      leds[NUM_LEDS - 1 - i] = leds[(NUM_LEDS)-i - 2];
    }
  }

  FastLED.show();
  //delay(25);
}


void FillLEDsFromPaletteColors(uint8_t colorIndex) {
  uint8_t brightness = 255;

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = ColorFromPalette(currentPalette, colorIndex, brightness, currentBlending);
    colorIndex += 3;
  }
}


// There are several different palettes of colors demonstrated here.
//
// FastLED provides several 'preset' palettes: RainbowColors_p, RainbowStripeColors_p,
// OceanColors_p, CloudColors_p, LavaColors_p, ForestColors_p, and PartyColors_p.
//
// Additionally, you can manually define your own color palettes, or you can write
// code that creates color palettes on the fly.  All are shown here.

void ChangePalettePeriodically() {
  uint8_t secondHand = (millis() / 1000) % 60;
  static uint8_t lastSecond = 99;

  if (lastSecond != secondHand) {
    lastSecond = secondHand;
    if (secondHand == 0) {
      currentPalette = RainbowColors_p;
      currentBlending = LINEARBLEND;
    }
    if (secondHand == 10) {
      currentPalette = RainbowStripeColors_p;
      currentBlending = NOBLEND;
    }
    if (secondHand == 15) {
      currentPalette = RainbowStripeColors_p;
      currentBlending = LINEARBLEND;
    }
    if (secondHand == 20) {
      SetupPurpleAndGreenPalette();
      currentBlending = LINEARBLEND;
    }
    if (secondHand == 25) {
      SetupTotallyRandomPalette();
      currentBlending = LINEARBLEND;
    }
    if (secondHand == 30) {
      SetupBlackAndWhiteStripedPalette();
      currentBlending = NOBLEND;
    }
    if (secondHand == 35) {
      SetupBlackAndWhiteStripedPalette();
      currentBlending = LINEARBLEND;
    }
    if (secondHand == 40) {
      currentPalette = CloudColors_p;
      currentBlending = LINEARBLEND;
    }
    if (secondHand == 45) {
      currentPalette = PartyColors_p;
      currentBlending = LINEARBLEND;
    }
    if (secondHand == 50) {
      currentPalette = myRedWhiteBluePalette_p;
      currentBlending = NOBLEND;
    }
    if (secondHand == 55) {
      currentPalette = myRedWhiteBluePalette_p;
      currentBlending = LINEARBLEND;
    }
  }
}

// This function fills the palette with totally random colors.
void SetupTotallyRandomPalette() {
  for (int i = 0; i < 16; i++) {
    currentPalette[i] = CHSV(random8(), 255, random8());
  }
}

// This function sets up a palette of black and white stripes,
// using code.  Since the palette is effectively an array of
// sixteen CRGB colors, the various fill_* functions can be used
// to set them up.
void SetupBlackAndWhiteStripedPalette() {
  // 'black out' all 16 palette entries...
  fill_solid(currentPalette, 16, CRGB::Black);
  // and set every fourth one to white.
  currentPalette[0] = CRGB::White;
  currentPalette[4] = CRGB::White;
  currentPalette[8] = CRGB::White;
  currentPalette[12] = CRGB::White;
}

// This function sets up a palette of purple and green stripes.
void SetupPurpleAndGreenPalette() {
  CRGB purple = CHSV(HUE_PURPLE, 255, 255);
  CRGB green = CHSV(HUE_GREEN, 255, 255);
  CRGB black = CRGB::Black;

  currentPalette = CRGBPalette16(
    green, green, black, black,
    purple, purple, black, black,
    green, green, black, black,
    purple, purple, black, black);
}


// This example shows how to set up a static color palette
// which is stored in PROGMEM (flash), which is almost always more
// plentiful than RAM.  A static PROGMEM palette like this
// takes up 64 bytes of flash.
const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM = {
  CRGB::Red,
  CRGB::Gray,  // 'white' is too bright compared to red and blue
  CRGB::Blue,
  CRGB::Black,

  CRGB::Red,
  CRGB::Gray,
  CRGB::Blue,
  CRGB::Black,

  CRGB::Red,
  CRGB::Red,
  CRGB::Gray,
  CRGB::Gray,
  CRGB::Blue,
  CRGB::Blue,
  CRGB::Black,
  CRGB::Black
};



// Additional notes on FastLED compact palettes:
//
// Normally, in computer graphics, the palette (or "color lookup table")
// has 256 entries, each containing a specific 24-bit RGB color.  You can then
// index into the color palette using a simple 8-bit (one byte) value.
// A 256-entry color palette takes up 768 bytes of RAM, which on Arduino
// is quite possibly "too many" bytes.
//
// FastLED does offer traditional 256-element palettes, for setups that
// can afford the 768-byte cost in RAM.
//
// However, FastLED also offers a compact alternative.  FastLED offers
// palettes that store 16 distinct entries, but can be accessed AS IF
// they actually have 256 entries; this is accomplished by interpolating
// between the 16 explicit entries to create fifteen intermediate palette
// entries between each pair.
//
// So for example, if you set the first two explicit entries of a compact
// palette to Green (0,255,0) and Blue (0,0,255), and then retrieved
// the first sixteen entries from the virtual palette (of 256), you'd get
// Green, followed by a smooth gradient from green-to-blue, and then Blue.
