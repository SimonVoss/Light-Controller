/*
Software written by Simon Voss on behaf of CTU
This software is for a handmade lightcontroller, that simplifies controlling of 
ALL lights in the slyngelstue.

Last updated: 05-01-2024


Improvements
Include Mini moving heads
Improve computing by making display more efficient
Create more advaned light scenarioes
*/


#include <DmxSimple.h>
#include <FastLED.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "PinChangeInterrupt.h"

//WS2812B LED initialicering
#define LED_TYPE WS2812B
#define COLOR_ORDER RGB

#define DMX_Pin 2

#define LED_Pin1 3
#define LED_Pin2 4
#define LED_Pin3 5
#define LED_Pin4 6
#define LED_Pin5 7

#define NUM_LEDS1 146  // 0-1 Ile
#define NUM_LEDS2 146  // 0-2 Ile
#define NUM_LEDS3 324  // 0-3 Ile
#define NUM_LEDS4 450  // Disp
#define NUM_LEDS5 900  // Dancefloor

#define BRIGHTNESS0 255  //Max Strip Brightness
#define BRIGHTNESS1 255  //Max Strip Brightness
#define BRIGHTNESS2 255  //Max Strip Brightness
#define BRIGHTNESS3 255  //Max Strip Brightness
#define BRIGHTNESS4 255  //Max Strip Brightness

#define UPDATES_PER_SECOND 150  //Strip Lights

CRGB leds1[NUM_LEDS1];
CRGB leds2[NUM_LEDS2];
CRGB leds3[NUM_LEDS3];
CRGB leds4[NUM_LEDS4];
CRGB leds5[NUM_LEDS5];

CRGBPalette16 currentPalette;
TBlendType currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

//DMX initialicering
//Kanaler
//First parameter is the light the second is the channel

int const A = 63;  //FloodPanel150      Set to 8 channels
int const B = 12;  //LedBar2408         Set to 5 channels
int const C = 20;  //MovingHeadMH710    Set to 9 channels
int const D = 30;  //TriLED_Bar         Set to 7 channels
int const E = 39;  //UV600              Set to 4 channels
int const F = 50;  //MiniMovingHead     Set to 11 channels
int const G = 0;   //SpotCameo          Set to 9 channels
int const H = 75;  //MiniDekker         Set to 0 Channels



uint16_t FloodPanel150[4][9] = {
  { 0, A, 1 + A, 2 + A, 3 + A, 4 + A, 5 + A, 6 + A, 7 + A },
  { 0, 1 + A, 2 + A, 3 + A, 4 + A, 5 + A, 6 + A, 7 + A, 8 + A },
  { 0, 1 + A, 2 + A, 3 + A, 4 + A, 5 + A, 6 + A, 7 + A, 8 + A },
  { 0, 1 + A, 2 + A, 3 + A, 4 + A, 5 + A, 6 + A, 7 + A, 8 + A }
};


uint16_t LedBar2408[3][6] = {
  { 0, B, 1 + B, 2 + B, 3 + B, 4 + B },
  { 0, 1 + B, 2 + B, 3 + B, 4 + B, 5 + B },
  { 0, 1 + B, 2 + B, 3 + B, 4 + B, 5 + B }
};


uint16_t MovingHeadMH710[3][10] = {
  { 0, C, 1 + C, 2 + C, 3 + C, 4 + C, 5 + C, 6 + C, 7 + C, 8 + C },
  { 0, 1 + C, 2 + C, 3 + C, 4 + C, 5 + C, 6 + C, 7 + C, 8 + C, 9 + C },
  { 0, 1 + C, 2 + C, 3 + C, 4 + C, 5 + C, 6 + C, 7 + C, 8 + C, 9 + C }
};


uint16_t TriLED_Bar[2][8] = {
  { 0, D, 1 + D, 2 + D, 3 + D, 4 + D, 5 + D, 6 + D },
  { 0, 1 + D, 2 + D, 3 + D, 4 + D, 5 + D, 6 + D, 7 + D }
};


uint16_t UV600[2][5] = {
  { 0, E, 1 + E, 2 + E, 3 + E },
  { 0, 1 + E, 2 + E, 3 + E, 4 + E }
};


uint16_t MiniMovingHead[2][13] = {
  { 0, F, 1 + F, 2 + F, 3 + F, 4 + F, 5 + F, 6 + F, 7 + F, 8 + F, 9 + F, 10 + F, 11 + F },
  { 0, 1 + F, 2 + F, 3 + F, 4 + F, 5 + F, 6 + F, 7 + F, 8 + F, 9 + F, 10 + F, 11 + F, 12 + F }
};


uint16_t SpotCameo[2][10] = {
  { 0, G, 1 + G, 2 + G, 3 + G, 4 + G, 5 + G, 6 + G, 7 + G, 8 + G },
  { 0, 1 + G, 2 + G, 3 + G, 4 + G, 5 + G, 6 + G, 7 + G, 8 + G, 9 + G }
};


uint16_t MiniDekker[2][10] = {
  { 0, H, 1 + H, 2 + H, 3 + H, 4 + H, 5 + H, 6 + H, 7 + H, 8 + H },
  { 0, 1 + H, 2 + H, 3 + H, 4 + H, 5 + H, 6 + H, 7 + H, 8 + H, 9 + H }
};



uint8_t Button[13] = { 0, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 43 };                                                   //Inputs
bool ButtonState[12] = { HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH };                              //Sates
bool LastButtonState[15] = { HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH };  //Sates
/*
Button 0 is for All Auto sound reactive, all lights on
Button 1 is for strope, set RGB to what strope light you desire. Setting RGB to max will enable white light
Button 2 is for constant collor
Button 3 is for dissabling the light

Button 11 is for the rotary encoder
*/


// Interrupts Pins that can be used are 2, 3, 18, 19, (20, 21) https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
uint8_t Scroller[5] = { 0, 2, 3, 18, 19 };  //Inputs

//Rotary encoder
bool ScrollerCLKState[2] = { HIGH, HIGH };             //States
bool LastScrollerCLKState[4];                          //Sates
bool ScrollerDTState[4] = { HIGH, HIGH, HIGH, HIGH };  //States
int aState;
int aLastState;
int counter = 0;



//uint8_t PotentiometerPin[6] = { 0, A0, A1, A2, A3, A4 };  //Inputs
uint8_t PotentiometerPin[6] = { 0, A10, A8, A12, A9, A14 };  //Inputs
uint16_t Potentiometer[6] = { 0, 0, 0, 0, 0, 0 };            //Value
uint16_t LastPotentiometer[6] = { 0, 0, 0, 0, 0, 0 };        //Value
int Wheel_Scroller = 0;


//Display initialicering
LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
int DisplayUpdateInterval = 150;
int DisplayUpdateCounter = 0;
bool ClearScreen = HIGH;


//Generalle initialiceringer
uint8_t Wheel_Dimmer = 255;  //Color  dimmer
int Wheel_Time = 255;        //


uint32_t Wheel_Red = 255;           //Color value of global red
uint32_t Wheel_Red_Previous = 0;    //Previous color value of global red
uint32_t Wheel_Green = 255;         //Color value of global green
uint32_t Wheel_Green_Previous = 0;  //Previous color value of global Green
uint32_t Wheel_Blue = 255;          //Color value of global blue
uint32_t Wheel_Blue_Previous = 0;   //Previous color value of global Blue


int Dimmer_FloodPanel150 = 150;
int Dimmer_MovingHeadMH710 = 255;
int Dimmer_LedBar2408 = 255;
int Dimmer_TriLED_Bar = 255;
int Dimmer_UV600 = 255;
int Dimmer_MiniMovingHead = 255;
int Dimmer_SpotCameo = 255;
int Dimmer_MiniDekker = 255;

//Timer function to work
unsigned long previousMillis[10] = { 0 };
const long interval[6] = { 1, 5, 10, 15, 20, 30 };  // interval at which to
int currentState = 0;



void setup() {

  //Serial Comunication
  Serial.begin(115200);


  //DMX initialicering
  DmxSimple.usePin(DMX_Pin);
  DmxSimple.maxChannel(512);  //Max channels

  // initialize the lcd
  lcd.init();


  // Pinmode setup til LED Outputs
  pinMode(LED_Pin1, OUTPUT);
  pinMode(LED_Pin2, OUTPUT);
  pinMode(LED_Pin3, OUTPUT);
  pinMode(LED_Pin4, OUTPUT);
  pinMode(LED_Pin5, OUTPUT);




  for (int i = 1; i < 15; i++) {
    pinMode(Button[i], INPUT_PULLUP);
  }

  /*
  for (int i = 1; i < 3; i++) {
    pinMode(Scroller[i], INPUT);
  }
  */


  // Enable pull-up resistors on encoder pins
  digitalWrite(Scroller[3], HIGH);
  digitalWrite(Scroller[4], HIGH);

  //Interrupts for the rotary encoder
  //attachInterrupt(digitalPinToInterrupt(Scroller[3]), scroller, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(Scroller[4]), scroller, CHANGE);


  //Start screen
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("---------CTU--------");
  lcd.setCursor(7, 1);
  lcd.print("Welcome");
  lcd.setCursor(2, 2);
  lcd.print("Light Controller");
  lcd.setCursor(0, 3);
  lcd.print("Design by  Voss Boss");


  //Fast LED initialiceringer
  FastLED.addLeds<WS2812B, LED_Pin1, RGB>(leds1, NUM_LEDS1);  //LED Linje 1
  FastLED.addLeds<WS2812B, LED_Pin2, RGB>(leds2, NUM_LEDS2);  //LED Linje 2
  FastLED.addLeds<WS2812B, LED_Pin3, RGB>(leds3, NUM_LEDS3);  //LED Linje 3
  //FastLED.addLeds<WS2812B, LED_Pin4, RGB>(leds4, NUM_LEDS4); //LED Linje 4
  //FastLED.addLeds<WS2812B, LED_Pin5, RGB>(leds5, NUM_LEDS5); //LED Linje 5
  //FastLED.setBrightness(BRIGHTNESS);
  currentPalette = RainbowColors_p;
  currentBlending = LINEARBLEND;
  //Bootup delay
  delay(2500);
}


void loop() {
  unsigned long currentMillis = millis();

  //Updating inputs
  updateButtons();
  updatePotentiometer();
  //Update mode based on input buttons
  Mode();

  /*if (currentMillis - previousMillis[9] >= 400) {  //Wheel_Time
    previousMillis[9] = currentMillis;
    Display(1);
  }*/
  Display(1);

  //LED_StripCall(4);

  //Serial.println("test");
  //Service
  SerialPrintMaintenence(0);
}



void Mode() {
  unsigned long currentMillis = millis();

  if (1) {  //Dissabling all other modes than the one pressed
    if (digitalRead(Button[1]) == LOW) {
      ButtonState[2] = HIGH;
      ButtonState[3] = HIGH;
      ButtonState[6] = HIGH;
      ButtonState[7] = HIGH;
      ButtonState[8] = HIGH;
    }
    if (digitalRead(Button[2]) == LOW) {
      ButtonState[1] = HIGH;
      ButtonState[3] = HIGH;
      ButtonState[6] = HIGH;
      ButtonState[7] = HIGH;
      ButtonState[8] = HIGH;
    }
    if (digitalRead(Button[3]) == LOW) {
      ButtonState[2] = HIGH;
      ButtonState[1] = HIGH;
      ButtonState[6] = HIGH;
      ButtonState[7] = HIGH;
      ButtonState[8] = HIGH;
    }
    if (digitalRead(Button[6]) == LOW) {
      ButtonState[2] = HIGH;
      ButtonState[3] = HIGH;
      ButtonState[1] = HIGH;
      ButtonState[7] = HIGH;
      ButtonState[8] = HIGH;
    }
    if (digitalRead(Button[7]) == LOW) {
      ButtonState[2] = HIGH;
      ButtonState[3] = HIGH;
      ButtonState[6] = HIGH;
      ButtonState[1] = HIGH;
      ButtonState[8] = HIGH;
    }
    if (digitalRead(Button[8]) == LOW) {
      ButtonState[2] = HIGH;
      ButtonState[3] = HIGH;
      ButtonState[6] = HIGH;
      ButtonState[7] = HIGH;
      ButtonState[1] = HIGH;
    }
  }

  //Enabeling sound enabeld funktion for all lights
  if (ButtonState[1] == LOW) {

    MiniMovingHeadCall(0);
    TriLED_BarCall(0);
    //LED_StripCall(0);
    //SpotCameoCall(3);
    //LedBar2408Call5(0);
    FloodPanel150Call(0);
    UV600Call(3);
    MiniDekkerCall(4);
  }
  if (LastButtonState[1] == LOW) {  // Disables strope correctly
    FloodPanel150Call(3);
    TriLED_BarCall(3);
    //SpotCameoCall(3);
    updateButtons();
  }


  //Enabeling strope lights, can be controlled by the potentiometers with color and speed
  if (ButtonState[2] == LOW) {

    MiniMovingHeadCall(3);
    FloodPanel150Call(3);
    TriLED_BarCall(1);

    //LED_StripCall(1);
    //SpotCameoCall(3);
    //LedBar2408Call5(1);
    UV600Call(4);
    //MiniDekkerCall(1);
  }
  if (LastButtonState[2] == LOW) {  // Disables strope correctly
    FloodPanel150Call(3);
    TriLED_BarCall(3);
    //SpotCameoCall(3);
    UV600Call(3);
    updateButtons();
  }


  //Enabeling Wheel funktion and makes the color knobs tunable with constant light
  if (ButtonState[3] == LOW) {

    MiniMovingHeadCall(3);
    TriLED_BarCall(2);
    //LED_StripCall(2);
    //SpotCameoCall(4);
    //LedBar2408Call5(2);
    FloodPanel150Call(2);
    MiniDekkerCall(2);
    UV600Call(3);
  }

  if (LastButtonState[3] == LOW) {  // Disables Light correctly
    FloodPanel150Call(3);
    TriLED_BarCall(3);
    //SpotCameoCall(4);
    //LedBar2408Call5(3);
    updateButtons();
  }


  //Disabling dance area except UV light
  if (ButtonState[4] == LOW) {
    SpotCameoCall(4);
  }
  if (ButtonState[4] == HIGH) {
    SpotCameoCall(3);
  }



  if (ButtonState[5] == LOW) {
    Dimmer_MiniMovingHead = 0;
  }
  if (LastButtonState[5] == HIGH) {
    Dimmer_MiniMovingHead = 255;
  }

  if (ButtonState[6] == LOW) {
    //Enabeling

    //MiniMovingHeadCall(3);
    TriLED_BarCall(5);
    //LED_StripCall(4);
    //SpotCameoCall(3);
    //LedBar2408Call5(2);
    FloodPanel150Call(4);
    //MiniDekkerCall(2);
    UV600Call(3);
  }

  if (LastButtonState[6] == LOW) {  // Disables Light correctly
    FloodPanel150Call(3);
    TriLED_BarCall(3);
    //SpotCameoCall(3);
    //LedBar2408Call5(3);
    updateButtons();
  }

  if (ButtonState[7] == LOW) {
    //Enabeling
    FloodPanel150Call(3);
    TriLED_BarCall(4);
    UV600Call(4);
  }
  if (LastButtonState[7] == LOW) {
    //Disabling
    TriLED_BarCall(3);
    UV600Call(3);
  }


  //Disabling dance area except UV light
  if (ButtonState[8] == LOW) {
    MiniMovingHeadCall(3);
    TriLED_BarCall(3);
    //LED_StripCall(4);
    //SpotCameoCall(3);
    //LedBar2408Call5(2);
    FloodPanel150Call(3);
    MiniDekkerCall(2);
    UV600Call(3);
  }


  //Disabling dance area except UV light
  if (ButtonState[9] == LOW) {
    Dimmer_FloodPanel150 = 0;
    Dimmer_TriLED_Bar = 0;
    Dimmer_MiniMovingHead = 0;
    Dimmer_UV600 = 0;
    Dimmer_MiniDekker = 0;
  }

  //Enabling dance area except UV light
  if (ButtonState[9] == HIGH) {
    Dimmer_FloodPanel150 = 255;
    Dimmer_TriLED_Bar = 255;
    Dimmer_MiniMovingHead = 255;
    Dimmer_UV600 = 255;
    Dimmer_MiniDekker = 255;
  }

  //Toggeling LED Bar 2408
  if (ButtonState[10] == LOW) {
    Dimmer_LedBar2408 = 0;
    LED_StripCall(3);
    DmxSimple.write(13, 0);    //TEMP
    DmxSimple.write(14, 220);  //TEMP
  }
  if (ButtonState[10] == HIGH) {
    Dimmer_LedBar2408 = 255;
    LED_StripCall(4);
    DmxSimple.write(13, 80);   //TEMP
    DmxSimple.write(14, 220);  //TEMP
  }
}



void Display(int mode) {
  unsigned long currentMillis = millis();
  int update = 0;
  for (int i = 0; i <= 11; i++) {
    if (ButtonState[i] != LastButtonState[i]) {
      update = 1;
    }
  }
  for (int i = 0; i <= 6; i++) {
    if (LastPotentiometer[i] > Potentiometer[i] - 3 || LastPotentiometer[i] < Potentiometer[i] + 3){
      update = 1;
    }
  }
  if (update == 1) {
    lcd.clear();
    switch (mode) {
      case 0:
        lcd.setCursor(0, 0);
        lcd.print("---------CTU--------");
        lcd.setCursor(7, 1);
        lcd.print("Welcome");
        lcd.setCursor(2, 2);
        lcd.print("Light Controller");
        lcd.setCursor(0, 3);
        lcd.print("Design by  Voss Boss");
        break;
      case 1:

        if (ButtonState[1] == LOW) {
          lcd.setCursor(7, 0);
          lcd.print("Sound");
        }
        if (ButtonState[2] == LOW) {
          lcd.setCursor(7, 0);
          lcd.print("Strope");
        }
        if (ButtonState[3] == LOW) {
          lcd.setCursor(4, 0);
          lcd.print("Static Color");
        }

        if (ButtonState[4] == LOW) {
          lcd.setCursor(14, 1);
          lcd.print("Glb:H");
        }
        if (ButtonState[4] == HIGH) {
          lcd.setCursor(14, 1);
          lcd.print("Glb:L");
        }

        if (ButtonState[5] == LOW) {
          lcd.setCursor(7, 3);
          lcd.print("Mov:L");
        }
        if (ButtonState[5] == HIGH) {
          lcd.setCursor(7, 3);
          lcd.print("Mov:H");
        }

        if (ButtonState[6] == LOW) {
          lcd.setCursor(3, 0);
          lcd.print("Rotating  Show");
        }

        if (ButtonState[7] == LOW) {
          lcd.setCursor(1, 0);
          lcd.print("Fixed White Strobe");
        }

        if (ButtonState[9] == LOW) {
          lcd.setCursor(1, 3);
          lcd.print("DJ:L");
        }
        if (ButtonState[9] == HIGH) {
          lcd.setCursor(1, 3);
          lcd.print("DJ:H");
        }

        if (ButtonState[10] == LOW) {
          lcd.setCursor(14, 3);
          lcd.print("Bar:L");
        }
        if (ButtonState[10] == HIGH) {
          lcd.setCursor(14, 3);
          lcd.print("Bar:H");
        }

        if (ButtonState[1] == HIGH && ButtonState[2] == HIGH && ButtonState[3] == HIGH && ButtonState[6] == HIGH && ButtonState[7] == HIGH /*&& ButtonState[8] == HIGH*/) {
          lcd.setCursor(0, 0);
          lcd.print("Design by  Voss Boss");
        }
        lcd.setCursor(1, 2);
        lcd.print("R:");
        lcd.print(Wheel_Red);
        lcd.setCursor(7, 2);
        lcd.print("G:");
        lcd.print(Wheel_Green);
        lcd.setCursor(14, 2);
        lcd.print("B:");
        lcd.print(Wheel_Blue);

        lcd.setCursor(1, 1);
        lcd.print("T:");
        lcd.print(Wheel_Time);

        lcd.setCursor(7, 1);
        lcd.print("D:");
        lcd.print(Wheel_Dimmer);
        break;
    }
  }
}


//If the time has elaped 1 is return if not 0
int Wait(int intervalArray, int previousMillisArray) {
  unsigned long currentMillis = millis();  // Get the current time
  // Check if it's time to switch states
  if (currentMillis - previousMillis[previousMillisArray] >= interval[intervalArray]) {
    // Save the current time
    previousMillis[previousMillisArray] = currentMillis;

    // Toggle between state 1 and state 2
    currentState = 1 - currentState;

    // Update the output pins based on the current state
    if (currentState == 0) {
      return 1;
    } else {
      return 0;
    }
  }
}

void SerialPrintMaintenence(int input) {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis[2] >= interval[1] * 70) {  //Wheel_Time
    // Save the last time the LED state was toggled
    previousMillis[2] = currentMillis;
    //Buttons
    Serial.print("Buttons:: ");
    for (int i = 1; i < 12; i++) {
      Serial.print(i);
      Serial.print(" = ");
      Serial.print(ButtonState[i]);
      Serial.print(", ");
    }
    Serial.println("");

    //Potentiometer
    Serial.print("Wheels:: ");
    Serial.print("Red: ");
    Serial.print(Wheel_Red);
    Serial.print(" Green: ");
    Serial.print(Wheel_Green);
    Serial.print(" Blue: ");
    Serial.print(Wheel_Blue);
    Serial.print(" Time: ");
    Serial.print(Wheel_Time);
    Serial.print(" Wheel_Dimmer: ");
    Serial.print(Wheel_Dimmer);
    Serial.print(" Wheel_Scroller: ");
    Serial.print(counter);
    Serial.println("");
  }
}

/*
void scroller(){
  aState = digitalRead(Scroller[3]); // Reads the "current" state of the Scroller[3]
   // If the previous and the current state of the Scroller[3] are different, that means a Pulse has occured
   if (aState != aLastState){     
     // If the Scroller[4] state is different to the Scroller[3] state, that means the encoder is rotating clockwise
     if (digitalRead(Scroller[4]) != aState) { 
       counter += 2;
       Serial.println("højre");
     } else {
       counter --;
       Serial.println("venstre");
     }
     //Serial.print("Position: ");
     //Serial.println(counter);
   } 
   aLastState = aState; // Updates the previous state of the Scroller[3] with the current state
}

void updateWheelTEST3() {
  // Read the current state of CLK
  // If last and current state of CLK are different, then we can be sure that the pulse occurred

    //RED
    if (digitalRead(Scroller[3]) == HIGH && digitalRead(Scroller[4]) == LOW) {
      // Encoder is rotating clockwise so we increese the color value
      Wheel_Scroller++;
    }
    if (digitalRead(Scroller[3]) == LOW && digitalRead(Scroller[4]) == HIGH) {
      // Encoder is rotating counterclockwise so we increese the color value
      Wheel_Scroller--;
    }
  
  Serial.print("Wheel_Scroller: ");
  Serial.println(Wheel_Scroller);
}

void updateWheelTEST3() {
  // Read the current state of CLK
  ScrollerCLKState[1] = digitalRead(Scroller[1]);
  // If last and current state of CLK are different, then we can be sure that the pulse occurred
  if (ScrollerCLKState[1] == 0) {
    //RED
    if (digitalRead(Scroller[2]) != ScrollerCLKState[1] && digitalRead(Scroller[3]) == HIGH && digitalRead(Scroller[4]) == HIGH) {
      // Encoder is rotating clockwise so we increese the color value

      Wheel_Scroller++;
    }
    if (digitalRead(Scroller[2]) == ScrollerCLKState[1] && digitalRead(Scroller[3]) == HIGH && digitalRead(Scroller[4]) == HIGH) {
      // Encoder is rotating counterclockwise so we increese the color value
      Wheel_Scroller--;
    }
  }
}

void updateWheelTEST3() {
  // Read the current state of the encoder pins
  boolean readingA = digitalRead(Scroller[2]);
  boolean readingB = digitalRead(Scroller[3]);

  // Check for state changes
  if (Scroller[2] != aSet || Scroller[3] != bSet) {
    // Update the encoder position based on the direction of rotation
    if (Scroller[2] == Scroller[3]) {
      encoderPosition++;
    } else {
      encoderPosition--;
    }

    // Store the current state for the next iteration
    aSet = Scroller[2];
    bSet = Scroller[3];

    // You can add your own code here to handle the encoder position change
    // For example, print the position to the Serial Monitor
    Serial.print("Encoder Position: ");
    Serial.println(encoderPosition);
  }
}
*/




/*
All funktions for lights is so that input:
0 - Auto based on music
1 - Strope
2 - Static light
3 - Lights off if not dimmer is set to offLedBar2408Call
4 and onwards is other user programmed setups
*/

//Set to 8 channels
void FloodPanel150Call(int input) {
  Dimmer_FloodPanel150 = Dimmer_FloodPanel150 * Wheel_Dimmer;
  Dimmer_FloodPanel150 = Dimmer_FloodPanel150 / 255;
  switch (input) {
    case 0:  //Auto sound reactive
      //Sound reactive
      //channel 6 = 250
      DmxSimple.write(FloodPanel150[1][6], 255);                   //Sound control
      DmxSimple.write(FloodPanel150[1][1], Dimmer_FloodPanel150);  //Dimmer
      DmxSimple.write(FloodPanel150[1][2], Wheel_Time);
      break;
    case 1:
      for (int i = 0; i < 4; i++) {  //Setting all the FloodPanel150 lights to the same
        //Strope
        DmxSimple.write(FloodPanel150[i][1], Dimmer_FloodPanel150);  //Dimmer
        DmxSimple.write(FloodPanel150[i][6], 0);                     //Initialicer for input colors to work
        DmxSimple.write(FloodPanel150[i][5], Wheel_Time);            //Strope Speed 0 Slow 255 Fast
        DmxSimple.write(FloodPanel150[i][2], Wheel_Red);             //Red Color
        DmxSimple.write(FloodPanel150[i][3], Wheel_Green);           //Green Color
        DmxSimple.write(FloodPanel150[i][4], Wheel_Blue);            //Blue Color
      }
      break;
    case 2:
      for (int i = 0; i < 5; i++) {  //Setting all the FloodPanel150 lights to the same
        //Constant collors
        //channel 6 = 0
        DmxSimple.write(FloodPanel150[i][1], Dimmer_FloodPanel150);  //Dimmer
        DmxSimple.write(FloodPanel150[i][2], Wheel_Red);             //Color Red
        DmxSimple.write(FloodPanel150[i][3], Wheel_Green);           //Color Green
        DmxSimple.write(FloodPanel150[i][4], Wheel_Blue);            //Color Blue
        DmxSimple.write(FloodPanel150[i][6], 0);                     //Initialicer for input colors to work
      }
      break;
    case 3:  //LED OFF
      for (int i = 0; i < 6; i++) {
        DmxSimple.write(FloodPanel150[i][5], 0);  //Strope Speed 0 Slow 255 Fast
        DmxSimple.write(FloodPanel150[i][1], 0);  //Dimmer
        DmxSimple.write(FloodPanel150[i][6], 0);  //Initialicer for input colors to work
      }
      break;
    case 4:
      for (int i = 0; i < 5; i++) {  //Setting all the FloodPanel150 lights to the same
        //Constant collors
        //channel 6 = 0
        DmxSimple.write(FloodPanel150[1][1], Dimmer_FloodPanel150);  //Dimmer
        DmxSimple.write(FloodPanel150[1][6], 150);                   //
      }
      break;
  }
}

//Set to 5 channels
void LedBar2408Call5(int input) {
  Dimmer_LedBar2408 = Dimmer_LedBar2408 * Wheel_Dimmer;
  Dimmer_LedBar2408 = Dimmer_LedBar2408 / 255;
  switch (input) {
    case 1:                          //Strope
      for (int i = 0; i < 5; i++) {  //Setting all the LedBar2408 lights to the same
        DmxSimple.write(LedBar2408[i][1], Wheel_Red);
        DmxSimple.write(LedBar2408[i][2], Wheel_Green);
        DmxSimple.write(LedBar2408[i][3], Wheel_Blue);
        DmxSimple.write(LedBar2408[i][4], Dimmer_LedBar2408);
        DmxSimple.write(LedBar2408[i][5], Wheel_Time);
      }
      break;
    case 2:  //Constant collors
      for (int i = 0; i < 5; i++) {
        DmxSimple.write(LedBar2408[i][1], Wheel_Red);
        DmxSimple.write(LedBar2408[i][2], Wheel_Green);
        DmxSimple.write(LedBar2408[i][3], Wheel_Blue);
        DmxSimple.write(LedBar2408[i][4], Dimmer_LedBar2408);
        DmxSimple.write(LedBar2408[i][5], 0);
      }
      break;
    case 3:                          //LED OFF
      for (int i = 0; i < 5; i++) {  //Setting all the LedBar2408 lights to the same
        DmxSimple.write(LedBar2408[i][4], 0);
        DmxSimple.write(LedBar2408[i][5], 0);
      }
      break;
  }
}

//Set to 2 channels
void LedBar2408Call2(int input) {
  switch (input) {
    case 0:                          //Auto sound reactive
      for (int i = 0; i < 5; i++) {  //Setting all the LedBar2408 lights to the same
        //Sound reactive channel 1 = 255
        DmxSimple.write(LedBar2408[i][1], 255);         //Sound reactive
        DmxSimple.write(LedBar2408[i][2], Wheel_Time);  //Sound sensitivity
      }
      break;
    case 1:                          //White
      for (int i = 0; i < 5; i++) {  //Setting all the LedBar2408 lights to the same
        DmxSimple.write(LedBar2408[i][1], 56);
        if (Wheel_Dimmer < 125) {
          DmxSimple.write(LedBar2408[i][1], 0);  //
        }
      }
      break;
    case 2:  //Constant collors
      for (int i = 0; i < 5; i++) {
        if (Wheel_Dimmer < 125) {
          DmxSimple.write(LedBar2408[i][1], 0);
        }
      }
      /*
      if (Wheel_Dimmer > 125) {
        for (int i = 0; i < 5; i++) {  //Setting all the LedBar2408 lights to the same
          //Single collor
          if (Wheel_Red > 125 && Wheel_Green < 125 && Wheel_Blue < 125) {
            DmxSimple.write(LedBar2408[i][1], 8);
          }
          if (Wheel_Green > 125 && Wheel_Red < 125 && Wheel_Blue < 125) {
            DmxSimple.write(LedBar2408[i][1], 24);
          }
          if (Wheel_Blue > 125 && Wheel_Red < 125 && Wheel_Green < 125) {
            DmxSimple.write(LedBar2408[i][1], 40);
          }
          //Duel collor
          if (Wheel_Red > 125 && Wheel_Green > 125 && Wheel_Blue < 125) {
            DmxSimple.write(LedBar2408[i][1], 16);
          }
          if (Wheel_Red > 125 && Wheel_Blue > 125 && Wheel_Green < 125) {
            DmxSimple.write(LedBar2408[i][1], 48);
          }
          if (Wheel_Green > 125 && Wheel_Blue > 125 && Wheel_Red < 125) {
            DmxSimple.write(LedBar2408[i][1], 32);
          }
          //All collors
          if (Wheel_Red > 125 && Wheel_Green > 125 && Wheel_Blue > 125) {
            DmxSimple.write(LedBar2408[i][1], 56);
          }
          //off
          if (Wheel_Red < 125 && Wheel_Green < 125 && Wheel_Blue < 125) {
            DmxSimple.write(LedBar2408[i][1], 0);
          }
        }
      }
      */
      break;
    case 3:                                    //LED OFF
      for (int i = 0; i < 5; i++) {            //Setting all the LedBar2408 lights to the same
        DmxSimple.write(LedBar2408[i][1], 0);  //
      }
      break;
    case 4:                                    //LED OFF
      for (int i = 0; i < 5; i++) {            //Setting all the LedBar2408 lights to the same
        DmxSimple.write(LedBar2408[i][1], 0);  //
        DmxSimple.write(LedBar2408[i][1], Wheel_Time);
      }
      break;
  }
}

//Set to 9 channels
void MovingHeadMH710Call(int mode) {
  Dimmer_MovingHeadMH710 = Dimmer_MovingHeadMH710 * Wheel_Dimmer;
  Dimmer_MovingHeadMH710 = Dimmer_MovingHeadMH710 / 255;
  switch (mode) {
    case 0:                                                              //Auto sound reactive
      for (int i = 0; i < 3; i++) {                                      //Setting all the MovingHeadMH710 lights to the same
        DmxSimple.write(MovingHeadMH710[i][8], Dimmer_MovingHeadMH710);  //Dimmer
        DmxSimple.write(MovingHeadMH710[i][9], 240);                     //Sound reactive channel 9 = 240..255
      }
      break;
    case 1:                          //White
      for (int i = 0; i < 3; i++) {  //Setting all the MovingHeadMH710 lights to the same
        DmxSimple.write(MovingHeadMH710[i][3], 132);
        DmxSimple.write(MovingHeadMH710[i][8], Dimmer_MovingHeadMH710);  //Dimmer
        DmxSimple.write(MovingHeadMH710[i][4], Wheel_Red);               //Red
        DmxSimple.write(MovingHeadMH710[i][5], Wheel_Green);             //Green
        DmxSimple.write(MovingHeadMH710[i][6], Wheel_Blue);              //Blue
      }
      break;
    case 2:                          //Constant Collors
      for (int i = 0; i < 3; i++) {  //Setting all the MovingHeadMH710 lights to the same
        DmxSimple.write(MovingHeadMH710[i][3], 132);
        DmxSimple.write(MovingHeadMH710[i][8], Dimmer_MovingHeadMH710);  //Dimmer
        DmxSimple.write(MovingHeadMH710[i][4], Wheel_Red);               //Red
        DmxSimple.write(MovingHeadMH710[i][5], Wheel_Green);             //Green
        DmxSimple.write(MovingHeadMH710[i][6], Wheel_Blue);              //Blue
      }
      break;

    case 3:                          //OFF
      for (int i = 0; i < 3; i++) {  //Setting all the MovingHeadMH710 lights to the same
        //DmxSimple.write(MovingHeadMH710[i][3], 132);
        //DmxSimple.write(MovingHeadMH710[i][8], 0);  //Dimmer
      }
      break;
  }
}

//Set to 4 channels
void UV600Call(int mode) {
  Dimmer_UV600 = Dimmer_UV600 * Wheel_Dimmer;
  Dimmer_UV600 = Dimmer_UV600 / 255;
  switch (mode) {
    case 0:                          //Auto sound reactive
      for (int i = 0; i < 3; i++) {  //Setting all the UV600 lights to the same
        //Sound reactive
        DmxSimple.write(UV600[i][1], Dimmer_UV600);  //Dimmer
        DmxSimple.write(UV600[i][4], 125);           //Sound control + sensetivity 6 to 255
      }
      break;
    case 1:                                          //Strope
      for (int i = 0; i < 3; i++) {                  //Setting all the UV600 lights to the same
        DmxSimple.write(UV600[i][1], Dimmer_UV600);  //Dimmer
        DmxSimple.write(UV600[i][4], 0);             //Sound control OFF
        DmxSimple.write(UV600[i][2], Wheel_Time);    //Strope from 6 to 255
        DmxSimple.write(UV600[i][3], 125);           //Strope duration 1ms to 510ms
      }
      break;
    case 2:                                          //Constant
      for (int i = 0; i < 3; i++) {                  //Setting all the UV600 lights to the same
        DmxSimple.write(UV600[i][1], Dimmer_UV600);  //Dimmer
        DmxSimple.write(UV600[i][4], 0);             //Sound control OFF
        DmxSimple.write(UV600[i][2], 0);             //Strope OFF
      }
      break;
    case 3:                             //LED OFF
      DmxSimple.write(UV600[1][1], 0);  //Dimmer
      break;
    case 4:
      DmxSimple.write(UV600[1][1], 255);  //Dimmer
      break;
  }
}

//Set to 11 channels
void MiniMovingHeadCall(int mode) {
  Dimmer_MiniMovingHead = Dimmer_MiniMovingHead * Wheel_Dimmer;
  Dimmer_MiniMovingHead = Dimmer_MiniMovingHead / 255;
  switch (mode) {
    case 0:                          //Auto sound reactive
      for (int i = 0; i < 5; i++) {  //Setting all the MiniMovingHead lights to the same
        //Sound reactive
        DmxSimple.write(MiniMovingHead[i][8], Dimmer_MiniMovingHead);  //Dimmer
        DmxSimple.write(MiniMovingHead[i][10], 250);                   //Sound control
        //DmxSimple.write(MiniMovingHead[i][11], 125);           //Sound sensetivity
      }
      break;
    case 1:                                                            //Strope
      for (int i = 0; i < 5; i++) {                                    //Setting all the MiniMovingHead lights to the same
        DmxSimple.write(MiniMovingHead[i][8], Dimmer_MiniMovingHead);  //Dimmer
        DmxSimple.write(MiniMovingHead[i][9], Wheel_Time);
      }
      break;
    case 2:  //Constant collors
      /*
      for (int i = 0; i < 5; i++) {
        DmxSimple.write(MiniMovingHead[i][5], 0);
        DmxSimple.write(MiniMovingHead[i][1], Wheel_Red);
        DmxSimple.write(MiniMovingHead[i][2], Wheel_Green);
        DmxSimple.write(MiniMovingHead[i][3], Wheel_Blue);
        DmxSimple.write(MiniMovingHead[i][4], Wheel_Dimmer);
      }
      */
      break;
    case 3:                          //LED OFF
      for (int i = 0; i < 5; i++) {  //Setting all the TriLED_Bar lights to the same
        DmxSimple.write(MiniMovingHead[i][8], 0);
      }
      break;
  }
}

//Set to 7 channels
void TriLED_BarCall(int mode) {
  Dimmer_TriLED_Bar = Dimmer_TriLED_Bar * Wheel_Dimmer;
  Dimmer_TriLED_Bar = Dimmer_TriLED_Bar / 255;

  switch (mode) {
    case 0:                          //Auto sound reactive
      for (int i = 0; i < 5; i++) {  //Setting all the TriLED_Bar lights to the same
        //Sound reactive
        DmxSimple.write(TriLED_Bar[i][5], 0);
        DmxSimple.write(TriLED_Bar[i][4], Dimmer_TriLED_Bar);  //Dimmer
        DmxSimple.write(TriLED_Bar[i][6], 250);                //Sound control
        DmxSimple.write(TriLED_Bar[i][7], 80);                 //Sound sensetivity
      }
      break;
    case 1:                          //Strope
      for (int i = 0; i < 5; i++) {  //Setting all the TriLED_Bar lights to the same
        DmxSimple.write(TriLED_Bar[i][1], Wheel_Red);
        DmxSimple.write(TriLED_Bar[i][5], Wheel_Time);
        DmxSimple.write(TriLED_Bar[i][2], Wheel_Green);
        DmxSimple.write(TriLED_Bar[i][3], Wheel_Blue);
        DmxSimple.write(TriLED_Bar[i][4], Dimmer_TriLED_Bar);
      }
      break;
    case 2:  //Constant collors
      for (int i = 0; i < 5; i++) {
        DmxSimple.write(TriLED_Bar[i][5], 0);
        DmxSimple.write(TriLED_Bar[i][1], Wheel_Red);
        DmxSimple.write(TriLED_Bar[i][2], Wheel_Green);
        DmxSimple.write(TriLED_Bar[i][3], Wheel_Blue);
        DmxSimple.write(TriLED_Bar[i][4], Dimmer_TriLED_Bar);
      }
      break;
    case 3:                          //LED OFF
      for (int i = 0; i < 5; i++) {  //Setting all the TriLED_Bar lights to the same
        DmxSimple.write(TriLED_Bar[i][6], 0);
        DmxSimple.write(TriLED_Bar[i][5], 0);
        DmxSimple.write(TriLED_Bar[i][4], 0);
      }
      break;
    case 4:                                      //Strope
      for (int i = 0; i < 5; i++) {              //Setting all the TriLED_Bar lights to the same
        DmxSimple.write(TriLED_Bar[i][1], 255);  //Red
        DmxSimple.write(TriLED_Bar[i][5], 236);  //Time
        DmxSimple.write(TriLED_Bar[i][2], 255);  //Green
        DmxSimple.write(TriLED_Bar[i][3], 255);  //Blue
        DmxSimple.write(TriLED_Bar[i][4], Dimmer_TriLED_Bar);
      }
      break;
    case 5:
      for (int i = 0; i < 5; i++) {  //Setting all the TriLED_Bar lights to the same
        DmxSimple.write(TriLED_Bar[i][5], 0);
        DmxSimple.write(TriLED_Bar[i][7], Wheel_Time);
        DmxSimple.write(TriLED_Bar[i][6], 80);  //Pre programmed show
        DmxSimple.write(TriLED_Bar[i][4], Dimmer_TriLED_Bar);
      }
      break;
  }
}


//Set to 9 channels
void SpotCameoCall(int mode) {
  Dimmer_SpotCameo = Dimmer_SpotCameo * Wheel_Dimmer;
  Dimmer_SpotCameo = Dimmer_SpotCameo / 255;
  switch (mode) {
    case 1:                          //Strope
      for (int i = 0; i < 5; i++) {  //Setting all the TriLED_Bar lights to the same
        int Wheel_TimeTranslated = map(Wheel_Time, 0, 255, 128, 250);
        DmxSimple.write(SpotCameo[1][1], Dimmer_SpotCameo);
        DmxSimple.write(SpotCameo[1][3], Wheel_TimeTranslated);
        DmxSimple.write(SpotCameo[1][4], Wheel_Red);
        DmxSimple.write(SpotCameo[1][5], Wheel_Green);
        DmxSimple.write(SpotCameo[1][6], Wheel_Blue);
      }
      break;
    case 2:  //Constant collors
      DmxSimple.write(SpotCameo[1][1], Dimmer_SpotCameo);
      DmxSimple.write(SpotCameo[1][3], 0);
      DmxSimple.write(SpotCameo[1][4], Wheel_Red);
      DmxSimple.write(SpotCameo[1][5], Wheel_Green);
      DmxSimple.write(SpotCameo[1][6], Wheel_Blue);

      break;
    case 3:  //LED OFF
      DmxSimple.write(SpotCameo[1][1], 0);
      DmxSimple.write(SpotCameo[1][3], 0);
      DmxSimple.write(SpotCameo[1][8], 0);
      break;
    case 4:  //Rotating fading lights
      int Wheel_TimeTranslated = map(Wheel_Time, 0, 255, 128, 191);
      DmxSimple.write(SpotCameo[1][8], 255);
      DmxSimple.write(SpotCameo[1][1], 255);
      DmxSimple.write(SpotCameo[1][3], 0);  //Strope off

      break;
  }
}


void MiniDekkerCall(int mode) {
  Dimmer_MiniDekker = Dimmer_MiniDekker * Wheel_Dimmer;
  Dimmer_MiniDekker = Dimmer_MiniDekker / 255;
  switch (mode) {
    case 1:                          //Strope
      for (int i = 0; i < 5; i++) {  //Setting all the SpotCameo lights to the same

        int Wheel_TimeTranslated = map(Wheel_Time, 0, 255, 10, 255);
        DmxSimple.write(SpotCameo[i][9], 0);  //Enableing DMX
        DmxSimple.write(SpotCameo[i][6], Dimmer_MiniDekker);
        DmxSimple.write(SpotCameo[i][5], Wheel_TimeTranslated);
        DmxSimple.write(SpotCameo[i][1], Wheel_Red);
        DmxSimple.write(SpotCameo[i][2], Wheel_Green);
        DmxSimple.write(SpotCameo[i][3], Wheel_Blue);
      }
      break;
    case 2:                                   //Constant collors
      for (int i = 0; i < 5; i++) {           //Setting all the SpotCameo lights to the same
        DmxSimple.write(SpotCameo[i][9], 0);  //Enableing DMX

        DmxSimple.write(SpotCameo[i][5], 0);
        DmxSimple.write(SpotCameo[i][7], 0);
        DmxSimple.write(SpotCameo[i][8], 0);

        DmxSimple.write(SpotCameo[i][6], Dimmer_MiniDekker);
        DmxSimple.write(SpotCameo[i][1], Wheel_Red);
        DmxSimple.write(SpotCameo[i][2], Wheel_Green);
        DmxSimple.write(SpotCameo[i][3], Wheel_Blue);
      }
      break;
    case 3:                                   //LED OFF
      for (int i = 0; i < 5; i++) {           //Setting all the SpotCameo lights to the same
        DmxSimple.write(SpotCameo[i][9], 0);  //Enableing DMX
        DmxSimple.write(SpotCameo[i][5], 0);
        DmxSimple.write(SpotCameo[i][6], 0);
      }
      break;
    case 4:                          //Rotating lights
      for (int i = 0; i < 5; i++) {  //Setting all the SpotCameo lights to the same

        int Wheel_TimeTranslated1 = map(Wheel_Time, 0, 255, 128, 255);
        int Wheel_TimeTranslated2 = map(Wheel_Time, 0, 255, 85, 255);
        DmxSimple.write(SpotCameo[i][9], 0);  //Enableing DMX
        DmxSimple.write(SpotCameo[i][6], Dimmer_MiniDekker);
        DmxSimple.write(SpotCameo[i][7], Wheel_TimeTranslated1);  //Motor control
        DmxSimple.write(SpotCameo[i][8], Wheel_TimeTranslated2);  //Color fading
      }
      break;
  }
}


//Data send to ESP32
void LED_StripCall(int mode) {
  switch (mode) {
    case 0:  //Auto sound reactive
      while (0)
        ;
      break;
    case 1:  //Strope
      while (0)
        ;
      break;
    case 2:  //Constant collors
      while (0)
        ;
      break;
    case 3:  //LED OFF
      for (int i = 0; i < NUM_LEDS1; i++) {
        leds1[i] = CRGB(0, 0, 0);
        leds2[i] = CRGB(0, 0, 0);
      }
      for (int i = 0; i < NUM_LEDS3; i++) {
        leds3[i] = CRGB(0, 0, 0);
      }
      FastLED.show();
      break;
    case 4:  //Show 1
      ChangePalettePeriodically();

      static uint8_t startIndex = 0;
      startIndex = startIndex + 3; /* motion speed */


      FillLEDsFromPaletteColors(startIndex);
      FastLED.show();
      FastLED.delay(1000 / UPDATES_PER_SECOND);
      break;
  }
}



void updatePotentiometer() {
  for (int i = 1; i < 7; i++) {
    LastPotentiometer[i] = Potentiometer[i];
    Potentiometer[i] = analogRead(PotentiometerPin[i]);
  }
  Wheel_Dimmer = map(Potentiometer[5], 0, 1024, 255, 0);

  Wheel_Time = map(Potentiometer[4], 0, 1024, 255, 0);

  Wheel_Blue = map(Potentiometer[3], 0, 1024, 255, 0);
  Wheel_Blue = Wheel_Blue * Wheel_Dimmer;
  Wheel_Blue = Wheel_Blue / 255;

  Wheel_Green = map(Potentiometer[2], 0, 1024, 255, 0);
  Wheel_Green = Wheel_Green * Wheel_Dimmer;
  Wheel_Green = Wheel_Green / 255;

  Wheel_Red = map(Potentiometer[1], 0, 1024, 255, 0);
  Wheel_Red = Wheel_Red * Wheel_Dimmer;
  Wheel_Red = Wheel_Red / 255;

  //Fixing hardware issues

  if (Wheel_Red < 45) {
    Wheel_Red = 0;
  }
  if (Wheel_Red > 245) {
    Wheel_Red = 255;
  }

  if (Wheel_Green < 42) {
    Wheel_Green = 0;
  }
  if (Wheel_Green > 250) {
    Wheel_Green = 255;
  }

  if (Wheel_Blue < 42) {
    Wheel_Blue = 0;
  }
  if (Wheel_Blue > 250) {
    Wheel_Blue = 255;
  }

  if (Wheel_Dimmer < 0) {
    Wheel_Dimmer = 0;
  }


  if (Wheel_Dimmer > 250) {
    Wheel_Dimmer = 255;
  }

  if (Wheel_Time <= 0) {
    Wheel_Time = 1;
  }
  if (Wheel_Time > 249) {
    Wheel_Time = 255;
  }
}

void updateButtons() {
  //Update all buttons buttons
  for (int i = 1; i < 17; i++) {
    int reading = digitalRead(Button[i]);

    if (reading == LOW && LastButtonState[i] == HIGH) {
      ButtonState[i] = !ButtonState[i];
    }
    LastButtonState[i] = reading;
  }
}

//LED STRIP funktions
void FillLEDsFromPaletteColors(uint8_t colorIndex) {
  uint8_t brightness = 255;

  for (int i = 0; i < NUM_LEDS1; i++) {
    leds1[i] = ColorFromPalette(currentPalette, colorIndex, brightness, currentBlending);
    leds2[i] = ColorFromPalette(currentPalette, colorIndex, brightness, currentBlending);
    colorIndex += 3;
  }

  for (int i = 0; i < NUM_LEDS3; i++) {
    leds3[i] = ColorFromPalette(currentPalette, colorIndex, brightness, currentBlending);
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