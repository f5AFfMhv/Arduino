/*
  PC Controller with Keyboard Schortcuts

  For the Arduino Pro Micro.

  Sends keyboard schortcuts to PC

  created 2020-04-03
  by Martynas J.

  This code is in the public domain.

  https://github.com/f5AFfMhv
  https://www.instructables.com/member/f5AFfMhv/instructables/
  
*/

#include "Keyboard.h"
#include <Encoder.h>

/*        Arduino Pro Micro Pinout
           (PWM pins in brackets)
  
                   |﹉﹉|   
  TX    1     O--------------O RAW
  RX    0     O              O GND
          GND O              O RST
          GND O              O VCC
  SDA   2     O              O      21    A3
  SCL  (3)    O              O      20    A2
  A6    4     O              O      19    A1    
       (5)    O              O      18    A0
  A7   (6)    O              O      15    SCLK
        7     O              O      14    MISO
  A8    8     O   Pro Micro  O      16    MOSI
  A9   (9)    O______________O     (10)   A10
*/

#define ALL_BUTTONS 15
//        buttonID:   0  1  2  3  4  5  6  7  8   9   10  11  12  13  14
const int button[] = {1, 0, 4, 5, 6, 7, 8, 9, 10, 16, 14, 15, 18, 19, 20}; // pins for button inputs
Encoder myEnc(2, 3);    // rotary encoder pin A and B

int buttonState[ALL_BUTTONS] = {HIGH}; // state of each button in current cycle
int previousButtonState[ALL_BUTTONS] = {HIGH}; // state of each button in previous cycle

bool debugFlag = 0;            // flag for debug output over serial monitor
bool encoderButtonFlag = 0;    // for switching encoder functions

long oldPosition  = -999;     // for rotary encoder

void keyboard_shortcut(int buttonID);

void setup() {
  if (debugFlag) {
    //enable serial monitor
    Serial.begin(9600);
    Serial.println("Starting...");
  }
  // make the pushButton pins an input:
    for (int i=0;i<ALL_BUTTONS;i++) {
    pinMode(button[i], INPUT);
  }
  // initialize control over the keyboard:
  Keyboard.begin();
}

void loop() {
  // read the pushbuttons:
  for (int i=0;i<ALL_BUTTONS;i++) {
    buttonState[i] = digitalRead(button[i]);
    // if button pressed execute function with button ID:
    if ((buttonState[i] != previousButtonState[i]) && (buttonState[i] == HIGH)) {
      keyboard_shortcut(i); 
    }
    // save the current button state for comparison next time:
    previousButtonState[i] = buttonState[i];
  }
  // read the encoder
  long newPosition = myEnc.read();
  
  // check for encoder rotation
  if (newPosition != oldPosition) {
    if (newPosition > oldPosition) {
      if (encoderButtonFlag) {
        keyboard_shortcut(15);
      }
      else {
        keyboard_shortcut(16);
      }
    }
    else {
      if (encoderButtonFlag) {
        keyboard_shortcut(17);
      }
      else {
        keyboard_shortcut(18);
       }
    }
    oldPosition = newPosition;
    delay(200);
  }
}

void keyboard_shortcut(int buttonID) {
  // CTRL-SHIFT-x:
  Keyboard.press(KEY_LEFT_CTRL);
  Keyboard.press(KEY_LEFT_SHIFT);
  switch (buttonID) {
    case 0:
      Keyboard.press('w');
      encoderButtonFlag = !encoderButtonFlag; 
      if (debugFlag) {
        Serial.print("ENCODER_FLAG=");
        Serial.println(encoderButtonFlag);
      }
      break;
    case 1:
      Keyboard.press('A');
      break;
    case 2:
      Keyboard.press('B');
      break;
    case 3:
      Keyboard.press('C');
      break;
    case 4:
      Keyboard.press('D');
      break;
    case 5:
      Keyboard.press('F');
      break;
    case 6:
      Keyboard.press('G');
      break;
    case 7:
      Keyboard.press('H');
      break;
    case 8:
      Keyboard.press('I');
      break;
    case 9:
      Keyboard.press('J');
      break;
    case 10:
      Keyboard.press('K');
      break;
    case 11:
      Keyboard.press('L');
      break;
    case 12:
      Keyboard.press('M');
      break;
    case 13:
      Keyboard.press('O');
      break;
    case 14:
      Keyboard.press('P');
      break;
    case 15:
      Keyboard.press('R');
      break;
    case 16:
      Keyboard.press('T');
      break;
    case 17:
      Keyboard.press('U');
      break;
    case 18:
      Keyboard.press('V');
      break;
    default:
      break;
  }  
  delay(100);
  Keyboard.releaseAll();
  if (debugFlag) {
    Serial.print("Key pressed: ");
    Serial.println(buttonID);
  }
}
