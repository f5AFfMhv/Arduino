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

const int buttonPin = 4;          // input pin for pushbutton
int previousButtonState = HIGH;   // for checking the state of a pushButton
String path = "~/skriptai/";

void execute_shell();

void setup() {
  // make the pushButton pin an input:
  pinMode(buttonPin, INPUT);
  // initialize control over the keyboard:
  Keyboard.begin();
}

void loop() {
  // read the pushbutton:
  int buttonState = digitalRead(buttonPin);
  // if the button state has changed and its high
  if ((buttonState != previousButtonState) && (buttonState == HIGH)) {
    delay(50);
    // CTRL-SHIFT-}:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press('}');
    delay(100);
    Keyboard.releaseAll();
    //execute_shell("pc_load.sh");
    // save the current button state for comparison next time:
  }
   previousButtonState = buttonState;
}

void execute_shell(String script) {
    // Open terminal
    // CTRL-ALT-T:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_ALT);
    Keyboard.press('t');
    delay(100);
    Keyboard.releaseAll();
    Keyboard.println(path + script);
    Keyboard.press(KEY_RETURN);
    delay(100);
    Keyboard.releaseAll();
    Keyboard.println("exit");
    Keyboard.press(KEY_RETURN);
    delay(100);
    Keyboard.releaseAll();
  }
