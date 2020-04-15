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

const int button1 = 4;          // input pin for button1
const int button2 = 5;          // input pin for button2
const int button3 = 6;          // input pin for button3
const int button4 = 8;          // input pin for button4
const int button5 = 9;          // input pin for button5
const int button6 = 10;         // input pin for button6
const int button7 = 16;         // input pin for button7
const int button8 = 14;         // input pin for button8
const int button9 = 15;         // input pin for button9
const int button10 = 18;        // input pin for button10
const int button11 = 19;        // input pin for button11
const int button12 = 20;        // input pin for button12
const int button13 = 21;        // input pin for button13
const int button14 = 0;         // input pin for button14
const int button15 = 1;         // input pin for button15
const int encoderButton = 7;    // input pin for encoder button

int previousButton1State = HIGH;         // for checking the state of a Button1
int previousButton2State = HIGH;         // for checking the state of a Button2
int previousButton3State = HIGH;         // for checking the state of a Button3
int previousButton4State = HIGH;         // for checking the state of a Button4
int previousButton5State = HIGH;         // for checking the state of a Button5
int previousButton6State = HIGH;         // for checking the state of a Button6
int previousButton7State = HIGH;         // for checking the state of a Button7
int previousButton8State = HIGH;         // for checking the state of a Button8
int previousButton9State = HIGH;         // for checking the state of a Button9
int previousButton10State = HIGH;        // for checking the state of a Button10
int previousButton11State = HIGH;        // for checking the state of a Button11
int previousButton12State = HIGH;        // for checking the state of a Button12
int previousButton13State = HIGH;        // for checking the state of a Button13
int previousButton14State = HIGH;        // for checking the state of a Button14
int previousButton15State = HIGH;        // for checking the state of a Button15
int previousEncoderButtonState = LOW;    // for checking the state of a encoder button

bool debugFlag = 0;            // flag for debug output over serial monitor
bool encoderButtonFlag = 0;    // for switching encoder volume-0/zoom-1 functions

String path = "~/skriptai/";  // path to folder containing scripts

long oldPosition  = -999;     // for rotary encoder

Encoder myEnc(2, 3);          // rotary encoder pin A and B

void execute_shell(String script); // not mapped anywhere
void shutdown_pc();     // mapped to button1
void monitor_control(); // mapped to button2
void pc_load_info();    // mapped to button3
void dummy1();          // mapped to button4
void dummy2();          // mapped to button5
void dummy3();          // mapped to button6
void dummy4();          // mapped to button7
void dummy5();          // mapped to button8
void dummy6();          // mapped to button9
void dummy7();          // mapped to button10
void dummy8();          // mapped to button11
void dummy9();          // mapped to button12
void dummy10();         // mapped to button13
void dummy11();         // mapped to button14
void dummy12();         // mapped to button15
void increase_sound();  // mapped to rotary encoder turning clockwise when flag is 0
void decrease_sound();  // mapped to rotary encoder turning counter clockwise when flag is 0
void zoom_in();         // mapped to rotary encoder turning clockwise when flag is 1
void zoom_out();        // mapped to rotary encoder turning counter clockwise when flag is 1

void setup() {
  if (debugFlag) {
    //enable serial monitor
    Serial.begin(9600);
    Serial.println("Starting...");
  }
  // make the pushButton pins an input:
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);
  pinMode(button4, INPUT);
  pinMode(button5, INPUT);
  pinMode(button6, INPUT);
  pinMode(button7, INPUT);
  pinMode(button8, INPUT);
  pinMode(button9, INPUT);
  pinMode(button10, INPUT);
  pinMode(button11, INPUT);
  pinMode(button12, INPUT);
  pinMode(button13, INPUT);
  pinMode(button14, INPUT);
  pinMode(button15, INPUT);
  pinMode(encoderButton, INPUT);
  
  // initialize control over the keyboard:
  Keyboard.begin();
}

void loop() {
  // read the pushbuttons:
  int button1State = digitalRead(button1);
  int button2State = digitalRead(button2);
  int button3State = digitalRead(button3);
  int button4State = digitalRead(button4);
  int button5State = digitalRead(button5);
  int button6State = digitalRead(button6);
  int button7State = digitalRead(button7);
  int button8State = digitalRead(button8);
  int button9State = digitalRead(button9);
  int button10State = digitalRead(button10);
  int button11State = digitalRead(button11);
  int button12State = digitalRead(button12);
  int button13State = digitalRead(button13);
  int button14State = digitalRead(button14);
  int button15State = digitalRead(button15);
  int encoderButtonState = digitalRead(encoderButton);
  
  // if the button1 state has changed to high:
  if ((button1State != previousButton1State) && (button1State == HIGH)) {
    shutdown_pc(); 
  }
  // if the button2 state has changed to high:
  if ((button2State != previousButton2State) && (button2State == HIGH)) {
    monitor_control(); 
  }
  // if the button3 state has changed to high:
  if ((button3State != previousButton3State) && (button3State == HIGH)) {
    pc_load_info(); 
  }

  // if the button4 state has changed to high:
  if ((button4State != previousButton4State) && (button4State == HIGH)) {
    dummy1(); 
  }
  // if the button5 state has changed to high:
  if ((button5State != previousButton5State) && (button5State == HIGH)) {
    dummy2(); 
  }
  // if the button6 state has changed to high:
  if ((button6State != previousButton6State) && (button6State == HIGH)) {
    dummy3(); 
  }
  // if the button7 state has changed to high:
  if ((button7State != previousButton7State) && (button7State == HIGH)) {
    dummy4(); 
  }
  // if the button8 state has changed to high:
  if ((button8State != previousButton8State) && (button8State == HIGH)) {
    dummy5(); 
  }
  // if the button9 state has changed to high:
  if ((button9State != previousButton9State) && (button9State == HIGH)) {
    dummy6(); 
  }
  // if the button10 state has changed to high:
  if ((button10State != previousButton10State) && (button10State == HIGH)) {
    dummy7(); 
  }
  // if the button11 state has changed to high:
  if ((button11State != previousButton11State) && (button11State == HIGH)) {
    dummy8(); 
  }
  // if the button12 state has changed to high:
  if ((button12State != previousButton12State) && (button12State == HIGH)) {
    dummy9(); 
  }
  // if the button13 state has changed to high:
  if ((button13State != previousButton13State) && (button13State == HIGH)) {
    dummy10(); 
  }
  // if the button14 state has changed to high:
  if ((button14State != previousButton14State) && (button14State == HIGH)) {
    dummy11(); 
  }
  // if the button15 state has changed to high:
  if ((button15State != previousButton15State) && (button15State == HIGH)) {
    dummy12(); 
  }
  
  // if the encoderButton state has changed to low:
  if ((encoderButtonState != previousEncoderButtonState) && (encoderButtonState == LOW)) {
    delay(50);
    encoderButtonFlag = !encoderButtonFlag; 
    if (debugFlag) {
      Serial.print("ENCODER_FLAG=");
      Serial.println(encoderButtonFlag);
    }
  }
  
  // save the current button state for comparison next time:
  previousButton1State = button1State;
  previousButton2State = button2State;
  previousButton3State = button3State;
  previousButton4State = button4State;
  previousButton5State = button5State;
  previousButton6State = button6State;
  previousButton7State = button7State;
  previousButton8State = button8State;
  previousButton9State = button9State;
  previousButton10State = button10State;
  previousButton11State = button11State;
  previousButton12State = button12State;
  previousButton13State = button13State;
  previousButton14State = button14State;
  previousButton15State = button15State;
  previousEncoderButtonState = encoderButtonState;

  // read the encoder
  long newPosition = myEnc.read();
  
  // check for encoder rotation
  if (newPosition != oldPosition) {
    if (newPosition > oldPosition) {
      if (encoderButtonFlag) {
        zoom_in();
      }
      else {
        increase_sound();
      }
    }
    else {
      if (encoderButtonFlag) {
        zoom_out();
      }
      else {
        decrease_sound();
      }
    }
    oldPosition = newPosition;
  }
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
void shutdown_pc() {
    // CTRL-SHIFT-A:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press('A');
    delay(100);
    Keyboard.releaseAll();
}
void monitor_control() {
    // CTRL-SHIFT-B:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press('B');
    delay(100);
    Keyboard.releaseAll();
}
void pc_load_info() {
    // CTRL-SHIFT-C:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press('C');
    delay(100);
    Keyboard.releaseAll();
}
void increase_sound() {
    // CTRL-SHIFT-D:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press('D');
    delay(100);
    Keyboard.releaseAll();
}
void decrease_sound() {
    // CTRL-SHIFT-E:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press('E');
    delay(100);
    Keyboard.releaseAll();
}
void zoom_in() {
    // CTRL-SHIFT-F:
    Keyboard.press(KEY_LEFT_CTRL);
    //Keyboard.press(KEY_LEFT_SHIFT);
    //Keyboard.press('F');
    Keyboard.press('+');
    delay(100);
    Keyboard.releaseAll();
}
void zoom_out() {
    // CTRL-SHIFT-G:
    Keyboard.press(KEY_LEFT_CTRL);
    //Keyboard.press(KEY_LEFT_SHIFT);
    //Keyboard.press('G');
    Keyboard.press('-');
    delay(100);
    Keyboard.releaseAll();
}
void dummy1() {
    // CTRL-SHIFT-H:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press('H');
    delay(100);
    Keyboard.releaseAll();
}
void dummy2() {
    // CTRL-SHIFT-I:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press('I');
    delay(100);
    Keyboard.releaseAll();
}
void dummy3() {
    // CTRL-SHIFT-J:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press('J');
    delay(100);
    Keyboard.releaseAll();
}
void dummy4() {
    // CTRL-SHIFT-K:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press('K');
    delay(100);
    Keyboard.releaseAll();
}
void dummy5() {
    // CTRL-SHIFT-L:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press('L');
    delay(100);
    Keyboard.releaseAll();
}
void dummy6() {
    // CTRL-SHIFT-M:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press('M');
    delay(100);
    Keyboard.releaseAll();
}

void dummy7() {
    // CTRL-SHIFT-N:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press('N');
    delay(100);
    Keyboard.releaseAll();
}
void dummy8() {
    // CTRL-SHIFT-O:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press('O');
    delay(100);
    Keyboard.releaseAll();
}
void dummy9() {
    // CTRL-SHIFT-P:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press('P');
    delay(100);
    Keyboard.releaseAll();
}
void dummy10() {
    // CTRL-SHIFT-R:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press('R');
    delay(100);
    Keyboard.releaseAll();
}
void dummy11() {
    // CTRL-SHIFT-S:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press('S');
    delay(100);
    Keyboard.releaseAll();
}
void dummy12() {
    // CTRL-SHIFT-T:
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.press(KEY_LEFT_SHIFT);
    Keyboard.press('T');
    delay(100);
    Keyboard.releaseAll();
}
