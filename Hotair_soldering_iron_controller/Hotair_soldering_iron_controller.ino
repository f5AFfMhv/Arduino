/* 
This sketch is for hotair soldering iron controller.
Microcontroller: Atmega 328P.

Copyright (C) 2019 Martynas J. 
f5AFfMhv@protonmail.com  
https://github.com/f5AFfMhv
 */

#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>

// Pinout
#define heater 10
#define air 11
#define thermocouple A0
#define air_flow_level A1
#define button A3
#define red_led A2
#define green_led 13
#define reed_sw 9

LiquidCrystal_PCF8574 lcd(0x27);  // set the LCD address to 0x27 for a 16 chars and 2 line display
unsigned long currentTime;
unsigned long oldTime = 0;

const float ref_v = 5.1; //measured LDO voltage
const float k = 200.3; //linear temperature dependancy over thermocouple amp voltage coeficient k
const float b = -251.0; //linear temperature dependancy over thermocouple amp voltage coeficient b
const float coef = (k*ref_v)/1023.0; //simplify temp calculations
const int OVER_TEMP = 600;
const int SAFE_TEMP = 40;

int temp, flow, mflow;
int counter = 0;

String heater_state = " OFF";
String fan_state = " OFF";

//flags
int debug_flag = 0;
int interrupt_flag = 0;
int temp_check_flag = 0;
int on_state = 0;

//Interrupt service routines
//reed switch closed
ISR(PCINT0_vect) {
    delay(10);
    interrupt_flag = 1;
    if (digitalRead(reed_sw) == 0){
      on_state = 0;
    }
}
//button pressed
ISR(PCINT1_vect) {
  delay(10);
  if (digitalRead(button) == 0){
    interrupt_flag = 1;
    on_state = 1;
  }
}

void setup() {
  pinMode(heater, OUTPUT);
  pinMode(air, OUTPUT);
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(button, INPUT);
  pinMode(reed_sw, INPUT);

  //initial output levels
  digitalWrite(heater, LOW);
  digitalWrite(air, LOW);
  digitalWrite(red_led, HIGH);
  digitalWrite(green_led, LOW);
  
  //reed switch interrupt setup
  PCICR |= (1<<PCIE0); //enable PCINT[7:0] pin interrupts
  PCMSK0 &= ~(1<<PCINT1); //disable interrupt on PCINT1 pin (reed switch)
  //button interrupt setup
  PCICR |= (1<<PCIE1); //enable PCINT[14:8] pin interrupts
  PCMSK1 |= (1<<PCINT11); //enable interrupt on PCINT11 pin (button)
  sei(); // global interrupts enable

  if (debug_flag == 1){
    Serial.begin(9600);
    Serial.println("START");
  }
  
  Wire.begin();
  Wire.beginTransmission(0x27);

  lcd.begin(16, 2); // initialize the lcd
  lcd.setBacklight(255);
  lcd.home(); 
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T_C:");
  lcd.setCursor(0, 1);
  lcd.print("Air:");
  lcd_update_state();

}

void loop() {

  // measure and calculate temperature and air flow level
  temp = ((analogRead(thermocouple)*coef)+b);
  flow = analogRead(air_flow_level);
  mflow = map(flow, 540, 910, 0, 100);

  // over temperature shutdown
  if (temp > OVER_TEMP){
    interrupt_flag = 1;
    on_state = 0;
    if (debug_flag == 1){
      Serial.println("over temperature");
    }
  }

  if (on_state == 1 && digitalRead(reed_sw) == 1){
    PCMSK0 |= (1<<PCINT1); //enable interrupt on PCINT1 pin (reed switch)
  }
  else{
    PCMSK0 &= ~(1<<PCINT1); //disable interrupt on PCINT1 pin (reed switch)
  }
    
  if (interrupt_flag == 1){
    if (on_state){
      digitalWrite(green_led, HIGH);
      digitalWrite(heater, HIGH);
      digitalWrite(air, HIGH);
      digitalWrite(red_led, LOW);
      heater_state = "  ON";
      fan_state = "  ON";
      temp_check_flag = 0;
    }
    else{
      digitalWrite(green_led, LOW);
      digitalWrite(heater, LOW);
      digitalWrite(red_led, HIGH);
      heater_state = " OFF";
      temp_check_flag = 1;
    }
  interrupt_flag = 0;
  lcd_update_state();
  }

  if (temp_check_flag == 1){
    if (temp < SAFE_TEMP){
      digitalWrite(air, LOW);
      fan_state = " OFF";
      temp_check_flag = 0;
      lcd_update_state();
    }
  }
  // what time it is?
  currentTime = millis();
  // update LCD every 200 ms
  if (currentTime-oldTime > 200){
    oldTime = currentTime;
    if (debug_flag == 1){
      counter++;
    }
    lcd_update();
  }
    
  //debug values every second
  if (debug_flag == 1 && counter == 5){
    counter = 0;
    Serial.print("ADCv");
    Serial.print("\t");
    Serial.print("T_V");
    Serial.print("\t");
    Serial.print("TEMP");
    Serial.print("\t");
    Serial.print("Hstate");
    Serial.print("\t");
    Serial.print("Fstate");
    Serial.print("\t");
    Serial.print("Tchk");
    Serial.print("\t");
    Serial.println("Time");
    Serial.print((((float)flow*5.0)/1023.0));
    Serial.print("\t");
    Serial.print((((float)flow*5.0*5.7)/1023.0));
    Serial.print("\t");
    Serial.print(temp);
    Serial.print("\t");
    Serial.print(heater_state);
    Serial.print("\t");
    Serial.print(fan_state);
    Serial.print("\t");
    Serial.print(temp_check_flag);
    Serial.print("\t");
    Serial.println(currentTime/1000);
    Serial.println();
  }
}

void lcd_update(){
  lcd.setCursor(7, 0);
  if (temp < 100){
    lcd.print(" ");}
  lcd.print(temp);
  
  lcd.setCursor(7, 1);
  if (mflow < 0){
    lcd.print("  0");
  }
  if (mflow < 10){
    lcd.print("  ");}
  else if (mflow < 100){
    lcd.print(" ");}
  lcd.print(mflow);
}

void lcd_update_state(){
  lcd.setCursor(12, 0);
  lcd.print(heater_state);

  lcd.setCursor(12, 1);
  lcd.print(fan_state);
}

