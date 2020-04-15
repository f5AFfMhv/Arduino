/* 
This sketch is for quadcopter controller.
Microcontroller: Atmega 328P.

Copyright (C) 2019 Martynas J. 
f5AFfMhv@protonmail.com  
https://github.com/f5AFfMhv
 */

/*****************************Bibliotekos*****************************/
#include <avr/interrupt.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <TFT_ILI9163C.h>
/*******************************Isvadai*******************************/
// Sviesos diodai
#define green_led 4 //PORTD
#define red_led 3 //PORTC
// Vairalazdes
#define JS_R_X A4 // Joystick Right X axis
#define JS_R_Y A5

#define JS_L_X A0
#define JS_L_Y A1

// Mygtukai
#define button_l 7 //PORTD
#define button_r 5 //PORTD
#define JS_R_B 2 //PORTD
#define JS_L_B 2 //PORTC
// Puse akumuliatoriaus itampos
#define bat A7 

// SPI interfeisas TFT LCD ekranui
#define __DC 9        // A0
#define __CS 10       // CS                
#define __RST 12      // RESET
// SCLK --> (SCK)
// MOSI --> (SDA) 
#define LCD_BCL 6    //PORTD    Ekrano PWM apsvietimas
/***********************Kintamieji ir konstantos*********************/
// Ekrano spalvos
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0  
#define WHITE   0xFFFF

int LCD_light = 250; // Pradine ekrano apsvietimo reiksme
int L_X, R_X, L_Y, R_Y, BATTERY, bat_old, L_X_old, R_X_old, L_Y_old, R_Y_old, L_X_offset, R_X_offset, L_Y_offset, R_Y_offset = 0; // Kintamieji ASK vertems saugoti
int L_X_map, R_X_map, L_Y_map, R_Y_map, LCD_bcl_map;
int flight_flag = 0;
bool X_change = 0;
bool Y_change = 0;
char incomingByte = ' '; 
String serial_string = "";
int color = RED;
int color_mask = RED;
volatile int interrupt_count = 0;
const int ANGLE_LIMIT = 100;
// Skraidykles akumuliatoriaus celes
String cell_1 = "-";
String cell_1_old = "-";
// Kintamieji skirti kalibravimui
struct calibration {
  String accX = "-";
  String accY = "-"; 
  String accZ = "-";
  String gyrX = "-";
  String gyrY = "-";
  String gyrZ = "-";
};

calibration angles;

TFT_ILI9163C tft = TFT_ILI9163C(__CS, __DC, __RST); // Ekrano isvadu susiejimas su biblioteka
/***********************Pertraukciu paprogrames***********************/
ISR(TIMER2_OVF_vect) { // (1024 / 12M) * 255
   interrupt_count++;
   if (interrupt_count == 2757){ // Ivyksta apytiksliai kas 1min (1024 / 12M) * 255 * 2757 = 60
    interrupt_count = 0;
    Battery_update();
   }
}

/****************************MV nustatymas****************************/
void setup() {
  // MV isvadu nustatymas
//  green_led         PD4
//  red_led           PC3
//  LCD_BCL           PD6
//  button_l          PD7
//  button_r          PD5
//  JS_R_B            PD2
//  JS_L_B            PC2

  // Pull-up ijungimas
  PORTB |= (1<<PB0); // Neprijungtas
  PORTC |= (1<<JS_L_B); 
  PORTD |= (1<<JS_R_B)|(1<<PD3)|(1<<button_r)|(1<<LCD_BCL)|(1<<button_l); 
  // Ivestis/isvestis 
  DDRB &= ~(1<<DDB0); // Neprijungtas
  DDRC &= ~(1<<JS_L_B); 
  DDRC |= (1<<red_led); 
  DDRD = 0; 
  DDRD |= (1<<green_led)|(1<<LCD_BCL);
  
  // Pertraukciu nustatymas
  TCCR2A = 0; // TMR2 nustatomas normaliam veikimui
  TIMSK2 |= (1<<TOIE2); // TMR2 perpildos pertraukties aktyvavimas
  sei(); // Globalus pertraukciu ijungimas
  TCCR2B |= (1<<CS22)|(1<<CS21)|(1<<CS20); // System clock 1024 prescaler
  
  // Nuosekliojo monitoriaus inicializacija
  Serial.begin(38400);
  // Ekrano inicializacija
  tft.begin();  
  main_window();
  // Vairalazdziu 0 padeties nuskaitymas
  L_X_offset = map(analogRead(JS_L_X), 0, 1023, -100, 100);
  R_X_offset = map(analogRead(JS_R_X), 0, 1023, -ANGLE_LIMIT, ANGLE_LIMIT);
  L_Y_offset = map(analogRead(JS_L_Y), 0, 1023, -100, 100);
  R_Y_offset = map(analogRead(JS_R_Y), 0, 1023, -ANGLE_LIMIT, ANGLE_LIMIT);
  // Raudonas LED rodo parengties rezima
  PORTD &= ~(1<<green_led);
  PORTC |= (1<<red_led);
}
/*************************Pagrindine programa**************************/
void loop() {
  // Skrydzio rezimas
  if ((PIND & (1<<JS_R_B)) == 0) { // Nuspaustas JS_R_B
    Serial.print("EN");
    delay(30);
    if (serial_string == "OK") {
      PORTD |= (1<<green_led);
      PORTC &= ~(1<<red_led);
      flight_flag = 1;
      serial_string = "";
    } 
  }
  // Parengties rezimas
  else if ((PINC & (1<<JS_L_B)) == 0) { // Nuspaustas JS_L_B
    Serial.print("DIS");
    delay(30);
    if (serial_string == "OK") {
      PORTD &= ~(1<<green_led);
      PORTC |= (1<<red_led);
      flight_flag = 0;
      serial_string = "";
    }
  }
  // Kalibravimo rezimas
  else if ((PIND & (1<<button_l)) == 0) { // Nuspaustas button_l
    Serial.print("K");
    delay(30);
    if (/*(serial_string == "OK") & */(flight_flag == 0)) {
      serial_string = "";
      Calibrate();
    }
  }
  // Skrydzio valdymas
  else if (L_Y_map >= 40 && L_Y_map < 90) {
    Serial.print("P1");
    delay(30);
  }
  else if (L_Y_map >= 90) {
    Serial.print("P5");
    delay(30);
  }
  else if (L_Y_map <= -40 && L_Y_map > -90) {
    Serial.print("M1");
    delay(30);
  }
  else if (L_Y_map <= -90) {
    Serial.print("M5");
    delay(30);
  }
  if (X_change == 1) {
    Serial.print("X");
    Serial.print(R_X_map);
    delay(30);
  }
  if (Y_change == 1) {
    Serial.print("Y");
    Serial.print(R_Y_map);
    delay(30);
  }
  
  if (Serial.available() > 0) {
    serial_string = "";
    serial_constr();
  if (serial_string.length()<6) {
    if (serial_string.substring(0,1) == "C") {
      cell_1 = serial_string.substring(1);
      }
    else if (serial_string.substring(0,1) == "X") {
      info_update(serial_string, 1);
    }
    else if (serial_string.substring(0,1) == "Y") {
      info_update(serial_string, 2);
    }
    else if (serial_string.substring(0,1) == "Z") {
      info_update(serial_string, 3);
    }
    else if (serial_string.substring(0,2) == "Gp") {
      info_update(serial_string, 4);
    }
    else if (serial_string.substring(0,2) == "Px") {
      info_update(serial_string, 5);
    }
    else if (serial_string.substring(0,2) == "Py") {
      info_update(serial_string, 6);
    }
    else if (serial_string.substring(0,2) == "Ps") {
      info_update(serial_string, 7);
    }
  }
  else {
    serial_string = "";
  }
    }

  LCD_update();           
}
/*****************************Paprogrames******************************/
void info_update(String data, int j) {
      switch(j){
        case 1:
          tft.fillRect(30, 65, 30, 8, color_mask);
          tft.setCursor(30, 65);
          tft.print(data.substring(1));
          break;
        case 2:
          tft.fillRect(30, 75, 30, 8, color_mask);
          tft.setCursor(30, 75);
          tft.print(data.substring(1));
          break;
        case 3:
          tft.fillRect(30, 85, 30, 8, color_mask);
          tft.setCursor(30, 85);
          tft.print(data.substring(1));
          break;
          
        case 4:
          tft.fillRect(5, 105, 30, 8, color_mask);
          tft.setCursor(5, 105);
          tft.print(data.substring(2));
          break;
        case 5:
          tft.fillRect(35, 105, 30, 8, color_mask);
          tft.setCursor(40, 105);
          tft.print(data.substring(2));
          break;
        case 6:
          tft.fillRect(65, 105, 30, 8, color_mask);
          tft.setCursor(70, 105);
          tft.print(data.substring(2));
          break;
        case 7:
          tft.fillRect(95, 105, 35, 8, color_mask);
          tft.setCursor(100, 105);
          tft.print(data.substring(2));
          break;
        default:
          break;
      }
}

void serial_constr() {
  while (Serial.available() > 0) {
    incomingByte = Serial.read();
    serial_string += incomingByte;
    delay(1);
  }
}

struct calibration getValue(String data, char separator)
{
  calibration temp;
  int maxIndex = data.length()-1;
  int dot_matrix[6];
  int num = 0;
  for(int i=0; i <= maxIndex; i++){
    if(data.charAt(i)==separator){
        dot_matrix[num] = i;
        num++;
    }
  }
  temp.accX = data.substring(dot_matrix[0]+1, dot_matrix[1]);
  temp.accY = data.substring(dot_matrix[1]+1, dot_matrix[2]);
  temp.accZ = data.substring(dot_matrix[2]+1, dot_matrix[3]);
  temp.gyrX = data.substring(dot_matrix[3]+1, dot_matrix[4]);
  temp.gyrY = data.substring(dot_matrix[4]+1, dot_matrix[5]);
  temp.gyrZ = data.substring(dot_matrix[5]+1, dot_matrix[6]-1);
  return temp;
}

void Battery_update() {
  BATTERY = analogRead(bat);
  Serial.print("V");
}

void main_window(){
// Ekrano nustatymas
  tft.fillScreen(color); // Isvalomas ekranas
  tft.setTextColor(BLACK);  
  tft.setTextSize(1);
  tft.setCursor(5, 5);
  tft.println("-------PULTAS-------");
  tft.setTextColor(WHITE);
  tft.setCursor(60, 15);
  tft.print("X");
  tft.setCursor(60, 25);
  tft.print("Y");
  tft.setCursor(30, 35);
  tft.print(" V");
  tft.setCursor(60, 35);
  tft.println("Ak. itampa");
  tft.setCursor(5, 45);
  tft.setTextColor(BLACK);
  tft.println("-----SKRAIDYKLE-----");
  tft.setTextColor(WHITE);
  tft.setCursor(5, 55);
  tft.print(cell_1);
  tft.setCursor(30, 55);
  tft.print(" V");
  tft.setCursor(60, 55);
  tft.println("Ak. itampa");
  tft.setCursor(5, 65);
  tft.print("X:"); 
  tft.setCursor(60, 65);
  tft.print("laipsn.");
  tft.setCursor(5, 75);
  tft.print("Y:");
  tft.setCursor(60, 75);
  tft.print("laipsn.");
  tft.setCursor(5, 85);
  tft.print("Z:");
  tft.setCursor(60, 85);
  tft.print("laipsn.");

  tft.setCursor(5, 95);
  tft.print("PWM1");
  tft.setCursor(35, 95);
  tft.print("PWM2");
  tft.setCursor(65, 95);
  tft.print("PWM3");
  tft.setCursor(95, 95);
  tft.print("PWM4");
  
  tft.setCursor(5, 115);
  tft.setTextColor(BLUE);
  tft.println("Kalibr.");
  tft.setTextColor(WHITE);  
  L_X_old = -100;
  L_Y_old = -100;
  R_X_old = -100;
  R_Y_old = -100;
  bat_old = 5;
  Battery_update();
}

void Calibrate() {
  calibration calib_old;
  calibration calib_new;

  int calib_f = 0;
  int exit_f = 0;
  int started_f = 0;
  int load_s = 0;
  
  tft.fillScreen(color); // Isvalomas ekranas
  tft.setTextColor(BLACK);  
  tft.setCursor(5, 5);
  tft.println("----KALIBRAVIMAS----");
  tft.setTextColor(WHITE);
  tft.setCursor(5, 15);
  tft.println("Kalibravimo duomenys");
  tft.setCursor(35, 30);
  tft.println("Seni");
  tft.setCursor(75, 30);
  tft.println("Nauji");
  tft.setCursor(5, 40);
  tft.println("aX");
  tft.setCursor(5, 50);
  tft.println("aY");
  tft.setCursor(5, 60);
  tft.println("aZ");
  tft.setCursor(5, 70);
  tft.println("gX");
  tft.setCursor(5, 80);
  tft.println("gY");
  tft.setCursor(5, 90);
  tft.println("gZ");
  tft.setCursor(5, 115);
  tft.setTextColor(BLUE);
  tft.println("Kalibruoti");
  tft.setCursor(95, 115);
  tft.println("Atgal");
  tft.setTextColor(WHITE);

  
  calib_info_update(calib_old, 0);
  calib_info_update(calib_new, 1);
        
 while (exit_f == 0) { 
  if (Serial.available() > 0) {
      serial_string = "";
      serial_constr();
      if (serial_string.substring(0,1) == ".") {
        if (calib_f == 0) {
          calib_old = getValue(serial_string, '.');
          calib_info_update(calib_old, 2);
          delay(1000);
        }
        else {
          calib_new = getValue(serial_string, '.');
          calib_info_update(calib_new, 3);
          delay(1000);
        }
      }
      else if (serial_string == "ERROR") {
        tft.fillRect(20, 40, 100, 60, color_mask);
        tft.setCursor(40, 60);
        tft.setTextSize(2);
        tft.setTextColor(BLACK);
        tft.println("KLAIDA!");
        exit_f = 1;
        delay(5000);
      }
      else if (serial_string == "DONE") {
         calib_f = 1;
         tft.fillRect(5, 100, 120, 10, color_mask);
         tft.setTextColor(BLUE);
         tft.setCursor(95, 115);
         tft.println("Atgal");
         tft.setTextColor(WHITE);
      }
    }
  if ((PIND & (1<<button_l)) == 0) { // Nuspaustas button_l
    if (started_f == 0) {  
    Serial.print("S");
    delay(30);
    if (serial_string == "OK") {
      tft.fillRect(5, 115, 120, 20, color_mask);
      started_f = 1;
     }
    }
  }
  else if (calib_f^started_f) {
    if (load_s > 3){
      load_s = 0;
    }
    tft.fillRect(5, 100, 120, 10, color_mask);
    tft.setCursor(5, 100);
    switch (load_s){
      case 0: tft.print("------------------- "); break;
      case 1: for(int i = 0; i< 10; i++) {
        tft.print(char(92)); 
        tft.print(char(32));
      }break;
      case 2: tft.print("||||||||||||||||||| "); break;
      case 3: tft.print("/ / / / / / / / / / "); break;
      default: tft.print("------------------- ");
    }
     load_s++;
     delay(50);
  }
  else if (((PIND & (1<<button_r)) == 0) && !(calib_f^started_f)) { // Nuspaustas button_r
    Serial.print("B");
    delay(30);
    exit_f = 1;
  }
  }
  main_window();
}

void calib_info_update (struct calibration values, int R) {
  int X;
  switch (R) {
    case 0:
      X = 40;
      tft.fillRect(20, 40, 50, 60, color_mask);
      break;
    case 1:
      X = 90;
      tft.fillRect(70, 40, 50, 60, color_mask);
      break;
    case 2:
      X = 30;
      tft.fillRect(20, 40, 50, 60, color_mask);
      break;
    case 3:
      X = 70;
      tft.fillRect(70, 40, 50, 60, color_mask);
      break;
    default:
      X = 40;
      tft.fillRect(20, 40, 50, 60, color_mask);
  }
  tft.setCursor(X, 40);
  tft.println(values.accX);
  tft.setCursor(X, 50);
  tft.println(values.accY);
  tft.setCursor(X, 60);
  tft.println(values.accZ);
  tft.setCursor(X, 70);
  tft.println(values.gyrX);
  tft.setCursor(X, 80);
  tft.println(values.gyrY);
  tft.setCursor(X, 90);
  tft.println(values.gyrZ);
}

void LCD_update() {  
  
//  if (digitalRead(button_l) == 0) {
//    if (LCD_light > 10) {
//      LCD_light = LCD_light - 10;
//    }
//    else {
//      LCD_light = 0;
//    }
//    analogWrite(LCD_BCL, LCD_light);
//    LCD_bcl_map = map(LCD_light, 0, 250, 0, 100);
//    delay(150);
//    tft.fillRect(5, 65, 20, 8, color); //LCD_bcl
//    tft.setCursor(5, 65);
//    tft.print(LCD_bcl_map);
//  }
//  else if (digitalRead(button_r) == 0) {
//    if (LCD_light < 240) {
//      LCD_light = LCD_light + 10;
//    }
//    else {
//      LCD_light = 250;
//    }
//    analogWrite(LCD_BCL, LCD_light);
//    LCD_bcl_map = map(LCD_light, 0, 250, 0, 100);
//    delay(150);
//    tft.fillRect(5, 65, 20, 8, color); //LCD_bcl
//    tft.setCursor(5, 65);
//    tft.print(LCD_bcl_map);
//  }
  
  L_X = analogRead(JS_L_X);
  L_X_map = map(L_X, 0, 1023, -100, 100);
  if (L_X_old != L_X_map) {
    L_X_old = L_X_map;
    tft.fillRect(5, 15, 24, 8, color_mask); //JS_L_X
    tft.setCursor(5, 15);
    tft.print(L_X_map-L_X_offset);
  }
  
  R_X = analogRead(JS_R_X);  
  R_X_map = map(R_X, 0, 1023, -ANGLE_LIMIT, ANGLE_LIMIT);
  if (R_X_old != R_X_map) {
    X_change = 1;
    R_X_old = R_X_map;
    tft.fillRect(100, 15, 24, 8, color_mask); //JS_R_X
    tft.setCursor(100, 15);
    tft.println(R_X_map-R_X_offset);
  }
  else {
    X_change = 0;
  }
  
  L_Y = analogRead(JS_L_Y);
  L_Y_map = map(L_Y, 0, 1023, -100, 100);
  if (L_Y_old != L_Y_map) {
    L_Y_old = L_Y_map;
    tft.fillRect(5, 25, 24, 8, color_mask); //JS_L_Y
    tft.setCursor(5, 25);
    tft.print(L_Y_map-L_Y_offset);
  }
  
  R_Y = analogRead(JS_R_Y);
  R_Y_map = map(R_Y, 0, 1023, -ANGLE_LIMIT, ANGLE_LIMIT);
  if (R_Y_old != R_Y_map) {
    Y_change = 1;
    R_Y_old = R_Y_map;
    tft.fillRect(100, 25, 24, 8, color_mask); //JS_R_Y
    tft.setCursor(100, 25);
    tft.println(R_Y_map-R_Y_offset);
  }
  else {
    Y_change = 0;
  }

  if (bat_old != BATTERY) {
    bat_old = BATTERY;
    tft.fillRect(5, 35, 24, 8, color_mask); //bat
    tft.setCursor(5, 35);
    tft.print((2*BATTERY*3.3)/1023);
  }
  
  if (cell_1_old != cell_1){
    cell_1_old = cell_1;
    tft.fillRect(5, 55, 24, 8, color_mask);
    tft.setCursor(5, 55);
    tft.print(cell_1);
  }
    

}

