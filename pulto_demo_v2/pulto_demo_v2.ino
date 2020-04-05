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
char incomingByte = ' '; 
String serial_string = "";
int color = RED;
int interrupt_count = 0;

TFT_ILI9163C tft = TFT_ILI9163C(__CS, __DC, __RST); // Ekrano isvadu susiejimas su biblioteka
/***********************Pertraukciu paprogrames***********************/
ISR(TIMER2_OVF_vect) { // (1024 / 12M) * 255
   interrupt_count++;
   if (interrupt_count == 8272){ // Ivyksta apytiksliai kas 3min (1024 / 12M) * 255 * 8272 = 180
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
  Serial.begin(9600);
   
  // Ekrano nustatymas
  tft.begin(); // Ekrano inicializacija
  tft.fillScreen(RED); // Isvalomas ekranas
  tft.setTextColor(WHITE);  
  tft.setTextSize(1);
  tft.setCursor(60, 5);
  tft.print("X");
  tft.setCursor(60, 25);
  tft.print("Y");
  tft.setCursor(30, 45);
  tft.print(" V");
  tft.setCursor(60, 45);
  tft.println("Ak. itampa");
  tft.setCursor(60, 65);
  tft.println("Apsvietimas");
  tft.setCursor(5, 90);
  tft.println("Nuosekl.mon.[9600]:");
  tft.setCursor(5, 65);
  LCD_bcl_map = map(LCD_light, 0, 250, 0, 100);
  tft.print(LCD_bcl_map);
  Battery_update();
  
  // Vairalazdziu 0 padeties nuskaitymas
  L_X_offset = map(analogRead(JS_L_X), 0, 1023, -100, 100);
  R_X_offset = map(analogRead(JS_R_X), 0, 1023, -100, 100);
  L_Y_offset = map(analogRead(JS_L_Y), 0, 1023, -100, 100);
  R_Y_offset = map(analogRead(JS_R_Y), 0, 1023, -100, 100);
  
}
/*************************Pagrindine programa**************************/
void loop() {
 
  if ((PIND & (1<<JS_R_B)) == 0) { // Nuspaustas JS_R_B
    PORTD |= (1<<green_led);
    delay(30);
  }
  else if ((PINC & (1<<JS_L_B)) == 0) { // Nuspaustas JS_L_B
    PORTC |= (1<<red_led);
    delay(30);
  }
  else {
    PORTD &= ~(1<<green_led);
    PORTC &= ~(1<<red_led);
  }
  if (Serial.available() > 0) {
    serial_string = "";
    serial_constr();
    tft.fillRect(0, 100, 135, 30, color); //Serial
    tft.setCursor(5, 110);
    tft.println(serial_string);
  }

  LCD_update();           
}
/*****************************Paprogrames******************************/
void serial_constr() {
  while (Serial.available() > 0) {
    incomingByte = Serial.read();
    serial_string += incomingByte;
    delay(1);
  }
}

void Battery_update() {
  BATTERY = analogRead(bat);
  if (bat_old != BATTERY) {
    bat_old = BATTERY;
    tft.fillRect(5, 45, 24, 8, color); //bat
    tft.setCursor(5, 45);
    tft.print((2*BATTERY*3.3)/1023);
  }
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
    tft.fillRect(5, 5, 24, 8, color); //JS_L_X
    tft.setCursor(5, 5);
    tft.print(L_X_map-L_X_offset);
  }
  
  R_X = analogRead(JS_R_X);  
  R_X_map = map(R_X, 0, 1023, -100, 100);
  if (R_X_old != R_X_map) {
    R_X_old = R_X_map;
    tft.fillRect(100, 5, 24, 8, color); //JS_R_X
    tft.setCursor(100, 5);
    tft.println(R_X_map-R_X_offset);
  }
  
  L_Y = analogRead(JS_L_Y);
  L_Y_map = map(L_Y, 0, 1023, -100, 100);
  if (L_Y_old != L_Y_map) {
    L_Y_old = L_Y_map;
    tft.fillRect(5, 25, 24, 8, color); //JS_L_Y
    tft.setCursor(5, 25);
    tft.print(L_Y_map-L_Y_offset);
  }
  
  R_Y = analogRead(JS_R_Y);
  R_Y_map = map(R_Y, 0, 1023, -100, 100);
  if (R_Y_old != R_Y_map) {
    R_Y_old = R_Y_map;
    tft.fillRect(100, 25, 24, 8, color); //JS_R_Y
    tft.setCursor(100, 25);
    tft.println(R_Y_map-R_Y_offset);
  }

}

