/* 
This sketch is for quadcopter (similar to Syma X5C).
Microcontroller: Atmega 328P.

Copyright (C) 2019 Martynas J. 
f5AFfMhv@protonmail.com  
https://github.com/f5AFfMhv
 */

/*****************************Bibliotekos*****************************/
#include <avr/interrupt.h>
#include <Wire.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "Kalman.h"
/*******************************Isvadai*******************************/
// Sviesos diodai
#define green_led 2 //PC2
#define red_led 1 //PC1

#define USB_on 7 //PD7
// Varikliai
#define M1 6 // PD6
#define M2 5 // PD5
#define M3 1 // PB1
#define M4 2 // PB2
// Akumuliatoriaus itampa
#define bat_half A6 

/***********************Kintamieji ir konstantos*********************/
int send_count = 0;
const int MPU=0x69;  // MPU-6050 I2C adresas
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float cell_1;
char incomingByte = ' '; 
String serial_string = "";
int PWM[4] = {0, 0, 0, 0};
int PWM_old[4] = {255, 255, 255, 255};
int pwm = 0;
int pwm_old = 255;
int X_old = 360;
int Y_old = 360;
int Z_old = 360;
bool ON_flag = 0;

// EEPROM atmintyje saugomu verciu adresai
int calibr_addr = 0;

// Kintamieji PID valdikliams
double Xdif, Ydif, Zdif;
double Xdif_old, Ydif_old, Zdif_old;
double setX, setX_old, setY, setZ;
double X, Y, Z;
double X_sum, Y_sum, Z_sum;

/**********************Parametrai**********************/
                    double KP = 2.4; 
                    double KI = 0; 
                    double KD = 2.5;
                    const int MAX_PWM = 100;
                    int LIMIT = 100;
                    bool debug = 0;
/******************************************************/

PID X_PID(&X, &Xdif, &setX,KP,KI,KD, DIRECT);
//PID Y_PID(&Y, &Ydif, &setY,KP,KI,KD, DIRECT);
//PID Z_PID(&Z, &Zdif, &setZ,KP,KI,KD, DIRECT);

// Kintamieji skirti kalibravimui
int buffersize=1000;     //Matavimu skaicius, daugiau matavimu - didesnis tikslumas, bet ilgesne trukme  (default:1000)
int acel_deadzone=8;     //Leistina akselerometro paklaida  (default:8)
int giro_deadzone=1;     //Leistina giroskopo paklaida  (default:1)
int devStatus;

// Kintamieji MPU6050 
MPU6050 mpu(MPU);

volatile int interrupt_count = 0;

int16_t ax, ay, az, gx, gy, gz;
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

struct calibration {
  int16_t accX;
  int16_t accY; 
  int16_t accZ;
  int16_t gyrX;
  int16_t gyrY;
  int16_t gyrZ;
};

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
int prv_angle = 999;
/***********************Pertraukciu paprogrames***********************/
ISR(TIMER2_OVF_vect) { // (1024 / 8M) * 255
   interrupt_count++;
   if (interrupt_count == 3){ // Ivyksta apytiksliai kas 0.1s (1024 / 8M) * 255 * 3 = 0.1
    interrupt_count = 0;
    serial_update();
   }
}

/****************************MV nustatymas****************************/
void setup() {
//int main() {
  // MV isvadu nustatymas
  DDRB |= (1<<M3)|(1<<M4); // M3, M4 - isejimai
  DDRC |= (1<<red_led)|(1<<green_led); // green_led, red_led - isejimai
  DDRD |= (1<<M2)|(1<<M1); // M1, M2 - isejimai
  DDRD &= ~(1<<USB_on); // USB_on - iejimas

  // Pertraukciu nustatymas
  TCCR2A = 0; // TMR2 nustatomas normaliam veikimui
  TIMSK2 |= (1<<TOIE2); // TMR2 perpildos pertraukties aktyvavimas
  sei(); // Globalus pertraukciu ijungimas
  TCCR2B |= (1<<CS22)|(1<<CS21)|(1<<CS20); // System clock 1024 prescaler

  // Timer0 ir Timer1 nustatymas
  //TIMER0 default (64) prescaler pakeistas i 1, TIMER0 naudojamas delay() funkcijoje
  TCCR0B = 0b00000001; // set timer 0 divisor to 1 for PWM frequency of 31250 Hz, kai clk = 8MHz
  TCCR1B = 0b00001001; // set timer 1 divisor to 1 for PWM frequency of 31250 Hz, kai clk = 8MHz
  
  // Nuosekliojo monitoriaus inicializacija
  Serial.begin(38400);
  
  // Pid valdikliu aktyvavimas
    Xdif = 0.0;
    Ydif = 0.0;
    //Zdif = 0.0;
    setX = 0.0;
    setY = 0.0;
    //setZ = 0.0;
    X = 0.0;
    Y = 0.0;
    //Z = 0.0;

    X_PID.SetOutputLimits(-LIMIT, LIMIT);
    //Y_PID.SetOutputLimits(-LIMIT, LIMIT);
    //Z_PID.SetOutputLimits(-LIMIT, LIMIT);
    
  // Pradines isvadu busenos
  // Varikliu valdymo isvadams priskiriamas zemas loginis lygis
  PORTB &= ~(1<<M3) & ~(1<<M4);
  PORTD &= ~(1<<M2) & ~(1<<M1);
  // Sviesos diodai
  PORTC &= ~(1<<green_led);
  PORTC |= (1<<red_led);

  mpu_start();
  
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(6400); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);

  timer = micros();
}
/*************************Pagrindine programa**************************/
void loop() {
  //Jei kraunama pereinama i itampos stebejima
   if ((PIND & (1<<USB_on)) == (1<<USB_on)) {
      ON_flag = 0;
      for(int j=0;j<4;j++){
        PWM[j] = 0;
      }
      PWM_update();
      pwm = 0;
      // Isjungiamas varikliu PWM
      TCCR0A = 0;
      TCCR1A = 0;
      // Varikliu valdymo isvadams priskiriamas zemas loginis lygis
      PORTB &= ~(1<<M3) & ~(1<<M4);
      PORTD &= ~(1<<M2) & ~(1<<M1);
      All_PID_disable();
      PORTC &= ~(1<<green_led);
      PORTC &= ~(1<<red_led);
      while((PIND & (1<<USB_on)) == (1<<USB_on)){
        delay(320000);
        Serial.print("T");
        delay(320000);
        battery_update();
      }
      PORTC |= (1<<red_led);
    }
     
   if (Serial.available() > 0) {
    serial_string = "";
    serial_constr();
    if (serial_string == "V") {
      battery_update();
    }
    else if (serial_string == "EN") {
      ON_flag = 1;
      //PWM nustatymas
      X_PID.SetMode(AUTOMATIC);
      //Y_PID.SetMode(AUTOMATIC);
      //Z_PID.SetMode(AUTOMATIC);
      //M1 ir M2
      TCCR0A = 0b10100011; // non-inverting, fast PWM mode on OC0A and OC0B 
      //M3 ir M4
      TCCR1A = 0b10100001; // non-inverting, fast PWM mode on OC1A and OC1B
       
      PORTC &= ~(1<<red_led);
      PORTC |= (1<<green_led); 
      Serial.print("OK");
    }
    else if (serial_string == "DIS") {
      ON_flag = 0;
      for(int j=0;j<4;j++){
        PWM[j] = 0;
      }
      PWM_update();
      pwm = 0;
      // Isjungiamas varikliu PWM
      TCCR0A = 0;
      TCCR1A = 0;
      // Varikliu valdymo isvadams priskiriamas zemas loginis lygis
      PORTB &= ~(1<<M3) & ~(1<<M4);
      PORTD &= ~(1<<M2) & ~(1<<M1);
      All_PID_disable();
      PORTC &= ~(1<<green_led);
      PORTC |= (1<<red_led);
      Serial.print("OK");
    }
    else if (serial_string == "K" && ON_flag == 0) {
      Serial.print("OK");
      IMU_calibration();
    }
    else if (serial_string.substring(0,2) == "kp" && ON_flag == 0) {
      KP = (serial_string.substring(2)).toFloat();
      X_PID.SetTunings(KP, KI, KD);
      //Y_PID.SetTunings(KP, KI, KD);
      //Z_PID.SetTunings(KP, KI, KD);
      if (debug) {
        Serial.println();
        Serial.println(KP);
        Serial.println(KI);
        Serial.println(KD);
        Serial.println();
      }
    }
    else if (serial_string.substring(0,2) == "ki" && ON_flag == 0) {
      KI = (serial_string.substring(2)).toFloat();
      X_PID.SetTunings(KP, KI, KD);
      //Y_PID.SetTunings(KP, KI, KD);
      //Z_PID.SetTunings(KP, KI, KD);
      if (debug) {
        Serial.println();
        Serial.println(KP);
        Serial.println(KI);
        Serial.println(KD);
        Serial.println();
      }
    }
    else if (serial_string.substring(0,2) == "kd" && ON_flag == 0) {
      KD = (serial_string.substring(2)).toFloat();
      X_PID.SetTunings(KP, KI, KD);
      //Y_PID.SetTunings(KP, KI, KD);
      //Z_PID.SetTunings(KP, KI, KD);
      if (debug) {
        Serial.println();
        Serial.println(KP);
        Serial.println(KI);
        Serial.println(KD);
        Serial.println();
      }
    }

    else if (serial_string.substring(0,1) == "A") {
      KP=KP+0.1;
      X_PID.SetTunings(KP, KI, KD);
    }
     else if (serial_string.substring(0,1) == "Z") {
      KP=KP-0.1;
      X_PID.SetTunings(KP, KI, KD);
    }
    else if (serial_string.substring(0,1) == "J") {
      KD=KD+0.1;
      X_PID.SetTunings(KP, KI, KD);
    }
     else if (serial_string.substring(0,1) == "N") {
      KD=KD-0.1;
      X_PID.SetTunings(KP, KI, KD);
    }
    else if (serial_string.substring(0,1) == "H") {
      KI=KI+0.1;
      X_PID.SetTunings(KP, KI, KD);
    }
     else if (serial_string.substring(0,1) == "B") {
      KI=KI-0.1;
      X_PID.SetTunings(KP, KI, KD);
    }
    if (ON_flag == 1) {
   
     if (serial_string == "P1") {
      if (pwm < MAX_PWM) {
          pwm++;
      }
     }
     else if (serial_string == "P5") {

      if (pwm <= MAX_PWM-5) {
          pwm += 5;
      }
      else if (pwm > 255) {
          pwm = 255;
      }
      
     }
     else if (serial_string == "M1") {
      if (pwm > 0) {
          pwm--;
      }
     }
     else if (serial_string == "M5") {
      if (pwm >= 5) {
          pwm -= 5;
      }
      else if (pwm < 5) {
          pwm = 0;
      }
     }
     // Pasvirimo kampu valdymas
     else if (serial_string.substring(0,1) == "X") {
      setX = (serial_string.substring(1)).toInt();
     }
     else if (serial_string.substring(0,1) == "Y") {
      setY = (serial_string.substring(1)).toInt();
     }
    }
   }
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 64000000; // Calculate delta time                       
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
   roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    kalAngleY = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif
  delay(128);                                                                                     
/*************VYKSTA PAPILDOMAS KAMPU FILTRAVIMAS**********************/
  if (prv_angle == 999){
    prv_angle = round(kalAngleX);
  }
  else{
    if ((round(kalAngleX) < (prv_angle - 30)) || (round(kalAngleX) > (prv_angle + 30))){
      kalAngleX = prv_angle;
    }
    else {
      prv_angle = kalAngleX;
    }
  }
          X = round(kalAngleX);
          Y = round(kalAngleY);

          //Serial.print(X); Serial.print("\t");
          //Serial.print(Y); Serial.print("\t");
          //Serial.print(round(Xdif)); Serial.print("\t");
          //Serial.println();
  
          PID_update();
          PWM[0] = pwm - round(Xdif);
          //PWM[1] = pwm - round(Xdif);
          //PWM[2] = pwm + round(Xdif);
          PWM[3] = pwm + round(Xdif);

          for(int n=0;n<4;n++){
            if(PWM[n] > 255){
              PWM[n] = 255;
          }
            if(PWM[n] < 0){
              PWM[n] = 0;
          }
          }
          PWM_update();
           
}

/*****************************Paprogrames******************************/
uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(MPU);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(MPU);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(MPU, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}

void PID_update(){
    if (pwm >= 100) {
           //Update Pids
           X_PID.Compute();
           //Y_PID.Compute();
           //Z_PID.Compute();
    }
}

void All_PID_disable() {
    Xdif = 0.0;
    Ydif = 0.0;
    //Zdif = 0.0;
    setX = 0.0;
    setY = 0.0;
    //setZ = 0.0;
    X = 0.0;
    Y = 0.0;
    //Z = 0.0;
    
    X_PID.SetMode(MANUAL);
    //Y_PID.SetMode(MANUAL);
    //Z_PID.SetMode(MANUAL);
}

void mpu_start() {
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(1);
  if (debug == 1) {
    if (mpu.testConnection()) {
      Serial.println("MPU connected");
     }
    else {
      Serial.println("MPU connection failed");
    }
  }
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();
  if (debug == 1){
    if (!devStatus) {
      Serial.println("DMP initialized");
   }
    else {
      Serial.println("DMP initialized failed");
    }
  }
  MPU_set_offsets();
}

void serial_update() {
if (send_count < 6) {
  switch(send_count) {
  case 0:
  if (X_old != X) {
    X_old = X;
    Serial.print("X");
    Serial.print(X);
  }
  break;
  case 1:
  if (Y_old != Y) {
    Y_old = Y;
    Serial.print("Y");
    Serial.print(Y);
  }
  break;
  case 2:
  if (ON_flag) {
    if(PWM_old[0] != PWM[0]){
      PWM_old[0] = PWM[0];
      Serial.print("Gp");
      Serial.print(PWM[0]);
    }
  }
  break;
  case 3:
  if (ON_flag) {
    if(PWM_old[1] != PWM[1]){
      PWM_old[1] = PWM[1];
      Serial.print("Px");
      Serial.print(PWM[1]);
    }
  }
  break;
  case 4:
  if (ON_flag) {
  if (PWM_old[2] != PWM[2]) {
    PWM_old[2] = PWM[2];
    Serial.print("Py");
    Serial.print(PWM[2]);
  }
  }
  break;
  case 5:
  if (ON_flag) {
  if (PWM_old[3] != PWM[3]) {
    PWM_old[3] = PWM[3];
    Serial.print("Ps");
    Serial.print(PWM[3]);
  }
  }
  break;
  default:
  break;
  }
  send_count++;
}
else {
  send_count = 0;
}
}

void serial_constr() {
  while (Serial.available() > 0) {
    incomingByte = Serial.read();
    serial_string += incomingByte;
    delay(64); // 1ms
  }
}

void battery_update(){
  cell_1 = (2*3.27*analogRead(bat_half))/1023;
  Serial.print("C");
  Serial.print(cell_1);
  delay(6400); // 100ms
}

void PWM_update() {
  // M1
  OCR0A = PWM[0];
  // M2
  OCR0B = PWM[1];
  // M3
  OCR1A = PWM[2];
  // M4
  OCR1B = PWM[3];
}

void MPU_set_offsets() {
  calibration offsets;
  EEPROM.get(calibr_addr, offsets);
  delay(19200); // 300ms

  mpu.setXAccelOffset(offsets.accX);
  mpu.setYAccelOffset(offsets.accY);
  mpu.setZAccelOffset(offsets.accZ);
  mpu.setXGyroOffset(offsets.gyrX);
  mpu.setYGyroOffset(offsets.gyrY);
  mpu.setZGyroOffset(offsets.gyrZ);
}

void IMU_calibration() {
  int calib_flag = 0; 
  int start_flag = 0;
  int state = 0;
  calibration calib_old;
  
  EEPROM.get(calibr_addr, calib_old);
  delay(128000); // 2s
  Serial.print(".");
  Serial.print(calib_old.accX);
  Serial.print(".");
  Serial.print(calib_old.accY);
  Serial.print(".");
  Serial.print(calib_old.accZ);
  Serial.print(".");
  Serial.print(calib_old.gyrX);
  Serial.print(".");
  Serial.print(calib_old.gyrY);
  Serial.print(".");
  Serial.print(calib_old.gyrZ);
  delay(128000); // 2s

  do {
    if (Serial.available() > 0) {
      serial_string = "";
      serial_constr();
      if (serial_string == "S") {
        calib_flag = 1;
        start_flag = 1;
        Serial.print("OK");
      }
      else if (serial_string == "B") {
        calib_flag = 1;
        start_flag = 0;
        Serial.print("OK");
      }
    }
  }while (calib_flag == 0);
  
  if (start_flag == 1) {
  delay(64000); // 1s
  // verify connection
  Serial.println(mpu.testConnection() ? " " : "ERROR");
  delay(64000); // 1s
  
  // reset offsets
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  if (state==0){
    meansensors();
    state++;
    delay(128000); // 2s
  }

  if (state==1) {
    offsetsensors();
    state++;
    delay(128000); // 2s
  }

  if (state==2) {
    meansensors();
    calibration calib_new = {
    ax_offset,
    ay_offset,
    az_offset,
    gx_offset,
    gy_offset,
    gz_offset
  };
    EEPROM.put(calibr_addr, calib_new);

    if (debug) {
      Serial.println(mean_ax); 
      Serial.println(mean_ay); 
      Serial.println(mean_az); 
      Serial.println(mean_gx); 
      Serial.println(mean_gy); 
      Serial.println(mean_gz);
    }
   
  delay(128000); // 2s 
  Serial.print("DONE");
  delay(128000); // 2s
  Serial.print(".");
  Serial.print(calib_new.accX);
  Serial.print(".");
  Serial.print(calib_new.accY);
  Serial.print(".");
  Serial.print(calib_new.accZ);
  Serial.print(".");
  Serial.print(calib_new.gyrX);
  Serial.print(".");
  Serial.print(calib_new.gyrY);
  Serial.print(".");
  Serial.print(calib_new.gyrZ);
  delay(128000); // 2s
  }
}
}

void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
      i++;
    delay(128); // 2ms Needed so we don't get repeated measures
  }
}

void offsetsensors(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    meansensors();

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }
}


