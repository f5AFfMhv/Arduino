  #include "Timer.h"
  
  Timer t;
  
  int A = 8; // 7s pin = arduino digital pin
  int B = 12;
  int C = 7;
  int D = 9;
  int E = 10;
  int F = 11;
  int G = 6;
  int gnd2 = 5; // tens 7s ground pin
  int gnd1 = 13; // first 7s ground pin
  
  int number = 0; // first digit
  int tens = 0; // tens digit
  int delay_time2 = 10; // delay for 7s swap
  
void setup() {                
  // initialize the digital pins as an output.
  /*0 1 2 3 4 5 6 7 8 9
  A + - + + - + + + + +
  B + + + + + - - + + +
  C + + - + + + + + + +
  D + - + + - + + - + -
  E + - + - - - + - + -
  F + - - - + + + - + +
  G - - + + + + + - + +
  DP
  */
  
  pinMode(A, OUTPUT);     
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);     
  pinMode(D, OUTPUT);
  pinMode(E, OUTPUT);
  pinMode(F, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(gnd1, OUTPUT);
  pinMode(gnd2, OUTPUT);
  
  int tickEvent = t.every(500, increment);
}

// the loop routine runs over and over again forever:
void loop() {
  t.update(); 
  digitalWrite(gnd1, LOW);  
  digitalWrite(gnd2, HIGH);
  
 if (number == 0){
  digitalWrite(A, HIGH);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, HIGH);
  digitalWrite(E, HIGH);
  digitalWrite(F, HIGH);
  digitalWrite(G, LOW);
  }
  else if (number == 1){
  digitalWrite(A, LOW);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, LOW);
  digitalWrite(E, LOW);
  digitalWrite(F, LOW);
  digitalWrite(G, LOW);
  }
  else if (number == 2){
  digitalWrite(A, HIGH);  
  digitalWrite(B, HIGH);
  digitalWrite(C, LOW); 
  digitalWrite(D, HIGH);
  digitalWrite(E, HIGH);
  digitalWrite(F, LOW);
  digitalWrite(G, HIGH);
  }
  else if (number == 3){
  digitalWrite(A, HIGH);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, HIGH);
  digitalWrite(E, LOW);
  digitalWrite(F, LOW);
  digitalWrite(G, HIGH);
  }
  else if (number == 4){
  digitalWrite(A, LOW);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, LOW);
  digitalWrite(E, LOW);
  digitalWrite(F, HIGH);
  digitalWrite(G, HIGH);
  }
  else if (number == 5){
  digitalWrite(A, HIGH);  
  digitalWrite(B, LOW);
  digitalWrite(C, HIGH); 
  digitalWrite(D, HIGH);
  digitalWrite(E, LOW);
  digitalWrite(F, HIGH);
  digitalWrite(G, HIGH);
  }
  else if (number == 6){
  digitalWrite(A, HIGH);  
  digitalWrite(B, LOW);
  digitalWrite(C, HIGH); 
  digitalWrite(D, HIGH);
  digitalWrite(E, HIGH);
  digitalWrite(F, HIGH);
  digitalWrite(G, HIGH);
  }
  else if (number == 7){
  digitalWrite(A, HIGH);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, LOW);
  digitalWrite(E, LOW);
  digitalWrite(F, LOW);
  digitalWrite(G, LOW);
  }
  else if (number == 8){
  digitalWrite(A, HIGH);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, HIGH);
  digitalWrite(E, HIGH);
  digitalWrite(F, HIGH);
  digitalWrite(G, HIGH);
  }
  else if (number == 9){
  digitalWrite(A, HIGH);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, LOW);
  digitalWrite(E, LOW);
  digitalWrite(F, HIGH);
  digitalWrite(G, HIGH);
  number = 0;
  tens++;
  }
  
  delay(delay_time2); 
  digitalWrite(gnd1, HIGH);  
  digitalWrite(gnd2, LOW);
  
  if (tens == 0){
  digitalWrite(A, HIGH);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, HIGH);
  digitalWrite(E, HIGH);
  digitalWrite(F, HIGH);
  digitalWrite(G, LOW);
  }
  else if (tens == 1){
  digitalWrite(A, LOW);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, LOW);
  digitalWrite(E, LOW);
  digitalWrite(F, LOW);
  digitalWrite(G, LOW);
  }
  else if (tens == 2){
  digitalWrite(A, HIGH);  
  digitalWrite(B, HIGH);
  digitalWrite(C, LOW); 
  digitalWrite(D, HIGH);
  digitalWrite(E, HIGH);
  digitalWrite(F, LOW);
  digitalWrite(G, HIGH);
  }
  else if (tens == 3){
  digitalWrite(A, HIGH);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, HIGH);
  digitalWrite(E, LOW);
  digitalWrite(F, LOW);
  digitalWrite(G, HIGH);
  }
  else if (tens == 4){
  digitalWrite(A, LOW);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, LOW);
  digitalWrite(E, LOW);
  digitalWrite(F, HIGH);
  digitalWrite(G, HIGH);
  }
  else if (tens == 5){
  digitalWrite(A, HIGH);  
  digitalWrite(B, LOW);
  digitalWrite(C, HIGH); 
  digitalWrite(D, HIGH);
  digitalWrite(E, LOW);
  digitalWrite(F, HIGH);
  digitalWrite(G, HIGH);
  }
  else if (tens == 6){
  digitalWrite(A, HIGH);  
  digitalWrite(B, LOW);
  digitalWrite(C, HIGH); 
  digitalWrite(D, HIGH);
  digitalWrite(E, HIGH);
  digitalWrite(F, HIGH);
  digitalWrite(G, HIGH);
  }
  else if (tens == 7){
  digitalWrite(A, HIGH);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, LOW);
  digitalWrite(E, LOW);
  digitalWrite(F, LOW);
  digitalWrite(G, LOW);
  }
  else if (tens == 8){
  digitalWrite(A, HIGH);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, HIGH);
  digitalWrite(E, HIGH);
  digitalWrite(F, HIGH);
  digitalWrite(G, HIGH);
  }
  else if (tens == 9){
  digitalWrite(A, HIGH);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, LOW);
  digitalWrite(E, LOW);
  digitalWrite(F, HIGH);
  digitalWrite(G, HIGH);
  tens = 0;
  }
  delay(delay_time2);
}

void increment()
{
  if (number == 10)
     number = 0;
  else 
     number++;
}
