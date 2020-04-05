  int A = 8;
  int B = 12;
  int C = 7;
  int D = 9;
  int E = 10;
  int F = 11;
  int G = 6;
  int gnd2 = 5;
  int gnd1 = 13;
  
  int number = 0;
  int delay_time = 500;
  
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
}

// the loop routine runs over and over again forever:
void loop() {
  
  digitalWrite(gnd1, LOW);
  
  if (number == 0){
  digitalWrite(A, HIGH);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, HIGH);
  digitalWrite(E, HIGH);
  digitalWrite(F, HIGH);
  digitalWrite(G, LOW);
  number++;
  delay(delay_time);
  }
  else if (number == 1){
  digitalWrite(A, LOW);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, LOW);
  digitalWrite(E, LOW);
  digitalWrite(F, LOW);
  digitalWrite(G, LOW);
  number++;
  delay(delay_time);
  }
  else if (number == 2){
  digitalWrite(A, HIGH);  
  digitalWrite(B, HIGH);
  digitalWrite(C, LOW); 
  digitalWrite(D, HIGH);
  digitalWrite(E, HIGH);
  digitalWrite(F, LOW);
  digitalWrite(G, HIGH);
  number++;
  delay(delay_time);
  }
  else if (number == 3){
  digitalWrite(A, HIGH);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, HIGH);
  digitalWrite(E, LOW);
  digitalWrite(F, LOW);
  digitalWrite(G, HIGH);
  number++;
  delay(delay_time);
  }
  else if (number == 4){
  digitalWrite(A, LOW);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, LOW);
  digitalWrite(E, LOW);
  digitalWrite(F, HIGH);
  digitalWrite(G, HIGH);
  number++;
  delay(delay_time);
  }
  else if (number == 5){
  digitalWrite(A, HIGH);  
  digitalWrite(B, LOW);
  digitalWrite(C, HIGH); 
  digitalWrite(D, HIGH);
  digitalWrite(E, LOW);
  digitalWrite(F, HIGH);
  digitalWrite(G, HIGH);
  number++;
  delay(delay_time);
  }
  else if (number == 6){
  digitalWrite(A, HIGH);  
  digitalWrite(B, LOW);
  digitalWrite(C, HIGH); 
  digitalWrite(D, HIGH);
  digitalWrite(E, HIGH);
  digitalWrite(F, HIGH);
  digitalWrite(G, HIGH);
  number++;
  delay(delay_time);
  }
  else if (number == 7){
  digitalWrite(A, HIGH);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, LOW);
  digitalWrite(E, LOW);
  digitalWrite(F, LOW);
  digitalWrite(G, LOW);
  number++;
  delay(delay_time);
  }
  else if (number == 8){
  digitalWrite(A, HIGH);  
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH); 
  digitalWrite(D, HIGH);
  digitalWrite(E, HIGH);
  digitalWrite(F, HIGH);
  digitalWrite(G, HIGH);
  number++;
  delay(delay_time);
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
  delay(delay_time);
  }
}
