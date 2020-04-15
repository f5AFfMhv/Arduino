/* 
This sketch is for controlling Moon Lamp with android smartphone app.
Microcontroller: ESP8266 (tested with ESP-12E module).

Copyright (C) 2019 Martynas J. 
f5AFfMhv@protonmail.com  
https://github.com/f5AFfMhv

This file is part of mood_lamp_mod.

    mood_lamp_mod is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    mood_lamp_mod is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with mood_lamp_mod.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <ESP8266WiFi.h>

/***************** CHANGE THESE PARAMETERS *****************/
const char* ssid = "YOUR_WIFI_AP_NAME"; // WiFi access point name
const char* password = "YOUR_WIFI_AP_PASSWORD"; // WiFi access point password
/*                                                                 
   +VCC           ESP8266 ADC reference voltage = 1V, resolution 2^10-1=1023.
    |                    
   +-+                    (R1+R2)                    ADCvalue
   | | R1         div_c = -------;    VCC = div_c x ----------;
   +-+                      R2                         1023
    |     ADCvalue
    +-----(0-1023)
    |
   +-+
   | | R2
   +-+
    |
   GND
*/    
const double div_c = 5.7; // voltage divider for ADC (R1 = 47K; R2 = 10K)
int debug = 0; // enable/disable debuging (starts serial interface, sends flag values alongside regular information)
/************************************************************/

// these flags defines state of the program
int blueONflag = 0; // blue LEDs on/off
int orangeONflag = 0; // orange LEDs on/off
int holdFlag = 0; // hold button pressed/released
int updateFlag = 1; // update or not orangeONflag and blueONflag

IPAddress ipaddr;
String response = "";
double inputVoltage = 0.0;

// create an instance of the server
// specify the port to listen on as an argument
WiFiServer server(80);

// function to simulate pushbutton press event
void pushButton();

// initialization function
void setup() {
  if (debug == 1){
    Serial.begin(115200);
    delay(10);
  }
  
  // prepare GPIO2 (pushbutton)
  pinMode(2, OUTPUT);
  digitalWrite(2, 1);

  pinMode(4, INPUT); // orange LED control voltage input
  pinMode(5, INPUT); // blue LED control voltage input
  
  // Connect to WiFi network
  if (debug == 1){
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
  }
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (debug == 1){
      Serial.print(".");
    }
  }
  if (debug == 1){
    Serial.println("");
    Serial.println("WiFi connected");
  }
  
  // Start the server
  server.begin();
  ipaddr = WiFi.localIP();
  if (debug == 1){
    Serial.println("Server started");
    // Print the IP address
    Serial.println(ipaddr);
  }
}

void loop() {
  // check if still connected to WiFI, reconnect if not
  if (WiFi.status() != WL_CONNECTED){
       if (debug == 1){
          Serial.println("Connection lost. Reconnecting");
        }
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        if (debug == 1){
          Serial.print(".");
        }
      }
      if (debug == 1){
          Serial.println("Connected");
       }
  }
  
  // check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
  
  // Wait until the client sends some data
  if (debug == 1){
    Serial.println("new client");
  }
  
  while(!client.available()){
    delay(1);
  }
  
  // Read the first line of the request
  String req = client.readStringUntil('\r');
  client.flush();
  
  // read input voltage
  inputVoltage = div_c*((double)analogRead(A0)/1023.0);
  if (updateFlag == 1){
    // read LEDs controll voltage states
    blueONflag = digitalRead(5);
    orangeONflag = digitalRead(4);
  }
  // match requests:
  // brightness controll match
  if (req.indexOf("/light/0") != -1){
    updateFlag = 0;
    if (holdFlag == 0){
      digitalWrite(2, 0); // simulate button press
      holdFlag = 1;
      response = "Brightness button pressed";
      delay(50);
    }
    else{
      digitalWrite(2, 1); // simulate button release
      holdFlag = 0;
      response = "Brightness button released";  
      delay(10);    
    }
    }
  // blue LED on match
  else if (req.indexOf("/light/1") != -1){
    switch(blueONflag){
      case 0:
        if (orangeONflag == 0){
          pushButton();
        }
        else{
          pushButton();
          pushButton();
          pushButton();
        }
        break;
      case 1:
        if (orangeONflag == 0){
          break;
        }
        else{
          pushButton();
          pushButton();
        }
        break;
      default:
        break;
    }
    response = "Blue light ON";
    updateFlag = 1;
    }
  // orange LED on match
  else if (req.indexOf("/light/2") != -1){
    switch(blueONflag){
      case 0:
        if (orangeONflag == 0){
          pushButton();
          pushButton();
        }
        else{
          break;
        }
        break;
      case 1:
        if (orangeONflag == 0){
          pushButton();
        }
        else{
          pushButton();
          pushButton();
          pushButton();
        }
        break;
      default:
        break;
    }
    response = "Orange light ON";
    updateFlag = 1;
    }
  // blue and orange LED on match
  else if (req.indexOf("/light/3") != -1){
    switch(blueONflag){
      case 0:
        if (orangeONflag == 0){
          pushButton();
          pushButton();
          pushButton();
        }
        else{
          pushButton();
        }
        break;
      case 1:
        if (orangeONflag == 0){
          pushButton();
          pushButton();
        }
        else{
          break;
        }
        break;
      default:
        break;
    }
    response = "White light ON";
    updateFlag = 1;
    }
  // all LED off match
  else if (req.indexOf("/light/4") != -1){
    switch(blueONflag){
      case 0:
        if (orangeONflag == 0){
          break;
        }
        else{
          pushButton();
          pushButton();
        }
        break;
      case 1:
        if (orangeONflag == 0){
          pushButton();
          pushButton();
          pushButton();
        }
        else{
          pushButton();
        }
        break;
      default:
        break;
    }
    response = "Lights OFF";
    updateFlag = 1;
    }
  // just update flags match
  else if (req.indexOf("/light/5") != -1){
    updateFlag = 1;
  }
  else {
    client.stop();
    return;
  }

  client.flush();
  // update flags
  if (updateFlag == 1){
    blueONflag = digitalRead(5);
    orangeONflag = digitalRead(4);
  };
  
  // Prepare the response (putting all info into html <title> tag
  String s = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<!DOCTYPE HTML>\r\n<html>\r\n<head>\r\n<title>\r\n";
  s += "Connected to: ";
  s += ipaddr.toString();
  s += "<br>";
  s += "Response: ";
  s += response;
  s += "<br>";
  s += "Input voltage: \n";
  s += inputVoltage;
  s += "V\n";
  if (debug == 1){
    s += "<br>";
    s += "Blue flag: ";
    s += blueONflag;
    s += "<br>";
    s += "Orange flag: ";
    s += orangeONflag;
    s += "<br>";
    s += "Hold flag: ";
    s += holdFlag;
    s += "<br>";
    s += "Update flag: ";
    s += updateFlag;
  }
  s += "</title>\r\n</head>\r\n</html>\r\n";

  // Send the response to the client
  client.print(s);
  delay(1);
  
  if (debug == 1){
      Serial.println(response);
      Serial.println(inputVoltage);
    }

}

// functions
void pushButton(){
    digitalWrite(2, 0);
    delay(50);
    digitalWrite(2, 1);
    delay(10);
}
