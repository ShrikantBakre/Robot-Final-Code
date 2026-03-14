#include <CytronMotorDriver.h>
#include "HardwareSerial.h"

#define piston 14

String var;

HardwareSerial espSerial(0);

CytronMD M(PWM_DIR, 4, 2) ;

void setup() {
  espSerial.begin(115200);
  pinMode(piston, OUTPUT);
}

void loop() {
  if (espSerial.available()) {
    var = espSerial.readStringUntil('\n');
    var.trim(); // Remove whitespace and newline
  }

  // Piston control
  if (var == "A") {
    digitalWrite(piston, LOW);  
    // espSerial.println(var) ;
  } else if (var == "a") {
    digitalWrite(piston, HIGH); 
    // espSerial.println(var) ;
  }

  // Motor control
  if (var == "b") {
    M.setSpeed(-150);  // Backward
    espSerial.println(var) ;
  } else if (var == "c") {
    M.setSpeed(150);   // Forward
    espSerial.println(var) ;
  } else if (var == "d") {
    M.setSpeed(0);     // Stop
    espSerial.println(var) ;
  }
  // espSerial.println(var) ;
}