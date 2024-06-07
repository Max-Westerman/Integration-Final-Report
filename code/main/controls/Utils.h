#ifndef UTILS_H
#define UTILS_H
// =====================

#include <Arduino.h>

void endProgram(){
  Serial.println("Ended program.");
  while (true){
    delay(1000);
  }
}

#endif