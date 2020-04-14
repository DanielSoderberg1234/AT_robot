/*
 * created by Martin Gordon, gordonm@student.chalmers.se
 * 
 * The code was created for the use of several ultrasonic
 * sensors simultaneously. It takes a pointer to the first
 * object of an array with pins and the total number of pins.
 * The function 'distance()' return the smallest distance of
 * all sensors in cm. 
 *  
 
 */

#include "Arduino.h"
#include "USSensor.h"

USSensor::USSensor(int *pointer, int n)
{
    nPins = n;        //Number of pins
    pinPointer = pointer;  //Pointer to the pin-array
    
}

int USSensor::distance()
{


  
  /*Set all pins to OUTPUT
  and make sure everything is set to LOW*/
  for(int i = 0; i < nPins; i++) {
    pinMode(pinPointer[i], OUTPUT);
    digitalWrite(pinPointer[i], LOW);
  } delayMicroseconds(2);


  /*Send out a HIGH pulse for every pin, 
   wait 14 microseconds, 
   then set every pin to LOW
    */
  signalTime = micros();
  for(int i = 0; i < nPins; i++) { 
    digitalWrite(pinPointer[i], HIGH);
  } 
  while(signalTime+14>micros()){}
  for(int i = 0; i < nPins; i++) { //16 us
    digitalWrite(pinPointer[i], LOW);
  }


  //Set all pins to input
  for(int i = 0; i < nPins; i++) {
    pinMode(pinPointer[i], INPUT);
  }
  //Start timer to check if no signal is received
  signalError = millis();
  /*Loop through every pin and if a pin is HIGH(has 
   received a signal), then measure the signal and
  return the distance in cm*/
  for(int i = 0; signalError+100>millis() ; i = (i+1)%(nPins)) {
    if(digitalRead(pinPointer[i]) == HIGH) {
      timerStart = micros();
      while(digitalRead(pinPointer[i]) == HIGH){}
      duration = (micros()-timerStart);
      cm = (duration/2) / 29.1;   
      return cm;
    }
  }
  /*if no signal is received before 0.1 seconds passed,
  return -1.*/
  return -1;
}



int* USSensor::getPointer()
{
  return pinPointer;
}
