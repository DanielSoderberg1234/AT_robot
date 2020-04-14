#ifndef USSensor_h
#define USSensor_h
#include "Arduino.h"

class USSensor
{
  public:
    USSensor(int *pointer, int n);
    int distance();
    int* getPointer();
  private:
    int nPins;        //Number of pins
    int *pinPointer;  //Pointer to the pin-array
    unsigned long timerStart;
    unsigned long signalTime;
    unsigned long signalError;
    long duration;
    long cm;
    

};
#endif
