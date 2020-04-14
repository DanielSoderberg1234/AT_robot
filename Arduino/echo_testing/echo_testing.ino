 /*
 * created by Martin Gordon, gordonm@student.chalmers.se
 * 
 * Example code using the USSensor library.
 *
    Ultrasonic sensor Pins:
        VCC: +5VDC
        Trig/Echo : (INPUT/OUTPUT) - Pin2-Pin13
        GND: GND
 */
 
#include <USSensor.h>

//Array of all sensor pins 
int senPins[] = {2, 10};  
long distanceCm;
unsigned long operationTimer = 0;

/*Create an object with USSensor(int *pointer, int n), where 
*pointer is a pointer to the first object of the pin-array, 
and n is the total number of pins*/
USSensor ussensor(&senPins[0], sizeof(senPins)/sizeof(senPins[0]));  


void setup() {
  Serial.begin (9600);
}
 
void loop() {

  operationTimer = millis(); //start operation timer
  distanceCm = ussensor.distance();
  operationTimer = millis()-operationTimer; //end operation timer
  
  Serial.print(distanceCm);
  Serial.print(" cm");
  Serial.print("\t\t");
  Serial.print(operationTimer);
  Serial.print(" ms");
  Serial.println();
  delay(1000);
}
