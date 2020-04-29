 /*
 * created by Rui Santos, https://randomnerdtutorials.com
 * 
 * Complete Guide for Ultrasonic Sensor HC-SR04
 *
    Ultrasonic sensor Pins:
        VCC: +5VDC
        Trig : Trigger (INPUT) - Pin11
        Echo: Echo (OUTPUT) - Pin 12
        GND: GND
*/

 /*
int senPin = 9; 
int senPins[] = {9, 10};   
long duration, cm, inches;
unsigned long timerStart = 0;
unsigned long signalTime = 0;
unsigned long signalError = 0;
unsigned long operationTimer = 0;
 
void setup() {
  //Serial Port begin
  Serial.begin (9600);
  //Define inputs and outputs
  pinMode(senPin, OUTPUT);
  pinMode(senPin, INPUT);
}
 
void loop() {
  


  //Send signal for each sensor
  for(int i = 0; i < sizeof(senPins)/2; i++) {
    pinMode(senPins[i], OUTPUT);
    digitalWrite(senPins[i], LOW);
  } delayMicroseconds(2);

  signalTime = micros();
  for(int i = 0; i < sizeof(senPins)/2; i++) { 
    digitalWrite(senPins[i], HIGH);
  } 
  while(signalTime+14>micros()){}
  for(int i = 0; i < sizeof(senPins)/2; i++) { //16 us
    digitalWrite(senPins[i], LOW);
  }
  
  
  //pinMode(senPins[0], OUTPUT);
  //digitalWrite(senPins[0], LOW);
  //delayMicroseconds(5);
  //digitalWrite(senPins[0], HIGH);
  //delayMicroseconds(10);
  //digitalWrite(senPins[0], LOW);
 


  //operationTimer = micros();
  //operationTimer = micros()-operationTimer;
  //Serial.println(operationTimer);
  
  
  for(int i = 0; i < sizeof(senPins)/2; i++) {
    pinMode(senPins[i], INPUT);
  }
  signalError = millis();
  for(int i = 0; signalError+100>millis() ; i = (i+1)%(sizeof(senPins)/2)) {
    if(digitalRead(senPins[i]) == HIGH) {
      timerStart = micros();
      while(digitalRead(senPins[i]) == HIGH){}
      duration = (micros()-timerStart);
      cm = (duration/2) / 29.1;   

      Serial.print(cm);
      Serial.print(" cm");
      Serial.println();
  
      break;
    }
  }

  

  /*
  pinMode(senPins[0], INPUT);
  signalError = millis();
  while(digitalRead(senPins[0]) == LOW && signalError+100>millis()){}
  timerStart = micros();
  while(digitalRead(senPins[0]) == HIGH){}
  duration = (micros()-timerStart);
  cm = (duration/2) / 29.1;     

   
  Serial.print(cm);
  Serial.print(" cm");
  Serial.println();
* /

  
  //operationTimer = millis()-operationTimer;
  //Serial.println(operationTimer);
  delay(250);
  Serial.print("hej");
}

 */
