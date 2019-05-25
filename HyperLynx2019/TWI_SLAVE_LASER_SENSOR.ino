/*
 *    Code To Track Reflective Stripes Using Laser Sensors
 *    Initializes Arduino as I2C Slave device to communicate with the RPi
 */

#include <Wire.h>
#define SLAVE_ADDR 0x5F         //Slave address for device
volatile int stripeCount = 0;   //Keep tally on stripe readings

void setup() {
  Wire.begin(SLAVE_ADDR);       //Initialize as slave         
  pinMode(2, INPUT);            //Interrupt pin as input
  Wire.onRequest(sendCount);    //When requested by RPi, send stripe count
  //Serial.begin(9600);           //Need only for testing
  initializeInterrupt();
}

void loop() {
  //Serial.println(stripeCount);  //Need only for Testing
}

void sendCount(){
  Wire.write(stripeCount);      //Send stripe count to RPi
}

void initializeInterrupt(){
  EIMSK |= (1<<INT0);           //PIN 2 as external Interrupt
  EICRA |= (1<<ISC01);          //Trigger Falling Edge
  sei();                        //Enable Interrupts
}

ISR(INT0_vect){
  stripeCount++;              //Increment stripe count upon signal from Sick Lasers
}

