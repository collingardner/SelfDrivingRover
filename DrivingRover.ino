#include <Wire.h>

//Included for the Wire transmission
String passback = "";


#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/
/*                SETTING THE PINS FOR SENSOR AND ROVOR SHIELD    */
//Pins from the Ultrasonic Sensor
int triggerPin = 2; //triggering on pin 2
int echoPin = 3;    //echo on pin 3

//Pins for the Rover shield
int rightMotorPin = 5;
int leftMotorPin = 6;
int rightDirectionPin = 7;
int leftDirectionPin = 8;
int leftspeed = 255; //setmaximum speed, goes constant speed
int rightspeed = 250;//makes it drive straight

int distance, duration;

// Create the bluefruit object, either software serial...uncomment these lines

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                              BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

/*=====================*/
void setup(void)
{
  /* SETUP for SENSOR and ROVER SHEILD */
  pinMode(triggerPin, OUTPUT); //defining pins
  pinMode(echoPin, INPUT);

  //Shield Code
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightDirectionPin, OUTPUT);
  pinMode(leftDirectionPin, OUTPUT);

  //Set up for wire Tranmissions
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);


  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);
  /* Print Bluefruit information */
  ble.info();
  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }
  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);
}

void loop(void)
{
  /*This for calculating the distance objects in cm from the Ultrasonic Sensor*/
  int duration;   //Adding duration and distance
  boolean selfDrive = false;
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  /* This if statement waits for the button presses for the controls*/
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    boolean selfDrive = false;
    if (buttnum == 3 ) { //if User presses this button, vehicle goes in self driving mode
      selfDrive = true;

      Serial.println("Self Driving Mode"); //Delete THis
    }


    else if (buttnum == 1 ) { //Sets Rover's speed to low
      leftspeed = 200;
      rightspeed = 200;
    }
    else if (buttnum == 2 ) { //Sets the Rover's speed to high
      leftspeed = 255;
      rightspeed = 250;
    }
    else if (buttnum == 4) {
      forward(255, 200);

    }
    else if (buttnum == 5 && pressed == true) {
      selfDrive = false;
      forward (leftspeed, rightspeed);
    }
    else if (buttnum == 7 && pressed == true) {
      selfDrive = false;
      turnLeft (leftspeed, rightspeed);
    }
    else if (buttnum == 8 && pressed == true ) {
      turnRight (leftspeed, rightspeed);
    }
    else if (buttnum == 6 && pressed == true) {
      reverse (leftspeed, rightspeed);
    }
    else {
      stop();
    }

    while (selfDrive) { //THIS IS IF IT HAS TO TURN AROUND MORE THAN 3 TIMES THE VEHICLE STOPS AND THE USER HAS TO CONTROL IT FROM HERE



      if (distance < 10 && distance > 0) { //If vehicle within 10 cm of something it stops,
        Serial.println("Collision Stopped "); //Delete THis
        ble.println("Collision Stopped");
        stop();//stop for some time, then check the objects around iit
        delay(1000);// 1 second delay for show


        if (passback.equals("DontTurnRight")) {
          Serial.println("Turn Left "); //Delete THis
          ble.println("Turning Right");
          Serial.println("Turn Right "); //Delete THis
          ble.println("Turning Right");
          turnLeft(leftspeed, rightspeed);
          delay(1200);
          stop();
          delay(1000);
          forward(leftspeed, rightspeed);
          delay(1500);
          if (distance < 20) { //Stop and go around te other way
            stop();
            delay(1000);
            turnRight(leftspeed, rightspeed);
            delay(1200);  //may need a stop and delay here
            if (distance < 30) { //to see if path is clear
              forward(leftspeed, rightspeed);
            }
            else {
              turnRight(leftspeed, rightspeed); //path is not clear, so give up and and let user take backover
              delay(1200);
              selfDrive = false;
            }

          } else {
            delay(2500); //continue going
            stop();
            delay(1000);
            turnRight(leftspeed, rightspeed);
            delay(1000);
            forward(leftspeed, rightspeed);
            delay(1500);
            if (distance < 15) {
              stop();
              delay(1500);
              selfDrive = false;
              break;
            }
            turnRight(leftspeed, rightspeed);
            delay(1000);
            forward(leftspeed, rightspeed);
            delay(1500);
            if (distance < 15) {
              if (!(passback.equals("DontTurnRight"))) {
                stop();
                delay(1500);
                selfDrive = false;//
                break;
              }
            }
            else {
              forward(leftspeed, rightspeed);
              delay(2500);
              turnLeft(leftspeed, rightspeed);
              delay(1000);
            }
          }

          passback = "Everything is fine";
        }


        else if (passback.equals("DontTurnLeft")) {  //This is pretty much complete so far
          Serial.println("Turn Right "); //Delete THis
          ble.println("Turning Right");
          Serial.println("Turn Right "); //Delete THis
          ble.println("Turning Right");
          turnRight(leftspeed, rightspeed);
          delay(1200);
          stop();
          delay(1000);
          forward(leftspeed, rightspeed);
          delay(1500);
          if (distance < 20) { //Stop and go around te other way
            stop();
            delay(1000);
            turnLeft(leftspeed, rightspeed);
            delay(1200);  //may need a stop and delay here
            if (distance < 30) { //to see if path is clear
              forward(leftspeed, rightspeed);
            }
            else {
              turnLeft(leftspeed, rightspeed); //path is not clear, so give up and and let user take backover
              delay(1200);
              selfDrive = false;
            }

          } else {
            delay(2500); //continue going
            stop();
            delay(1000);
            turnLeft(leftspeed, rightspeed);
            delay(1000);
            forward(leftspeed, rightspeed);
            delay(1500);
            if (distance < 15) {
              stop();
              delay(1500);
              selfDrive = false;
              break;
            }
            turnLeft(leftspeed, rightspeed);
            delay(1000);
            forward(leftspeed, rightspeed);
            delay(1500);
            if (distance < 15) {
              if (!(passback.equals("DontTurnRight"))) {
                stop();
                delay(1500);
                selfDrive = false;//
                break;
              }
            }
            else {
              forward(leftspeed, rightspeed);
              delay(2500);
              turnRight(leftspeed, rightspeed);
              delay(1000);
            }
          }

          passback = "Everything is fine";
        }
        else if (passback.equals("Reverse")) {
          reverse(leftspeed, rightspeed);
          delay(2500);
          selfDrive = false;
          break;
        }

        else {
          Serial.println("Turn Right "); //Delete THis
          ble.println("Turning Right");
          Serial.println("Turn Right "); //Delete THis
          ble.println("Turning Right");
          turnRight(leftspeed, rightspeed);
          delay(1200);
          stop();
          delay(1000);
          forward(leftspeed, rightspeed);
          delay(1500);
          if (distance < 20) { //Stop and go around te other way
            stop();
            delay(1000);
            turnLeft(leftspeed, rightspeed);
            delay(1200);  //may need a stop and delay here
            if (distance < 30) { //to see if path is clear
              forward(leftspeed, rightspeed);
            }
            else {
              turnLeft(leftspeed, rightspeed); //path is not clear, so give up and and let user take backover
              delay(1200);
              selfDrive = false;
            }

          } else {
            delay(2500); //continue going
            stop();
            delay(1000);
            turnLeft(leftspeed, rightspeed);
            delay(1000);
            forward(leftspeed, rightspeed);
            delay(1500);
            if (distance < 15) {
              stop();
              delay(1500);
              selfDrive = false;
              break;
            }
            turnLeft(leftspeed, rightspeed);
            delay(1000);
            forward(leftspeed, rightspeed);
            delay(1500);
            if (distance < 15) {
              if (!(passback.equals("DontTurnRight"))) {
                stop();
                delay(1500);
                selfDrive = false;//
                break;
              }
            }
            else {
              forward(leftspeed, rightspeed);
              delay(2500);
              turnRight(leftspeed, rightspeed);
              delay(1000);
            }
          }
          passback = "Everything is fine";
        }
      } //turn on self driving mode. in while loop


      else {
        Serial.println("Driving");
        ble.println("Driving");
        if ( (buttnum == 1 ) || (buttnum == 2 && pressed == true) || (buttnum == 4 && pressed == true) || (buttnum == 5 && pressed == true) || (buttnum == 6 && pressed == true) || (buttnum == 7 && pressed == true) || (buttnum == 8 && pressed == true) ) {
          selfDrive = false;
          break;
        }
        forward(leftspeed, rightspeed);
      }


    }


  }


}
//METHODS FOR TRAVELING
void stop(void) //Stop
{
  digitalWrite(leftMotorPin, LOW);
  digitalWrite(rightMotorPin, LOW);
}
void collisionStop(void) //Stop
{

  digitalWrite(leftMotorPin, LOW);
  digitalWrite(rightMotorPin, LOW);
  delay(10000);
  ble.println("Collision Avoided");
  ble.println(" Rover is stopped, use directional keys to drive, and button 3 for self driving");
  digitalWrite(leftMotorPin, LOW);
  digitalWrite(rightMotorPin, LOW);
}
void forward(char a, char b)
{
  analogWrite (leftMotorPin, a);
  digitalWrite(leftDirectionPin, LOW);
  analogWrite (rightMotorPin, b);
  digitalWrite(rightDirectionPin, LOW);
}

void reverse (char a, char b)
{
  analogWrite (leftMotorPin, a);
  digitalWrite(leftDirectionPin, HIGH);
  analogWrite (rightMotorPin, b);
  digitalWrite(rightDirectionPin, HIGH);
}

void turnLeft(char a, char b)
{
  analogWrite (leftMotorPin, b);
  digitalWrite(leftDirectionPin, LOW);
  analogWrite (rightMotorPin, a);
  digitalWrite(rightDirectionPin, HIGH);
}

void turnRight(char a, char b)
{
  analogWrite (leftMotorPin, b);
  digitalWrite(leftDirectionPin, HIGH);
  analogWrite (rightMotorPin, a);
  digitalWrite(rightDirectionPin, LOW);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
int old = 0;
int x = 1;
int LeftDist = 1000;
int RightDist = 10000;
void receiveEvent(int howMany) {

  //  Serial.println("Self Driving");
  digitalWrite(triggerPin, HIGH); //triggering the wave(like blinking an LED)
  delay(10);
  digitalWrite(triggerPin, LOW);
  duration = pulseIn(echoPin, HIGH); //a special function for listening and waiting for the wave
  distance = (duration / 2) / 29.1; //transforming the number to cm(if you want inches, you have to change the 29.1 with a suitable number
  Serial.println(distance);

  if (old != x) {

    String str = "";
    while (1 < Wire.available()) { // loop through all but the last
      char c = Wire.read(); // receive byte as a character
      Serial.print(c);// print the character
      str.concat(c);
    }
    int x = Wire.read();    // receive byte as an integer
    //passback = str + x ;
    if (x < 20 && x > 0 && str.equals("L")) {
      passback = "DontTurnLeft";
      LeftDist = x;
      delay(250);
    }
    else if (x < 20 && x > 0 && str.equals("R")) {
      passback = "DontTurnRight";
      RightDist = x;
      delay(250);
    }
    else if (x < 20 && x > 0 && str.equals("B")) {
      passback = "Dont Go Back";
      delay(250);
    }
    else if (RightDist < 15 && LeftDist < 15 && RightDist > 0 && LeftDist > 0) {
      passback = "Reverse";
      delay(250);
    }
    else {

    }
  }
}

