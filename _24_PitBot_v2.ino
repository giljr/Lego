/*LEGO Episode #24
  Project :     PitBot — A Star Is Born
                Working at The First Structure in Our Sparring Robot
  Ino File:     _24_PitBot_v2.ino
  Flavour 02
  Date: Jul 2019
  Description:  This code is for test Sparkfun Monster Moto shield
  Board:        Sparkfun Monster Motor Shield & Arduino Uno IDE 1.8.5
  Web Page:     https://medium.com/kidstronics/pitbot-is-agressive-well-no-worries-c801adba4b4c
  Power Supply: 2 Baterries:                
                14500 6.4V/500mAh LiFePo4 Battery right into 5.5mm Jack of Arduino UNO
                ZIPPY Compact 1000mAh 3S 35C Lipo Pack for 2 DC Motor 12 v 150 RPM
  Pixy Camera:  Processor: NXP LPC4330, 204 MHz, dual core;
.               Image sensor: Omnivision OV9715, 1/4", 1280x800; 
                Visit: https://charmedlabs.com/default/ 
  Credits:      This sketch was written by CharmedLab, Adafruit and modifeid by J3
  Based on:     https://learn.adafruit.com/pixy-pet-robot-color-vision-follower-using-pixycam/playing-with-your-pet?source=post_page
  copyright:    Adafruit Industries LLC, 2019 and Charmed Labs, 2019
                This code is public domain, enjoy!
  Terms of Use: This program is free software: you can redistribute it and/or modify
                it under the terms of the GNU General Public License v3 as published by
                the Free Softw are Foundation - There is no guarantee whatsoever :)
                Use at your own risk!
*/
//==========================================================================
//
//  Pixy Pet Robot
//
//   Adafruit invests time and resources providing this open source code,
//  please support Adafruit and open-source hardware by purchasing
//  products from Adafruit!
//
// Written by: Bill Earl for Adafruit Industries
//
//==========================================================================
// begin license header
//
// All Pixy Pet source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
//
// end license header
//
//==========================================================================
//
// Portions of this code are derived from the Pixy CMUcam5 pantilt example code.
//
//==========================================================================
#include <SPI.h>
#include <Pixy.h>

// Sparkfun Monster Motor

#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100

/*  VNH2SP30 pin definitions
  xxx[0] controls '1' outputs
  xxx[1] controls '2' outputs */
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
int cspin[2] = {2, 3}; // CS: Current sense ANALOG input
int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)

//int statpin = 13;

//#include <ZumoMotors.h>

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2) // 160L
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2) // 100L 
#define RCS_MIN_POS     0L
#define RCS_MAX_POS     1000L
#define RCS_CENTER_POS	((RCS_MAX_POS-RCS_MIN_POS)/2)

//---------------------------------------
// Servo Loop Class
// A Proportional/Derivative feedback
// loop for pan/tilt servo tracking of
// blocks.
// (Based on Pixy CMUcam5 example code)
//---------------------------------------
class ServoLoop
{
  public:
    ServoLoop(int32_t proportionalGain, int32_t derivativeGain);

    void update(int32_t error);

    int32_t m_pos;
    int32_t m_prevError;
    int32_t m_proportionalGain;
    int32_t m_derivativeGain;
};

// ServoLoop Constructor
ServoLoop::ServoLoop(int32_t proportionalGain, int32_t derivativeGain)
{
  m_pos = RCS_CENTER_POS;
  m_proportionalGain = proportionalGain;
  m_derivativeGain = derivativeGain;
  m_prevError = 0x80000000L;
}

// ServoLoop Update
// Calculates new output based on the measured
// error and the current state.
void ServoLoop::update(int32_t error)
{
  long int velocity;
  char buf[32];
  if (m_prevError != 0x80000000)
  {
    velocity = (error * m_proportionalGain + (error - m_prevError) * m_derivativeGain) >> 10;

    m_pos += velocity;
    if (m_pos > RCS_MAX_POS)
    {
      m_pos = RCS_MAX_POS;
    }
    else if (m_pos < RCS_MIN_POS)
    {
      m_pos = RCS_MIN_POS;
    }
  }
  m_prevError = error;
}
// End Servo Loop Class
//---------------------------------------

Pixy pixy;  // Declare the camera object

ServoLoop panLoop(200, 200);  // Servo loop for pan
ServoLoop tiltLoop(150, 200); // Servo loop for tilt
//ServoLoop panLoop(300, 500);  // Servo loop for pan
//ServoLoop tiltLoop(500, 700); // Servo loop for tilt

//ZumoMotors motors;  // declare the motors on the zumo

//---------------------------------------
// Setup - runs once at startup
//---------------------------------------
void setup()
{
  //Serial.begin(9600);
  //Serial.print("Starting...\n");

  // Sparkfun Moster Motor
  //pinMode(statpin, OUTPUT);

  // Initialize digital pins as outputs
  for (int i = 0; i < 2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }

  // Pixy
  pixy.init();
}

uint32_t lastBlockTime = 0;

//---------------------------------------
// Main loop - runs continuously after setup
//---------------------------------------
void loop()
{
  uint16_t blocks;
  blocks = pixy.getBlocks();

  // If we have blocks in sight, track and follow them
  if (blocks)
  {
    int trackedBlock = TrackBlock(blocks);
    FollowBlock(trackedBlock);
    lastBlockTime = millis();
  }
  else if (millis() - lastBlockTime > 100)
  {
    //motors.setLeftSpeed(0);
    //motors.setRightSpeed(0);
    motorGo(0, CCW, 0);
    motorGo(1, CCW, 0);
    ScanForBlocks();
  }
}

int oldX, oldY, oldSignature;

//---------------------------------------
// Track blocks via the Pixy pan/tilt mech
// (based in part on Pixy CMUcam5 pantilt example)
//---------------------------------------
int TrackBlock(int blockCount)
{
  int trackedBlock = 0;
  long maxSize = 0;

  //Serial.print("blocks =");
  //Serial.println(blockCount);

  for (int i = 0; i < blockCount; i++)
  {
    if ((oldSignature == 0) || (pixy.blocks[i].signature == oldSignature))
    {
      long newSize = pixy.blocks[i].height * pixy.blocks[i].width;
      if (newSize > maxSize)
      {
        trackedBlock = i;
        maxSize = newSize;
      }
    }
  }

  int32_t panError = X_CENTER - pixy.blocks[trackedBlock].x;
  int32_t tiltError = pixy.blocks[trackedBlock].y - Y_CENTER;

  panLoop.update(panError);
  tiltLoop.update(tiltError);

  pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);

  oldX = pixy.blocks[trackedBlock].x;
  oldY = pixy.blocks[trackedBlock].y;
  oldSignature = pixy.blocks[trackedBlock].signature;
  return trackedBlock;
}

//---------------------------------------
// Follow blocks via the Zumo robot drive
//
// This code makes the robot base turn
// and move to follow the pan/tilt tracking
// of the head.
//---------------------------------------
int32_t size = 400;
void FollowBlock(int trackedBlock)
{
  int32_t followError = RCS_CENTER_POS - panLoop.m_pos;  // How far off-center are we looking now?

  // Size is the area of the object.
  // We keep a running average of the last 8.
  size += pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height;
  size -= size >> 3;

  // Forward speed decreases as we approach the object (size is larger)
  int forwardSpeed = constrain(400 - (size / 256), 0, 400);

  // Steering differential is proportional to the error times the forward speed
  int32_t differential = (followError + (followError * forwardSpeed)) >> 8;

  // Adjust the left and right speeds by the steering differential.
  int leftSpeed = constrain(forwardSpeed + differential, 0, 150);
  int rightSpeed = constrain(forwardSpeed - differential, 0, 150);

  // And set the motor speeds
  //motors.setLeftSpeed(leftSpeed);
  //motors.setRightSpeed(rightSpeed);
  motorGo(0, CCW, leftSpeed);
  motorGo(1, CCW, rightSpeed);
}

//---------------------------------------
// Random search for blocks
//
// This code pans back and forth at random
// until a block is detected
//---------------------------------------
int scanIncrement = (RCS_MAX_POS - RCS_MIN_POS) / 200;
uint32_t lastMove = 0;

void ScanForBlocks()
{
  if (millis() - lastMove > 20)
  {
    lastMove = millis();
    panLoop.m_pos += scanIncrement;
    if ((panLoop.m_pos >= RCS_MAX_POS) || (panLoop.m_pos <= RCS_MIN_POS))
    {
      tiltLoop.m_pos = random(RCS_MAX_POS * 0.4, RCS_MAX_POS);
      scanIncrement = -scanIncrement;
      if (scanIncrement < 0)
      {
        //motors.setLeftSpeed(-250);
        //motors.setRightSpeed(250);
        motorGo(0, CCW, 153);
        motorGo(1, CCW, 150);

      }
      else
      {
        //motors.setLeftSpeed(+180);
        //motors.setRightSpeed(-180);
        motorGo(0, CCW, 83);
        motorGo(1, CCW, 80);
      }
      delay(random(250, 500));
    }

    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
  }
}

void motorOff(int motor)
{
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
}

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <= 4)
    {
      // Set inA[motor]
      if (direct <= 1)
      //if (direct = 1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct == 0) || (direct == 2))
      //if (direct == 2)
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}

