/*LEGO Episode #24
  Project :     PitBot — A Star Is Born
                Working at The First Structure in Our Sparring Robot
  Ino File:     _24_PitBot_v1.ino
  Flavour 01
  Date: Jul 2019
  Description:  This code is for Pixy test
  Board:        Sparkfun Monster Motor Shield & Arduino Uno IDE 1.8.5
  Web Page:     https://medium.com/kidstronics/pitbot-a-star-is-born-e04a0f305fb6
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
//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// This sketch is a simple tracking demo that uses the pan/tilt unit.  For
// more information, go here:
//
// http://cmucam.org/projects/cmucam5/wiki/Run_the_Pantilt_Demo
//

#include <SPI.h>  
#include <Pixy.h>

Pixy pixy;

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)       
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

class ServoLoop
{
public:
  ServoLoop(int32_t pgain, int32_t dgain);

  void update(int32_t error);
   
  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_pgain;
  int32_t m_dgain;
};


ServoLoop panLoop(300, 500);
ServoLoop tiltLoop(500, 700);

ServoLoop::ServoLoop(int32_t pgain, int32_t dgain)
{
  m_pos = PIXY_RCS_CENTER_POS;
  m_pgain = pgain;
  m_dgain = dgain;
  m_prevError = 0x80000000L;
}

void ServoLoop::update(int32_t error)
{
  long int vel;
  char buf[32];
  if (m_prevError!=0x80000000)
  {	
    vel = (error*m_pgain + (error - m_prevError)*m_dgain)>>10;
    //sprintf(buf, "%ld\n", vel);
    //Serial.print(buf);
    m_pos += vel;
    if (m_pos>PIXY_RCS_MAX_POS) 
      m_pos = PIXY_RCS_MAX_POS; 
    else if (m_pos<PIXY_RCS_MIN_POS) 
      m_pos = PIXY_RCS_MIN_POS;
  }
  m_prevError = error;
}



void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");
  
  pixy.init();
}

void loop()
{ 
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  int32_t panError, tiltError;
  
  blocks = pixy.getBlocks();
  
  if (blocks)
  {
    panError = X_CENTER-pixy.blocks[0].x;
    tiltError = pixy.blocks[0].y-Y_CENTER;
    
    panLoop.update(panError);
    tiltLoop.update(tiltError);
    
    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
    
    i++;
    
    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%50==0) 
    {
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
      for (j=0; j<blocks; j++)
      {
        sprintf(buf, "  block %d: ", j);
        Serial.print(buf); 
        pixy.blocks[j].print();
      }
    }
  }  
}
