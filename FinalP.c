// Final Project.c
// Runs on MSP432
// Implementation of the control system.
// Andrew Lee
// December 4, 2020

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

#include <stdint.h>
#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/Bump.h"
#include "../inc/TimerA1.h"
#include "../inc/IRDistance.h"
#include "../inc/Tachometer.h"
#include "../inc/Reflectance.h"

#define REDLED (*((volatile uint8_t *)(0x42098060)))
#define BLUELED (*((volatile uint8_t *)(0x42098068)))

struct State {
  int state;
  const struct State *next[3]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

void FollowLine(void);
void TurnAround(void);
void FlashLights(void);

#define Follow    &fsm[0]
#define Turn      &fsm[1]
#define Blink     &fsm[2]

State_t *Spt;

State_t fsm[3]={
  {0, {Follow, Turn, Blink}},  // Follow
  {1, {Follow, Follow, Follow}},  // Turn
  {2, {Blink, Blink, Blink}}  //Flash Lights
};


volatile uint32_t ControllerFlag; // set every 10ms on controller execution

int32_t Mode = 0;
int32_t UL, UR;             // Controller output PWM duty 2 to 14,998
uint32_t Time;             // in 0.01 sec


/**************Follow Line******************************************/
#define PWMNOMINAL3 2500
#define SWING3 1000
#define PWMIN3 (PWMNOMINAL3-SWING3)
#define PWMAX3 (PWMNOMINAL3+SWING3)

#define TACHBUFF 10
uint16_t LeftTach[TACHBUFF];
enum TachDirection LeftDir;
uint16_t RightTach[TACHBUFF];
enum TachDirection RightDir;
int32_t LeftSteps;
int32_t RightSteps;

int32_t collisionBefore = 0;

// Proportional controller to drive robot using line following
uint8_t LineData;       // direct measure from line sensor
int32_t Position;      // position in 0.1mm relative to center of line
int32_t Kp3= 15;
int32_t i = 0;
int32_t LastStep = 0;

void Controller(void){

    i = (i + 1)%10;

    if(Spt->state == *Turn.state){
        //this is the turning state
        Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
        Motor_Left(PWMNOMINAL3, PWMNOMINAL3);
        if((RightSteps - LastStep) > 360){
            Spt = Spt->next[0];
        }
    }
    else if(Spt->state == *Blink.state){
        //Blink state
        Motor_Stop();

        if(Time < 200){
            LaunchPad_Output(1);
            LaunchPad_Output(4);
        } else {
            LaunchPad_Output(0);
        }

        Time = (Time + 1) % 500;
    }
    else{
        //forward state
        if(Time == 0){
            Reflectance_Start();
        }
        if(Time == 1){
              LineData =  ~Reflectance_End();
              Position = Reflectance_Position(LineData);
              if(Mode){
                   UR = PWMNOMINAL3 - (Kp3 * Position);
                   UL = PWMNOMINAL3 + (Kp3 * Position);
                   if(UR > PWMAX3)
                       UR = PWMAX3;

                   if(UR < PWMIN3)
                       UR = PWMIN3;

                   if(UL > PWMAX3)
                       UL = PWMAX3;

                   if(UL < PWMIN3)
                      UL = PWMIN3;

                   Motor_Forward(UL, UR);
                   i = (i+1)%TACHBUFF;

                   Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
                   LastStep = RightSteps;
                 }
      }

      //change states depending on the number of collisions
      if(Bump_Read() && collisionBefore == 0){
        collisionBefore += 1;
        Spt = Spt->next[collisionBefore];
      }else if(Bump_Read() && collisionBefore != 0){
        collisionBefore += 1;
        Spt = Spt->next[collisionBefore];
      }

      Time = (Time + 1) % 10;
    }
}


int main(void){
    //Initializations
    Tachometer_Init();
    Clock_Init48MHz();
    LaunchPad_Init();
    Bump_Init();
    Reflectance_Init();
    Motor_Init();
    Mode = 1;
    Time = 0;
    UR = UL = PWMNOMINAL3;
    EnableInterrupts();
    TimerA1_Init(&Controller,500);
    Spt = Follow;
}


