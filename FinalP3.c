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
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/Motor.h"
#include "../inc/LaunchPad.h"
#include "../inc/Bump.h"
#include "../inc/TimerA1.h"
#include "../inc/Tachometer.h"

int32_t state;

int32_t UL, UR;             // Controller output PWM duty 2 to 14,998

#define DESIRED_SPEED 50
                     // number of elements in tachometer array
#define PWMNOMINAL 5000
#define PWMIN1 (PWMNOMINAL-SWING1)
#define PWMAX1 (PWMNOMINAL+SWING1)

#define TACHBUFF 10               // X* - X'
uint16_t LeftTach[TACHBUFF];             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir;              // direction of left rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps;                       // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
uint16_t RightTach[TACHBUFF];            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection RightDir;             // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t RightSteps;

int32_t LastStepR;
int32_t LastStepL;

//prevent getting stuck in corner
int32_t stateNext = 0;

int i = 0;

//states
int32_t straight = 0;
int32_t reverse = 1;
int32_t softL = 2;
int32_t hardL = 3;
int32_t softR = 4;
int32_t hardR = 5;


//**************************************************
// Proportional controller to drive straight with input
//  from the tachometers
void Controller1(void){
    i = (i+1)%10;
    if(state == straight){
        Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
        Motor_Forward(PWMNOMINAL, PWMNOMINAL);
        LastStepR = RightSteps;
        LastStepL = LeftSteps;

        if(Bump_Read()){
           //hard coded values are the bump values
           if(Bump_Read() == 12){
               state = reverse;
               stateNext = hardL;
           }
           else if(Bump_Read() <= 3){
               state = reverse;
               stateNext = softL;
           }
           else if(Bump_Read() <= 7){
               state = reverse;
               stateNext = hardL;
           }
           else if(Bump_Read() >= 24){
               state = reverse;
               stateNext = hardR;
           }
           else{
               state = reverse;
               stateNext = softR;
          }
        }
    }

    //switch state may have been better BUT
    //I used if's since I wrote and tested one state at a time
    if(state == reverse){
        //reverse state
        Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
        Motor_Backward(PWMNOMINAL, PWMNOMINAL);

        if((RightSteps - LastStepR) < -70){
            state = stateNext;
        }

    }

    if(state == softL){
        //soft left turn
        Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
        Motor_Left(PWMNOMINAL, PWMNOMINAL);

        if((RightSteps - LastStepR) > 3)
            state = straight;

    }

    if(state == hardL){
        //hard left turn
        Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
        Motor_Left(PWMNOMINAL, PWMNOMINAL);

        if((RightSteps - LastStepR) > 20)
            state = straight;


    }

    if(state == softR){
        //soft right turn
        Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
        Motor_Right(PWMNOMINAL, PWMNOMINAL);

        if((LeftSteps - LastStepL) > 3)
            state = straight;



    }

    if(state == hardR){
        //hard right turn
        Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
        Motor_Right(PWMNOMINAL, PWMNOMINAL);

        if((LeftSteps - LastStepL) > 20){
           state = straight;

        }
    }
}


int main(void){
    Tachometer_Init();
    DisableInterrupts();
    Clock_Init48MHz();
    LaunchPad_Init();
    Bump_Init();
    Motor_Init();

    TimerA1_Init(&Controller1,500);

    Motor_Stop();
    UR = UL = PWMNOMINAL;
    EnableInterrupts();

    state = straight;
}





