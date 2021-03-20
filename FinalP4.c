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
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/ADC14.h"
#include "../inc/TimerA1.h"
#include "../inc/IRDistance.h"
#include "../inc/LPF.h"
#include "../inc/SysTickInts.h"


volatile uint32_t ControllerFlag; // set every 10ms on controller execution

int32_t Mode = 0;
int32_t UL, UR;             // Controller output PWM duty 2 to 14,998

#define PWMNOMINAL2 6750
#define SWING2 3000
#define PWMIN2 (PWMNOMINAL2-SWING2)
#define PWMAX2 (PWMNOMINAL2+SWING2)

volatile uint32_t nr, nc, nl; // raw distance values
int32_t Left, Center, Right; // IR distances in mm
volatile uint32_t ADCflag; // Set every 500us on ADC sample
int32_t SetPoint = 172;
int32_t Error;
int32_t tJoint = 11;
int32_t lJoint = 12;


// proportional controller gain
// experimentally determine value that creates a stable system
int32_t Kp2= 17;

void IRsampling(void){
    uint32_t raw17, raw12, raw16;
    ADC_In17_14_16(&raw17, &raw12, &raw16);
    nr = LPF_Calc(raw17);
    nc = LPF_Calc2(raw12);
    nl = LPF_Calc3(raw16);
    Left = LeftConvert(nl);
    Center = CenterConvert(nc);
    Right = RightConvert(nr);
    ADCflag = 1;
}

/*
* Proportional controller to keep robot in
* center of two walls using IR sensors.
*/
void SysTick_Handler(void){
    if(Mode){
        //for debugging purposes
       int type =  Classify(Left, Center, Right);

       if(type == lJoint){
           Left = Right;
           //set to straight
       }

        if(type == tJoint){
           Left = 180;
           //set fake left wall
       }

        SetPoint = (Right + Left) / 2 ;

        // set error based off set point
        Error = SetPoint - Right;


        UR = PWMNOMINAL2 + Kp2 * Error;
        UL = PWMNOMINAL2 - Kp2 * Error;

        if(UR < PWMIN2)
            UR = PWMIN2;

        if(UL > PWMAX2)
            UL = PWMAX2;

        if(UL < PWMIN2)
            UL = PWMIN2;

        if(UR > PWMAX2)
            UR = PWMAX2;

        // update motor values
        Motor_Forward(UL, UR);
        ControllerFlag = 1;
    }
}

// proportional control, wall distance
int main(void){
    uint32_t raw17,raw14,raw16;
    DisableInterrupts();
    Clock_Init48MHz();
    LaunchPad_Init();
    Motor_Init();
    TimerA1_Init(&IRsampling,  250);
    Motor_Stop();
    Mode = 0;
    UR = UL = PWMNOMINAL2;
    ADCflag = ControllerFlag = 0;   // semaphores
    ADC0_InitSWTriggerCh17_14_16();   // initialize channels 17,12,16
    ADC_In17_14_16(&raw17,&raw14,&raw16);  // sample
    LPF_Init(raw17,64);     // P9.0/channel 17
    LPF_Init2(raw14,64);    // P4.1/channel 12
    LPF_Init3(raw16,64);    // P9.1/channel 16
    Mode = 1;
    SysTick_Init(48000, 2);
    EnableInterrupts();

}
