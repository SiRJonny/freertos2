/*
 * config.cpp
 *
 *  Created on: 2016. febr. 2.
 *      Author: Gabor
 */

#include <config.hpp>

int SERVO_RANGE_MOTOR = 700;	// max eltérés 0-tól, 1500us +/- SERVO_RANGE a max kiadott jel
int SERVO_RANGE_STEERING = 260;	// max eltérés 0-tól, 1500us +/- SERVO_RANGE a max kiadott jel



//sebességes dolgok
float SLOW = 2.0;

float FAST = 2.8;

float STOP = 0.0;

float SET_SPEED = 0;
float speed_global = 0;

float SKILLSLOW = 1.0;

//kanyarszervo
int servoOffset = -60;


//encoder
int encoderPos = 1000000000;
float encoderIncrementToMeter = 0;

int globalDistance = 0;

//szenzorsor
uint32_t ADC1_BUFFER[4];
uint32_t szenzorsor_1[32];
uint32_t szenzorsor_2[32];

uint32_t szenzorsor_temp_1[32];
uint32_t szenzorsor_temp_2[32];

//vonal
float linePosM;		// vonalpozíció méterben
float angle = 0;
float control = 0;
float speed_error = 0;
float speed_control = 0;
float last_speed_control = 0;
float last_active_line_pos1 = 15.5;
float last_active_line_pos2 = 15.5;

LineState globalLines;

bool stable3lines;
float activeLine1 = 0;  // középsõ vonal kiválasztása
float activeLine2 = 0;

//szabályzók
float ACC_MAX = 200;		// egy szabályzó periódusban max ennyivel növekedhet a motor szervo jele
int NO_LINE_CYCLES = 0;

float A = 0.4;	// sebesség függés	// d5% = v*A + B
float B = 0.4;	// konstans

bool TunePID = false;
int pid = 0;

PID_struct PIDs;
PID_struct PIDm;

//állapotgépek

StateContext stateContext;


//egyeb
char buffer[10];	//bt msg hez
int timer = 0; // idõméréshez

int stopped = 0;
