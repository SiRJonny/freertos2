/*
 * config.cpp
 *
 *  Created on: 2016. febr. 2.
 *      Author: Gabor
 */

#include <config.hpp>

int SERVO_RANGE_MOTOR = 700;	// max eltérés 0-tól, 1500us +/- SERVO_RANGE a max kiadott jel
int SERVO_RANGE_STEERING = 260;	// max eltérés 0-tól, 1500us +/- SERVO_RANGE a max kiadott jel
int SERVO_RANGE_SENSOR = 400;

StateData stateData;

//sebességes dolgok
bool speed_control_enabled = true;

volatile float SLOW = 1.5;

volatile float FAST = 4.5;

float SAFETY_SPEED_LIMIT = 1.7;

float STOP = 0.0;

float SET_SPEED = 0;
float SET_DISTANCE = 0.011;
float speed_global = 0;

float SKILLSLOW = 1.0;

float SAFETYFAST = 1.7;
float SAFETYSLOW = 1.1;

bool speed_under_X = false;
float speed_limit = 1.3f;

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

//többi szenzor
float giro_drift_Y;
float giro_drift_Z;
float giro_accu_Y;
float giro_accu_Z;
bool fal_bal = false;
bool fal_jobb = false;
bool bordas_bal = false;
bool bordas_jobb = false;
int Distance_sensors[5];
float FrontSensorMedian = 100;
float FrontSensorAverage = 100;

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
float ACC_MAX = 100;		// egy szabályzó periódusban max ennyivel növekedhet a motor szervo jele
int NO_LINE_CYCLES = 50;

float A = 0.4;	// sebesség függés	// d5% = v*A + B
float B = 0.4;	// konstans

bool TunePID = false;
int pid = 0;

bool safety_car = true;

PID_struct PIDs;
PID_struct PIDm;
static PID_struct PIDk;

//állapotgépek

StateContext stateContext;
SkillStateContext skillStateContext;

//egyeb
char buffer[10];	//bt msg hez
int timer = 0; // idõméréshez

int stopped = 0;
