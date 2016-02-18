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
float SET_DISTANCE = 0.010;
float speed_global = 0;

float SKILLSLOW = 0.5;
float SKILLMEDIUM = 0.8;
float SKILLPARK = 0.4;


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
bool giro_stopped = true;
bool giro_fall = false;
bool giro_lejto = false;
bool giro_emelkedo = false;

bool fal_bal = false;
bool fal_jobb = false;
bool bordas_bal = false;
bool bordas_jobb = false;
int Distance_sensors[5];
int FrontSensorTurn = 140;

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

bool keresztvonal;
bool stable0lines;
bool stable1lines;
bool stable2lines;
bool stable3lines;

bool stable1linesForBoth;
bool stable0linesForBoth;

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

bool steeringControl = true;

//állapotgépek

StateContext stateContext;
SkillStateContext skillStateContext;

//egyeb
char buffer[10];	//bt msg hez
int timer = 0; // idõméréshez

bool skillTrack = true;
Direction direction = UNDEFINED;
SkillTrackEvent currentState = NONE;
bool checkDirection = true;

int stopped = 0;

int timeCounter = 0;

bool start_radio_done;

