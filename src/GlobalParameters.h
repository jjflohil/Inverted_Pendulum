#include <Arduino.h>

//Wifi parameters
const char* ssid2 = "MilperHome";
const char* password2 = "********";

// Acces point parameters
const char* ssid     = "InvertedPendulum_AccesPoint";
const char* password = "printerprint";


uint8_t ActiveConn = 2;
const char* identifier = "InvertedPendulum";
const uint16_t ID = 1;
IPAddress local_IP2(192, 168, 2, 15);
IPAddress gateway2(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);

//Artnet parameters
const int startUniverse = 0;
bool sendFrame = 1;
int previousDataLength = 0;

// DMX Receiving parameters
uint8_t Vel;
uint8_t R;
uint8_t G;
uint8_t B;
uint8_t W;
uint8_t Mode = 0;
uint8_t Mode_p;
uint8_t Vel_1;
uint8_t R_1;
uint8_t G_1;
uint8_t B_1;
uint8_t W_1;
uint8_t WW;



//------------------ Timer parameters----------------//
uint32_t  t_timeout = 120;  // Timeout time
uint32_t  t_main    = 0;   // Current time main
uint16_t  ts_main   = 1;   // Sample time main
uint32_t  t_effect   = 0;   // Current time effect
uint16_t  ts_effect  = 1;   // Sample time effect
uint32_t  t_effect2   = 0;   // Current time effect
uint16_t  ts_effect2  = 100;   // Sample time effect
uint32_t  t_msgeq7   = 0;   // Current time effect
uint16_t  ts_msgeq7  = 10;   // Sample time effect
uint32_t  t_WifiLoop   = 0;   // Current time effect
uint16_t  ts_WifiLoop  = 10;   // Sample time effect
uint32_t  t_serial   = 0;   // Current time effect
uint16_t  ts_serial  = 50;   // Sample time effect
uint32_t  t_movingAvg   = 0;   // Current time effect
uint16_t  ts_movingAvg  = 15;   // Sample time effect
uint32_t  t_rbs   = 0;   // Current time effect
uint16_t  ts_rbs  = 200;   // Sample time effect
uint32_t  tc = 0;
uint16_t  ts = 1;
float     tr = 0;

//Function Parameters
uint8_t ModeP = 0;

// Set PWM properties
const int freq = 20000;
const int motorChannel = 0;
const int resolution = 8;

//Other Parameters
const float pi = 3.14159265359;

struct component {
  float refPos;
  float refVel;
  float refAcc;
  float position;
  float velocity;
  float acceleration;
  float lastPosition;
  float lastVelocity;
  float positionFiltered;
  float velocityFiltered;
  float accelerationFiltered;
  float e_pos;
  float e_vel;
  float e_acc;
};

struct EncoderPulse {
	const uint8_t PIN;
	int32_t position;
};

struct motor {
  const uint8_t VoltPin;
  const uint8_t DirPin0;
  const uint8_t DirPin1;
  uint32_t Volt;
  bool Direction;
};

EncoderPulse Enc0_0 = {21, 0};
EncoderPulse Enc0_1 = {22, 0};
EncoderPulse Enc1_0 = {32, 0};
EncoderPulse Enc1_1 = {33, 0};

component cart = {0,0,0,0,0,0,0};
component pend = {0,0,0,0,0,0,0};

motor mot = {26,18,19,0,false};

//Control parameters
float ctrl = 0;
float ctrl_cur = 0;
float ctrl_past = 0;
float errInt = 0;
float PendErrInt = 0;
float errPast = 0;
float PosScaled = 0;
float VelScaled = 0;
float lastP = 0;
float lastV = 0;
float Pgain = 0;
float Dgain = 0;
float Again = 0;
float Igain = 0;
float Xgain = 0;
float LP = 1.0;
float pendPosGain = 0.0;
float pendVelGain = 0.0;
