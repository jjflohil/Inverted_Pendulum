#include <Arduino.h>

//Wifi parameters
const char* ssid2 = "MilperHome";
const char* password2 = "2TXTpvb6Xwxpn7hj";

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

//Function Parameters
uint8_t ModeP = 0;
uint8_t nLEDs_H = 12;
uint32_t LeftColor[12];
uint32_t RightColor[12];
uint32_t TotalColor[23];




