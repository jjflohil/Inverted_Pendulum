#include <Arduino.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <ArtnetWifi.h>
#include <ArduinoOTA.h>
#include <WiFiUdp.h>
#include <WiFiClient.h> 

//Load global parameters
#include <GlobalParameters.h>

// Create a task handler to be able to use processor 0 for separate tasking 
TaskHandle_t Task0_Wifi;
TaskHandle_t Task1_ControlSystem;

ArtnetWifi artnet;
ArtnetWifi artnetSend;

WiFiUDP UDP;
IPAddress macbookIP(192,168,2,23);
unsigned int localUdpPort = 4210;
//Parameters
const float pi = 3.1415;

struct component {
  float refPos;
  float refVel;
  float position;
  float velocity;
  float lastPosition;
  float lastVelocity;
  float positionFiltered;
  float velocityFiltered;
  float e_pos;
  float e_vel;
};

struct EncoderPulse {
	const uint8_t PIN;
	int32_t position;
};

struct motor {
  const uint8_t VoltPin;
  const uint8_t DirPin0;
  const uint8_t DirPin1;
  int Volt;
  bool Direction;
};

EncoderPulse Enc0_0 = {21, 0};
EncoderPulse Enc0_1 = {22, 0};
EncoderPulse Enc1_0 = {32, 0};
EncoderPulse Enc1_1 = {33, 0};

component cart = {0,0,0,0,0};
component pend = {0,0,0,0,0};

motor mot = {26,18,19,0,false};

//Time parameters
long  tc = 0;
int   ts = 10;
float tr = 0;
float errInt = 0;
float errPast = 0;
//Temporary control parameters
float ctrl = 0;


//Initialize functions
void onDmxFrame(uint16_t , uint16_t , uint8_t , uint8_t*);
void Task0_WifiCode( void *);
void Task1_ControlSystemCode( void *);


//Encoder interrupts
void IRAM_ATTR isr00() {
  if(digitalRead(Enc0_0.PIN)==digitalRead(Enc0_1.PIN)){
    Enc0_0.position++;
  }
  else{
    Enc0_0.position--;
  }
}
void IRAM_ATTR isr01() {
  if(digitalRead(Enc0_0.PIN)!=digitalRead(Enc0_1.PIN)){
    Enc0_0.position++;
  }
  else{
    Enc0_0.position--;
  }
}
void IRAM_ATTR isr10() {
  if(digitalRead(Enc1_0.PIN)!=digitalRead(Enc1_1.PIN)){
    Enc1_0.position++;
  }
  else{
    Enc1_0.position--;
  }
}
void IRAM_ATTR isr11() {
  if(digitalRead(Enc1_0.PIN)==digitalRead(Enc1_1.PIN)){
    Enc1_0.position++;
  }
  else{
    Enc1_0.position--;
  }
}

float PosScaled = 0;
float VelScaled = 0;
float lastP = 0;
float lastV = 0;

float Pgain = 0;
float Vgain = 0;
float Igain = 0;
float LP = 1.0;
//========================Setup========================//
void setup() {
  xTaskCreatePinnedToCore(Task0_WifiCode, "Task0", 10000,  NULL, 1, &Task0_Wifi,0); 
  xTaskCreatePinnedToCore(Task1_ControlSystemCode, "Task1", 10000,  NULL, 1, &Task1_ControlSystem,1); 

}


void Task0_WifiCode( void * pvParameters ){
  //------------------------------Local Setup --------------------------------//
  //WiFi.mode(WIFI_AP);
  //WiFi.softAPConfig(local_IP2, gateway2, subnet);
  //WiFi.softAP(ssid, password);
  Serial.begin(9600);
  if(1){
    //Wifi setup
    WiFi.mode(WIFI_STA);
    WiFi.config(local_IP2, gateway2, subnet);
    WiFi.begin(ssid2, password2);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
      delay(3000);
    }

    //Artnet receive data
    artnet.begin();
    artnet.setArtDmxCallback(onDmxFrame);

    //OTA programming
    ArduinoOTA.setHostname(identifier);
    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";
    });
    ArduinoOTA.onEnd([]() {});
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {});
    ArduinoOTA.onError([](ota_error_t error) {});
    ArduinoOTA.begin(); 
    
    //------------------------------ Local Loop --------------------------------//
    for(;;){
      if(millis() >= t_WifiLoop+ts_WifiLoop){
        t_WifiLoop = millis();
        Serial.println(tr);
        //If internet is connected then perform artnet read and OTA
        if(WL_CONNECTED){
          artnet.read();
          ArduinoOTA.handle();
        }
      }
      if(WL_CONNECTED){
        if(millis() >= (t_serial+ts_serial)){
          t_serial = millis();
          UDP.beginPacket(macbookIP, 4210);
          UDP.printf(String(uint32_t(5000000+tr*1000)).c_str(),10);
          UDP.printf(",");
          UDP.printf(String(uint32_t(5000000+cart.refPos*1000)).c_str(),10);
          UDP.printf(",");
          UDP.printf(String(uint32_t(5000000+cart.position*1000)).c_str(),10);
          UDP.printf(",");
          UDP.printf(String(uint32_t(5000000+cart.refVel*1000)).c_str(),10);
          UDP.printf(",");
          UDP.printf(String(uint32_t(5000000+cart.velocityFiltered*1000)).c_str(),10);
          UDP.printf(",");
          UDP.printf(String(uint32_t(5000000+pend.position*1000)).c_str(),10);
          UDP.printf(",");
          UDP.printf(String(uint32_t(5000000+pend.velocityFiltered*1000)).c_str(),10);
          UDP.printf(",");
          UDP.printf(String(uint32_t(5000000+ctrl*1000)).c_str(),10);
          UDP.endPacket();
        }
      }
    }
  }
}




//======================Main Loop========================//

void Task1_ControlSystemCode( void * pvParameters ){

  pinMode(Enc0_0.PIN, INPUT_PULLUP);
  pinMode(Enc0_1.PIN, INPUT_PULLUP);
  pinMode(Enc1_0.PIN, INPUT_PULLUP);
  pinMode(Enc1_1.PIN, INPUT_PULLUP);
  pinMode(mot.VoltPin, OUTPUT);
  pinMode(mot.DirPin0, OUTPUT);
  pinMode(mot.DirPin1, OUTPUT);
	attachInterrupt(Enc0_0.PIN, isr00, CHANGE);
  attachInterrupt(Enc0_1.PIN, isr01, CHANGE);
  attachInterrupt(Enc1_0.PIN, isr10, CHANGE);
  attachInterrupt(Enc1_1.PIN, isr11, CHANGE);

  //Inverted Pedulum control loop
  for(;;){
    if(millis() >= (tc+ts)){
      tc = millis();
      tr = tr+float(ts)/1000.0;

      //Calculate and lowpass filter cart position and velocity
      cart.lastPosition = cart.position;
      cart.lastVelocity = cart.velocity;
      cart.position = float(Enc0_0.position)/5800*0.25-0.125; //[m]
      cart.velocity = (cart.position-cart.lastPosition)*10;
      cart.positionFiltered = cart.positionFiltered + 1.0 * (cart.position - cart.positionFiltered);
      cart.velocityFiltered = cart.velocityFiltered + 0.2 * (cart.velocity - cart.velocityFiltered);

      pend.lastPosition = pend.position;
      pend.lastVelocity = pend.velocity;
      pend.position = float(Enc1_0.position)/2400*2*pi;  //[rad]
      pend.velocity = (pend.position-pend.lastPosition)*10;
      pend.positionFiltered = pend.positionFiltered + 1.0 * (pend.position - pend.positionFiltered);
      pend.velocityFiltered = pend.velocityFiltered + 0.9 * (pend.velocity - pend.velocityFiltered);

      // Stationary
      if(Mode==0){
        Pgain = 2.0;
        Vgain = 1.0;
        Igain = 5.0;
        cart.refPos = -0.125;
        cart.refVel = 0.0;
      }
      //step
      else if(Mode==1){
        Pgain = 5;
        Igain = 5;
        Vgain = 1;
        cart.refPos = 0.1*round(sin(2*pi*0.1*tr));
        cart.refVel = 0.0;
      }
      //sine
      else if(Mode==2){
        Pgain = 20;
        Vgain = 60;
        Igain = 20;// 0..25
        cart.refPos = 0.1*sin(2*pi*0.1*tr);
        cart.refVel = 0.1*0.1*cos(2*pi*0.1*tr);
      }
      //track user input
      else if(Mode==3){
        Pgain = 15;
        Vgain = 0.01;
        Igain = 10.0; // 0..25
        cart.refPos = 0.24 * (R-127)/255;
        cart.refVel = 0.0;
      }
      //track pendulum down
      else if(Mode==4){
        Pgain = 10;
        Vgain = 0.01;
        cart.refPos = 0.01 * pend.position + 0.05 * pend.velocityFiltered;
        cart.refVel = 0.0;
      }
      else{
        Pgain = 0;
        Vgain = 0;
      }

      cart.e_pos = cart.refPos-cart.positionFiltered;
      cart.e_vel = cart.refVel-cart.velocityFiltered;
      //Integral action
      errInt = errInt + cart.e_pos/10;
      //Anti-windup
      if((errInt > 0 && cart.e_pos < 0) || (errInt < 0 && cart.e_pos > 0)){
        errInt = 0;
      }
      if(errInt > 10.0){
        errInt = 10.0;
      }
      if(errInt < -10.0){
        errInt = -10.0;
      }
      ctrl = Pgain * cart.e_pos + Igain * errInt + Vgain * cart.e_vel;
      
      
      if(ctrl > 0){
        mot.Volt = int(ctrl*255);
      }
      else if(ctrl<0){
        mot.Volt = int(-ctrl*255);
      }
      else{
        mot.Volt = 0;
      }
      if(mot.Volt>255){
        mot.Volt = 255;
      }
      if(Mode>0 && ((cart.position > 0.1 && ctrl > 0) || (cart.position < -0.1 && ctrl < 0))){
        mot.Volt = 0;
      }
      //friction compensation right
      if(ctrl > 0 && mot.Volt > 0 && mot.Volt < 90){
        mot.Volt = 90;
      }
      //friction compensation left
      if(ctrl < 0 && mot.Volt > 0 && mot.Volt < 90){
        mot.Volt = 90;
      }

      mot.Direction = (ctrl<=0.0);

      if(1){
      analogWrite(mot.VoltPin,mot.Volt);
      }
      digitalWrite(mot.DirPin0,mot.Direction);
      digitalWrite(mot.DirPin1,1-mot.Direction);
      
    }
  }
}

void loop() {
  ;
}




//==============================================================================//
//==============================================================================//
//                                  Functions                                   //
//==============================================================================//
//==============================================================================//


void onDmxFrame(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data){
  sendFrame = 1;
  // set brightness of the whole strip 
  if(universe==2){
    Mode  = data[(ID-1)*4 + 0];
    if(Mode!=13){
      R     = data[(ID-1)*4 + 1];
      G     = data[(ID-1)*4 + 2];
      B     = data[(ID-1)*4 + 3];
      W     = data[(ID-1)*4 + 4];
      Vel   = data[(ID-1)*4 + 5];
    }
    
  }
  if(universe==3){
    Mode  = data[(ID-1)*4 + 0];
    if(Mode!=13){
      R_1     = data[(ID-1)*4 + 1];
      G_1     = data[(ID-1)*4 + 2];
      B_1     = data[(ID-1)*4 + 3];
      W_1     = data[(ID-1)*4 + 4];
      Vel_1   = data[(ID-1)*4 + 5];
    }
  }
}