// This script is used to control an inverted pedulum based om an old inkject printer
//  - The controller is an ESP32 wroom development board
//  - The printer has two optical encoders (1 linear and 1 rotational) and a DC motor
//  - A pythonista interface is used to send artnet DMX messages to control the modes and parameters
//
//  This code can be freely used for personal and educational purposes
//
//  Last update: 18 Jan 2024,  Jasper Flohil

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

// Create Artnet and UPD objects 
ArtnetWifi artnet;
ArtnetWifi artnetSend;
WiFiUDP UDP;
IPAddress macbookIP(192,168,2,23);
unsigned int localUdpPort = 4210;

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

//========================Setup========================//
void setup() {
  //Initialize processor tasks
  xTaskCreatePinnedToCore(Task0_WifiCode, "Task0", 10000,  NULL, 1, &Task0_Wifi,0); 
  xTaskCreatePinnedToCore(Task1_ControlSystemCode, "Task1", 10000,  NULL, 1, &Task1_ControlSystem,1); 

}

//Task 0 runs on processor core 0 and controls WIFI, all communication and OTA programming
void Task0_WifiCode( void * pvParameters ){
  //------------------------------Local Setup --------------------------------//
  Serial.begin(9600);
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
    //Send UPD message with system data
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
        UDP.printf(String(uint32_t(5000000+cart.accelerationFiltered*1000)).c_str(),10);
        UDP.printf(",");
        UDP.printf(String(uint32_t(5000000+pend.position*1000)).c_str(),10);
        UDP.printf(",");
        UDP.printf(String(uint32_t(5000000+pend.velocityFiltered*1000)).c_str(),10);
        UDP.printf(",");
        UDP.printf(String(uint32_t(5000000+ctrl*255*1000)).c_str(),10);
        UDP.printf(",");
        UDP.printf(String(uint32_t(5000000+Mode*1000)).c_str(),10);
        UDP.printf(",");
        UDP.printf(String(uint32_t(5000000+Pgain*1000)).c_str(),10);
        UDP.printf(",");
        UDP.printf(String(uint32_t(5000000+Igain*1000)).c_str(),10);
        UDP.printf(",");
        UDP.printf(String(uint32_t(5000000+Dgain*1000)).c_str(),10);
        UDP.printf(",");
        UDP.printf(String(uint32_t(5000000+Xgain*1000)).c_str(),10);
        UDP.endPacket();
      }
    }
  }
}

//Taks1 controls the motion of the pendulum
void Task1_ControlSystemCode( void * pvParameters ){
  //------------------------------Local Setup --------------------------------//
  // configure LED PWM functionalitites
  ledcSetup(motorChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(mot.VoltPin, motorChannel);

  pinMode(Enc0_0.PIN, INPUT_PULLUP);
  pinMode(Enc0_1.PIN, INPUT_PULLUP);
  pinMode(Enc1_0.PIN, INPUT_PULLUP);
  pinMode(Enc1_1.PIN, INPUT_PULLUP);
  //pinMode(mot.VoltPin, OUTPUT);
  pinMode(mot.DirPin0, OUTPUT);
  pinMode(mot.DirPin1, OUTPUT);
	attachInterrupt(Enc0_0.PIN, isr00, CHANGE);
  attachInterrupt(Enc0_1.PIN, isr01, CHANGE);
  attachInterrupt(Enc1_0.PIN, isr10, CHANGE);
  attachInterrupt(Enc1_1.PIN, isr11, CHANGE);

  //------------------------------ Local Loop --------------------------------//
  for(;;){
    if(millis() >= (tc+ts)){
      tc = millis();
      tr = tr+float(ts)/1000.0;

      //Calculate and lowpass filter cart position and velocity
      cart.lastPosition = cart.position;
      cart.lastVelocity = cart.velocity;
      cart.position = float(Enc0_0.position)/(5800/0.25)-0.125; //[m]
      cart.velocity = (cart.position-cart.lastPosition)*(1000/float(ts));
      cart.acceleration = (cart.velocity-cart.lastVelocity)*(1000/float(ts));
      cart.positionFiltered = cart.positionFiltered + 0.1 * (cart.position - cart.positionFiltered);
      cart.velocityFiltered = cart.velocityFiltered + 0.1 * (cart.velocity - cart.velocityFiltered);
      cart.accelerationFiltered = cart.accelerationFiltered + 0.1 * (cart.acceleration - cart.accelerationFiltered);

      pend.lastPosition = pend.position;
      pend.lastVelocity = pend.velocity;
      pend.position = float(Enc1_0.position)/(2400/(2*pi));  //[rad]
      pend.velocity = (pend.position-pend.lastPosition)*(1000/float(ts));
      pend.positionFiltered = pend.positionFiltered + 0.1 * (pend.position - pend.positionFiltered);
      pend.velocityFiltered = pend.velocityFiltered + 0.1 * (pend.velocity - pend.velocityFiltered);

      //Switch off
      if(Mode==0){
        Pgain = 0;
        Dgain = 0;
        Igain = 0;
        cart.refPos = -0.125;
        cart.refVel = 0;
      }
      // Stationary
      if(Mode==1){
        Pgain = float(G)/5;//5.0;
        Igain = float(B)/5;//0.0;
        Dgain = float(W)/10;//0.0;
        cart.refPos = -0.125;
        cart.refVel = 0.0;
      }
      //step response
      else if(Mode==2){
        Pgain = float(G)/5;
        Igain = float(B)/5;
        Dgain = float(W)/10;
        cart.refPos = 0.1*round(sin(2*pi*0.1*tr));
        cart.refVel = 0.0;
      }
      //sine
      else if(Mode==3){
        Pgain = float(G)/3;
        Igain = float(B)/3;
        Dgain = float(W)/10;
        cart.refPos = 0.1*sin(2*pi*0.1*tr);
        cart.refVel = 0.1*cos(2*pi*0.1*tr);
      }
      //track user position input
      else if(Mode==4){
        Pgain = float(G)/3;
        Igain = float(B)/3;
        Dgain = float(W)/10;
        cart.refPos = 0.24 * (float(R)-127)/255;
        cart.refVel = 0.0;
      }
      //stabilize pendulum down position
      else if(Mode==5){
        //Pendulum reference to center cart
        pend.refPos = cart.position;
        pend.e_pos = pend.refPos - pend.position;

        //Integrator
        if(pend.position > -0.5 && pend.position < 0.5){
          PendErrInt = PendErrInt + pend.e_pos * float(ts)/1000;
          if(PendErrInt > 1){
          PendErrInt = 1;
          }
          if(PendErrInt < -1){
            PendErrInt = -1;
          }
          ctrl = -10.0 * pend.e_pos - 0.0 * PendErrInt - 0.0 * pend.velocityFiltered;
        }
        else{
          ctrl = 0;
        }
      }
      //stabilize pendulum up position
      else if(Mode==6){

        //Control Parameters
        Pgain = 10.0;
        Igain = 150.0;
        Dgain = 0.1;
        Pgain = float(G)/3;
        Igain = float(B)*2;
        Dgain = float(W)/50;
        Xgain = float(Vel)/50;

        //Pendulum reference to center cart
        pend.refPos =  Dgain * cart.position;
        pend.e_pos = (pend.refPos - (pend.position-pi));

        
        //Integrator
        if(pend.position > (pi-0.5) && pend.position < (pi+0.5)){
          PendErrInt = PendErrInt + pend.e_pos * float(ts)/1000;
          if(PendErrInt > 1){
          PendErrInt = 1;
          }
          if(PendErrInt < -1){
            PendErrInt = -1;
          }
          ctrl = Pgain * pend.e_pos + Igain * PendErrInt - Xgain * pend.velocityFiltered;
        }
        else{
          ctrl = 0;
        }
        
      }
      //RBS
      else if(Mode==7){
        if(millis() >= (t_rbs+ts_rbs)){
          t_rbs = millis();
          float amp = float(random(100,220))/255;
          float direction = float(random(0,2));
          ts_rbs = random(10,90);
          if(direction){
            ctrl = amp;
          }
          else{
            ctrl = -amp; 
          }
        }
      }
      else if(Mode==14){
        Enc0_0.position = 0;
        Enc0_1.position = 0;
        Enc1_0.position = 0;
        Enc1_1.position = 0;
      }
      else{
        Pgain = 0;
        Igain = 0;
        Dgain = 0;
      }

      cart.e_pos = cart.refPos-cart.positionFiltered;
      cart.e_vel = cart.refVel-cart.velocityFiltered;

      //Integral action only in small error band of +-2cm
      if(cart.e_pos < 0.02 && cart.e_pos > -0.02){
        errInt = errInt + cart.e_pos*float(ts)/1000;
      }
      //Limit Integrator action
      if(errInt > 0.5){
        errInt = 0.5;
      }
      if(errInt < -0.5){
        errInt = -0.5;

      }
      if(Mode<5){
        ctrl = Pgain * cart.e_pos + Igain * errInt + Dgain * cart.e_vel;
      }
      
      //Write control calue to output value which is unsigned
      if(ctrl > 0){
        mot.Volt = int(ctrl*255);
      }
      else if(ctrl<0){
        mot.Volt = int(-ctrl*255);
      }
      else{
        mot.Volt = 0;
      }

      //Limit motor voltage
      if(mot.Volt>255){
        mot.Volt = 255;
      }
      //Cutoff control output when cart reaches end of track
      if(Mode > 1 && ((cart.position > 0.11 && ctrl > 0) || (cart.position < -0.11 && ctrl < 0))){
        mot.Volt = 0;
      }
      
      //Coulomb friction compensation right
      float FrStaticR = 150;
      if(ctrl > 0 && mot.Volt > 0 && mot.Volt < FrStaticR){
        mot.Volt = FrStaticR;
      }
      //Coulomb friction compensation left
      float FrStaticL = 155;
      if(ctrl < 0 && mot.Volt > 0 && mot.Volt < FrStaticL){
        mot.Volt = FrStaticL;
      }
      
      //Determine motor direction
      mot.Direction = (ctrl<=0.0);
      
      //Write control value to motor
      if(Mode == 0){
        ledcWrite(motorChannel,0);
      }
      else{
        ledcWrite(motorChannel,mot.Volt);
      }
      //Write motor direction to motor
      digitalWrite(mot.DirPin0,mot.Direction);
      digitalWrite(mot.DirPin1,1-mot.Direction);
    }
  }
}

// Arduino code loop (needs to be here but has no function)
void loop() {
  ;
}

//==============================================================================//
//==============================================================================//
//                                  Functions                                   //
//==============================================================================//
//==============================================================================//

//Read DMX values when available
void onDmxFrame(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data){
  sendFrame = 1;
  // set brightness of the whole strip 
  if(universe==2){
    Mode  = data[(ID-1)*4 + 0];
      R     = data[(ID-1)*4 + 1];
      G     = data[(ID-1)*4 + 2];
      B     = data[(ID-1)*4 + 3];
      W     = data[(ID-1)*4 + 4];
      Vel   = data[(ID-1)*4 + 5];
    
  }
  if(universe==3){
    Mode  = data[(ID-1)*4 + 0];
      R_1     = data[(ID-1)*4 + 1];
      G_1     = data[(ID-1)*4 + 2];
      B_1     = data[(ID-1)*4 + 3];
      W_1     = data[(ID-1)*4 + 4];
      Vel_1   = data[(ID-1)*4 + 5];
  }
}
