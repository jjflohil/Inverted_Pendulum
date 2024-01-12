#include <Arduino.h>


//Parameters
const float pi = 3.1415;

struct component {
  float position;
  float velocity;
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

component cart = {0,0};
component pend = {0,0};

motor mot = {26,18,19,0,false};

//Time parameters
long  tc = 0;
int   ts = 1;
float tr = 0;
long  t_serial = 0;
int   ts_serial = 1;

//Temporary control parameters
float ref[2] = {0.0, 0.0}; //[m, m/s]
float ctrl = 0;


//Encoder interrupts
void IRAM_ATTR isr0() {
  if(digitalRead(Enc0_1.PIN)){
    Enc0_0.position++;
  }
  else{
    Enc0_0.position--;
  }
}
void IRAM_ATTR isr() {
  if(digitalRead(Enc1_0.PIN)){
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

float Pgain = 10;
float Vgain = 0.0001;
float LP = 1.0;
//========================Setup========================//
void setup() {
	Serial.begin(9600);
	pinMode(Enc0_0.PIN, INPUT_PULLUP);
  pinMode(Enc0_1.PIN, INPUT_PULLUP);
  pinMode(Enc1_0.PIN, INPUT_PULLUP);
  pinMode(Enc1_1.PIN, INPUT_PULLUP);
  pinMode(mot.VoltPin, OUTPUT);
  pinMode(mot.DirPin0, OUTPUT);
  pinMode(mot.DirPin1, OUTPUT);
	attachInterrupt(Enc0_0.PIN, isr0, RISING);
  attachInterrupt(Enc1_1.PIN, isr, RISING);
}


//======================Main Loop========================//
void loop() {

  //Sine reference of 

  if(millis() > (tc+ts)){
    tc = millis();
    tr = tr+0.001;

    //Calculate and lowpass filter cart position and velocity
    PosScaled = float(Enc0_0.position)/1400*0.247; //[m]
    VelScaled = (PosScaled-lastP)/1400*0.247*1000;

    cart.position = lastP + LP*(PosScaled - lastP);
    cart.velocity = lastV + LP*(VelScaled - lastV);
    
    lastP = cart.position;
    lastV = cart.velocity;

    pend.position = float(Enc1_0.position)/600*2*pi;  //[rad]

    ref[0] = 0.1*round(sin(2*pi*0.5*tr));
    ref[1] = //0.5*0.1*cos(2*pi*0.5*tr);
    
    ctrl = Pgain * (ref[0]-cart.position) + Vgain*(ref[1]-cart.velocity);
    
    //ctrl = sin(2*pi*0.2*tr);
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
    //friction compensation right
    if(ctrl > 0 && mot.Volt > 0 && mot.Volt < 145){
      mot.Volt = 145;
    }
    //friction compensation left
    if(ctrl < 0 && mot.Volt > 0 && mot.Volt < 150){
      mot.Volt = 150;
    }

    mot.Direction = (ctrl<=0.0);
    if(1){
    analogWrite(mot.VoltPin,mot.Volt);
    }
    digitalWrite(mot.DirPin0,mot.Direction);
    digitalWrite(mot.DirPin1,1-mot.Direction);
    
  }
  /*
  if(millis() > (t_serial+ts_serial)){
    t_serial = millis();
    Serial.print(ref[0]);
    Serial.print("  ,  ");
    Serial.print(cart.position);
    Serial.print("  ,  ");
    Serial.print(ref[1]);
    Serial.print("  ,  ");
    //Serial.println(pend.position);
    Serial.println(cart.velocity);
  }
  */
}