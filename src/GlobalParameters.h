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
uint32_t  t_rbs   = 0;   // Current time effect
uint16_t  ts_rbs  = 200;   // Sample time effect

//Function Parameters
uint8_t ModeP = 0;
uint8_t nLEDs_H = 12;
uint32_t LeftColor[12];
uint32_t RightColor[12];
uint32_t TotalColor[23];


//State Feedback Parameters
float A_system[1];
//Observer Parameters


//State feedback LQR
struct LQR5{
    float Q[5][5];
    float R;
    float K[1][5];
};
//State space representation with 5 states, 2 outputs and 1 input
struct StateSpace_5_2_1{
    float A[5][5];
    float B[1][5];
    float C[5][2];
    float D[1][2];
};

StateSpace_5_2_1 Observer = {{1,         0.005,  0,          -0.002      ,   0       ,
                              0.0905,    1,      0.0763,     -0.9047     ,   0.0131  ,
                              0,         0,      1,          0.0048      ,   0       ,
                              0,         0,      0,          0.8984      ,   00032   ,
                              0,         0,      -0.0036,    -34.6930    ,   0.3417  },
                            
                            {0		    ,-0.0029,   0,	    -0.0008,    	0.2370  },
                            
                            {1, 0, 0, 0, 0, 
                             0, 0, 1, 0, 0},

                            {0, 0}
                             };
   

float MatMul(float Mat0In[], float Mat1In[], int Size0, int Size1){
    float MatOut[Size0][Size1];

    for(int i=0;i<Size0;i++){
        MatOut[i][0] =+ Mat0In[i]*Mat0In[i+Size1]; 
    }

}
//m(0, 0) = 3;
//m(1, 0) = 2.5;
//m(0, 1) = -1;
//m(1, 1) = m(1, 0) + m(0, 1);
//MatrixXd m = MatrixXd::Random(3, 3);
//m = (m + MatrixXd::Constant(3, 3, 1.2)) * 50;

//LQR5 LQR_UP
/*
//Q = diag([1 0.01 20000 2.2 0]);
//R = 20*10^-9;
//K_ul = dlqr(A_ou,B_ou,Q,R);
//u = round(-K_ul*[theta ; x_o(2) ; q ; x_o(4) ; x_o(5)] + G*r);

% State feedback LQR
Q = diag([5 0.4 4000 2 0]);
R = 20*10^-6;
K_u = dlqr(A_ou,B_ou,Q,R);

% Observer LQR
Q = diag([1 0 1 0.1 0]);
R = [10 0 ; 0 50];
L_u = dlqr(A_ou',C_ou',Q,R)';

% State feedback LQR down
Q = diag([30 0.1 10000 0.6 0]);
R = 20*10^-9;
K_d = dlqr(A_od,B_od,Q,R);

% Observer LQR down
Q = diag([1 1 1 1 0]);
R = [1 0 ; 0 10];
L_d = dlqr(A_od',C_od',Q,R)';

% Gain which sets DC gain to 1 (Only perfect in case the model is perfect)
G =  1./(Du(2)-(Cu(2,:)-Du(2)*K_u)*inv(Au-Bu*K_u)*Bu);
*/
