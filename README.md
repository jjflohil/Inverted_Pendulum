# Inverted-pendulum-code-for-tuning-and-testing-controllers-on-a-ESP32-module

In this project an inverted pendulum is controlled using an ESP32 module. The inverted pendulum is based on an old inkjet printer which has a DC motor and a linear  optical encoder. On what used to be the ink holder platform, a rotational optical encoder is placed to measure the pendulum angle. The motor is controlled using a l298n motor driver with a 12v power supply. 

Here is a brief explanation of the files: 

- main.cpp - Main code that runs on the ESP32 module to control the inverted pendulum
- GlobalParameters.h - Contain global parameters used in main code
- "receiveUDP.py" - Python script that receives UPD messages and generates live plot of the data

In CVS folder, csv files are saved with measured data. The data consists of input U(voltage) and outputs X(position cart) and Theta(angle pendulum). The data can be used to calculate controller performance or to identify a system model.