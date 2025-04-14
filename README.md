# This repository consist of MATLAB code to perform Model Predictive Control on DC Motor driven car. This repository also consist of Simulink block diagrm that uses ordinary Propotional Integral Control.

## runDynamics.m consist of MATLAB code that performs the main operation of Model predictive Control

## solveMPC.m builds the algorithm for Model Predictive Control and solves non linear optimization

## RHS_DCCar.m consist of MATLAB code that defines the Right hand side of the Ordinary differential equation with controls
## RHS_DCCar_cons.m consist of MATLAB code that defines the Right hand side of the Ordinary differential equation without controls

## phasepotrait.m consist of model that plots 2D phase potrait of the non linear system

![Description](https://github.com/tejasbg004/Model-Predictive-Control-driven-DC_Car/blob/main/Feedback%20system.png)

![Description](https://github.com/tejasbg004/Model-Predictive-Control-driven-DC_Car/blob/main/Main%20system.png)



