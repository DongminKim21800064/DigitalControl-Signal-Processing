#pragma once

#define   _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <time.h>
#include <math.h>
#include <conio.h>

#include "NIDAQmx.h"


// main
void DAQ_Initialization(void);
void gyro_offset_calc(void);
void Initialization(void);
void Import_Data(void);
void Controller(void);
void Kbhit(void);
double Vcmd_safe(double input);
void Angle_stop(void);
void Check_Gyro(void);
void Export_Data(void);
void Time_Management(void);
void bufData(void);
void Check_Stop_Condition(void);
void DAQ_Terminatation(void);
void File_Writing(void);

// controller
double P_Controller(double error, double gain);
double I_Controller(double Prev_I_out, double cur_error, double prev_error, double gain, double sampling_period);
double D_Controller(double error, double gain);

// signal
void Triangle_Wave_Signal(void);
void Sine_Wave_Signal(void);
void Step_Signal(void);