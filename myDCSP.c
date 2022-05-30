#include "myDCSP.h"

//-------------------------------------------------
// Macros
//-------------------------------------------------

#define NEW_LINE                printf("\n");
#define SEP_LINE                printf("-------------------------------------------------\n");

#define MAX( X, LIM_MAX )       ((X<LIM_MAX)?X:LIM_MAX)
#define MIN( X, LIM_MIN )       ((X<LIM_MIN)?X:LIM_MIN)

//-------------------------------------------------
// definitions
//-------------------------------------------------

#define   SAMPLING_FREQ      (double)(                      200 )
#define   SAMPLING_TIME      (double)(          1/SAMPLING_FREQ )
#define   FINAL_TIME         (double)(                    20)   // length of whole time
#define   STOP_TIME          (double)(                      3.0 )

#define   N_STEP             (int)   ( FINAL_TIME*SAMPLING_FREQ )

#define   UNIT_PI            (double)(         3.14159265358979 )

#define   V_REF              (double)(                      0.0 )   // target of gimbal
#define   V_STOP             (double)(                      2.5 )

#define   A_CCW              (double)(      0.2273)
#define   B_CCW              (double)(        1.67 )
#define   A_CW               (double)(        0.2273 )
#define   B_CW               (double)(       2.1936)

#define   READ(Task,data,size)       DAQmxReadAnalogF64(Task, DAQmx_Val_Auto, FINAL_TIME, DAQmx_Val_GroupByChannel, data, size * sizeof(float64), NULL, NULL)
#define   WRITE(Task,data)           DAQmxWriteAnalogF64(Task, 1, DAQmx_Val_Task_Start, FINAL_TIME, DAQmx_Val_GroupByChannel, data, NULL, NULL)

//-------------------------------------------------
// global variables
//-------------------------------------------------

static double      iniTime = 0.0; // initial time 
static double      curTime = 0.0; // current time 
static double      delTime = 0.0; // time deviation (time elapsed)
static double      simTime = 0.0;
static double      preTime = 0.0;
static int         count   = 0;


//-------------------------------------------------
// data buffer
//-------------------------------------------------

static double      OutDeg [N_STEP]  = { 0.0, };
static double      OutTime[N_STEP]  = { 0.0, };
static double      OutVc  [N_STEP]  = { 0.0, };
static double      OutVcmd[N_STEP]  = { 0.0, };
static double      OutGyro[N_STEP]  = { 0.0, };


//--------------------------------------------------
// Input data
//--------------------------------------------------

static double      V_flag_on        = 5.0;
static double      V_flag_off       = 0.0;
static double      Vcmd_stop        = 2.5;
static double      Vc               = 2.5;
static double      Vcmd             = 0.0;
static double      V_DEL            = 0.01;
static double      Vc_CW_buf[] = {0.0,};
static double      Vc_CCW_buf[] = {0.0,};
static double      Vcmd_CW_buf[] = { 0.0, };
static double      Vcmd_CCW_buf[] = { 0.0, };
static double       CW_Gyro_buf[] = { 0.0 };
static double       CCW_Gyro_buf[] = { 0.0 };



double             DEL = 0.01;



static double      Sin_Freq         = 10.5;      // Starting Freq of Sine Wave
static double      FreqDel          = 0.05;      // 주파수 간격
static double      Sin_Mag          = 2.50;

enum DAQ_PIN_MAP_MOTOR_OUTPUT { Left_Command, Right_Command };
float64            Wcmd[2]          = { 0.0, }; // [deg/s]

enum SensorMeasure_DATA { DIR = 0, DOA, GYRO, POTENTIO, N_MEAS }; // N_MEAS = #data, 4
float64            Vin[N_MEAS]      = { 0.0, }; // DIR(sign), DOA(mag), Gyro, Potentio

static double      PsiG             = 0.0;   // gimbal angle [deg]
static double      Gyro             = 0.0;   // [deg/sec]
static double      Gyro_offset      = 0.0;   // [V]
static double      Gyro_offset_sum  = 0.0;   // [V]
static double      RelPsiG          = 0.0;   // gimbal angle command [deg]
static double      ErrPsiG          = 0.0;   // gimbal angle errer [deg]

//-------------------------------------------------
// output data
//--------------------------------------------------

double             Freq             = 1.0;
static double      Standard         = 2.5;


//----------------------------------------------
// Controller 변수
//--------------------------------------------------

double             P_gain       = 0.0;
double             P_out        = 0.0;

double             I_gain       = 0.0;
double             I_out        = 0.0;
double             Prev_I_out   = 0.0;

double             D_gain       = 0.0;
double             D_out        = 0.0;

double             Error        = 0.0;
double             Prev_Error   = 0.0;

double             gain         = 0.025;
//--------------------------------------------------
// File save etc
double         DataNum          = 0;
char           OutFileName[100] = { "" };
char           name        [40] = "Gyro_Vcmd = ";   // 파일이름
char           extension   [10] = ".txt";           // 확장자

int            idx              = 0;
unsigned       i                = 0;

//----------------
// gyro vs Vcmd 
int            N                = 1;
int            key;

TaskHandle     TaskAi           = 0;
TaskHandle     TaskAo           = 0;
TaskHandle     TaskAo_flag      = 0;

//-------------------------------------------------
// DAQ Operation
//-------------------------------------------------

#define        MAX_VOLT    (double)( 5.0 )
#define        MIN_VOLT    (double)( 0.0 )

#define        AO_Vcmd      DAQmxWriteAnalogF64(TaskAo, 1, 0, FINAL_TIME, DAQmx_Val_GroupByChannel, &Vc, NULL, NULL)
#define        AO_STOP      DAQmxWriteAnalogF64(TaskAo, 1, 0, FINAL_TIME, DAQmx_Val_GroupByChannel, &Vcmd_stop, NULL, NULL)
#define        MOTOR_ON     DAQmxWriteAnalogF64(TaskAo_flag, 1, 0, FINAL_TIME, DAQmx_Val_GroupByChannel, &V_flag_on, NULL, NULL)
#define        MOTOR_OFF    DAQmxWriteAnalogF64(TaskAo_flag, 1, 0, FINAL_TIME, DAQmx_Val_GroupByChannel, &V_flag_off, NULL, NULL)



//-------------------------------------------------
// function declaration
//-------------------------------------------------

// 1. check the windows time in [ms] 
double GetWindowTime(void)
{
    LARGE_INTEGER   liEndCounter, liFrequency;
    QueryPerformanceCounter(&liEndCounter);
    QueryPerformanceFrequency(&liFrequency);
    return(liEndCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);
}

void Linearization(void)
{
    if      (Vc > 2.5)     Vcmd = A_CW * Vc + B_CW;
    else if (Vc < 2.5)     Vcmd = A_CCW * Vc + B_CCW;
    else                   Vcmd = V_STOP;

    OutVc[count] = Vc;
    OutVcmd[count] = Vcmd;
}


// 2. Initialization
void DAQ_Initialization(void)
{
    DAQmxCreateTask("", &TaskAi);
    DAQmxCreateTask("", &TaskAo);
    DAQmxCreateTask("", &TaskAo_flag);

    // 태스크는 타이밍, 트리거, 기타 프로퍼티를 가진 하나 또는 그 이상 버추얼 채널의 모음입니다. 
    // 개념적으로 태스크는 실행하려는 측정(Measurement)이나 생성(Generation)을 나타냅니다. 
    DAQmxCreateAIVoltageChan(TaskAi, "Dev1/ai0:3", "", DAQmx_Val_RSE, MIN_VOLT, MAX_VOLT, DAQmx_Val_Volts, NULL);

    // task handle, physical channel, virtual name, terminal config, min, max,단위, custom scale name
    // Creates channel(s) to measure voltage and adds the channel(s) 
    // to the task you specify with taskHandle.

    DAQmxCreateAOVoltageChan(TaskAo, "Dev1/ao1", "", MIN_VOLT, MAX_VOLT, DAQmx_Val_Volts, NULL);
    DAQmxCreateAOVoltageChan(TaskAo_flag, "Dev1/ao0", "", MIN_VOLT, MAX_VOLT, DAQmx_Val_Volts, NULL);

    DAQmxStartTask(TaskAi);
    DAQmxStartTask(TaskAo);
    DAQmxStartTask(TaskAo_flag);

    printf("DAQ Setting complete.\n");
}

void gyro_offset_calc(void)
{
    for (i = 0; i < 1000; i++)
    {
        READ(TaskAi, Vin, N_MEAS);
        Gyro_offset_sum += Vin[GYRO];
    }
    Gyro_offset = Gyro_offset_sum / 1000;
    printf("Gyro offset = %lf\n", Gyro_offset);
}

void Initialization(void)
{
    DAQ_Initialization();
    gyro_offset_calc();
    printf("Press any key to start the program.... \n");
    getchar();

    iniTime = GetWindowTime() * 0.001;
    Vc      = V_STOP;
    RelPsiG = V_REF;
    MOTOR_ON;
}



// 3. Import Data 
void Import_Data(void)
{
    const double SACLE_POTENTIO_V2DEG = 70.11;
    const double OFFSET_POTENTIO_V2DEG = 181.9;
    
    READ(TaskAi, Vin, N_MEAS);

    Gyro    = (Vin[GYRO] - Gyro_offset) * 1000 / 0.67;                      // [deg/sec]
    PsiG    = SACLE_POTENTIO_V2DEG * Vin[POTENTIO] - OFFSET_POTENTIO_V2DEG; // [deg]
    ErrPsiG = RelPsiG - PsiG;
}


// 4. Controller
void Controller(void)
{
    //double P_Controller(double ErrPsiG, double gain);
    //ErrPsiG = ErrPsiG * gain + 2.5;
}

// 5. Export Data
void Kbhit(void)
{
    if (_kbhit())
    {
        key = _getch();
        switch (key)
        {
        case 75: printf("LEFT\n");  Vc += V_DEL; break;
        case 77: printf("RIGHT\n"); Vc -= V_DEL; break;
        case 80: printf("STOP\n");  Vc = V_STOP; break;
        }
    }
}
double Vcmd_safe(double input)
{
    if (input >= 5)
        return 5;
    else if (input <= 0)
        return 0;
    else
        return input;
}

void Angle_stop(void)
{
    if (Vin[POTENTIO] >= 1.7 + 0.074 * 22) // 원하는 voltage
        Vc = V_STOP;
}
void Check_Gyro(void)
{
    Vc = 0.1 * Vcmd_stop;
}

void Export_Data(void)
{
    //Triangle_Wave_Signal();
    Sine_Wave_Signal();
    //Step_Signal();

    Linearization();
    Vcmd = Vcmd_safe(Vcmd);
    
    DAQmxWriteAnalogF64(TaskAo, 1, 0, FINAL_TIME, DAQmx_Val_GroupByChannel, &Vcmd, NULL, NULL);
  
}



// 6. Time Management
void Time_Management(void)
{
    while (1)
    {
        curTime = GetWindowTime() / 1000;
        delTime = curTime - iniTime - simTime;
        if (delTime >= (SAMPLING_TIME))   break;
    }
    
    bufData();

    simTime = SAMPLING_TIME * ((double)count + 1);
    count++;
}


void bufData(void)
{
    OutTime[count] = simTime;
    OutVc  [count] = Vc;
    OutVcmd[count] = Vcmd;
    OutDeg [count] = Vin[POTENTIO];// 김발 헤드 각도
    OutGyro[count] = Gyro;
}

// 7. Check Stop Condition
void Check_Stop_Condition(void)
{
    //if (N % 2 == 1)
    //{
    //    if (simTime < STOP_TIME)
    //        return 1;
    //    else
    //        return 0;
    //}
    //else
    //{
    //    if (simTime < FINAL_TIME)
    //        return 1;
    //    else
    //        return 0;
    //}
    if (simTime < FINAL_TIME)
        return 1;
    else
        return 0;
}




// 8. Stop DAQ
void DAQ_Terminatation(void)
{
    AO_STOP;
    MOTOR_OFF;
    printf("Press any key to terminate the Program.... \n");
    getchar();

    DAQmxStopTask(TaskAi);
    DAQmxStopTask(TaskAo);
    DAQmxStopTask(TaskAo_flag);
    // 자동으로 실행 정지를 막기 위함

    DAQmxClearTask(TaskAi);
    DAQmxClearTask(TaskAo);
    DAQmxClearTask(TaskAo_flag);
}



// 9. File Writing
void File_Writing(void)
{
    FILE* pFile = NULL;

    sprintf(OutFileName, "Step_Response");
    pFile = fopen(strcat(OutFileName, ".txt"), "w+t");

    printf("%s\n", OutFileName);

    for (idx = 0; idx < N_STEP; idx++)
    {
        fprintf(pFile, "\%20.10f %20.10f %20.10f %20.10f\n",
            OutTime[idx], OutVc[idx], OutVcmd[idx], OutGyro[idx]);
    }
    _fcloseall();
}



//=============================================================//
// triangular
void Triangle_Wave_Signal(void)
{
    Vc += DEL;
    if (Vc > 5 || Vc < 0)
        DEL = -DEL;
}


void Sine_Wave_Signal(void)
{
    Vc = Sin_Mag * sin(2 * UNIT_PI * Sin_Freq * simTime) + 2.5;
}


void Step_Signal(void) 
{
    if (simTime >= 1.0 && simTime < 4.0)
        Vc = 5.0;
    else if (simTime >= 5.0 && simTime < 9.0)
        Vc = 0.0;
    else if (simTime >= 10.0 && simTime < 14.0)
        Vc = 5.0;
    else if (simTime >= 15.0 && simTime < 19.0)
        Vc = 0.0;
    else
        Vc = 2.5;
}

//=============================================================//
//// P 제어기
//double P_Controller(double error, double gain)
//{
//    P_out = error * gain;
//    return P_out;
//}
//
//// I 제어기 - Trapezoidal rule
//double I_Controller(double Prev_I_out, double cur_error, double prev_error, double gain, double sampling_period)
//{
//    I_out = Prev_I_out + gain * sampling_period * (cur_error + prev_error) / 2;
//}
//
//// D 제어기 
//double D_Controller(double error, double gain)
//{
//    // = gain * 
//}

void Static_Characteristic(void)
{
    Linearization();
    DAQmxWriteAnalogF64(TaskAo, 1, 0, FINAL_TIME, DAQmx_Val_GroupByChannel, &Vcmd, NULL, NULL);
    double Vin_buf = 0;
    for (i = 0; i < 300; i++)
    {
        if (i > 100 && i < 200) {
            READ(TaskAi, Vin, N_MEAS);
            Vin_buf += Vin[GYRO];
        }
        Sleep(1);
    }
    Vin[GYRO] = Vin_buf / 100;
    Gyro = (Vin[GYRO] - Gyro_offset) * 1000 / 0.67 +20;

    //if (count % 2 == 0) {
    //    Vc_CCW_buf[count] = Vc;
    //    OutVcmd[count] = Vcmd_CCW_buf[count];
    //    OutGyro[count] = CCW_Gyro_buf[count];
    //}
    //else if (count % 2 == 1) {
    //    OutVc[count] = Vc_CW_buf[count];
    //    OutVcmd[count] = Vcmd_CW_buf[count];
    //    OutGyro[count] = CW_Gyro_buf[count];
    //}

    if (Vc > 2.0 && Vc < 3.0){
        DEL = 0.01;
    } else{
        DEL = 0.05;
    }
    Vc += DEL;
    if (count == 0)
        Vc = 0;
    /*if (Vc > 3 || Vc < 2) {
        DEL = -DEL;
        Vc = 2.5;
    }*/

   
    Vcmd = Vcmd_safe(Vcmd);

   
    DAQmxWriteAnalogF64(TaskAo, 1, 0, FINAL_TIME, DAQmx_Val_GroupByChannel, &Vcmd_stop, NULL, NULL);
    Sleep(500);

    while (1)
    {
        curTime = GetWindowTime() / 1000;
        delTime = curTime - iniTime - simTime;
        if (delTime >= (SAMPLING_TIME))   break;
    }

    bufData();
    
    OutTime[count] = simTime;
    
 /*   if (count % 2 == 0) {
        OutVc[count] = Vc_CCW_buf[count];
        OutVcmd[count] = Vcmd_CCW_buf[count];
        OutGyro[count] = CCW_Gyro_buf[count];
    }
    else if (count % 2 == 1) {
        OutVc[count] = Vc_CW_buf[count];
        OutVcmd[count] = Vcmd_CW_buf[count];
        OutGyro[count] = CW_Gyro_buf[count];
    }*/
    OutVc[count] = Vc;
    OutVcmd[count] = Vcmd;
    OutDeg[count] = Vin[POTENTIO];// 김발 헤드 각도
    OutGyro[count] = Gyro;
    printf("time = %f  Vc = %f  Vcmd = %f  Wgyro = %f\n\n", OutTime[count], OutVc[count], OutVcmd[count], OutGyro[count]);

    simTime = SAMPLING_TIME * ((double)count + 1);
    count++;

}