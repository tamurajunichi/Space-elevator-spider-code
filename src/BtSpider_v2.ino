/******************************************************
 * @name: AUTO SPIDER No,2 (version0.01)              *
 *                                                    *
 * @brief:                                            *
 *         Climbing SPIDER program with Arduino       *
 *                                                    *
 * @author: Junichi Tamura                            *
 *                                                    *
 * @date: Winter 2016                                 *
 ******************************************************/
#include"Servo.h"
#include"MsTimer2.h"
#include <SoftwareSerial.h>

/*
 * Defined Pins
 */
const int  EscPin = 9;
const int  BrakeSvPin = 10;
const int  EncAPin = 2;
const int EncBPin = 3;
const int  Sw1Pin = 12;
const int  Sw2Pin = 13;
const int Led1Pin = 7;
const int Led2Pin = 8;
const int RxPin = 5;
const int TxPin = 4;
const int UsrSwPin = 6;

/*
 * Objects
 */
Servo Esc,BrakeSv;
SoftwareSerial BtSerial(RxPin,TxPin),logger(1,0);

float RunTime,PhaseTime;
const float WaitTime = 100;
int DataPrintTime;
int LastPhase,CurPhase,NxtPhase;
bool StopPhaseFlag;
int EscPwm;
const int AccelRate = 20;
int BrakeStat,BrakePwm;
long Cnt,OldCnt;
long EncCnt;
int Sum;
const int EncP = 50;
int LedFlag;
float UnitDist,Velo;
float Dist;
const float TargetDist = 2.0,WheelDia = 0.060,Pi = 3.14;
int Sw1Stat,Sw2Stat;
float Sw1Time,Sw2Time;
int SerialStat,UsrSwStat,BtStat;
float UsrSwTime;
int SafetyFlag,StackFlag;
float SlipTime,StackTime,FallTime;
int ReptCnt = 0;
int bt_flag = 0;

#define INIT 0
#define STOP 1
#define UP 2
#define DOWN 3
#define PHASE 4
#define SUCCESS 0
#define SAFETY 1
#define SLIP_ERROR 2
#define STACK_ERROR 3
#define FALL_ERROR 4
#define ERROR 5
#define ON 1
#define OFF 0
#define CW_MAX_PWM 1900
#define STOP_PWM 1500
#define CCW_MAX_PWM 1200
#define _0DEG 1000
#define _180DEG 2000
#define BRAKE_ON_PWM 1400
#define BRAKE_OFF_PWM 1900

void _init();
void stop();
void up();
void down();
void led();
void esc();
int brake(int);
void usr_sw();
void tisr_sw();
void sw1_chk();
void sw2_chk();
void isr_enc();
void safety();
void usb_serial_print();
void log_serial_print();

void setup(){

  BtSerial.begin(4800);
    Serial.begin(115200);
    logger.begin(115200);
    while(!Serial);

    MsTimer2::set(10,tisr_sw);
    MsTimer2::start();

    _init();
}

void loop(){

    /*if(BtSerial.available() > 0){
      Serial.write(BtSerial.read());
      bt_flag = 1;
    }else{
      bt_flag = 0;
    }*/
    bt_flag = BtSerial.available(); /*omazinai*/


  if(bt_flag == 1){
    if(millis() == RunTime + WaitTime){

        RunTime += WaitTime;
        PhaseTime++;
        DataPrintTime++;
        SerialStat = PHASE;

        UnitDist = (float)EncCnt / ((float)EncP * 4) * Pi *  WheelDia;
        Dist += UnitDist;
        Velo = UnitDist / 0.1;
        if(Velo < 0)Velo = -1 * Velo;

       if(BrakePwm == BRAKE_ON_PWM){
            digitalWrite(Led1Pin,OFF);
            digitalWrite(Led2Pin,ON);
        }else if(BrakePwm == BRAKE_OFF_PWM){
            digitalWrite(Led1Pin,ON);
            digitalWrite(Led2Pin,OFF);
        }

        switch(CurPhase){
            case INIT:
                _init();
                break;

            case STOP:
                stop();
                break;

            case UP:
                up();
                break;
            case DOWN:
                down();
                break;
        }

        esc();
        safety();


    if(StopPhaseFlag == false){
        LastPhase = CurPhase;
        CurPhase = NxtPhase;
        }

    } else if(DataPrintTime == 1){
        DataPrintTime = 0;
        usb_serial_print();
        log_serial_print();
        UnitDist = 0;
        EncCnt = 0;
    }
  }
}


/**
 * @fn
 *
 * @brief
 * @param
 * @param
 * @return
 * @sa
 * @detail
 */
void _init(){

    SerialStat = INIT;
    pinMode(Sw1Pin,INPUT_PULLUP);
    pinMode(Sw2Pin,INPUT_PULLUP);

    pinMode(UsrSwPin,INPUT_PULLUP);

    pinMode(Led1Pin,OUTPUT);
    pinMode(Led2Pin,OUTPUT);
    //LedFlag = ON;

    Esc.attach(EscPin);
    EscPwm = STOP_PWM;
    Esc.writeMicroseconds(EscPwm);

    BrakeSv.attach(BrakeSvPin,_0DEG,_180DEG);
    BrakeStat = brake(ON);

    pinMode(EncAPin,INPUT);
    pinMode(EncBPin,INPUT);
    attachInterrupt(0,isr_enc,CHANGE);
    attachInterrupt(1,isr_enc,CHANGE);
    EncCnt = 0;
    Cnt = 0;
    OldCnt = 0;
    Sum = 0;

    UnitDist = 0;
    Dist = 0;
    Velo = 0;

    Sw1Time = 0;
    Sw2Time = 0;

    UsrSwTime = 0;

    RunTime = millis();
    PhaseTime = 0;

    SafetyFlag = OFF;
    StackFlag = OFF;

    SlipTime = 0;
    StackTime = 0;
    FallTime = 0;

    NxtPhase = STOP;

}


/**
 * @fn
 *
 * @brief
 * @param
 * @param
 * @return
 * @sa
 * @detail
 */
void stop(){

    esc();
    brake(ON);
    StopPhaseFlag = true;
    SlipTime = 0;
    StackTime = 0;
    FallTime = 0;

    switch(LastPhase){
        case INIT:

            usr_sw();
            if(UsrSwStat == ON){
                NxtPhase = UP;
                PhaseTime = 0;
                StopPhaseFlag = false;
            }
            break;

        case UP:

            if(PhaseTime >= 50){
                NxtPhase = DOWN;
                PhaseTime = 0;
                StopPhaseFlag = false;
                }
            break;

        case DOWN:

            if(PhaseTime == 5){

                if(ReptCnt > 0){
                    NxtPhase = UP;
                    ReptCnt--;
                    PhaseTime = 0;
                    StopPhaseFlag = true;
                } else {
                    NxtPhase = STOP;
                    PhaseTime = 0;
                    StopPhaseFlag = false;
                }
            }
            break;

        case STOP:

            usr_sw();
            if(UsrSwStat == ON || UsrSwTime == 1.0){
                NxtPhase = INIT;
                PhaseTime = 0;
                StopPhaseFlag = false;
            }
            break;

    }
}


/**
 * @fn
 *
 * @brief
 * @param
 * @param
 * @return
 * @sa
 * @detail
 */
void up(){

    brake(OFF);

        esc();

    if(Dist >= TargetDist || Sw2Stat == ON || BtStat == ON){

        NxtPhase = STOP;
        PhaseTime = 0;
    }
}


/**
 * @fn
 *
 * @brief
 * @param
 * @param
 * @return
 * @sa
 * @detail
 */
void down(){


    if(Velo > 1.0){
        brake(ON);
    }else{
        brake(OFF);
    }
    esc();

    if(Dist < 1.0 || Sw1Stat == ON || BtStat == ON){
        NxtPhase = STOP;
        PhaseTime = 0;
    }
    if(StackFlag == ON && PhaseTime >= 50 && Velo < 0.1){
        NxtPhase = STOP;
        CurPhase = STOP;
        PhaseTime = 0;
    }

}


/**
 * @fn
 *
 * @brief
 * @param
 * @param
 * @return
 * @sa
 * @detail
 */
void usb_serial_print(){
    switch(SerialStat){
        case INIT:
            Serial.print("\nRunTime");
            Serial.print("\t");
            Serial.print("BrakePwm");
            Serial.print("\t");
            Serial.print("escPwm");
            Serial.print("\t");
            Serial.print("EncCnt");
            Serial.print("\t");
            Serial.print("UnitDist");
            Serial.print("\t");
            Serial.print("Dist");
            Serial.print("\t");
            Serial.print("Velo");
            Serial.print("\t");
            Serial.print("CurPhase");
            Serial.print("\t");
            Serial.print("PhaseTime");
            Serial.print("\t  ");
            Serial.print("SafetyTime");
            Serial.print("\t\t");
            Serial.print("Sw1Stat");
            Serial.print("\t");
            Serial.print("Sw2Stat");
            Serial.print("\t");
            Serial.println("BtStat");
            break;

        case PHASE:
            Serial.print(RunTime/1000);
            Serial.print("\t");
            Serial.print(BrakePwm);
            Serial.print("\t\t");
            Serial.print(EscPwm);
            Serial.print("\t");
            Serial.print(EncCnt);
            Serial.print("\t");
            Serial.print(UnitDist);
            Serial.print("\t\t");
            Serial.print(Dist);
            Serial.print("\t");
            Serial.print(Velo);
            Serial.print("\t");
            Serial.print(CurPhase);
            Serial.print("\t\t");
            Serial.print(PhaseTime/10);
            Serial.print("\t\t");
            Serial.print("  ");
            Serial.print(SlipTime);
            Serial.print("  ");
            Serial.print(StackTime);
            Serial.print("  ");
            Serial.print(FallTime);
            Serial.print("\t");
            Serial.print(Sw1Stat);
            Serial.print("\t");
            Serial.print(Sw2Stat);
            Serial.print("\t");
            Serial.println(BtStat);
            break;
    }
}


/**
 * @fn
 *
 * @brief
 * @param
 * @param
 * @return
 * @sa
 * @detail
 */
void log_serial_print(){
    switch(SerialStat){
        case INIT:
            logger.print("\nRunTime");
            logger.print("\t");
            logger.print("BrakePwm");
            logger.print("\t");
            logger.print("escPwm");
            logger.print("\t");
            logger.print("EncCnt");
            logger.print("\t");
            logger.print("UnitDist");
            logger.print("\t");
            logger.print("Dist");
            logger.print("\t");
            logger.print("Velo");
            logger.print("\t");
            logger.print("CurPhase");
            logger.print("\t");
            logger.print("PhaseTime");
            logger.print("\t  ");
            logger.print("SafetyTime");
            logger.print("\t\t");
            logger.print("Sw1Stat");
            logger.print("\t");
            logger.print("Sw2Stat");
            logger.print("\t");
            logger.println("BtStat");
            break;

        case PHASE:
            logger.print(RunTime/1000);
            logger.print("\t");
            logger.print(BrakePwm);
            logger.print("\t\t");
            logger.print(EscPwm);
            logger.print("\t");
            logger.print(EncCnt);
            logger.print("\t");
            logger.print(UnitDist);
            logger.print("\t\t");
            logger.print(Dist);
            logger.print("\t");
            logger.print(Velo);
            logger.print("\t");
            logger.print(CurPhase);
            logger.print("\t\t");
            logger.print(PhaseTime/10);
            logger.print("\t\t");
            logger.print("  ");
            logger.print(SlipTime);
            logger.print("  ");
            logger.print(StackTime);
            logger.print("  ");
            logger.print(FallTime);
            logger.print("\t");
            logger.print(Sw1Stat);
            logger.print("\t");
            logger.print(Sw2Stat);
            logger.print("\t");
            logger.println(BtStat);
            break;
    }
}


/**
 * @fn
 *
 * @brief
 * @param
 * @param
 * @return
 * @sa
 * @detail
 */
void usr_sw(){

    if(!digitalRead(UsrSwPin)){
        UsrSwTime += 0.1;
        UsrSwStat = ON;
    } else {
        UsrSwStat = OFF;
        UsrSwTime = 0;
    }
}


/**
 * @fn
 *
 * @brief
 * @param
 * @param
 * @return
 * @sa
 * @detail
 */
void esc(){

    switch(CurPhase){

        case UP:

            if(EscPwm == CW_MAX_PWM){
                EscPwm = CW_MAX_PWM;
            } else {
                EscPwm += AccelRate;
            }
            break;

        case DOWN:

            if(SafetyFlag == ON){
                EscPwm = STOP_PWM;
            }else if(BrakePwm == BRAKE_OFF_PWM){
                if(EscPwm == CCW_MAX_PWM){
                    EscPwm = CCW_MAX_PWM;
                } else {
                    EscPwm -= AccelRate;
                }
            } else {
                EscPwm = STOP_PWM;
            }
            break;
        case STOP:

            EscPwm = STOP_PWM;
            break;
    }

    Esc.writeMicroseconds(EscPwm);
}


/**
 * @fn
 *
 * @brief
 * @param
 * @param
 * @return
 * @sa
 * @detail
 */
int brake(int BrakeStat){

    switch(BrakeStat){
        case ON:
            BrakePwm = BRAKE_ON_PWM;
            BrakeSv.writeMicroseconds(BrakePwm);
            break;
        case OFF:
            BrakePwm = BRAKE_OFF_PWM;
            BrakeSv.writeMicroseconds(BrakePwm);
            break;
        default:
            return ERROR;
    }
    return SUCCESS;
}


/**
 * @fn
 *
 * @brief
 * @param
 * @param
 * @return
 * @sa
 * @detail
 */
void isr_enc(){

    int Var1 = digitalRead(EncAPin);
    int Var2 = digitalRead(EncBPin);

    Cnt = (Var1 << 1) | Var2;
    Sum = (OldCnt << 2) | Cnt;

    if(Sum == 0b1101 || Sum == 0b0100 || Sum == 0b0010 || Sum == 0b1011)EncCnt++;
    if(Sum == 0b0111 || Sum == 0b0001 || Sum == 0b1000 || Sum == 0b1110)EncCnt--;

    OldCnt=Cnt;
}

/**
 * @fn
 *
 * @brief
 * @param
 * @param
 * @return
 * @sa
 * @detail
 */
void tisr_sw(){
    sw1_chk();
    sw2_chk();
}

/**
 * @fn
 *
 * @brief
 * @param
 * @param
 * @return
 * @sa
 * @detail
 */
void sw1_chk(){

    if(!digitalRead(Sw1Pin)){
        Sw1Stat = ON;
        Sw1Time += 0.01;
    } else {
        Sw1Stat = OFF;
        Sw1Time = 0;
    }
}


/**
 * @fn
 *
 * @brief
 * @param
 * @param
 * @return
 * @sa
 * @detail
 */
void sw2_chk(){

    if(!digitalRead(Sw2Pin)){
        Sw2Stat = ON;
        Sw2Time += 0.01;
    } else {
        Sw2Stat = OFF;
        Sw2Time = 0;
    }
}


/**
 * @fn
 *
 * @brief
 * @param
 * @param
 * @return
 * @sa
 * @detail
 */
void safety(){

    //SLIP_CHECK
    if(EscPwm > 1650 && Velo < 0.2){
        SlipTime++;

        if(SlipTime >= 30){
            NxtPhase = STOP;
            CurPhase = UP;
            SafetyFlag = ON;
            PhaseTime = 0;
        }
    }

    //STACK_CHECK
    if(EscPwm > 1650 && Velo < 0.2){
        StackTime++;

        if(StackTime >= 30){
            NxtPhase = STOP;
            CurPhase = UP;
            SafetyFlag = ON;
            StackFlag = ON;
            PhaseTime = 0;
        }
    }

    //DOWN時にスタック判定される
    if(EscPwm < 1450 && UnitDist >= 0 && Velo > 0.2){
        StackTime++;

        if(StackTime >= 50){
            NxtPhase = STOP;
            CurPhase = UP;
            SafetyFlag = ON;
            StackFlag = ON;
            PhaseTime = 0;
        }
    }

    //FALL_CHECK
    if(EscPwm > 1500 && UnitDist < 0){
        FallTime++;

        if(FallTime >= 30){
            NxtPhase = STOP;
            CurPhase = UP;
            SafetyFlag = ON;
            PhaseTime = 0;
        }
    }

}
