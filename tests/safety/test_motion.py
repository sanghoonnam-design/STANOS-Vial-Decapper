import re, subprocess, pathlib, sys, os, shutil, tempfile
base=pathlib.Path(__file__).resolve().parent
root=pathlib.Path(sys.argv[1]).resolve() if len(sys.argv)>1 else base.parents[1]
def read(p):
    q=root/p
    return q.read_text(encoding='utf-8-sig', errors='replace')
def stripped(s): return re.sub(r'^\s*#include[^\n]*','',s,flags=re.M)+'\n'
def function(s,name):
    matches=list(re.finditer(r'(?m)^(?:static )?(?:bool|void|int|U08|S32)\s+'+name+r'\([^;]*?\)\s*(?://[^\n]*\n\s*)?\{' ,s))
    if not matches: return ''
    m=matches[-1]; start=m.start(); i=m.end(); depth=1
    # Bodies here contain no brace characters in string literals.
    while depth:
        if s[i]=='{': depth+=1
        if s[i]=='}': depth-=1
        i+=1
    return s[start:i]+'\n'
c=read('App/HW_Device/CDecap.c'); module=read('App/Task/_10_XCommand_Module.c')
prefix=r'''
#include <stdint.h>
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
typedef uint8_t U8,U08; typedef uint16_t U16; typedef uint32_t U32; typedef int32_t S32; typedef float F32;
#define YES 1
#define NO 0
#define TRUE 1
#define FALSE 0
#define ON 1
#define OFF 0
#define ENABLE 1
#define GPIO_PIN_SET 1
#define READ_IN 0
#define STEP_PULSE_RATE_R 122
#define A_MAX_BASE 3725
#define MAX_ACC 2047
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define pdTRUE 1
typedef void *SemaphoreHandle_t;
static int motionMutexToken;
static SemaphoreHandle_t xMutex_DecapperMotion = &motionMutexToken;
static bool motionMutexHeld;
static int xSemaphoreTake(SemaphoreHandle_t mutex,U32 ticks){
 if (mutex!=xMutex_DecapperMotion || motionMutexHeld) return 0;
 motionMutexHeld=true;
 return pdTRUE;
}
static void xSemaphoreGive(SemaphoreHandle_t mutex){
 if (mutex==xMutex_DecapperMotion) motionMutexHeld=false;
}
#define xprintf(...) ((void)0)
#define LOG_MSG_SEND(...) ((void)0)
#define ERR_MSG_SEND(...) ((void)0)
'''
tm=read('Drivers/BSP/STEP/TMC2660.h')
prefix+=stripped(tm[:tm.index('/*\n#pragma pack(push,1)')])+'\n#endif\n'
# mock TMC429 좌표를 앱 좌표로 쓰는 기존 기대값 유지: Z 부호 변환(Z_COORD_SIGN=-1)은 끈다(2026-11-20).
prefix+='#define Z_COORD_SIGN (1)\n'
prefix+=stripped(read('App/HW_Device/CDecap.h'))
prefix+=stripped(read('App/System/XSystem_DB.h'))
prefix+=stripped(read('App/Common/XErrorCode.h'))
prefix+=stripped(read('App/Common/XParser.h'))
drive=read('Drivers/BSP/STEP/Drive.h'); prefix+=drive[drive.index('typedef enum'):drive.index('DRIVE_EXT void Drive_Init')]
prefix+=r'''
bool gZCapUpPosSavePending,gRDecapPosSavePending; S32 gZCapUpPosPendingValue,gRDecapPosPendingValue;
tsXStateList xSL; tsXControlData xCD; tsXParameterList xPL; teXActionType xAT;
U32 gTriggerCount; static S32 position[2]; static int moveCount, stopCount; static sMotionCommand_t lastMove;
static int errorCode; static char response[128]; void *xSendMsg;
#define NO_COMMA 0
#define COMMA 1
void SetErrorCode(teErrorCode e,const char*f,int l){errorCode=e;}
int GetErrorCode_int(void){return errorCode;}
const char *GetErrorCode_char(void){return "E0000";}
const char *GetErrorMessage(void){return errorCode?"Generic error.":"No Error.";}
void ClearError(void){errorCode=0;}
void XBuffer_AddString(void*b,const char*s,int comma){snprintf(response,sizeof response,"%s",s);}
void xParser_HandleError(const char*f,teXParsingErrorCode c,U08 n){}
void Drive_SelMaxCurrent(U8 a,U8 b){} void Drive_SetCurrent(U8 a,F32 b,U8 d){}
void Drive_SetResoultion(U8 a,U8 b){} void Drive_SetHwLimit(U8 a,sMotionLimit_t b){}
void Drive_SetSwLimitPos(U8 a,sMotionSwLimitPos_t b){} void Drive_PowerEnable(U8 a,U8 b){}
void TMC429_SetPosition(U8 a,S32 p){position[a]=p;}
static bool monitorPositionLock;
static unsigned unlockedPositionReads;
S32 TMC429_GetPosition(U8 a){
 if (monitorPositionLock && !motionMutexHeld) unlockedPositionReads++;
 return position[a];
}
S32 TMC429_GetVelocity(U8 a){return 0;}
static bool useMotorFeedback;
static U8 motorFeedback[2];
static unsigned motorFeedbackReads;
U8 TMC2660_GetMotorRun(U8 axis){
 if (useMotorFeedback){motorFeedbackReads++;return motorFeedback[axis];}
 return axis==aR ? xSL.xR_Motor_Run.Motor_Run : xSL.xZ_Motor_Run.Motor_Run;
}
void IOEXP_WriteIObit(U8 a,U8 b){} U8 IOEXP_ReadIObit(U8 a,U8 b){return 1;}
U8 (*digitalRead)(U8,U8)=IOEXP_ReadIObit;
void Drive_RelMove(sMotionCommand_t*p){lastMove=*p;moveCount++;}
void Drive_AbsMove(sMotionCommand_t*p){lastMove=*p;moveCount++;}
void Drive_Stop(sMotionCommand_t*p){stopCount++;}
/* 구동/유지 전류 전환과 HOME 속도 이동의 가속도·전류 상태를 기록한다 */
static int runCurrentSets[2], holdCurrentSets[2], motorCurrentRun[2], homeMovesAtHold; static U32 lastHomeAcc;
void TMC2660_SetHoldCurrent(U8 a){if(a<2){holdCurrentSets[a]++;motorCurrentRun[a]=0;}}
void TMC2660_SetRunCurrent(U8 a){if(a<2){runCurrentSets[a]++;motorCurrentRun[a]=1;}}
static int homeMoves, homeStops;
static void homeVelNote(U8 a,U32 b){lastHomeAcc=b;if(a<2 && !motorCurrentRun[a])homeMovesAtHold++;}
U8 TMC429_VelMove(U8 a,U32 b,S32 d){homeMoves++;homeVelNote(a,b);return 0;}
void TMC429_MotorStop(U8 a,U32 b){homeStops++;lastHomeAcc=b;}
'''
code=prefix+stripped(c)
for name in ['CMD_RejectIfDecapperBusy','CMD_ReplyInvalidArgument','CMD_QueueAction','CMD_Handle_STOP','CMD_Handle_PAUSE','CMD_Handle_RESUME','CMD_Handle_HOME']:
    code+=function(module,name)
core=read('App/Task/_10_XCommand_Core.c')
code+=function(core,'CMD_Handle_GERR')+function(core,'CMD_Handle_GERD')
code+=r'''
static int failures;
#define CHECK(test,condition) do{if(!(condition)){printf("FAIL %s\n",test); failures++;}else printf("PASS %s\n",test);}while(0)
static void reset(void){
 memset(&xSL,0,sizeof xSL);memset(&xPL,0,sizeof xPL);memset(&xCD,0,sizeof xCD);
 xAT=ACTION_NONE;errorCode=0;gTriggerCount=0;
 useMotorFeedback=false;motorFeedbackReads=0;
 homeMoves=homeStops=0;
 memset(runCurrentSets,0,sizeof runCurrentSets);memset(holdCurrentSets,0,sizeof holdCurrentSets);
 memset(motorCurrentRun,0,sizeof motorCurrentRun);homeMovesAtHold=0;lastHomeAcc=0;
 xSL.CDecapping_Sensor.Z_H_Limit_Sensor=1;
 xSL.CDecapping_Sensor.Z_L_Limit_Sensor=1;
 xPL.Decapper.Limit_PosZ=160000;xPL.Decapper.Limit_PosR=100000;
 xPL.Decapper.SoftLimitEnable=1;
 xPL.Decapper.SwNegLimit[aZ]=-160000;xPL.Decapper.SwPosLimit[aZ]=160000;
 xPL.Decapper.SwNegLimit[aR]=-100000;xPL.Decapper.SwPosLimit[aR]=100000;
 CDecap_Init();CDecap_StartTimeout();moveCount=stopCount=0;
}
int main(void){tsXParsedData cmd={0};
 reset();position[aZ]=150000; CDecap_ManualMotorMove(SUB_zRMOVE,aZ,true,20000);
 CHECK("relative endpoint outside limit never sent",moveCount==0);
 reset();xPL.Decapper.Limit_PosZ=10000;xCD.Decapper.Motor_TargetPos[aZ]=11000;CDecap_Absmove_Z();
 CHECK("Z absolute uses configured limit",moveCount==0);
 reset();CDecap_ManualMotorMove(SUB_zRMOVE,aZ,true,500);
 CHECK("valid relative move unchanged",moveCount==1 && lastMove.Pos==500);
 reset();position[aZ]=159500;CDecap_ManualMotorMove(SUB_zRMOVE,aZ,true,500);
 CHECK("exact positive boundary accepted",moveCount==1 && lastMove.Pos==500);
 reset();position[aZ]=-159500;CDecap_ManualMotorMove(SUB_zRMOVE,aZ,true,-500);
 CHECK("exact negative boundary accepted",moveCount==1 && lastMove.Pos==-500);
 reset();position[aZ]=1000;CDecap_ManualMotorMove(SUB_zRMOVE,aZ,true,INT32_MAX);
 CHECK("relative addition cannot wrap into valid range",moveCount==0);
 reset();xPL.Decapper.SwPosLimit[aZ]=5000;CDecap_ManualMotorMove(SUB_zAMOVE,aZ,false,6000);
 CHECK("narrower software limit enforced",moveCount==0);
 reset();CHECK("invalid automatic target fails without motion",!CD_ABSMove(0,300000,20000,170000) && moveCount==0);
 reset();xPL.Decapper.SwPosLimit[aZ]=1000;position[aZ]=1001;xSL.xZ_Motor_Run.Motor_Run=1;sPhase=DFSM_AMOVEZ;
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("out of range moving axis stops",stopCount>=2 && xSL.isError);
 reset();xSL.isBusy=1;sPhase=DFSM_CAP;CMD_Handle_STOP(&cmd,0);CMD_Handle_RESUME(&cmd,0);CDecap_Action_Statemachine(xAT);
 CHECK("STOP survives later RESUME",stopCount>=2 && !xSL.isBusy);
 reset();gTriggerCount=501;CDecap_CheckTimeout(5000,"Test timeout");
 CHECK("timeout has a nonzero error code",errorCode!=0 && xSL.errorCode!=0);
 CDecap_Action_Statemachine(ACTION_NONE);CMD_Handle_HOME(&cmd,0);
 CHECK("fault prevents new motion request",xAT==ACTION_NONE);
 CDecap_Error_Clear();CMD_Handle_HOME(&cmd,0);
 CHECK("explicit idle clear permits new motion",!xSL.isError && xAT==ACTION_HOME && errorCode==0);
 reset();gTriggerCount=501;CDecap_CheckTimeout(5000,"Test timeout");CMD_Handle_STOP(&cmd,0);CDecap_Action_Statemachine(xAT);
 CHECK("STOP remains available during fault",xAT==ACTION_NONE && xSL.isError);
 reset();xSL.Decapper.Z_HL_isError=1;CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("latched diagnostics alone do not trigger a motion fault",!xSL.isError);
 reset();position[aZ]=200;position[aR]=0;useMotorFeedback=true;
 motorFeedback[aZ]=1;motorFeedback[aR]=0;
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("control tick reads Z and R feedback from the driver",
       motorFeedbackReads==2 && xSL.xZ_Motor_Run.Motor_Run && !xSL.xR_Motor_Run.Motor_Run);
 CHECK("running Z blocks idle requests",!CDecap_IsIdle());
 motorFeedback[aZ]=0;motorFeedback[aR]=1;
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("motor feedback follows each axis independently",
       motorFeedbackReads==4 && !xSL.xZ_Motor_Run.Motor_Run && xSL.xR_Motor_Run.Motor_Run);
 xSL.isError=1;motorFeedback[aR]=0;
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("motor feedback refreshes after a fault",motorFeedbackReads==6 && !xSL.xR_Motor_Run.Motor_Run);
 CDecap_Error_Clear();
 CHECK("confirmed standstill permits clearing the fault",!xSL.isError);
 CHECK("maintenance starts at standstill",CDecap_BeginMaintenance());
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("maintenance skips motor feedback reads",motorFeedbackReads==6);
 CDecap_EndMaintenance();
 CDecap_RequestStop();
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("pending STOP skips feedback and issues both stops",motorFeedbackReads==6 && stopCount>=2);
 reset();position[aZ]=position[aR]=0;
 sPhase=DFSM_LONGRUN;xSL.isBusy=YES;xAT=ACTION_LONGRUN;
 CMD_Handle_PAUSE(&cmd,0);
 CHECK("PAUSE is accepted during longrun",xAT==ACTION_PAUSE && !strcmp(response,"pause"));
 CMD_Handle_RESUME(&cmd,0);
 CHECK("early RESUME cannot replace a pending PAUSE",xAT==ACTION_PAUSE && strcmp(response,"resume"));
 CDecap_Action_Statemachine(xAT);
 CHECK("pending PAUSE is applied at the next tick",CDecap_IsPaused() && sPhase==DFSM_PAUSE && stopCount==2);
 CMD_Handle_RESUME(&cmd,0);
 CHECK("RESUME is accepted after the pause is applied",xAT==ACTION_RESUME && !strcmp(response,"resume"));
 CDecap_Action_Statemachine(xAT);
 CHECK("valid RESUME returns to the longrun phase",!CDecap_IsPaused() && sPhase==DFSM_LONGRUN);
 reset();sPhase=DFSM_AMOVEZ;xSL.isBusy=YES;xAT=ACTION_AMOVEZ;
 CMD_Handle_PAUSE(&cmd,0);
 CHECK("PAUSE on a manual move is rejected at acceptance",xAT==ACTION_AMOVEZ && strcmp(response,"pause"));
 reset();sPhase=DFSM_LONGRUN;xSL.isBusy=YES;xAT=ACTION_LONGRUN;
 CMD_Handle_PAUSE(&cmd,0);CDecap_Action_Statemachine(xAT);
 CMD_Handle_STOP(&cmd,0);CMD_Handle_RESUME(&cmd,0);
 CHECK("RESUME does not replace a pending STOP even in a paused phase",xAT==ACTION_STOP && strcmp(response,"resume"));
 CDecap_Action_Statemachine(xAT);
 CHECK("STOP clears the paused context and returns to idle",CDecap_IsIdle() && !CDecap_IsPaused());
 reset();sPhase=DFSM_LONGRUN;xCD.Decapper.phaseDecapCap=LONGRUN_CAP;
 CDecapping_Step=CDecapping_Move_PrevZ;xCD.Decapper.LongRunCount=123;
 gTriggerCount=MS_TO_TICKS(CAP_UNGRIP_WAIT_MS)+MS_TO_TICKS(GRIP_SENSOR_TIMEOUT_MS);
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("Cap release timeout stops motion and records the Cap sensor code",
       xSL.errorCode==2870 && stopCount==2 && CDecap_GetOperationResult()==4);
 SetErrorCode(ERROR_CODE_INVALID_ARGUMENT,__func__,__LINE__);
 CMD_Handle_GERR(&cmd,0);
 CHECK("GERR restores the first fault after an unrelated command error",errorCode==2870);
 CMD_Handle_GERD(&cmd,0);
 CHECK("GERD retains the LR half, step, completed count and Cap release condition",
       strstr(response,"LR/CAP") && strstr(response,"step=9") && strstr(response,"completed=123") &&
       strstr(response,"Cap release") && strstr(response,"Open=0 Close=0"));
 CMD_Handle_STOP(&cmd,0);CDecap_Action_Statemachine(xAT);CMD_Handle_GERD(&cmd,0);
 CHECK("STOP preserves the first fault detail and its operation result",
       strstr(response,"LR/CAP") && strstr(response,"completed=123") && CDecap_GetOperationResult()==4);
 CDecap_Error_Clear();CMD_Handle_GERD(&cmd,0);
 CHECK("CLR removes the fault detail",!strcmp(response,"No Error."));
 reset();sPhase=DFSM_Y_B_MOVE;gTriggerCount=1;
 xSL.CDecapping_Sensor.Y_B_Limit_Sensor=1;
 CDecap_Action_Statemachine(ACTION_NONE);
 gTriggerCount+=MS_TO_TICKS(READY_TIMEOUT_MS);
 CDecap_Action_Statemachine(ACTION_NONE);CMD_Handle_GERD(&cmd,0);
 CHECK("manual Y reports the missing limit and observed bits",
       xSL.errorCode==2861 && strstr(response,"Y Back limit not reached") && strstr(response,"F=1 B=0"));
 reset();sPhase=DFSM_HOME;HomingSeq.Step=CDECAP_HOMING_L_REACH_OFF;SET_STARTED(SUB_HOMEZ);
 xSL.CDecapping_Sensor.Z_L_Limit_Sensor=0;
 gTriggerCount=MS_TO_TICKS(HOMING_TIMEOUT_MS);
 CDecap_Action_Statemachine(ACTION_NONE);CMD_Handle_GERD(&cmd,0);
 CHECK("HOME reports Z sensor release failure and observed bits",
       xSL.errorCode==2851 && strstr(response,"Z home sensor did not release") && strstr(response,"H=0 L=1"));
 reset();sPhase=DFSM_HOME;HomingSeq.Step=CDECAP_HOMING_H_WAIT;SET_STARTED(SUB_HOMEZ);
 xSL.xZ_Motor_Run.Motor_Run=1;gTriggerCount=MS_TO_TICKS(HOMING_TIMEOUT_MS);
 CDecap_Action_Statemachine(ACTION_NONE);CMD_Handle_GERD(&cmd,0);
 CHECK("HOME motor stop timeout stays a motor error",
       xSL.errorCode==2800 && strstr(response,"Z stop not confirmed"));
 reset();sPhase=DFSM_AROTATE;xCD.Decapper.Motor_TargetPos[aR]=8388608;
 CDecap_Action_Statemachine(ACTION_NONE);CMD_Handle_GERD(&cmd,0);
 CHECK("invalid R target names the axis and rejected target",
       xSL.errorCode==ERROR_CODE_DECAP_LIMIT && strstr(response,"R target 8388608") &&
       strstr(response,"[-8388607,8388607]"));
 reset();sPhase=DFSM_AMOVEZ;position[aZ]=160001;xSL.xZ_Motor_Run.Motor_Run=1;
 CDecap_Action_Statemachine(ACTION_NONE);CMD_Handle_GERD(&cmd,0);
 CHECK("limit error distinguishes observed Z position from requested target",
       xSL.errorCode==ERROR_CODE_DECAP_LIMIT && strstr(response,"Z position 160001"));
 reset();sPhase=DFSM_AMOVEZ;xPL.Decapper.Limit_PosZ=0;
 CDecap_Action_Statemachine(ACTION_NONE);CMD_Handle_GERD(&cmd,0);
 CHECK("invalid configured limits are identified separately",
       xSL.errorCode==ERROR_CODE_DECAP_LIMIT && strstr(response,"Z limits invalid"));
 reset();position[aZ]=position[aR]=0;
 xSL.CDecapping_Sensor.CT_Detect_Sensor=1;
 CDecap_Action_Statemachine(ACTION_CAP);CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("automatic CAP before HOME retains its no-motion behavior",
       !xSL.isError && moveCount==0 && CDecap_GetOperationResult()==2);
 reset();xSL.isHomed=true;
 CDecap_Action_Statemachine(ACTION_DECAP);CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("automatic DECAP without a vial retains its no-motion behavior",
       !xSL.isError && moveCount==0 && CDecap_GetOperationResult()==2);
 reset();sPhase=DFSM_LONGRUN;xCD.Decapper.phaseDecapCap=LONGRUN_DECAP;
 CDecapping_Step=CDecapping_Move_PrevR;gTriggerCount=MS_TO_TICKS(DECAPPING_TIMEOUT_MS);
 sMovePending[aZ]=sMovePending[aR]=true;sMoveTarget[aZ]=0;sMoveTarget[aR]=-3200;
 CDecap_Action_Statemachine(ACTION_NONE);CMD_Handle_GERD(&cmd,0);
 CHECK("paired timeout identifies which axis still has an incomplete move",
       xSL.errorCode==ERROR_CODE_DECAP_TIMEOUT && strstr(response,"R stop or target not confirmed") &&
       !strstr(response,"Z/R stop"));
 /* Z 리미트는 원점 확정(isHomed) 뒤에만 검사한다. 확정 뒤에는 방향과 무관하게 한쪽만 감지돼도 정지한다. */
 reset();xSL.isHomed=1;position[aZ]=-100;xCD.Decapper.Motor_TargetPos[aZ]=0;
 xSL.CDecapping_Sensor.Z_H_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_AMOVEZ);CMD_Handle_GERD(&cmd,0);
 CHECK("homed: active High blocks positive absolute MOVE before launch",
       moveCount==0 && stopCount==2 && errorCode==2890 && strstr(response,"MOVE A Z") && strstr(response,"H=1"));
 reset();xSL.isHomed=1;position[aZ]=-100;xCD.Decapper.Motor_TargetPos[aZ]=-10;
 xSL.CDecapping_Sensor.Z_L_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_RMOVEZ);
 CHECK("homed: active Low blocks negative relative MOVE before launch",
       moveCount==0 && stopCount==2 && errorCode==2891);
 reset();xSL.isHomed=1;position[aZ]=0;xCD.Decapper.Motor_TargetPos[aZ]=-100;
 xSL.CDecapping_Sensor.Z_H_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_AMOVEZ);
 CHECK("homed: active High also blocks negative Z travel",moveCount==0 && stopCount==2 && errorCode==2890);
 reset();xSL.isHomed=1;position[aZ]=-100;xCD.Decapper.Motor_TargetPos[aZ]=100;
 xSL.CDecapping_Sensor.Z_L_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_RMOVEZ);
 CHECK("homed: active Low also blocks positive Z travel",moveCount==0 && stopCount==2 && errorCode==2891);
 reset();position[aZ]=-100;xCD.Decapper.Motor_TargetPos[aZ]=0;
 xSL.CDecapping_Sensor.Z_H_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_AMOVEZ);
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("before HOME an active High does not block a Z MOVE",moveCount==1 && !xSL.isError);
 reset();position[aZ]=0;xCD.Decapper.Motor_TargetPos[aZ]=-100;
 xSL.CDecapping_Sensor.Z_H_Limit_Sensor=xSL.CDecapping_Sensor.Z_L_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_AMOVEZ);
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("before HOME even both active Z sensors do not block a Z MOVE",moveCount==1 && !xSL.isError);
 reset();position[aZ]=-100;xCD.Decapper.Motor_TargetPos[aZ]=0;
 CDecap_Action_Statemachine(ACTION_AMOVEZ);
 position[aZ]=-3;xSL.xZ_Motor_Run.Motor_Run=1;
 xSL.CDecapping_Sensor.Z_H_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("before HOME High during travel does not stop the move",
       moveCount==1 && stopCount==0 && !xSL.isError);
 reset();xSL.isHomed=1;position[aZ]=-100;xCD.Decapper.Motor_TargetPos[aZ]=0;
 CDecap_Action_Statemachine(ACTION_AMOVEZ);
 position[aZ]=-3;xSL.xZ_Motor_Run.Motor_Run=1;
 xSL.CDecapping_Sensor.Z_H_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("homed: High during travel stops both axes before Z reaches zero",
       moveCount==1 && stopCount==2 && errorCode==2890 && CDecap_GetOperationResult()==4);
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("latched Z limit fault does not reissue motion",moveCount==1 && stopCount==2);
 reset();xSL.isHomed=1;position[aZ]=0;xCD.Decapper.Motor_TargetPos[aZ]=-100;
 CDecap_Action_Statemachine(ACTION_RMOVEZ);
 xSL.CDecapping_Sensor.Z_L_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("homed: Low trips a pending negative MOVE even before Run is asserted",
       moveCount==1 && stopCount==2 && errorCode==2891);
 reset();xSL.isHomed=1;position[aZ]=-100;xCD.Decapper.Motor_TargetPos[aZ]=100;
 CDecap_Action_Statemachine(ACTION_RMOVEZ);
 position[aZ]=-50;xSL.xZ_Motor_Run.Motor_Run=1;
 xSL.CDecapping_Sensor.Z_L_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("homed: Low during positive travel also stops the move",
       moveCount==1 && stopCount==2 && errorCode==2891);
 reset();xSL.isHomed=1;position[aZ]=0;xCD.Decapper.Motor_TargetPos[aZ]=-100;
 xSL.CDecapping_Sensor.Z_H_Limit_Sensor=xSL.CDecapping_Sensor.Z_L_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_AMOVEZ);CMD_Handle_GERD(&cmd,0);
 CHECK("homed: both active Z sensors block Z travel in either direction",
       moveCount==0 && stopCount==2 && errorCode==2892 && strstr(response,"H=1 L=1"));
 reset();xSL.isHomed=1;position[aZ]=-100;position[aR]=0;
 sPhase=DFSM_LONGRUN;xCD.Decapper.phaseDecapCap=LONGRUN_DECAP;
 xPL.Decapper.ZCap_UpPos=0;xPL.Decapper.RDecapPos=3200;
 CDecapping_Step=CDecapping_sData_Decapping;gTriggerCount=MS_TO_TICKS(CAP_GRIP_WAIT_MS);
 xSL.CDecapping_Sensor.Z_H_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_NONE);CMD_Handle_GERD(&cmd,0);
 CHECK("paired LR checks the Z physical limit before commanding R",
       moveCount==0 && stopCount==2 && errorCode==2890 && strstr(response,"LR/DECAP"));
 reset();xSL.isHomed=1;position[aZ]=-100;
 PauseContext.isPaused=true;PauseContext.resumeZ=true;PauseContext.resumeR=true;
 PauseContext.zTargetPos=0;PauseContext.rTargetPos=3200;PauseContext.savedPhase=DFSM_CAP;
 sPhase=DFSM_PAUSE;
 xSL.CDecapping_Sensor.Z_H_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_RESUME);
 CHECK("RESUME checks the Z physical limit before restarting either axis",
       moveCount==0 && stopCount==2 && errorCode==2890);
 reset();sPhase=DFSM_HOME;HomingSeq.Step=CDECAP_HOMING_H_SEARCH;SET_STARTED(SUB_HOMEZ);
 HomingSeq.H_Acc=HomingSeq.L_Acc=CDecap_ToDriverAcc(MOTOR_ACC); /* HOME 시작 시 설정되는 값 */
 xSL.CDecapping_Sensor.Z_L_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("HOME at an already active HOME(L) sensor advances without driving into it",
       homeMoves==0 && homeStops==1 && HomingSeq.Step==CDECAP_HOMING_H_WAIT && !xSL.isError);
 CDecap_Action_Statemachine(ACTION_NONE);
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("HOME still backs off an active HOME(L) sensor",
       homeMoves==1 && HomingSeq.Step==CDECAP_HOMING_L_REACH_OFF && !xSL.isError);
 CHECK("HOME backoff runs at run current with an in-range TMC429 acceleration",
       homeMovesAtHold==0 && runCurrentSets[aZ]==1 && lastHomeAcc==80);
 CHECK("acceleration conversion matches Drive_AbsMove and clamps",
       CDecap_ToDriverAcc(300000)==80 && CDecap_ToDriverAcc(800000)==214 &&
       CDecap_ToDriverAcc(0)==1 && CDecap_ToDriverAcc(0xFFFFFFFFu)==MAX_ACC);
 xSL.CDecapping_Sensor.Z_L_Limit_Sensor=1;
 xSL.CDecapping_Sensor.Z_H_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("H(Z+) during HOME backoff is not a fault (limits are unchecked until homed)",!xSL.isError && errorCode==0);
 /* HOME: High 감지 위치에서 Z_HOME_OFFSET_PULSE 물러난 곳이 0점이다 */
 reset();sPhase=DFSM_HOME;HomingSeq.Step=CDECAP_HOMING_Z_DONE;SET_STARTED(SUB_HOMEZ);
 position[aZ]=-4321;xSL.CDecapping_Sensor.Z_L_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("HOME labels the HOME(L) detection point +offset without moving yet",
       position[aZ]==Z_HOME_OFFSET_PULSE && moveCount==0 && HomingSeq.Step==CDECAP_HOMING_OFFSET_MOVE &&
       !xSL.isHomed && !xSL.isError);
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("HOME then backs off to Z=0 at the slow HOME speed and run current",
       moveCount==1 && lastMove.Axis==aZ && lastMove.Pos==0 && lastMove.Vel==MOTOR_SPD_HOME_SLOW &&
       lastMove.Acc==MOTOR_ACC && HomingSeq.Step==CDECAP_HOMING_OFFSET_WAIT && holdCurrentSets[aZ]==0 && !xSL.isError);
 position[aZ]=400;xSL.xZ_Motor_Run.Motor_Run=1;
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("HOME waits while the offset move is running",
       HomingSeq.Step==CDECAP_HOMING_OFFSET_WAIT && holdCurrentSets[aZ]==0 && !xSL.isHomed && !xSL.isError);
 position[aZ]=0;xSL.xZ_Motor_Run.Motor_Run=0;xSL.CDecapping_Sensor.Z_L_Limit_Sensor=1;
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("HOME accepts Z=0 once stopped with the HOME(L) sensor released, then drops Z to hold current",
       HomingSeq.Step==CDECAP_HOMING_Y_HOME && holdCurrentSets[aZ]==1 && position[aZ]==0 && moveCount==1 && !xSL.isError);
 reset();sPhase=DFSM_HOME;HomingSeq.Step=CDECAP_HOMING_OFFSET_WAIT;SET_STARTED(SUB_HOMEZ);
 sMovePending[aZ]=true;sMoveTarget[aZ]=0;position[aZ]=0;xSL.CDecapping_Sensor.Z_L_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_NONE);CMD_Handle_GERD(&cmd,0);
 CHECK("HOME faults when the HOME(L) sensor is still active at the offset position",
       xSL.errorCode==2851 && strstr(response,"HOME step=17") && strstr(response,"still active after home offset") &&
       strstr(response,"H=0 L=1 expected L=0") && !xSL.isHomed && stopCount==2);
 reset();sPhase=DFSM_HOME;HomingSeq.Step=CDECAP_HOMING_OFFSET_WAIT;SET_STARTED(SUB_HOMEZ);
 sMovePending[aZ]=true;sMoveTarget[aZ]=0;position[aZ]=500;xSL.xZ_Motor_Run.Motor_Run=1;
 xSL.CDecapping_Sensor.Z_H_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("H(Z+) during the HOME offset move is not a fault (limits are unchecked until homed)",
       !xSL.isError && errorCode==0 && HomingSeq.Step==CDECAP_HOMING_OFFSET_WAIT && !xSL.isHomed);
 reset();sPhase=DFSM_HOME;HomingSeq.Step=CDECAP_HOMING_OFFSET_WAIT;SET_STARTED(SUB_HOMEZ);
 sMovePending[aZ]=true;sMoveTarget[aZ]=0;position[aZ]=500;xSL.xZ_Motor_Run.Motor_Run=1;
 gTriggerCount=MS_TO_TICKS(HOMING_TIMEOUT_MS);
 CDecap_Action_Statemachine(ACTION_NONE);CMD_Handle_GERD(&cmd,0);
 CHECK("HOME offset move that never finishes is a motor timeout",
       xSL.errorCode==2800 && strstr(response,"after home offset") && !xSL.isHomed);
 /* 0점이 센서 아래에 있으므로 HOME 뒤의 + 이동에서 High가 켜지면 위치와 무관하게 E2890이다(예전 0점 근처 예외 없음) */
 reset();xSL.isHomed=1;position[aZ]=-50000;xCD.Decapper.Motor_TargetPos[aZ]=0;
 CDecap_Action_Statemachine(ACTION_AMOVEZ);
 position[aZ]=-30;xSL.xZ_Motor_Run.Motor_Run=1;xSL.CDecapping_Sensor.Z_L_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("homed return to Z=0 stops on the HOME(L) sensor even just below zero",
       moveCount==1 && stopCount==2 && errorCode==2891);
 reset();xSL.isHomed=1;position[aZ]=-50000;xCD.Decapper.Motor_TargetPos[aZ]=0;
 CDecap_Action_Statemachine(ACTION_AMOVEZ);
 position[aZ]=0;xSL.xZ_Motor_Run.Motor_Run=0;gTriggerCount=SEC_TO_TICKS(1)+1;
 CDecap_Action_Statemachine(ACTION_NONE);CDecap_Action_Statemachine(ACTION_NONE);
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("homed return to Z=0 completes with the HOME(L) sensor released",
       !xSL.isError && CDecap_GetPhase()==DFSM_IDLE && CDecap_GetOperationResult()==2);
 reset();xSL.isHomed=1;position[aZ]=-50000;
 CDecap_Action_Statemachine(ACTION_ORIGIN);
 position[aZ]=-Z_HOME_OFFSET_PULSE/2;xSL.xZ_Motor_Run.Motor_Run=1;xSL.CDecapping_Sensor.Z_L_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("ORG return to Z=0 stops on the HOME(L) sensor",
       moveCount==1 && stopCount==2 && errorCode==2891);
 reset();xSL.isHomed=1;position[aZ]=0;xCD.Decapper.Motor_TargetPos[aZ]=100;
 xSL.CDecapping_Sensor.Z_L_Limit_Sensor=0;
 CDecap_Action_Statemachine(ACTION_AMOVEZ);
 CHECK("homed target beyond the home reference is blocked by the HOME(L) sensor",
       moveCount==0 && stopCount==2 && errorCode==2891);
 /* 유지 전류 전환: 정지 명령 직후가 아니라 정지를 확인한 뒤에만 내린다 */
 reset();useMotorFeedback=true;motorFeedback[aZ]=motorFeedback[aR]=1;
 TMC2660_SetRunCurrent(aZ);TMC2660_SetRunCurrent(aR);
 sPhase=DFSM_LONGRUN;xSL.isBusy=YES;CDecap_RequestStop();CDecap_Action_Statemachine(xAT);
 CHECK("STOP sends the stop commands without dropping to hold current",
       stopCount==2 && motorCurrentRun[aZ] && motorCurrentRun[aR] && sPhase==DFSM_IDLE);
 CDecap_Action_Statemachine(xAT);
 CHECK("run current is kept while both axes are still decelerating",motorCurrentRun[aZ] && motorCurrentRun[aR]);
 motorFeedback[aZ]=0;CDecap_Action_Statemachine(xAT);
 CHECK("only the axis that has stopped drops to hold current",!motorCurrentRun[aZ] && motorCurrentRun[aR]);
 motorFeedback[aR]=0;CDecap_Action_Statemachine(xAT);
 CHECK("the other axis drops to hold current once it stops",!motorCurrentRun[aR]);
 reset();useMotorFeedback=true;TMC2660_SetRunCurrent(aZ);TMC2660_SetRunCurrent(aR);
 sPhase=DFSM_END_OK;xSL.isBusy=YES;CDecap_Action_Statemachine(ACTION_NONE);
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("idle after a normally completed operation returns both axes to hold current",
       sPhase==DFSM_IDLE && !motorCurrentRun[aZ] && !motorCurrentRun[aR] && !xSL.isError);
 reset();useMotorFeedback=true;TMC2660_SetRunCurrent(aZ);xSL.isHomed=1;
 sPhase=DFSM_CAP;xSL.isBusy=YES;CDecapping_Step=CDecapping_Cap_UpWait;previousTime[SUB_CAPPING]=gTriggerCount;
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("a dwell inside a running sequence keeps the run current",
       motorCurrentRun[aZ] && sPhase==DFSM_CAP && !xSL.isError);
 motorFeedback[aZ]=1;CDecap_Action_Statemachine(ACTION_PAUSE);CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("PAUSE keeps the run current while the axis is still moving",CDecap_IsPaused() && motorCurrentRun[aZ]);
 motorFeedback[aZ]=0;CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("a paused, stopped axis drops to hold current",CDecap_IsPaused() && !motorCurrentRun[aZ]);
 reset();useMotorFeedback=true;motorFeedback[aZ]=1;TMC2660_SetRunCurrent(aZ);
 sPhase=DFSM_CAP;xSL.isBusy=YES;CDecap_Fault(ERROR_CODE_DECAP_TIMEOUT,"current test");
 CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("a fault stop keeps the run current until the axis stops",xSL.isError && motorCurrentRun[aZ]);
 motorFeedback[aZ]=0;CDecap_Action_Statemachine(ACTION_NONE);
 CHECK("a stopped axis in the fault state drops to hold current",xSL.isError && !motorCurrentRun[aZ]);
 return failures?1:0;
}
'''
# COMPILE_AND_RUN
zig=os.environ.get('ZIG_EXE') or shutil.which('zig')
if not zig: sys.exit('Set ZIG_EXE to a Zig C compiler (tested with Zig 0.13.0).')
with tempfile.TemporaryDirectory(prefix='vial-motion-') as work:
    out=pathlib.Path(work)
    (out/'motion_test.c').write_text(code,encoding='utf-8')
    build=subprocess.run([zig,'cc','-std=c11','-Wno-format','-o',str(out/'motion_test.exe'),str(out/'motion_test.c')])
    result=build.returncode if build.returncode else subprocess.run([str(out/'motion_test.exe')]).returncode
sys.exit(result)
