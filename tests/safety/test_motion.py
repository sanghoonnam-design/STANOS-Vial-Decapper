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
void TMC2660_SetHoldCurrent(U8 a){}
U8 TMC429_VelMove(U8 a,U32 b,S32 d){return 0;}
void TMC429_MotorStop(U8 a,U32 b){}
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
 xPL.Decapper.Limit_PosZ=160000;xPL.Decapper.Limit_PosR=100000;
 xPL.Decapper.SoftLimitEnable=1;
 xPL.Decapper.SwNegLimit[0]=-160000;xPL.Decapper.SwPosLimit[0]=160000;
 xPL.Decapper.SwNegLimit[1]=-100000;xPL.Decapper.SwPosLimit[1]=100000;
 CDecap_Init();CDecap_StartTimeout();moveCount=stopCount=0;
}
int main(void){tsXParsedData cmd={0};
 reset();position[0]=150000; CDecap_ManualMotorMove(SUB_zRMOVE,aZ,true,20000);
 CHECK("relative endpoint outside limit never sent",moveCount==0);
 reset();xPL.Decapper.Limit_PosZ=10000;xCD.Decapper.Motor_TargetPos[0]=11000;CDecap_Absmove_Z();
 CHECK("Z absolute uses configured limit",moveCount==0);
 reset();CDecap_ManualMotorMove(SUB_zRMOVE,aZ,true,500);
 CHECK("valid relative move unchanged",moveCount==1 && lastMove.Pos==500);
 reset();position[0]=159500;CDecap_ManualMotorMove(SUB_zRMOVE,aZ,true,500);
 CHECK("exact positive boundary accepted",moveCount==1 && lastMove.Pos==500);
 reset();position[0]=-159500;CDecap_ManualMotorMove(SUB_zRMOVE,aZ,true,-500);
 CHECK("exact negative boundary accepted",moveCount==1 && lastMove.Pos==-500);
 reset();position[0]=1000;CDecap_ManualMotorMove(SUB_zRMOVE,aZ,true,INT32_MAX);
 CHECK("relative addition cannot wrap into valid range",moveCount==0);
 reset();xPL.Decapper.SwPosLimit[0]=5000;CDecap_ManualMotorMove(SUB_zAMOVE,aZ,false,6000);
 CHECK("narrower software limit enforced",moveCount==0);
 reset();CHECK("invalid automatic target fails without motion",!CD_ABSMove(0,300000,20000,170000) && moveCount==0);
 reset();xPL.Decapper.SwPosLimit[0]=1000;position[0]=1001;xSL.xZ_Motor_Run.Motor_Run=1;sPhase=DFSM_AMOVEZ;
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
 CHECK("deferred sensor interlock remains disabled",!xSL.isError);
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
       xSL.errorCode==2861 && strstr(response,"Y Back limit not reached") && strstr(response,"F=0 B=1"));
 reset();sPhase=DFSM_HOME;HomingSeq.Step=CDECAP_HOMING_L_REACH_OFF;SET_STARTED(SUB_HOMEZ);
 gTriggerCount=MS_TO_TICKS(HOMING_TIMEOUT_MS);
 CDecap_Action_Statemachine(ACTION_NONE);CMD_Handle_GERD(&cmd,0);
 CHECK("HOME reports Z sensor release failure and observed bits",
       xSL.errorCode==2851 && strstr(response,"Z home sensor did not release") && strstr(response,"H=0 L=0"));
 reset();sPhase=DFSM_HOME;HomingSeq.Step=CDECAP_HOMING_H_WAIT;SET_STARTED(SUB_HOMEZ);
 xSL.xZ_Motor_Run.Motor_Run=1;gTriggerCount=MS_TO_TICKS(HOMING_TIMEOUT_MS);
 CDecap_Action_Statemachine(ACTION_NONE);CMD_Handle_GERD(&cmd,0);
 CHECK("HOME motor stop timeout stays a motor error",
       xSL.errorCode==2800 && strstr(response,"Z stop not confirmed"));
 reset();sPhase=DFSM_AROTATE;xCD.Decapper.Motor_TargetPos[aR]=100001;
 CDecap_Action_Statemachine(ACTION_NONE);CMD_Handle_GERD(&cmd,0);
 CHECK("invalid R target names the axis and rejected target",
       xSL.errorCode==ERROR_CODE_DECAP_LIMIT && strstr(response,"R target 100001") &&
       strstr(response,"[-100000,100000]"));
 reset();sPhase=DFSM_AMOVEZ;position[aZ]=160001;xSL.xZ_Motor_Run.Motor_Run=1;
 CDecap_Action_Statemachine(ACTION_NONE);CMD_Handle_GERD(&cmd,0);
 CHECK("limit error distinguishes observed Z position from requested target",
       xSL.errorCode==ERROR_CODE_DECAP_LIMIT && strstr(response,"Z position 160001"));
 reset();sPhase=DFSM_AROTATE;xPL.Decapper.Limit_PosR=0;
 CDecap_Action_Statemachine(ACTION_NONE);CMD_Handle_GERD(&cmd,0);
 CHECK("invalid configured limits are identified separately",
       xSL.errorCode==ERROR_CODE_DECAP_LIMIT && strstr(response,"R limits invalid"));
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
