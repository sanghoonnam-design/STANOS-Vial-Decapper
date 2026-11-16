"""Exercise real API handlers with actual Decapper FSM and mocked hardware."""
import os, pathlib, subprocess, tempfile
base=pathlib.Path(__file__).resolve().parent
ns={"__file__":str(base/"test_motion.py")}
exec((base/"test_motion.py").read_text(encoding="utf-8").split("# COMPILE_AND_RUN")[0],ns)
code=ns["code"].split("static int failures;")[0]
code=code.replace('response[128]','response[512]')
code=code.replace('snprintf(response,sizeof response,"%s",s);',
                  'strncat(response,s,sizeof(response)-strlen(response)-1);if(comma)strcat(response,",");')
code+=r'''
void XBuffer_AddInt(void*b,int value,int comma){char s[32];snprintf(s,sizeof s,"%d",value);XBuffer_AddString(b,s,comma);}
void XBuffer_AddUInt(void*b,unsigned value,int comma){char s[32];snprintf(s,sizeof s,"%u",value);XBuffer_AddString(b,s,comma);}
void XBuffer_AddFloat(void*b,float value,int comma){char s[32];snprintf(s,sizeof s,"%.3f",value);XBuffer_AddString(b,s,comma);}
#define SLOT_COUNT 6
typedef int tsXCommandMapping;
bool gZCapUpPosSavePending; S32 gZCapUpPosPendingValue;
'''
code+=ns["stripped"](ns["read"]("App/Task/_10_XCommand_Module.h"))
for name in ["GetParamInt","GetParamFloat","RPL_SetParameter","API_NoArgs","API_AddParamFloat"]:
    code+=ns["function"](ns["module"].replace("__attribute__((unused)) ", ""),name)
for name in ["CMD_Handle_GPOS","CMD_Handle_TEACH","CMD_Handle_PGET","CMD_Handle_PCONFIG","CMD_Handle_GSTATE","CMD_Handle_GSENSOR","CMD_Handle_GLR","CMD_Handle_LongRun"]:
    code+=ns["function"](ns["module"],name) or ('void '+name+'(const tsXParsedData*p,U08 c){strcpy(response,"MISSING");}\n')
if "U32 CDecap_GetLongRunLimit(void)" not in ns["c"]:
    code+='U32 CDecap_GetLongRunLimit(void){return 0;}\n'
if "U32 CDecap_GetOperationId(void)" not in ns["c"]:
    code+='U32 CDecap_GetOperationId(void){return 0;} int CDecap_GetOperationResult(void){return 0;}\n'
code+=r'''
#define COMM_USB 1
#define xcprintf(...) ((void)0)
static void RPL_GetParameter(void){}
static void RPL_PrintHelp(void){}
'''
for name in ['CMD_Handle_Print_SL','CMD_Handle_Print_CD','CMD_Handle_Print_PL']:
    code+=ns['function'](ns['module'],name)
for name in ["CMD_RequestGrip", "CMD_Handle_MOVE", "CMD_Handle_READY", "CMD_Handle_CGRIP"]:
    code+=ns["function"](ns["module"],name)
parser=ns["read"]("App/Common/XParser.c").split("void xParser_HandleError(")[0]
code+=r"""#include <ctype.h>
#include <errno.h>
#include <limits.h>
#define UPCASE(c) toupper((unsigned char)(c))
static void xParser_PrintParsedData(const tsXParsedData *p){}
"""
code+=ns["stripped"](parser)
app_control=ns["read"]("App/Task/_06_XAppControl.c")
code+=r"""#include <setjmp.h>
#define FOREVER for (;;)
#define RTOS_WAIT_FOREVER 0
#define portMAX_DELAY 0
#define FW_MODE_IDLE 1
#define TP_IDX_taskAPC 0
#define __TASK_TRIGGER_START_Using(...) ((void)0)
#define __TASK_TRIGGER_END_Using(...) ((void)0)
#define XTP_CheckTaskUsingLED(...) ((void)0)
#define ITIMER_StartMeasure_us() 0
#define ITIMER_StopMeasure_us(tick) (tick)
static int Control_StartTime=200;
static int gTick_APC;
static bool __xTaskStatus[1];
static int normalMode(void){return 0;}
static const struct {int (*PL_Get_FW_Mode)(void);} xSystemInfo={normalMode};
static int tickToken,controlTicks;
static SemaphoreHandle_t semHD_APC=&tickToken;
static jmp_buf controlTickDone;
static int takeControlSemaphore(SemaphoreHandle_t semaphore,U32 ticks){
 if(semaphore!=semHD_APC) return xSemaphoreTake(semaphore,ticks);
 if(controlTicks++) longjmp(controlTickDone,1);
 return pdTRUE;
}
#define xSemaphoreTake(semaphore,ticks) takeControlSemaphore(semaphore,ticks)
"""
code+=ns["function"](app_control.replace("VOID TASK_ApplicationControl","void TASK_ApplicationControl"),"TASK_ApplicationControl")
code+=r"""#undef xSemaphoreTake
static void runControlTick(void){
 controlTicks=0;
 if(setjmp(controlTickDone)==0) TASK_ApplicationControl(NULL);
}
"""
code+=ns["function"](ns["code"],"reset").replace(" CDecap_Init();", " response[0]=0;CDecap_Init();")
code+=r'''
static int failures;
#define CHECK(n,c) do{printf("%s %s\n",(c)?"PASS":"FAIL",n);if(!(c))failures++;}while(0)
static tsXParsedData args(int n,int a,int b){tsXParsedData p={0};p.ParamCount=n;p.Params[0].type=p.Params[1].type=PARAM_TYPE_INT;p.Params[0].value._int=a;p.Params[1].value._int=b;return p;}
static bool parse(const char *command,tsXParsedData *p){
 char frame[BUFFER_SIZE_RX_MSG];
 snprintf(frame,sizeof frame,"%s\r\n",command);
 return xParser_ProcessReceivedData(frame,p,false)==PARSER_ERR_SUCCESS;
}
int main(void){
 setbuf(stdout,NULL);
 reset();
 tsXParsedData input;
 CHECK("parser retains fractional Grip input",parse("CGRIP 0.9",&input) && input.Params[0].type==PARAM_TYPE_FLOAT);
 CMD_Handle_CGRIP(&input,0);
 CHECK("fractional Grip never schedules a solenoid action",xAT==ACTION_NONE && strcmp(response,"OK"));
 reset();parse("READY 1.9",&input);CMD_Handle_READY(&input,0);
 CHECK("fractional READY never schedules a Y action",xAT==ACTION_NONE && strcmp(response,"OK"));
 reset();parse("MOVE A Z -99.9",&input);CMD_Handle_MOVE(&input,0);
 CHECK("fractional pulse never schedules a motor action",xAT==ACTION_NONE && xCD.Decapper.Motor_TargetPos[aZ]==0);
 reset();parse("MOVE A R 1e30",&input);CMD_Handle_MOVE(&input,0);
 CHECK("out of range float never reaches the pulse conversion",xAT==ACTION_NONE && xCD.Decapper.Motor_TargetPos[aR]==0);
 reset();parse("PL 42.9,0",&input);CMD_Handle_Print_PL(&input,0);
 CHECK("fractional PL index does not select a different setting",strcmp(response,"OK"));
 reset();parse("CGRIP 1.0",&input);CMD_Handle_CGRIP(&input,0);
 CHECK("integral decimal remains compatible with the Grip command",xAT==ACTION_CAP_GRIP && xCD.CT.Cap==1);
 reset();position[aZ]=100;parse("MOVE R Z -25",&input);CMD_Handle_MOVE(&input,0);
 CHECK("valid signed MOVE is accepted and only stores the request",xAT==ACTION_RMOVEZ && moveCount==0);
 CDecap_Action_Statemachine(xAT);
 CHECK("accepted MOVE issues one relative move with the supplied pulse",
       moveCount==1 && lastMove.Pos==-25 && sMoveTarget[aZ]==75);
 reset();gTriggerCount=300;position[aZ]=456;position[aR]=0;
 monitorPositionLock=true;unlockedPositionReads=0;
 runControlTick();
 CHECK("application tick holds the same mutex while accessing SPI",
       xCD.Decapper.Motor_CurPos==456 && unlockedPositionReads==0 && !motionMutexHeld);
 monitorPositionLock=false;
 reset();sOperationId=0;position[aZ]=position[aR]=0;
 xPL.Decapper.Limit_PosZ=160000;xPL.Decapper.Limit_PosR=100000;CDecap_Init();
 tsXParsedData p=args(1,'Z',0);position[0]=123;gZCapUpPosSavePending=true;gZCapUpPosPendingValue=42;
 response[0]=0;CMD_Handle_GPOS(&p,0);
 CHECK("pure GPOS preserves legacy candidate",!strcmp(response,"Z,123") && gZCapUpPosPendingValue==42 && gZCapUpPosSavePending);
 monitorPositionLock=true;unlockedPositionReads=0;
 response[0]=0;CMD_Handle_GPOS(&p,0);
 CHECK("GPOS reads the controller under the motion mutex",!strcmp(response,"Z,123") && unlockedPositionReads==0 && !motionMutexHeld);
 motionMutexHeld=true;response[0]=0;CMD_Handle_GPOS(&p,0);
 CHECK("GPOS returns BUSY when motion owns the bus",!strcmp(response,"BUSY") && motionMutexHeld);
 motionMutexHeld=false;monitorPositionLock=false;
 p=args(2,RPL_SET_DECAP_Z_CAP_UP_POS,234);response[0]=0;CMD_Handle_TEACH(&p,0);
 CHECK("explicit teaching writes selected target",xPL.Decapper.ZCap_UpPos==234 && !gZCapUpPosSavePending);
 p=args(2,RPL_SET_DECAP_Z_CAP_SIDE_POS,200000);response[0]=0;CMD_Handle_TEACH(&p,0);
 CHECK("out of range teaching leaves target unchanged",xPL.Decapper.ZCap_SidePos==0 && strcmp(response,"OK"));
 p=args(1,RPL_SET_DECAP_Z_CAP_UP_POS,0);response[0]=0;CMD_Handle_PGET(&p,0);
 CHECK("parameter query returns actual RAM value",strstr(response,",234")!=NULL);
 xPL.Decapper.RunCur[aZ]=0.00001f;p=args(1,RPL_SET_DECAP_RUN_CUR_Z,0);response[0]=0;CMD_Handle_PGET(&p,0);
 CHECK("PGET float readback preserves RAM precision",strtof(strchr(response,',')+1,0)==xPL.Decapper.RunCur[aZ]);
 p=args(0,0,0);response[0]=0;CMD_Handle_PCONFIG(&p,0);
 CHECK("capability schema and first Decapper index are stable",!strcmp(response,"1,36"));
 response[0]=0;CMD_Handle_Print_SL(&p,0);
 CHECK("SL returns state and sensor data on command channel",strstr(response,"STATE=") && strstr(response,"SENSOR="));
 xCD.Decapper.Motor_TargetPos[aZ]=987;response[0]=0;CMD_Handle_Print_CD(&p,0);
 CHECK("CD returns actual targets and longrun data",strstr(response,"TARGET_Z=987") && strstr(response,"LR="));
 response[0]=0;CMD_Handle_Print_PL(&p,0);
 int separators=0;for(char*c=response;*c;c++)if(*c==';')separators++;
 CHECK("PL returns all 22 supported parameters without truncation",separators==21 && strstr(response,",234"));
 p=args(2,RPL_SET_DECAP_STEP_RESOLUTION,3200);
 CHECK("invalid hardware resolution code rejected",RPL_SetParameter(p.Params[0].value._int,&p)==NO);
 p=args(2,RPL_SET_DECAP_STEP_RESOLUTION,0);
 CHECK("valid 256 microstep resolution code accepted",RPL_SetParameter(p.Params[0].value._int,&p)==YES && xPL.Decapper.StepResolution==0);
 p=args(2,RPL_SET_DECAP_SEL_MAX_CUR_Z,2);
 CHECK("invalid hardware current selection rejected",RPL_SetParameter(p.Params[0].value._int,&p)==NO);
 gZCapUpPosSavePending=gRDecapPosSavePending=true;
 bool legacyRejected=true;
 for(int index=RPL_SET_DOOR_DIRECTION;index<RPL_SET_DECAP_RUN_CUR_Z;index++){
  p=args(2,index,1);
  ClearError();response[0]=0;
  CMD_Handle_Print_PL(&p,0);
  legacyRejected &= strcmp(response,"OK") && GetErrorCode_int()==ERROR_CODE_INVALID_ARGUMENT;
 }
 CHECK("legacy Door and Servo indices return invalid argument",legacyRejected);
 CHECK("unsupported writes release maintenance and preserve teaching candidates",
       !CDecap_IsMaintenanceActive() && gZCapUpPosSavePending && gRDecapPosSavePending);
 ClearError();
 p=args(1,3,0);response[0]=0;CMD_Handle_LongRun(&p,0);
 CHECK("finite LR request registers limit",CDecap_GetLongRunLimit()==3 && xAT==ACTION_LONGRUN);
 CHECK("accepted operation has an identity before FSM tick",CDecap_GetOperationId()==1 && CDecap_GetOperationResult()==1);
 return failures?1:0;
}
'''
with tempfile.TemporaryDirectory(prefix="vial-api-") as work:
    source=pathlib.Path(work)/"test.c";exe=source.with_suffix(".exe")
    source.write_text(code,encoding="utf-8")
    subprocess.run([os.environ["ZIG_EXE"],"cc","-std=c11","-Wno-format",str(source),"-o",str(exe)],check=True)
    raise SystemExit(subprocess.run([str(exe)]).returncode)
