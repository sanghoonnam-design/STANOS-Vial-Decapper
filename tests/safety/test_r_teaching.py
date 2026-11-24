"""Run real command/FSM code with simulated motion registers and EEPROM failures."""
import os
import pathlib
import subprocess
import sys
import tempfile

root = pathlib.Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else pathlib.Path(__file__).resolve().parents[2]
helper = root / 'tests/safety/test_motion.py'
ns = {'__file__': str(helper)}
exec(helper.read_text(encoding='utf-8').split('# COMPILE_AND_RUN')[0], ns)
code = ns['code'].split('int main(void)')[0]
code = code.replace('tsXStateList xSL;', 'bool gZCapUpPosSavePending, gRDecapPosSavePending; S32 gZCapUpPosPendingValue, gRDecapPosPendingValue;\ntsXStateList xSL;')
code = code.replace('response[128]', 'response[512]')
code = code.replace('void IOEXP_WriteIObit(U8 a,U8 b){}', 'static int doLevels[16],bodyDoubleOn; void IOEXP_WriteIObit(U8 a,U8 b){doLevels[a]=b;if(doLevels[6] && doLevels[7])bodyDoubleOn++;}')
code = code.replace('S32 TMC429_GetVelocity(U8 a){return 0;}', 'static S32 actualVelocity[2]; S32 TMC429_GetVelocity(U8 a){return actualVelocity[a];}')
code += r'''
void XBuffer_AddInt(void*b,int v,int c){char text[32];snprintf(text,sizeof text,"%d",v);strncat(response,text,sizeof(response)-strlen(response)-1);if(c)strcat(response,",");}
void XBuffer_AddUInt(void*b,unsigned v,int c){snprintf(response,sizeof response,"%u",v);}
#define SLOT_COUNT 6
typedef int tsXCommandMapping;
static int timerStops,timerStarts,saves;
static S32 storedZ,storedR;
static bool saveOK=true,loadOK=true;
int gEEPROM;
#define XTimer_Stop() (timerStops++)
#define XTimer_Start() (timerStarts++)
U32 SWRTC_GetTime_YYMMDDHH(void){return 0;}
bool EEPROMPL_SaveVerified(void){saves++;storedZ=xPL.Decapper.ZCap_UpPos;storedR=xPL.Decapper.RDecapPos;return saveOK;}
bool EEPROMPL_SaveDecapperTarget(U08 axis,S32 position){
 saves++;
 if (!saveOK) return false;
 if (axis==aZ) storedZ=position;else storedR=position;
 return true;
}
bool EEPROMPL_LoadFromEEPROM(void*p){return loadOK;}
#define COMM_USB 1
#define xcprintf(...) ((void)0)
static void RPL_GetParameter(void){}
static void RPL_PrintHelp(void){}
'''
code += ns['stripped'](ns['read']('App/Task/_10_XCommand_Module.h'))
for name in ['GetParamInt', 'GetParamFloat', 'RPL_SetParameter', 'API_AddParamFloat', 'CMD_Handle_PGET', 'CMD_Handle_TEACH', 'CMD_Handle_RPOS', 'CMD_Handle_Print_PL', 'CMD_RequestGrip', 'CMD_Handle_BGRIP', 'CMD_Handle_CGRIP']:
    code += ns['function'](ns['module'].replace('__attribute__((unused)) ', ''), name)
core = ns['read']('App/Task/_10_XCommand_Core.c')
for name in ['CMD_BeginMaintenance', 'CMD_Handle_SAVEE', 'CMD_Handle_LOADE']:
    code += ns['function'](core, name)
code += r'''
static tsXParsedData args(int n,int a,int b){
 tsXParsedData p={0};p.ParamCount=n;p.Params[0].type=p.Params[1].type=PARAM_TYPE_INT;
 p.Params[0].value._int=a;p.Params[1].value._int=b;return p;
}
static void prepare(void){
 reset();gZCapUpPosSavePending=gRDecapPosSavePending=false;
 actualVelocity[0]=actualVelocity[1]=0;
 gZCapUpPosPendingValue=gRDecapPosPendingValue=0;response[0]=0;
 saves=timerStops=timerStarts=0;saveOK=loadOK=true;storedZ=2000;storedR=111;
 xPL.Decapper.ZCap_UpPos=2000;xPL.Decapper.ZCap_SidePos=4000;
 xPL.Decapper.ZDecapAcc=xPL.Decapper.RDecapAcc=300000;
 xPL.Decapper.ZDecapVel=xPL.Decapper.RDecapVel=20000;
 xPL.Decapper.RDecapPos=111;xSL.isHomed=true;
 xSL.CDecapping_Sensor.CT_Detect_Sensor=1;
 xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Open=1;
}
static void perform(teXActionType action){
 CDecap_TryRequestAction(action);
 for(int n=0;n<3000 && !xSL.isError;n++){
  CDecap_Action_Statemachine(xAT);
  for(int ch=0;ch<2;ch++)if(sMovePending[ch])position[ch]=sMoveTarget[ch];
  gTriggerCount++;
  if(sPhase==DFSM_IDLE && xAT==ACTION_NONE)break;
 }
}
int main(void){
 setbuf(stdout,NULL);tsXParsedData p,saveZ=args(1,'Z',0),saveR=args(1,'R',0);
 memset(doLevels,0,sizeof doLevels);prepare();p=args(1,1,0);CMD_Handle_BGRIP(&p,0);perform(ACTION_NONE);
 CHECK("BGRIP 1 drives physical DO7 ON and DO8 OFF",doLevels[6]==1 && doLevels[7]==0 && doLevels[4]==0 && doLevels[5]==0);
 p=args(1,0,0);CMD_Handle_BGRIP(&p,0);perform(ACTION_NONE);
 CHECK("BGRIP 0 drives physical DO7 OFF and DO8 ON",doLevels[6]==0 && doLevels[7]==1 && bodyDoubleOn==0);
 Air_CTBody_Gripper(CT_GRIP_STANDBY);
 CHECK("body standby clears both DO7 and DO8",doLevels[6]==0 && doLevels[7]==0);

 prepare();p=args(1,1,0);
 int capGripBefore=doLevels[2],capUngripBefore=doLevels[3];
 CMD_Handle_CGRIP(&p,0);
 CHECK("CGRIP is accepted before either output changes",
       !strcmp(response,"OK") && xAT==ACTION_CAP_GRIP &&
       doLevels[2]==capGripBefore && doLevels[3]==capUngripBefore);
 p=args(1,0,0);CMD_Handle_CGRIP(&p,0);
 CHECK("busy CGRIP preserves the accepted value",
       !strcmp(response,"BUSY") && xAT==ACTION_CAP_GRIP && xCD.CT.Cap==1);
 CDecap_Action_Statemachine(xAT);
 CHECK("Cap grip phase drives DO3 ON and DO4 OFF and enters END_OK",
       doLevels[2]==1 && doLevels[3]==0 && sPhase==DFSM_END_OK);
 CDecap_Action_Statemachine(xAT);
 CHECK("Cap grip returns to idle with completed status",
       CDecap_IsIdle() && CDecap_GetOperationResult()==2);
 CMD_Handle_CGRIP(&p,0);perform(ACTION_NONE);
 CHECK("CGRIP 0 drives DO3 OFF and DO4 ON",
       doLevels[2]==0 && doLevels[3]==1 && CDecap_IsIdle());

 prepare();position[aZ]=123;position[aR]=-3200;p=args(0,0,0);
 U32 operationBefore=CDecap_GetOperationId();
 monitorPositionLock=true;unlockedPositionReads=0;
 CMD_Handle_RPOS(&p,0);
 CHECK("RPOS reads both axes under the motion mutex",unlockedPositionReads==0 && !motionMutexHeld);
 monitorPositionLock=false;
 CHECK("RPOS directly returns Z,R without scheduling an action",!strcmp(response,"123,-3200") && xAT==ACTION_NONE && CDecap_IsIdle() && CDecap_GetOperationId()==operationBefore && moveCount==0);
 motionMutexHeld=true;position[aZ]=999;position[aR]=999;
 CMD_Handle_RPOS(&p,0);
 CHECK("busy RPOS does not replace either teaching snapshot",
       !strcmp(response,"BUSY") && motionMutexHeld && gZCapUpPosPendingValue==123 && gRDecapPosPendingValue==-3200);
 motionMutexHeld=false;
 prepare();position[aZ]=456;position[aR]=3200;xAT=ACTION_CAP;sPhase=DFSM_CAP;xSL.isBusy=true;CMD_Handle_RPOS(&p,0);
 CHECK("RPOS during CAP leaves its action and phase running",!strcmp(response,"456,3200") && xAT==ACTION_CAP && sPhase==DFSM_CAP && xSL.isBusy && moveCount==0);
 prepare();CDecap_Fault(ERROR_CODE_DECAP_LIMIT,"Test limit");position[aZ]=123;position[aR]=-3200;CMD_Handle_RPOS(&p,0);
 CHECK("RPOS reads coordinates while preserving a latched fault",!strcmp(response,"123,-3200") && xSL.isError && errorCode==ERROR_CODE_DECAP_LIMIT);
 prepare();position[aZ]=123;p=args(1,'Z',0);CMD_Handle_RPOS(&p,0);
 CHECK("explicit RPOS Z returns only Z without modifying RAM",!strcmp(response,"123") && gZCapUpPosSavePending && xPL.Decapper.ZCap_UpPos==2000);
 CMD_Handle_SAVEE(&saveZ,0);
 CHECK("explicit Z candidate saves independently",xPL.Decapper.ZCap_UpPos==123 && storedZ==123 && saves==1);

 for(int sign=-1;sign<=1;sign+=2){
  prepare();position[aZ]=123;position[aR]=sign*3200;p=args(0,0,0);CMD_Handle_RPOS(&p,0);
  const char *want=sign<0?"123,-3200":"123,3200";
  CHECK("plain RPOS returns both coordinates in Z,R order",!strcmp(response,want));
  CHECK("plain RPOS captures both signs without changing RAM",gZCapUpPosSavePending && gRDecapPosSavePending && gZCapUpPosPendingValue==123 && gRDecapPosPendingValue==sign*3200 && xPL.Decapper.ZCap_UpPos==2000 && xPL.Decapper.RDecapPos==111);
  position[aZ]=999;position[aR]=500;
  p=args(0,0,0);CMD_Handle_SAVEE(&p,0);
  CHECK("bare SAVE refuses to save both snapshots",!strcmp(response,"INVALID") && saves==0 && gZCapUpPosSavePending && gRDecapPosSavePending);
  p=args(2,'Z','R');CMD_Handle_SAVEE(&p,0);
  CHECK("SAVE Z,R is rejected without writing either axis",!strcmp(response,"INVALID") && saves==0 && timerStops==0);
  xPL.Decapper.RDecapPos=555;
  CMD_Handle_SAVEE(&saveZ,0);
  CHECK("SAVE Z persists the captured Z and preserves R RAM, EEPROM and candidate",
        storedZ==123 && storedR==111 && xPL.Decapper.ZCap_UpPos==123 &&
        xPL.Decapper.RDecapPos==555 && !gZCapUpPosSavePending && gRDecapPosSavePending);
  CMD_Handle_SAVEE(&saveR,0);
  CHECK("SAVE R persists the signed capture and consumes only its candidate",
        storedZ==123 && storedR==sign*3200 && xPL.Decapper.RDecapPos==sign*3200 &&
        !gZCapUpPosSavePending && !gRDecapPosSavePending);
 }
 for(int axis=0;axis<2;axis++){
  prepare();gZCapUpPosSavePending=gRDecapPosSavePending=true;gZCapUpPosPendingValue=222;gRDecapPosPendingValue=-444;
  position[aZ]=123;position[aR]=3200;p=args(0,0,0);
  if(axis==aZ)xSL.xZ_Motor_Run.Motor_Run=true;else xSL.xR_Motor_Run.Motor_Run=true;
  CMD_Handle_RPOS(&p,0);
  CHECK("RPOS reads during axis motion without changing controller state",!strcmp(response,"123,3200") && gZCapUpPosPendingValue==123 && gRDecapPosPendingValue==3200 && !CDecap_IsMaintenanceActive());
  xSL.xZ_Motor_Run.Motor_Run=xSL.xR_Motor_Run.Motor_Run=false;actualVelocity[axis]=10;response[0]=0;
  CMD_Handle_RPOS(&p,0);
  CHECK("RPOS reads current coordinates while either axis is coasting",!strcmp(response,"123,3200") && gZCapUpPosPendingValue==123 && gRDecapPosPendingValue==3200);
 }
 for(int axis=0;axis<2;axis++){
  prepare();gZCapUpPosSavePending=gRDecapPosSavePending=true;gZCapUpPosPendingValue=222;gRDecapPosPendingValue=-444;
  position[aZ]=axis==aZ?170000:123;position[aR]=axis==aR?100001:-3200;p=args(0,0,0);CMD_Handle_RPOS(&p,0);
  CHECK("RPOS reports raw coordinates outside teaching limits",gZCapUpPosPendingValue==(axis==aZ?170000:123) && gRDecapPosPendingValue==(axis==aR?100001:-3200) && !CDecap_IsMaintenanceActive());
  CMD_Handle_SAVEE(axis==aZ?&saveZ:&saveR,0);
  CHECK("SAVE rejects an out of range raw snapshot without changing settings",!strcmp(response,"RANGE") && saves==0 && xPL.Decapper.ZCap_UpPos==2000 && xPL.Decapper.RDecapPos==111);
 }
 prepare();xSL.isHomed=false;position[aZ]=123;position[aR]=-3200;p=args(0,0,0);CMD_Handle_RPOS(&p,0);
 CHECK("RPOS returns current coordinates before HOME",!strcmp(response,"123,-3200") && gZCapUpPosSavePending && gRDecapPosSavePending);
 prepare();position[aZ]=123;position[aR]=-3200;p=args(0,0,0);CMD_Handle_RPOS(&p,0);
 saveOK=false;CMD_Handle_SAVEE(&saveR,0);
 CHECK("failed SAVE R leaves both RAM values and candidates available",
       !strcmp(response,"FAIL") && xPL.Decapper.ZCap_UpPos==2000 && xPL.Decapper.RDecapPos==111 &&
       gZCapUpPosSavePending && gRDecapPosSavePending && timerStops==timerStarts && !CDecap_IsMaintenanceActive());
 saveOK=true;CMD_Handle_SAVEE(&saveR,0);
 CHECK("retry SAVE R preserves pending Z",storedZ==2000 && storedR==-3200 && gZCapUpPosSavePending && !gRDecapPosSavePending);
 CMD_Handle_SAVEE(&saveR,0);
 CHECK("missing R candidate never writes while Z remains available",!strcmp(response,"NO_RPOS") && saves==2 && gZCapUpPosSavePending);
 for(int sign=-1;sign<=1;sign+=2){
  prepare();position[aR]=sign*3200;p=args(1,'R',0);CMD_Handle_RPOS(&p,0);
  CHECK("RPOS R captures signed coordinate without modifying RAM",gRDecapPosSavePending && gRDecapPosPendingValue==sign*3200 && xPL.Decapper.RDecapPos==111 && atoi(response)==sign*3200);
  CMD_Handle_SAVEE(&saveR,0);
  CHECK("SAVE R persists signed R candidate and clears it",xPL.Decapper.RDecapPos==sign*3200 && storedR==sign*3200 && !gRDecapPosSavePending && !strcmp(response,"OK"));
 }
 prepare();position[aZ]=456;p=args(1,'Z',0);CMD_Handle_RPOS(&p,0);
 position[aR]=-3200;p=args(1,'R',0);CMD_Handle_RPOS(&p,0);CMD_Handle_SAVEE(&saveR,0);
 CHECK("SAVE R leaves separately captured Z pending",storedZ==2000 && storedR==-3200 && gZCapUpPosSavePending);
 CMD_Handle_SAVEE(&saveZ,0);
 CHECK("subsequent SAVE Z writes its own capture",storedZ==456 && storedR==-3200 && !gZCapUpPosSavePending && !gRDecapPosSavePending);
 prepare();gRDecapPosSavePending=true;gRDecapPosPendingValue=500;
 p=args(1,'X',0);CMD_Handle_RPOS(&p,0);
 CHECK("invalid RPOS axis preserves previous candidate",!strcmp(response,"INVALID") && gRDecapPosPendingValue==500);
 p=args(1,'R',0);xSL.isBusy=true;position[aR]=600;response[0]=0;CMD_Handle_RPOS(&p,0);
 CHECK("RPOS R returns the moving R coordinate",!strcmp(response,"600") && gRDecapPosPendingValue==600 && !CDecap_IsMaintenanceActive());
 xSL.isBusy=false;actualVelocity[aR]=10;position[aR]=700;response[0]=0;CMD_Handle_RPOS(&p,0);
 CHECK("RPOS R returns the current coordinate while coasting",!strcmp(response,"700") && gRDecapPosPendingValue==700);
 actualVelocity[aR]=0;
 xSL.isBusy=false;xSL.isHomed=false;response[0]=0;CMD_Handle_RPOS(&p,0);
 CHECK("RPOS R returns raw position with no HOME reference",!strcmp(response,"700") && gRDecapPosPendingValue==700);
 xSL.isHomed=true;position[aR]=100001;response[0]=0;CMD_Handle_RPOS(&p,0);
 CHECK("RPOS R prints a raw position outside configured limits",!strcmp(response,"100001") && gRDecapPosPendingValue==100001);
 prepare();gZCapUpPosSavePending=gRDecapPosSavePending=true;
 gZCapUpPosPendingValue=456;gRDecapPosPendingValue=3200;
 gRDecapPosPendingValue=8388608;CMD_Handle_SAVEE(&saveZ,0);
 CHECK("out of range R candidate does not block SAVE Z",!strcmp(response,"OK") && storedZ==456 && storedR==111 && gRDecapPosSavePending);
 CMD_Handle_SAVEE(&saveR,0);
 CHECK("SAVE R checks its range before accessing EEPROM",!strcmp(response,"RANGE") && saves==1 && xPL.Decapper.RDecapPos==111);
 prepare();gRDecapPosSavePending=true;gRDecapPosPendingValue=-3200;p=args(1,1,0);CMD_Handle_SAVEE(&p,0);
 CHECK("SAVE 1 ignores RPOS R candidate",storedR==111 && xPL.Decapper.RDecapPos==111 && !gRDecapPosSavePending);
 prepare();gRDecapPosSavePending=true;p=args(2,RPL_SET_DECAP_R_POS,-500);CMD_Handle_TEACH(&p,0);
 CHECK("explicit TEACH cancels stale RPOS R candidate",xPL.Decapper.RDecapPos==-500 && !gRDecapPosSavePending);
 prepare();gRDecapPosSavePending=true;p=args(2,RPL_SET_DECAP_R_POS,500);CMD_Handle_Print_PL(&p,0);
 CHECK("explicit PL cancels stale RPOS R candidate",xPL.Decapper.RDecapPos==500 && !gRDecapPosSavePending);
 prepare();gRDecapPosSavePending=true;p=args(0,0,0);CMD_Handle_LOADE(&p,0);
 CHECK("successful LOAD cancels R teaching candidate",!gRDecapPosSavePending && !xSL.isHomed);
 prepare();position[aR]=765;gRDecapPosSavePending=true;CMD_Handle_HOME(&p,0);
 CHECK("accepted HOME cancels R teaching candidate",!gRDecapPosSavePending && xAT==ACTION_HOME);
 for(int n=0;n<300;n++){ /* HOME은 Y 전진 전 Body 해제 대기(최대 100틱)를 포함한다 */
  if(HomingSeq.Step==CDECAP_HOMING_H_REACH || HomingSeq.Step==CDECAP_HOMING_S_REACH)xSL.CDecapping_Sensor.Z_L_Limit_Sensor=0;
  if(HomingSeq.Step==CDECAP_HOMING_L_REACH_OFF)xSL.CDecapping_Sensor.Z_L_Limit_Sensor=1;
  CDecap_Action_Statemachine(xAT);gTriggerCount++;
  if(sPhase==DFSM_IDLE && xAT==ACTION_NONE)break;
 }
 CHECK("HOME reestablishes R zero even after a previous HOME",xSL.isHomed && position[aR]==0 && position[aZ]==0 && !xSL.isError);
 prepare();position[aR]=765;HomingSeq.Step=CDECAP_HOMING_COMPLETE;SET_STARTED(SUB_HOMEZ);xSL.isHomed=false;xSL.xR_Motor_Run.Motor_Run=true;
 CDecap_StartTimeout();
 CDecap_Homing();
 CHECK("HOME never zeros R while running",position[aR]==765 && !xSL.isHomed);
 xSL.xR_Motor_Run.Motor_Run=false;actualVelocity[aR]=10;CDecap_Homing();
 CHECK("HOME also waits for actual R velocity to become zero",position[aR]==765 && !xSL.isHomed);
 actualVelocity[aR]=0;
 xSL.xR_Motor_Run.Motor_Run=false;CDecap_Homing();
 CHECK("HOME zeros R only after stop confirmation",position[aR]==0 && xSL.isHomed);
 for(int sign=-1;sign<=1;sign+=2){
  prepare();xPL.Decapper.RDecapPos=sign*3200;position[aR]=1500;
  perform(ACTION_UDECAP);
  CHECK("unit DECAP reaches signed absolute target",position[aR]==sign*3200 && !xSL.isError);
  perform(ACTION_UDECAP);
  CHECK("repeated unit DECAP never accumulates rotation",position[aR]==sign*3200 && !xSL.isError);
  perform(ACTION_UCAP);perform(ACTION_UCAP);
  CHECK("unit CAP always returns to zero without accumulation",position[aR]==0 && !xSL.isError);
  perform(ACTION_DECAP);
  CHECK("automatic DECAP uses same signed absolute target",position[aR]==sign*3200 && !xSL.isError);
  perform(ACTION_CAP);
  CHECK("automatic CAP returns R to zero",position[aR]==0 && !xSL.isError);
 }
 return failures?1:0;
}
'''
with tempfile.TemporaryDirectory(prefix='vial-r-teaching-') as work:
    source = pathlib.Path(work) / 'test.c'
    source.write_text(code, encoding='utf-8')
    exe = source.with_suffix('.exe')
    subprocess.run([os.environ['ZIG_EXE'], 'cc', '-std=c11', '-Wno-format', str(source), '-o', str(exe)], check=True)
    raise SystemExit(subprocess.run([str(exe)]).returncode)
