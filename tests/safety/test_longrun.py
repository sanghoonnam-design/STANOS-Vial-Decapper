"""Run the real Decapper FSM against a deterministic hardware model; never operates a device."""
import pathlib,sys,os,subprocess,tempfile,json,hashlib
base=pathlib.Path(__file__).resolve().parent
root=pathlib.Path(sys.argv[1]).resolve() if len(sys.argv)>1 else base.parents[1]
helper=root/'tests/safety/test_motion.py'
ns={'__file__':str(helper)}
exec(helper.read_text(encoding='utf-8').split('# COMPILE_AND_RUN')[0],ns)
code=ns['code'].split('int main(void)')[0]
code=code.replace('void IOEXP_WriteIObit(U8 a,U8 b){}', 'void model_io(U8 a,U8 b); void IOEXP_WriteIObit(U8 a,U8 b){model_io(a,b);}')
code=code.replace('void Drive_AbsMove(sMotionCommand_t*p){lastMove=*p;moveCount++;}', 'void model_abs(sMotionCommand_t*p); void Drive_AbsMove(sMotionCommand_t*p){lastMove=*p;moveCount++;model_abs(p);}')
code=code.replace('void Drive_Stop(sMotionCommand_t*p){stopCount++;}', 'void model_stop(U8 ch); void Drive_Stop(sMotionCommand_t*p){stopCount++;model_stop(p->Axis);}')
code+=ns['function'](ns['module'],'CMD_Handle_LongRun')
code+=ns['function'](ns['read']('App/System/XSystem_DB.c'),'SystemDB_PL_Init_Factory')
code+=r'''
typedef struct {S32 target; int speed,delay; bool active;} Axis;
static Axis axes[2];
static int mode, startDelay, overwritten, prematureGrip, io[16], rMoves;
static S32 rDistance[8];
static int reached[2][32];
void model_io(U8 ch,U8 value){
 if(ch==AIR_CT_CAP_GRIP_PIN && io[ch]!=value && (axes[0].active||axes[1].active)) prematureGrip++;
 io[ch]=value;
 /* 기본 모형은 출력 후 도착한다. 지연/미도착은 test_gripper_waits.py에서 검사한다. */
 if(ch==AIR_CT_CAP_GRIP_PIN || ch==AIR_CT_CAP_UNGRIP_PIN){
  xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Open=!io[AIR_CT_CAP_GRIP_PIN] && io[AIR_CT_CAP_UNGRIP_PIN];
  xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Close=0;
 }
 if(ch==AIR_CT_BODY_GRIP_PIN || ch==AIR_CT_BODY_UNGRIP_PIN){
  xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open=!io[AIR_CT_BODY_GRIP_PIN] && io[AIR_CT_BODY_UNGRIP_PIN];
  xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Close=0;
 }
}
void model_abs(sMotionCommand_t*p){
 Axis *a=&axes[p->Axis];
 if(a->active && a->target!=p->Pos) overwritten++;
 if(p->Axis==1 && rMoves<8)rDistance[rMoves++]=p->Pos-position[1];
 a->target=p->Pos; a->speed=abs(p->Vel); a->delay=startDelay; a->active=(a->target!=position[p->Axis]);
}
void model_stop(U8 ch){axes[ch].active=false;}
static void tick(void){
 gTriggerCount++;
 for(int ch=0;ch<2;ch++){
  Axis *a=&axes[ch];
  if(a->active && a->delay>0) a->delay--;
  else if(a->active && mode==0){
   S32 amount=a->speed/100;if(amount<1)amount=1;
   int64_t diff=(int64_t)a->target-position[ch];
   if(llabs(diff)<=amount){position[ch]=a->target;a->active=false;}
   else position[ch]+=diff>0?amount:-amount;
  }
  U8 busy=a->active && a->delay==0 && mode!=2;
  if(ch==0)xSL.xZ_Motor_Run.Motor_Run=busy;else xSL.xR_Motor_Run.Motor_Run=busy;
 }
 CDecap_Action_Statemachine(xAT);
 if(sPhase==DFSM_LONGRUN && CDecapping_Step<32) reached[xCD.Decapper.phaseDecapCap][CDecapping_Step]++;
}
static void setup(void){
 memset(axes,0,sizeof axes);memset(io,0,sizeof io);memset(reached,0,sizeof reached);
 mode=startDelay=overwritten=prematureGrip=rMoves=0;reset();
 xPL.Decapper.ZDecapAcc=300000;xPL.Decapper.ZDecapVel=20000;
 xPL.Decapper.RDecapAcc=300000;xPL.Decapper.RDecapVel=20000;
 xPL.Decapper.ZCap_SidePos=4000;xPL.Decapper.ZCap_UpPos=2000;
 xPL.Decapper.ZCap_Origin_Position=0;xPL.Decapper.RDecapPos=-1000;
 xSL.isHomed=true;xSL.CDecapping_Sensor.CT_Detect_Sensor=1;
}
static void start(void){tsXParsedData cmd={0};CMD_Handle_LongRun(&cmd,0);}
static void run(unsigned count,unsigned ticks){for(unsigned n=0;n<ticks && !xSL.isError && xCD.Decapper.LongRunCount<count;n++)tick();}
static void result(const char*name,bool ok){printf("%s | %s | count=%lu error=%d Z=%ld R=%ld overwrite=%d earlyGrip=%d\n",ok?"PASS":"FAIL",name,(unsigned long)xCD.Decapper.LongRunCount,errorCode,(long)position[0],(long)position[1],overwritten,prematureGrip);}

static void begin_unit(teXActionType action){CDecap_TryRequestAction(action);}
static void finish_unit(void){for(int n=0;n<3000 && !xSL.isError;n++){tick();if(sPhase==DFSM_IDLE && xAT==ACTION_NONE)break;}}
int main(void){
 setbuf(stdout,NULL);
 setup();position[1]=2500;start();run(1000,3000000);
 CHECK("1000 cycles return R to HOME zero from a nonzero start",xCD.Decapper.LongRunCount==1000 && !xSL.isError && position[1]==0);
 bool equal=true;for(int i=0;i<rMoves;i++)if(rDistance[i]!=(i==0?-3500:(i%2?1000:-1000)))equal=false;
 CHECK("DECAP/CAP use fixed absolute endpoints without accumulating",equal && rMoves==8);
 setup();mode=2;start();run(3,3000);
 CHECK("stopped feedback without arrival times out before grip/count",xCD.Decapper.LongRunCount==0 && errorCode==ERROR_CODE_DECAP_TIMEOUT && io[AIR_CT_CAP_GRIP_PIN]==0 && !overwritten);
 setup();startDelay=3;start();run(10,30000);
 CHECK("delayed start cannot overwrite target or change grip early",xCD.Decapper.LongRunCount==10 && !xSL.isError && !overwritten && !prematureGrip && position[1]==0);
 setup();mode=1;start();run(1,1000);
 CHECK("running feedback stuck faults with no count",errorCode==ERROR_CODE_DECAP_TIMEOUT && xCD.Decapper.LongRunCount==0);
 setup();xPL.Decapper.ZCap_SidePos=xPL.Decapper.ZCap_UpPos=0;xPL.Decapper.RDecapPos=0;start();run(5,10000);
 CHECK("already at target completes without observing motor ON",xCD.Decapper.LongRunCount==5 && !xSL.isError);
 setup();position[1]=4000;begin_unit(ACTION_UCAP);finish_unit();begin_unit(ACTION_UCAP);finish_unit();
 CHECK("repeated standalone CAP stays at zero",position[1]==0 && !xSL.isError);
 begin_unit(ACTION_UDECAP);finish_unit();begin_unit(ACTION_UDECAP);finish_unit();
 CHECK("repeated standalone DECAP stays at taught target",position[1]==-1000 && !xSL.isError);
 setup();xPL.Decapper.RDecapPos=8388608;begin_unit(ACTION_UDECAP);finish_unit();
 CHECK("absolute R endpoint beyond the TMC429 24-bit range faults",errorCode==ERROR_CODE_DECAP_LIMIT && rMoves==0);
 setup();xPL.Decapper.RDecapPos=INT32_MIN;begin_unit(ACTION_UDECAP);finish_unit();
 CHECK("invalid signed R target cannot enter a valid goal",errorCode==ERROR_CODE_DECAP_LIMIT && rMoves==0);
 setup();mode=2;xCD.Decapper.Motor_TargetPos[0]=500;begin_unit(ACTION_AMOVEZ);finish_unit();
 CHECK("manual absolute move needs arrival and times out",errorCode==ERROR_CODE_DECAP_TIMEOUT);
 setup();mode=2;xCD.Decapper.Motor_TargetPos[0]=500;begin_unit(ACTION_RMOVEZ);finish_unit();
 CHECK("manual relative move needs arrival and times out",errorCode==ERROR_CODE_DECAP_TIMEOUT);
 setup();startDelay=3;position[1]=4000;begin_unit(ACTION_DECAP);finish_unit();
 CHECK("automatic DECAP reaches taught absolute endpoint",!xSL.isError && position[1]==-1000 && !overwritten && !prematureGrip);
 begin_unit(ACTION_CAP);finish_unit();
 CHECK("automatic CAP returns to HOME zero",!xSL.isError && position[1]==0 && !overwritten && !prematureGrip);
 for(int automatic=0;automatic<2;automatic++){
  setup();position[aR]=-1000;xPL.Decapper.ZCap_SidePos=160001;
  begin_unit(automatic?ACTION_CAP:ACTION_UCAP);finish_unit();
  CHECK("invalid CAP Z endpoint is rejected before the paired R command",
        errorCode==ERROR_CODE_DECAP_LIMIT && rMoves==0 && position[aR]==-1000);
  setup();position[aR]=0;xPL.Decapper.ZCap_UpPos=160001;
  begin_unit(automatic?ACTION_DECAP:ACTION_UDECAP);finish_unit();
  CHECK("invalid DECAP Z endpoint is rejected before the paired R command",
        errorCode==ERROR_CODE_DECAP_LIMIT && rMoves==0 && position[aR]==0);
 }
 setup();position[aR]=0;xPL.Decapper.RDecapPos=8388608;
 begin_unit(ACTION_UDECAP);finish_unit();
 CHECK("invalid R endpoint also blocks the paired Z move",
       errorCode==ERROR_CODE_DECAP_LIMIT && rMoves==0 && moveCount==1);
 setup();startDelay=3;xCD.Decapper.Motor_TargetPos[0]=500;begin_unit(ACTION_AMOVEZ);finish_unit();
 CHECK("manual absolute valid move completes",!xSL.isError && position[0]==500 && sPhase==DFSM_IDLE);
 xCD.Decapper.Motor_TargetPos[0]=-200;begin_unit(ACTION_RMOVEZ);finish_unit();
 CHECK("manual relative valid move completes at computed goal",!xSL.isError && position[0]==300 && sPhase==DFSM_IDLE);
 setup();mode=1;xCD.Decapper.Motor_TargetPos[0]=500;begin_unit(ACTION_AMOVEZ);tick();position[0]=500;finish_unit();
 CHECK("position alone is insufficient while running feedback stays ON",errorCode==ERROR_CODE_DECAP_TIMEOUT);
 int cases=0,failed=0;
 for(int delay=0;delay<=3;delay+=3)for(int ph=0;ph<2;ph++)for(int st=0;st<20;st++){
  setup();startDelay=delay;start();bool found=false;
  for(int n=0;n<3000 && xCD.Decapper.LongRunCount<2;n++){tick();if(sPhase==DFSM_LONGRUN && xCD.Decapper.phaseDecapCap==ph && CDecapping_Step==st){found=true;break;}}
  if(!found)continue;
  cases++;tsXParsedData cmd={0};CMD_Handle_PAUSE(&cmd,0);tick();
  int beforeMoves=moveCount;for(int n=0;n<1000;n++)tick();
  bool quiet=(moveCount==beforeMoves && !xSL.isError && PauseContext.isPaused);
  CMD_Handle_RESUME(&cmd,0);tick();run(3,5000);
  if(!(quiet && xCD.Decapper.LongRunCount==3 && !xSL.isError && !overwritten && !prematureGrip && position[1]==0)){
   failed++;printf("DETAIL pause failure delay=%d phase=%d step=%d count=%lu error=%d R=%ld\n",delay,ph,st,(unsigned long)xCD.Decapper.LongRunCount,errorCode,(long)position[1]);
  }
 }
 /* LR은 자동 DECAP/CAP 전체 시퀀스: Body 해제확인·Y·Body 단계 + CAP Up 대기 포함 13+13단계 x 지연 2종 = 52 */
 CHECK("PAUSE/RESUME in 52 phase/step/delay cases preserves remaining motion",cases==52 && failed==0);
 setup();gTriggerCount=UINT32_MAX-80;start();run(3,5000);
 CHECK("tick wrap preserves target tracking",xCD.Decapper.LongRunCount==3 && !xSL.isError);
 setup();start();for(int n=0;n<150;n++)tick();tsXParsedData cmd={0};CMD_Handle_STOP(&cmd,0);CMD_Handle_RESUME(&cmd,0);tick();
 int beforeMoves=moveCount;for(int n=0;n<100;n++)tick();
 CHECK("STOP cannot be undone by RESUME",sPhase==DFSM_IDLE && moveCount==beforeMoves && !xSL.isBusy);
 setup();cmd.ParamCount=1;cmd.Params[0].type=PARAM_TYPE_INT;cmd.Params[0].value._int=3;CMD_Handle_LongRun(&cmd,0);
 for(int n=0;n<10000;n++){tick();if(sPhase==DFSM_IDLE)break;}
 CHECK("finite LR ends exactly after full CAP cycle",xCD.Decapper.LongRunCount==3 && sPhase==DFSM_IDLE && !xSL.isBusy && position[1]==0);
 /* LR 절반 사이 5초(LR_INTERVAL_MS) 대기: 출력·모터 변화 없음, PAUSE 시간은 대기에서 제외 */
 for(int paused=0;paused<2;paused++){
  setup();start();
  for(int n=0;n<5000 && xCD.Decapper.phaseDecapCap!=LONGRUN_CAP && !xSL.isError;n++) tick();
  int moves=moveCount;int snap[16];memcpy(snap,io,sizeof io);
  U32 interval=MS_TO_TICKS(LR_INTERVAL_MS);U32 waited=0;
  if(paused){
   tick();waited++;
   tsXParsedData c={0};CMD_Handle_PAUSE(&c,0);
   for(int n=0;n<500;n++)tick();
   CMD_Handle_RESUME(&c,0);
  }
  bool quiet=true;
  while(waited<interval-1){tick();waited++;if(moveCount!=moves||memcmp(snap,io,sizeof io)||CDecapping_Step!=CDecapping_Idle)quiet=false;}
  int started=0;for(int n=0;n<5 && !started;n++){tick();if(memcmp(snap,io,sizeof io)||moveCount!=moves||CDecapping_Step!=CDecapping_Idle)started=1;}
  CHECK(paused?"PAUSE during the LR rest excludes the paused time and keeps outputs still":
               "LR rests 1 s between DECAP and CAP with no output or motor change",
        quiet && started && !xSL.isError && xCD.Decapper.phaseDecapCap==LONGRUN_CAP);
 }
 /* LRCOUNT 알림: 사이클 완료마다 정확한 횟수가 한 번씩만 나오고, 새 LR 시작 시 이전 알림은 버린다 */
 {
  setup();start();U32 got=0,expect=1;bool ok=true;
  for(int n=0;n<60000 && xCD.Decapper.LongRunCount<3 && !xSL.isError;n++){
   tick();U32 c;
   if(CDecap_TakeLongRunReport(&c)){ if(c!=expect||c!=xCD.Decapper.LongRunCount)ok=false; expect++;got++; }
   if(CDecap_TakeLongRunReport(&c))ok=false; /* 같은 알림이 두 번 나오면 안 됨 */
  }
  CHECK("each completed LR cycle reports its count exactly once (LRCOUNT 1,2,3)",ok && got==3 && !xSL.isError);
  while(xCD.Decapper.LongRunCount<4 && !xSL.isError) tick();          /* 4번째 알림은 꺼내지 않음 */
  tsXParsedData c={0};CMD_Handle_STOP(&c,0);tick();tick();
  start();tick();U32 stale;
  CHECK("a new LR discards an unsent report from the previous run",!CDecap_TakeLongRunReport(&stale) && !xSL.isError);
 }
 return failures?1:0;
}
'''
code=code.replace('void Drive_RelMove(sMotionCommand_t*p){lastMove=*p;moveCount++;}', 'void model_abs(sMotionCommand_t*p); void Drive_RelMove(sMotionCommand_t*p){lastMove=*p;moveCount++;sMotionCommand_t a=*p;a.Pos=position[p->Axis]+p->Pos;model_abs(&a);}')
with tempfile.TemporaryDirectory(prefix='vial-longrun-regression-') as work:
    source=pathlib.Path(work)/'test.c';exe=pathlib.Path(work)/'test.exe'
    source.write_text(code,encoding='utf-8')
    subprocess.run([os.environ['ZIG_EXE'],'cc','-std=c11','-Wno-format',str(source),'-o',str(exe)],check=True)
    sys.exit(subprocess.run([str(exe)]).returncode)
