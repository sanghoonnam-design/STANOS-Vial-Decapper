"""Long-run soak: drives the real Decapper FSM through HOME and LR against a physical model.

The model places the Z High switch at the physical top of travel (HOME zero sits just past
its activation point), delays TMC2660 StandStill, delays motor start and gripper sensors with
a deterministic PRNG, and injects PAUSE/RESUME and faults. It never operates hardware.
Cycle count: LR_SOAK_CYCLES (default 50000).
"""
import pathlib, sys, os, subprocess, tempfile
base = pathlib.Path(__file__).resolve().parent
root = pathlib.Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else base.parents[1]
helper = root / 'tests/safety/test_motion.py'
ns = {'__file__': str(helper)}
exec(helper.read_text(encoding='utf-8').split('# COMPILE_AND_RUN')[0], ns)
code = ns['code'].split('int main(void)')[0]
cycles = int(os.environ.get('LR_SOAK_CYCLES', '50000'))

def swap(old, new):
    global code
    assert old in code, old
    code = code.replace(old, new)

swap('void TMC429_SetPosition(U8 a,S32 p){position[a]=p;}',
     'void model_setpos(U8 a,S32 p); void TMC429_SetPosition(U8 a,S32 p){model_setpos(a,p);}')
swap('S32 TMC429_GetVelocity(U8 a){return 0;}',
     'S32 model_velocity(U8 a); S32 TMC429_GetVelocity(U8 a){return model_velocity(a);}')
swap('void IOEXP_WriteIObit(U8 a,U8 b){}',
     'void model_io(U8 a,U8 b); void IOEXP_WriteIObit(U8 a,U8 b){model_io(a,b);}')
swap('void Drive_AbsMove(sMotionCommand_t*p){lastMove=*p;moveCount++;}',
     'void model_abs(U8 ch,S32 pos,S32 vel); void Drive_AbsMove(sMotionCommand_t*p){lastMove=*p;moveCount++;model_abs(p->Axis,p->Pos,p->Vel);}')
swap('void Drive_RelMove(sMotionCommand_t*p){lastMove=*p;moveCount++;}',
     'void model_abs(U8 ch,S32 pos,S32 vel); void Drive_RelMove(sMotionCommand_t*p){lastMove=*p;moveCount++;model_abs(p->Axis,position[p->Axis]+p->Pos,p->Vel);}')
swap('void Drive_Stop(sMotionCommand_t*p){stopCount++;}',
     'void model_stop(U8 ch); void Drive_Stop(sMotionCommand_t*p){stopCount++;model_stop(p->Axis);}')
swap('U8 TMC429_VelMove(U8 a,U32 b,S32 d){homeMoves++;homeVelNote(a,b);return 0;}',
     'void model_vel(U8 ch,S32 vel); U8 TMC429_VelMove(U8 a,U32 b,S32 d){homeMoves++;homeVelNote(a,b);model_vel(a,d);return 0;}')
swap('void TMC429_MotorStop(U8 a,U32 b){homeStops++;lastHomeAcc=b;}',
     'void TMC429_MotorStop(U8 a,U32 b){homeStops++;lastHomeAcc=b;model_stop(a);}')
code += ns['function'](ns['module'], 'CMD_Handle_LongRun')
code += r'''
/* ---------------- physical model ---------------- */
#define SOAK_CYCLES ''' + str(cycles) + r'''U
#define Z_HIGH_PHYS 0        /* High input active when physical Z >= this */
#define Z_LOW_PHYS (-150000) /* Low input active when physical Z <= this */
#define STANDSTILL_TICKS 7   /* TMC2660 StandStill flag needs ~65 ms without steps */
enum {M_IDLE,M_POS,M_VEL};
typedef struct {int mode; S32 target; S32 perTick; int dir; int startDelay; int coast; int standstill;} Axis;
static Axis ax[2];
static S32 offset[2];
static int io[16], capOpenDelay, capStuck, yDelay, highDead, bodyDelay;
static int yWhileMoving, bodyWhileMoving, axisOutsideBack, axisWithoutBody;
static int movingAtHold, idleAtRun, idleTicks; /* 전류: 움직이는 동안 유지 전류 / 대기 중 구동 전류 */
static unsigned long yBackStrokes; static int yMoveWithBodyHeld; /* 모든 동작 공통: Body 잡힌 채 Y 출력 */ static int io_prevB, yWithBody, gripBeforeBack;
static U32 rng=0x12345678u;
static U32 rnd(U32 n){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return n?rng%n:0;}
static int violations, overwrites, gripWhileMoving, dualGrip;
static S32 maxZ=INT32_MIN, minZ=INT32_MAX;
static unsigned long ticks, pauses, maxCycleTicks, cycleTicks;
static int maxStartDelay=2, capDelayMax=60;
static void violation(const char*what){if(violations<10)printf("VIOLATION tick=%lu %s count=%lu phase=%d step=%d\n",ticks,what,(unsigned long)xCD.Decapper.LongRunCount,(int)sPhase,(int)CDecapping_Step);violations++;}
void model_setpos(U8 a,S32 p){offset[a]+=position[a]-p;position[a]=p;}
S32 model_velocity(U8 a){return ax[a].mode==M_IDLE?0:ax[a].dir;}
static bool moving(int ch){return ax[ch].mode!=M_IDLE;}
void model_io(U8 ch,U8 value){
 bool axisMoving=moving(0)||moving(1);
 if((ch==AIR_CT_CAP_GRIP_PIN||ch==AIR_CT_CAP_UNGRIP_PIN) && io[ch]!=value && axisMoving) gripWhileMoving++;
 if((ch==Y_PIN_F_CONTROLLER||ch==Y_PIN_B_CONTROLLER) && io[ch]!=value && axisMoving && sPhase==DFSM_LONGRUN) yWhileMoving++;
 if((ch==AIR_CT_BODY_GRIP_PIN||ch==AIR_CT_BODY_UNGRIP_PIN) && io[ch]!=value && axisMoving && sPhase==DFSM_LONGRUN) bodyWhileMoving++;
 io[ch]=value;
 if(ch==AIR_CT_CAP_GRIP_PIN||ch==AIR_CT_CAP_UNGRIP_PIN) capOpenDelay=3+(int)rnd(capDelayMax);
 if(ch==AIR_CT_BODY_GRIP_PIN||ch==AIR_CT_BODY_UNGRIP_PIN) bodyDelay=3+(int)rnd(40);
 /* 순서 검사: Body를 잡은 채 Y를 움직이지 않고, Y Back 도착 전에는 Body를 잡지 않는다 */
 if((ch==Y_PIN_F_CONTROLLER||ch==Y_PIN_B_CONTROLLER) && value && io[AIR_CT_BODY_GRIP_PIN] && sPhase==DFSM_LONGRUN) yWithBody++;
 if(ch==AIR_CT_BODY_GRIP_PIN && value && !io[ch] && xSL.CDecapping_Sensor.Y_B_Limit_Sensor && sPhase==DFSM_LONGRUN) gripBeforeBack++;
 if(ch==Y_PIN_B_CONTROLLER && value && !io_prevB) yBackStrokes++;
 if(ch==Y_PIN_B_CONTROLLER) io_prevB=value;
 if(ch==Y_PIN_F_CONTROLLER||ch==Y_PIN_B_CONTROLLER) yDelay=10+(int)rnd(30);
}
void model_abs(U8 ch,S32 pos,S32 vel){
 Axis*a=&ax[ch];
 if(a->mode==M_POS && a->target!=pos) overwrites++;
 a->target=pos; a->perTick=abs(vel)/100; if(a->perTick<1)a->perTick=1;
 a->dir=pos>position[ch]?1:-1; a->coast=0;
 a->startDelay=(int)rnd(maxStartDelay+1);
 a->mode=(pos==position[ch])?M_IDLE:M_POS;
 motorCurrentRun[ch]=1; /* 실제 Drive_AbsMove/RelMove는 구동 전류로 올린다 */
}
void model_vel(U8 ch,S32 vel){
 Axis*a=&ax[ch]; a->mode=M_VEL; a->dir=vel>0?1:-1; a->perTick=abs(vel)*122/100; if(a->perTick<1)a->perTick=1;
 a->coast=0; a->startDelay=(int)rnd(maxStartDelay+1);
}
void model_stop(U8 ch){Axis*a=&ax[ch]; if(a->mode!=M_IDLE && !a->coast) a->coast=1+(int)rnd(2);} /* decel overshoot */
static void step_axis(int ch){
 Axis*a=&ax[ch];
 if(a->mode==M_IDLE){if(a->standstill)a->standstill--;return;}
 if(a->startDelay){a->startDelay--;return;}
 if(a->mode==M_POS){
  int64_t d=(int64_t)a->target-position[ch];
  if(llabs(d)<=a->perTick){position[ch]=a->target;a->mode=M_IDLE;a->standstill=STANDSTILL_TICKS;return;}
  position[ch]+=d>0?a->perTick:-a->perTick;
 } else position[ch]+=a->dir*a->perTick;
 if(a->coast && --a->coast==0){a->mode=M_IDLE;a->standstill=STANDSTILL_TICKS;}
}
static void tick(void){
 ticks++;cycleTicks++;gTriggerCount++;
 step_axis(0);step_axis(1);
 for(int ch=0;ch<2;ch++) motorFeedback[ch]=(ax[ch].mode!=M_IDLE && !ax[ch].startDelay) || ax[ch].standstill>0;
 S32 phys=position[0]+offset[0];
 if(phys>maxZ)maxZ=phys; if(phys<minZ)minZ=phys;
 /* UpdateSIG: active-low limit inputs, gripper and Y sensors */
 xSL.CDecapping_Sensor.Z_L_Limit_Sensor = (phys>=Z_HIGH_PHYS && !highDead) ? 0 : 1; /* 위쪽 = L(Z−, HOME) */
 xSL.CDecapping_Sensor.Z_H_Limit_Sensor = phys<=Z_LOW_PHYS ? 0 : 1; /* 아래쪽 = H(Z+) */
 if(capOpenDelay)capOpenDelay--;
 else {
  bool open=io[AIR_CT_CAP_UNGRIP_PIN] && !io[AIR_CT_CAP_GRIP_PIN];
  xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Open = open && !capStuck;
  xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Close = 0; /* a held cap keeps both off */
 }
 if(bodyDelay)bodyDelay--;
 else {
  xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open = io[AIR_CT_BODY_UNGRIP_PIN] && !io[AIR_CT_BODY_GRIP_PIN];
  xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Close = 0; /* a held vial keeps both off */
 }
 if(yDelay)yDelay--;
 else {
  if(io[Y_PIN_F_CONTROLLER] && !io[Y_PIN_B_CONTROLLER]){xSL.CDecapping_Sensor.Y_F_Limit_Sensor=0;xSL.CDecapping_Sensor.Y_B_Limit_Sensor=1;}
  if(io[Y_PIN_B_CONTROLLER] && !io[Y_PIN_F_CONTROLLER]){xSL.CDecapping_Sensor.Y_F_Limit_Sensor=1;xSL.CDecapping_Sensor.Y_B_Limit_Sensor=0;}
 }
 CDecapping_CheckSensorValidity();
 unsigned long before=xCD.Decapper.LongRunCount;
 CDecap_Action_Statemachine(xAT);
 for(int ch=0;ch<2;ch++) if(ax[ch].mode!=M_IDLE && !motorCurrentRun[ch]) movingAtHold++;
 if((sPhase==DFSM_IDLE||PauseContext.isPaused) && !motorFeedback[0] && !motorFeedback[1]){
  if(++idleTicks>3 && (motorCurrentRun[0]||motorCurrentRun[1])) idleAtRun++;
 } else idleTicks=0;
 if(io[AIR_CT_CAP_GRIP_PIN] && io[AIR_CT_CAP_UNGRIP_PIN]) dualGrip++;
 if((io[Y_PIN_F_CONTROLLER]||io[Y_PIN_B_CONTROLLER]) && io[AIR_CT_BODY_GRIP_PIN]) yMoveWithBodyHeld++;
 if(io[AIR_CT_BODY_GRIP_PIN] && io[AIR_CT_BODY_UNGRIP_PIN]) dualGrip++;
 if(sPhase==DFSM_LONGRUN && (moving(0)||moving(1))){
  if(xSL.CDecapping_Sensor.Y_B_Limit_Sensor) axisOutsideBack++;   /* Z/R work only at Y Back */
  if(!io[AIR_CT_BODY_GRIP_PIN]) axisWithoutBody++;                  /* vial body held during Z/R */
 }
 if(xCD.Decapper.LongRunCount>before){
  if(xCD.Decapper.LongRunCount!=before+1) violation("count jumped");
  if(cycleTicks>maxCycleTicks)maxCycleTicks=cycleTicks; cycleTicks=0;
 }
}
static void settle(int n){for(int i=0;i<n;i++)tick();}
static tsXParsedData noArgs(void){tsXParsedData p={0};return p;}
static void setup(U32 seed){
 reset(); rng=seed; memset(ax,0,sizeof ax); memset(io,0,sizeof io); offset[0]=offset[1]=0;
 useMotorFeedback=true; capStuck=highDead=0; capOpenDelay=yDelay=bodyDelay=0;
 violations=overwrites=gripWhileMoving=dualGrip=0; yWhileMoving=bodyWhileMoving=axisOutsideBack=axisWithoutBody=0; yBackStrokes=0; io_prevB=0; yMoveWithBodyHeld=0; yWithBody=gripBeforeBack=0; movingAtHold=idleAtRun=idleTicks=0; maxZ=INT32_MIN; minZ=INT32_MAX;
 ticks=pauses=maxCycleTicks=cycleTicks=0;
 position[0]=0; offset[0]=-20000;   /* power-on: physical Z is 20000 below High */
 position[1]=0; offset[1]=1234;
 xPL.Decapper.ZDecapAcc=300000;xPL.Decapper.ZDecapVel=20000;
 xPL.Decapper.RDecapAcc=300000;xPL.Decapper.RDecapVel=20000;
 xPL.Decapper.ZCap_SidePos=-40000;xPL.Decapper.ZCap_UpPos=-36000;
 xPL.Decapper.ZCap_Origin_Position=0;xPL.Decapper.RDecapPos=-64000;
 xSL.CDecapping_Sensor.Y_F_Limit_Sensor=0;xSL.CDecapping_Sensor.Y_B_Limit_Sensor=1; /* Y at Forward (loading) */
 xSL.CDecapping_Sensor.CT_Detect_Sensor=1; /* vial present */
}
static bool home(void){
 tsXParsedData p=noArgs(); CMD_Handle_HOME(&p,0);
 for(int i=0;i<3000;i++){tick(); if(sPhase==DFSM_IDLE && xAT==ACTION_NONE) break;}
 return xSL.isHomed && !xSL.isError;
}
static void startLR(U32 limit){
 tsXParsedData p={0};
 if(limit){p.ParamCount=1;p.Params[0].type=PARAM_TYPE_INT;p.Params[0].value._int=(int)limit;}
 response[0]=0; CMD_Handle_LongRun(&p,0);
}
static void report(const char*name){
 printf("  %s: count=%lu ticks=%lu (%.1f h simulated) pauses=%lu maxCycle=%lu ticks Zphys=[%ld,%ld] err=%d\n",
        name,(unsigned long)xCD.Decapper.LongRunCount,ticks,ticks/360000.0,pauses,maxCycleTicks,
        (long)minZ,(long)maxZ,errorCode);
 if(xSL.isError && CDecap_GetFaultDetail()) printf("  fault: %s\n",CDecap_GetFaultDetail());
}
static bool clean(void){
 if(movingAtHold||idleAtRun) printf("  current: movingAtHold=%d idleAtRun=%d\n",movingAtHold,idleAtRun);
 if(yWithBody||gripBeforeBack) printf("  order: yWithBody=%d gripBeforeBack=%d\n",yWithBody,gripBeforeBack);
 if(yWhileMoving||bodyWhileMoving||axisOutsideBack||axisWithoutBody)
  printf("  invariant: yWhileMoving=%d bodyWhileMoving=%d axisOutsideBack=%d axisWithoutBody=%d\n",yWhileMoving,bodyWhileMoving,axisOutsideBack,axisWithoutBody);
 return !violations && !overwrites && !gripWhileMoving && !dualGrip && !yWhileMoving && !bodyWhileMoving && !axisOutsideBack && !axisWithoutBody && !yWithBody && !gripBeforeBack && !movingAtHold && !idleAtRun;}
int main(void){
 setbuf(stdout,NULL);
 /* A: finite LR with HOME origin at Z=0, random latencies */
 setup(0xC0FFEEu);
 CHECK("HOME establishes Z zero on the High switch",home() && maxZ>=Z_HIGH_PHYS);
 CHECK("HOME passes the acceleration in TMC429 units (300000/3725=80), not raw pulse/s2",lastHomeAcc==MOTOR_ACC/A_MAX_BASE && lastHomeAcc==80);
 CHECK("every HOME Z velocity move starts at run current and Z ends at hold current",
       homeMovesAtHold==0 && runCurrentSets[aZ]>=3 && !motorCurrentRun[aZ]);
 startLR(SOAK_CYCLES);
 CHECK("finite LR accepted",!strcmp(response,"LR"));
 for(unsigned long i=0;i<(unsigned long)SOAK_CYCLES*4000UL && !xSL.isError;i++){tick();if(sPhase==DFSM_IDLE && xAT==ACTION_NONE)break;}
 report("A finite LR, origin 0");
 settle(20); /* 종료 후 대기 상태에서 정지가 확인되면 유지 전류로 내려가야 한다 */
 CHECK("A: both axes return to hold current after LR ends",!motorCurrentRun[aZ] && !motorCurrentRun[aR] && sPhase==DFSM_IDLE);
 CHECK("A: LR completes the requested cycles without fault",
       xCD.Decapper.LongRunCount==SOAK_CYCLES && !xSL.isError && sPhase==DFSM_IDLE && !xSL.isBusy && CDecap_GetOperationResult()==2);
 CHECK("A: no target overwrite, no grip change during motion, no dual grip",clean());
 CHECK("A: each LR cycle runs the full sequences (Y Back once per DECAP and once per CAP)",yBackStrokes==2UL*SOAK_CYCLES);
 CHECK("A: LR ends with Y Forward and the Body gripper released",!xSL.CDecapping_Sensor.Y_F_Limit_Sensor && !io[AIR_CT_BODY_GRIP_PIN] && io[AIR_CT_BODY_UNGRIP_PIN]);
 CHECK("A: R ends at zero and Z at origin",position[1]==0 && position[0]==0);
 CHECK("A: Z never travels past the High switch by more than the HOME overshoot",maxZ < Z_HIGH_PHYS+400);

 /* B: unlimited LR with random PAUSE/RESUME, then STOP */
 setup(0xBADC0DEu); maxStartDelay=3; capDelayMax=80;
 home(); startLR(0);
 unsigned long target=SOAK_CYCLES/5, nextPause=200+rnd(3000);
 for(unsigned long i=0;i<(unsigned long)target*5000UL && !xSL.isError && xCD.Decapper.LongRunCount<target;i++){
  tick();
  if(--nextPause==0){
   tsXParsedData p=noArgs(); response[0]=0; CMD_Handle_PAUSE(&p,0);
   if(!strcmp(response,"OK")||!strcmp(response,"PAUSE")||CDecap_IsPaused()||xAT==ACTION_PAUSE){
    settle(1+rnd(300));
    if(!CDecap_IsPaused()) violation("PAUSE accepted but not paused");
    p=noArgs(); CMD_Handle_RESUME(&p,0); pauses++;
   }
   nextPause=200+rnd(3000);
  }
 }
 { tsXParsedData p=noArgs(); CMD_Handle_STOP(&p,0); settle(20); }
 report("B unlimited LR with PAUSE/RESUME");
 CHECK("B: LR survives random PAUSE/RESUME",xCD.Decapper.LongRunCount>=target && !xSL.isError && pauses>=target/2);
 CHECK("B: STOP leaves the machine idle",sPhase==DFSM_IDLE && !xSL.isBusy && !moving(0) && !moving(1));
 CHECK("B: no target overwrite, no grip change during motion, no dual grip",clean());

 /* C: cap release sensor stuck mid-run -> E2870, then CLR/HOME/LR recovers */
 setup(0x5EED5EEDu); maxStartDelay=2; capDelayMax=60;
 home(); startLR(0);
 while(xCD.Decapper.LongRunCount<37 && !xSL.isError) tick();
 capStuck=1;
 for(int i=0;i<5000 && !xSL.isError;i++) tick();
 report("C fault injection");
 const char*detail=CDecap_GetFaultDetail();
 CHECK("C: stuck cap sensor stops LR with E2870 in the CAP half",
       errorCode==ERROR_CODE_DECAP_CAP_OPEN_NOT_CONFIRMED && detail && strstr(detail,"LR/CAP") && strstr(detail,"completed=37"));
 settle(20); capStuck=0; CDecap_Error_Clear();
 CHECK("C: CLR succeeds once stopped",!xSL.isError && errorCode==0);
 CHECK("C: HOME after the fault",home());
 startLR(200);
 for(int i=0;i<1000000 && !xSL.isError;i++){tick();if(sPhase==DFSM_IDLE && xAT==ACTION_NONE)break;}
 CHECK("C: LR resumes normally after recovery",xCD.Decapper.LongRunCount==200 && !xSL.isError && clean());

 /* D: origin taught below the switch, half speed */
 setup(0xD00Du); xPL.Decapper.ZCap_Origin_Position=-5000; CDecap_SetSpeedPercent(50);
 home(); startLR(SOAK_CYCLES/10);
 for(unsigned long i=0;i<(unsigned long)SOAK_CYCLES*800UL && !xSL.isError;i++){tick();if(sPhase==DFSM_IDLE && xAT==ACTION_NONE)break;}
 report("D origin -5000, 50% speed");
 CHECK("D: LR completes with a taught origin below High",xCD.Decapper.LongRunCount==SOAK_CYCLES/10 && !xSL.isError && clean() && position[0]==-5000);

 /* E: HOME from the bottom of travel needs longer than the fixed 5 s search window */
 setup(0xE0E0u); offset[0]=-140000;
 CHECK("E: HOME from 140000 pulses below High completes",home() && position[0]==0);
 /* F: High switch dead -> HOME search still times out with E2850 */
 setup(0xF00Du); offset[0]=-20000;
 { tsXParsedData p=noArgs(); CMD_Handle_HOME(&p,0); }
 highDead=1;
 for(int i=0;i<20000 && !xSL.isError;i++) tick();
 report("F dead High switch");
 CHECK("F: dead High switch still faults HOME within the computed window",
       errorCode==ERROR_CODE_DECAP_Z_HOME_NOT_REACHED && ticks<1300);
 /* H: HOME after a fault that left the Body gripped at Y Back -> Body opens before Y moves */
 setup(0x4848u);
 model_io(AIR_CT_BODY_GRIP_PIN,1);model_io(AIR_CT_BODY_UNGRIP_PIN,0);
 xSL.CDecapping_Sensor.Y_F_Limit_Sensor=1;xSL.CDecapping_Sensor.Y_B_Limit_Sensor=0;
 xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open=0;yMoveWithBodyHeld=0;
 CHECK("H: HOME with the Body held at Y Back completes",home() && !xSL.CDecapping_Sensor.Y_F_Limit_Sensor);
 CHECK("H: HOME never drives Y while the Body grip output is on",
       yMoveWithBodyHeld==0 && !io[AIR_CT_BODY_GRIP_PIN] && io[AIR_CT_BODY_UNGRIP_PIN]);
 /* G: vial removed between halves -> LR stops with E2820 instead of counting empty cycles */
 setup(0x6A6Au); home(); startLR(0);
 while(xCD.Decapper.LongRunCount<5 && !xSL.isError) tick();
 while(xCD.Decapper.phaseDecapCap!=LONGRUN_CAP && !xSL.isError) tick();      /* DECAP half of cycle 6 done */
 xSL.CDecapping_Sensor.CT_Detect_Sensor=0;
 for(int i=0;i<3000 && !xSL.isError;i++) tick();
 report("G vial removed mid-LR");
 detail=CDecap_GetFaultDetail();
 CHECK("G: missing CT stops LR with E2820 at the next half and keeps the count",
       errorCode==ERROR_CODE_DECAP_STATE && xCD.Decapper.LongRunCount==5 && detail && strstr(detail,"CT vial not detected"));
 return failures?1:0;
}
'''
with tempfile.TemporaryDirectory(prefix='vial-lr-soak-') as work:
    source = pathlib.Path(work) / 'soak.c'; exe = pathlib.Path(work) / 'soak.exe'
    source.write_text(code, encoding='utf-8')
    zig = os.environ.get('ZIG_EXE')
    if not zig: sys.exit('Set ZIG_EXE to a Zig C compiler (tested with Zig 0.13.0).')
    subprocess.run([zig, 'cc', '-std=c11', '-O2', '-Wno-format', str(source), '-o', str(exe)], check=True)
    sys.exit(subprocess.run([str(exe)]).returncode)
