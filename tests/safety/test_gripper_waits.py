"""Run the actual Decapper FSM with controlled gripper sensor endpoints."""
from pathlib import Path
import os
import subprocess
import sys
import tempfile

base = Path(__file__).resolve().parent
root = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else base.parents[1]
helper = root / "tests/safety/test_motion.py"
namespace = {"__file__": str(helper)}
exec(helper.read_text(encoding="utf-8").split("# COMPILE_AND_RUN")[0], namespace)
code = namespace["code"].split("int main(void)")[0]
code = code.replace("void IOEXP_WriteIObit(U8 a,U8 b){}", "static unsigned outputs; void IOEXP_WriteIObit(U8 a,U8 b){if(b)outputs|=1u<<a;else outputs&=~(1u<<a);}")
custom = "--custom" in sys.argv
if custom:
    override = "#undef BODY_GRIP_WAIT_MS\n#define BODY_GRIP_WAIT_MS 155U\n#undef CAP_UNGRIP_WAIT_MS\n#define CAP_UNGRIP_WAIT_MS 255U\n#undef CAP_GRIP_WAIT_MS\n#define CAP_GRIP_WAIT_MS 355U\n"
    source_start = "\ntsxCDcap xCDecap;\n"
    assert code.count(source_start) == 1
    code = code.replace(source_start, "\n" + override + source_start, 1)
code += "\n#define EXPECT_BODY_TICKS " + ("16" if custom else "100")
code += "\n#define EXPECT_CAP_TICKS " + ("26" if custom else "100")
code += "\n#define EXPECT_UPDWELL_TICKS 100"
code += "\n#define EXPECT_ENGAGE_TICKS " + ("36" if custom else "100") + "\n"
code += r'''
static void prepare(void){
    outputs=0;reset();Step_StartTick=gTriggerCount;
    xSL.isHomed=1;
    xSL.CDecapping_Sensor.CT_Detect_Sensor=1;
    xPL.Decapper.ZCap_Origin_Position=500;
}
static void frame(void){CDecap_Action_Statemachine(xAT);}
static void begin_body(teFSM_Decapper phase){
    prepare();sPhase=phase;xSL.isBusy=1;CDecapping_Step=CDecapping_Body_Grip;
    xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open=1;
    xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Close=0;
    frame();
}
static void begin_release(teFSM_Decapper phase){
    prepare();sPhase=phase;xSL.isBusy=1;CDecapping_Step=CDecapping_Move_PrevR;
    xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Open=0;
    xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Close=0;
    frame();
}
int main(void){
    for(int variant=0;variant<2;variant++){
        teFSM_Decapper phase=variant?DFSM_DECAP:DFSM_CAP;
        begin_body(phase);
        xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open=0;
        gTriggerCount=EXPECT_BODY_TICKS-1;frame();
        CHECK("Body grip minimum dwell blocks Z/R even with a ready sensor",moveCount==0 && (outputs&3u)==0);
        xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open=1;
        gTriggerCount=EXPECT_BODY_TICKS;frame();
        CHECK("Body Open still on blocks Z/R after dwell",moveCount==0 && (outputs&3u)==0);
        xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open=0;
        xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Close=1;frame();
        CHECK("Body Close on does not qualify as holding a vial",moveCount==0 && (outputs&3u)==0);
        xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Close=0;frame();
        CHECK("Body Open and Close both off start the Z approach with Y untouched",moveCount==1 && lastMove.Axis==aZ && (outputs&3u)==0);
        begin_body(phase);gTriggerCount=EXPECT_BODY_TICKS+500;frame();
        CHECK("missing Body confirmation faults without moving Z/R or Y",errorCode==2880 && moveCount==0 && (outputs&3u)==0);
        const char *detail = CDecap_GetFaultDetail();
        CHECK("Body hold fault shows observed and expected bits",
              detail && strstr(detail,"Open=1 Close=0") && strstr(detail,"expected 0/0"));
    }
    for(int variant=0;variant<2;variant++){
        teFSM_Decapper phase=variant?DFSM_UCAP:DFSM_CAP;
        begin_release(phase);
        xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Open=1;
        gTriggerCount=EXPECT_CAP_TICKS-1;frame();
        CHECK("Cap release minimum dwell blocks Z even with Open on",moveCount==0);
        xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Open=0;
        gTriggerCount=EXPECT_CAP_TICKS;frame();
        CHECK("Cap release waits for Open after dwell",moveCount==0);
        xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Open=1;
        xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Close=1;frame();
        CHECK("contradictory Cap sensors cannot release Z",moveCount==0);
        xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Close=0;frame();
        CHECK("Cap Open on and Close off permit one Z return",moveCount==1 && lastMove.Axis==aZ && lastMove.Pos==500);
        begin_release(phase);gTriggerCount=EXPECT_CAP_TICKS+500;frame();
        CHECK("missing Cap opening faults before any Z return",errorCode==2870 && moveCount==0);
        const char *detail = CDecap_GetFaultDetail();
        CHECK("Cap release fault shows observed and expected bits",
              detail && strstr(detail,"Open=0 Close=0") && strstr(detail,"expected 1/0"));
        begin_release(phase);CDecap_RequestStop();frame();
        gTriggerCount+=1000;frame();
        CHECK("STOP cancels Cap opening wait without a late Z move",sPhase==DFSM_IDLE && moveCount==0);
    }
    begin_release(DFSM_CAP);
    xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Open=1;
    xAT=ACTION_PAUSE;frame();
    gTriggerCount=700;frame();
    xAT=ACTION_RESUME;frame();
    gTriggerCount=700+EXPECT_CAP_TICKS-1;frame();
    CHECK("pause time is excluded from Cap minimum dwell",moveCount==0 && !xSL.isError);
    gTriggerCount++;frame();
    CHECK("resume finishes the remaining Cap dwell",moveCount==1 && !xSL.isError);
    prepare();gTriggerCount=UINT32_MAX-5;Step_StartTick=gTriggerCount;sPhase=DFSM_CAP;xSL.isBusy=1;
    CDecapping_Step=CDecapping_Move_PrevR;frame();
    xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Open=1;
    gTriggerCount+=EXPECT_CAP_TICKS-1;frame();
    CHECK("tick wrap preserves the Cap release dwell",moveCount==0);
    gTriggerCount++;frame();
    CHECK("tick wrap permits Z when the full dwell has elapsed",moveCount==1);
    for(int variant=0;variant<2;variant++){
        prepare();sPhase=variant?DFSM_UDECAP:DFSM_DECAP;xSL.isBusy=1;
        CDecapping_Step=CDecapping_sData_Decapping;
        xPL.Decapper.RDecapPos=1000;xPL.Decapper.ZCap_UpPos=2000;
        previousTime[SUB_CAPPING]=0;
        gTriggerCount=EXPECT_ENGAGE_TICKS-1;frame();
        CHECK("Cap grip minimum dwell delays rotation and Z movement",moveCount==0);
        xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Open=1;
        gTriggerCount++;frame();
        CHECK("Cap engagement keeps its existing dwell-only behavior",moveCount==2);
    }
    /* 새 순서: 작업 후 Body를 먼저 풀고, 열림 확인 또는 최소 시간 뒤에만 Y 전진 */
    for(int variant=0;variant<2;variant++){
        prepare();sPhase=variant?DFSM_DECAP:DFSM_CAP;xSL.isBusy=1;CDecapping_Step=CDecapping_Move_PrevY;
        outputs=(1u<<AIR_CT_BODY_GRIP_PIN);gTriggerCount=1000;Step_StartTick=gTriggerCount;frame();
        CHECK("Z return complete releases the Body before any Y output",
              !(outputs&(1u<<AIR_CT_BODY_GRIP_PIN)) && (outputs&(1u<<AIR_CT_BODY_UNGRIP_PIN)) && (outputs&3u)==0);
        gTriggerCount+=EXPECT_BODY_TICKS-1;frame();
        CHECK("Y Forward waits for the Body release dwell",(outputs&3u)==0);
        gTriggerCount++;frame();
        CHECK("Y Forward starts after the Body release dwell",(outputs&(1u<<Y_PIN_F_CONTROLLER)) && !(outputs&(1u<<Y_PIN_B_CONTROLLER)));
        prepare();sPhase=variant?DFSM_DECAP:DFSM_CAP;xSL.isBusy=1;CDecapping_Step=CDecapping_Move_PrevY;
        gTriggerCount=1000;Step_StartTick=gTriggerCount;frame();
        xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open=1;frame();
        CHECK("confirmed Body Open lets Y Forward start before the dwell",(outputs&(1u<<Y_PIN_F_CONTROLLER)) && !xSL.isError);
    }
    /* 새 순서: 시작 시 오류로 잡힌 채 남은 Body를 먼저 풀고 나서 Y 후진 */
    prepare();sPhase=DFSM_DECAP;xSL.isBusy=1;CDecapping_Step=CDecapping_Idle;
    outputs=(1u<<AIR_CT_BODY_GRIP_PIN);gTriggerCount=2000;frame();
    CHECK("start releases a held Body before pulling Y back",
          !(outputs&(1u<<AIR_CT_BODY_GRIP_PIN)) && (outputs&(1u<<AIR_CT_BODY_UNGRIP_PIN)) && (outputs&3u)==0);
    gTriggerCount+=EXPECT_BODY_TICKS-1;frame();
    CHECK("Y Back waits for the Body release dwell at start",(outputs&3u)==0);
    gTriggerCount++;frame();
    CHECK("Y Back starts after the release dwell, Body grip only after Y Back",
          (outputs&(1u<<Y_PIN_B_CONTROLLER)) && !(outputs&(1u<<AIR_CT_BODY_GRIP_PIN)) && moveCount==0);
    /* CAP: Z가 Up 위치에 도착한 뒤 CAP_UP_DWELL_MS 동안 회전·하강하지 않는다 */
    for(int variant=0;variant<2;variant++){
        prepare();sPhase=variant?DFSM_UCAP:DFSM_CAP;xSL.isBusy=1;CDecapping_Step=CDecapping_sData_Capping;
        xPL.Decapper.ZCap_UpPos=2000;xPL.Decapper.ZCap_SidePos=1000;position[aZ]=2000;position[aR]=500;
        gTriggerCount=5000;Step_StartTick=gTriggerCount;frame();
        CHECK("CAP Z arrival at Up starts the dwell without moving R/Z",moveCount==0 && CDecapping_Step==CDecapping_Cap_UpWait);
        gTriggerCount+=EXPECT_UPDWELL_TICKS-1;frame();
        CHECK("CAP Up dwell blocks rotation and descent",moveCount==0);
        gTriggerCount++;frame();
        CHECK("after the Up dwell R rotates to 0 and Z descends together",moveCount==2 && lastMove.Axis==aZ && lastMove.Pos==1000 && CDecapping_Step==CDecapping_Move_PrevR);
    }
    /* HOME: Z 원점 확정 뒤 Body를 먼저 풀고, 열림 확인 또는 최소 시간 뒤에만 Y 전진 */
    prepare();sPhase=DFSM_HOME;xSL.isBusy=1;xSL.isHomed=0;SET_STARTED(SUB_HOMEZ);HomingSeq.Step=CDECAP_HOMING_Y_HOME;
    outputs=(1u<<AIR_CT_BODY_GRIP_PIN);gTriggerCount=6000;frame();
    CHECK("HOME releases the Body before any Y output",
          !(outputs&(1u<<AIR_CT_BODY_GRIP_PIN)) && (outputs&(1u<<AIR_CT_BODY_UNGRIP_PIN)) && (outputs&3u)==0 &&
          HomingSeq.Step==CDECAP_HOMING_BODY_RELEASE);
    gTriggerCount+=EXPECT_BODY_TICKS-1;frame();
    CHECK("HOME Y Forward waits for the Body release dwell",(outputs&3u)==0);
    gTriggerCount++;frame();
    CHECK("HOME Y Forward starts after the Body release dwell",
          (outputs&(1u<<Y_PIN_F_CONTROLLER)) && !(outputs&(1u<<Y_PIN_B_CONTROLLER)) && HomingSeq.Step==CDECAP_HOMING_Y_WAIT);
    /* ORG: Z 0 복귀 → Body 해제(오류 후 잡힌 채 남은 경우) → 열림 확인 또는 최소 시간 → Y 전진 */
    prepare();position[aZ]=-50000;outputs=(1u<<AIR_CT_BODY_GRIP_PIN);gTriggerCount=3000;
    CDecap_Action_Statemachine(ACTION_ORIGIN);
    CHECK("ORG first returns Z to 0 without touching Body or Y",
          moveCount==1 && lastMove.Axis==aZ && lastMove.Pos==0 && (outputs&(1u<<AIR_CT_BODY_GRIP_PIN)) && (outputs&3u)==0);
    position[aZ]=0;frame();
    CHECK("ORG releases the Body after Z arrives, before any Y output",
          !(outputs&(1u<<AIR_CT_BODY_GRIP_PIN)) && (outputs&(1u<<AIR_CT_BODY_UNGRIP_PIN)) && (outputs&3u)==0);
    gTriggerCount+=EXPECT_BODY_TICKS-1;frame();
    CHECK("ORG Y Forward waits for the Body release dwell",(outputs&3u)==0);
    gTriggerCount++;frame();
    CHECK("ORG Y Forward starts after the Body release dwell",(outputs&(1u<<Y_PIN_F_CONTROLLER)) && !(outputs&(1u<<Y_PIN_B_CONTROLLER)));
    xSL.CDecapping_Sensor.Y_F_Limit_Sensor=0;xSL.CDecapping_Sensor.Y_B_Limit_Sensor=1;frame();frame();frame();
    CHECK("ORG completes at Y Forward with the Body released",
          CDecap_GetPhase()==DFSM_IDLE && CDecap_GetOperationResult()==2 && !xSL.isError && (outputs&3u)==0);
    prepare();position[aZ]=0;gTriggerCount=4000;CDecap_Action_Statemachine(ACTION_ORIGIN);frame();
    xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open=1;frame();
    CHECK("ORG with the Body already open moves Y without the dwell",(outputs&(1u<<Y_PIN_F_CONTROLLER)) && !xSL.isError);
    return failures?1:0;
}
'''
with tempfile.TemporaryDirectory(prefix="decapper-gripper-wait-") as directory:
    source = Path(directory) / "waits.c"
    exe = source.with_suffix(".exe")
    source.write_text(code, encoding="utf-8")
    subprocess.run([os.environ["ZIG_EXE"], "cc", "-std=c11", "-Wno-format", str(source), "-o", str(exe)], check=True)
    raise SystemExit(subprocess.run([str(exe)]).returncode)
