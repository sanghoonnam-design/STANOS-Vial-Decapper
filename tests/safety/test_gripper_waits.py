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
        CHECK("Body grip minimum dwell blocks Y even with a ready sensor",(outputs&3u)==0);
        xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open=1;
        gTriggerCount=EXPECT_BODY_TICKS;frame();
        CHECK("Body Open still on blocks Y after dwell",(outputs&3u)==0);
        xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open=0;
        xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Close=1;frame();
        CHECK("Body Close on does not qualify as holding a vial",(outputs&3u)==0);
        xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Close=0;frame();
        CHECK("Body Open and Close both off permit Y Back",(outputs&3u)==2u);
        begin_body(phase);gTriggerCount=EXPECT_BODY_TICKS+500;frame();
        CHECK("missing Body confirmation faults without moving Y",errorCode==2880 && (outputs&3u)==0);
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
    return failures?1:0;
}
'''
with tempfile.TemporaryDirectory(prefix="decapper-gripper-wait-") as directory:
    source = Path(directory) / "waits.c"
    exe = source.with_suffix(".exe")
    source.write_text(code, encoding="utf-8")
    subprocess.run([os.environ["ZIG_EXE"], "cc", "-std=c11", "-Wno-format", str(source), "-o", str(exe)], check=True)
    raise SystemExit(subprocess.run([str(exe)]).returncode)
