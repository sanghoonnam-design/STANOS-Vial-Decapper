"""Run the real diagnosis, sensor sampler and GSTA handler with simulated DI."""
import os, pathlib, subprocess, sys, tempfile

base = pathlib.Path(__file__).resolve().parent
root = pathlib.Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else base.parents[1]
motion = root / 'tests/safety/test_motion.py'
ns = {'__file__': str(motion)}
exec(motion.read_text(encoding='utf-8').split('# COMPILE_AND_RUN')[0], ns)
code = ns['code'].split('static int failures;')[0]
code = code.replace('U8 IOEXP_ReadIObit(U8 a,U8 b){return 1;}',
                    'static unsigned inputs; U8 IOEXP_ReadIObit(U8 a,U8 b){return (inputs >> b) & 1u;}')
code = code.replace('snprintf(response,sizeof response,"%s",s);',
                    'strncat(response,s,sizeof(response)-strlen(response)-1);if(comma)strcat(response,",");')
code += r'''
void XBuffer_AddInt(void *buf, int value, int comma){char s[24];snprintf(s,sizeof s,"%d",value);XBuffer_AddString(buf,s,comma);}
'''
code += ns['function'](ns['module'], 'CMD_Handle_GSTA')
header = ns['read']('App/HW_Device/CDecap.h')
code += '\n#define GripCap '+('isGripCap' if 'isGripCap' in header else 'Cap_is')+'\n'
code += '\n#define GripBody '+('isGripBody' if 'isGripBody' in header else 'Body_is')+'\n'
code += r'''
static int failures;
#define CHECK(label,expr) do{if(expr)puts("PASS " label);else{puts("FAIL " label);failures++;}}while(0)
/* Pin values come from the wired interface, independently of the implementation. */
static void sample(unsigned forward, unsigned back, unsigned ct, unsigned cap_open, unsigned cap_close){
    inputs = 3u | (forward<<2) | (back<<3) | (cap_open<<4) | (cap_close<<5) | (1u<<6) | (ct<<8);
    CDecap_Sensor_Update();
    CDecapping_CheckSensorValidity();
}
int main(void){
    tsXParsedData p = {0};
    sample(1,0,1,1,0);
    CHECK("initial Back CT does not establish vial presence", !xSL.Decapper.GripBody);
    sample(0,1,1,0,0);
    CHECK("Forward CT establishes vial presence even with Body Open", xSL.Decapper.GripBody);
    CHECK("Cap Open/Close both off reports cap held", xSL.Decapper.GripCap);
    xSL.isEnable=1; xSL.isHomed=1;
    CMD_Handle_GSTA(&p,0);
    CHECK("GSTA preserves first five fields and appends cap/body", !strcmp(response,"0,1,1,0,E0000,1,1"));
    sample(1,1,0,0,1);
    CHECK("moving away preserves vial detection despite CT off", xSL.Decapper.GripBody);
    CHECK("Cap Close on clears cap held", !xSL.Decapper.GripCap);
    sample(1,0,0,1,0);
    CHECK("Back CT off preserves vial detection", xSL.Decapper.GripBody);
    sample(0,0,0,1,1);
    CHECK("contradictory Y inputs preserve last vial reading", xSL.Decapper.GripBody);
    CHECK("existing Cap input contradiction still latches fault flag", xSL.Decapper.CT_Cap_Grip_isError);
    sample(0,1,0,1,0);
    CHECK("Forward with CT off clears vial presence", !xSL.Decapper.GripBody);
    response[0]=0; CMD_Handle_GSTA(&p,0);
    CHECK("GSTA publishes both flags cleared", !strcmp(response,"0,1,1,0,E0000,0,0"));
    return failures?1:0;
}
'''
with tempfile.TemporaryDirectory(prefix='decapper-grip-status-') as work:
    p = pathlib.Path(work)/'test.c'
    p.write_text(code, encoding='utf-8')
    exe = p.with_suffix('.exe')
    subprocess.run([os.environ['ZIG_EXE'],'cc','-std=c11','-Wno-format',str(p),'-o',str(exe)],check=True)
    raise SystemExit(subprocess.run([str(exe)]).returncode)
