"""DO must not touch outputs or SPI1 while the Decapper is not idle (real CMD_Handle_DO + FSM)."""
import os, pathlib, subprocess, sys, tempfile
base = pathlib.Path(__file__).resolve().parent
ns = {'__file__': str(base / 'test_motion.py')}
exec((base / 'test_motion.py').read_text(encoding='utf-8').split('# COMPILE_AND_RUN')[0], ns)
code = ns['code'].split('int main(void)')[0]
code = code.replace('void IOEXP_WriteIObit(U8 a,U8 b){}', 'static int ioWrites; void IOEXP_WriteIObit(U8 a,U8 b){ioWrites++;}')
code += r'''
#define XHW_PIN_DO_NUM 16
#define XHW_PIN_DO_FIRST 0
#define XHW_PIN_DO_END 15
#define FOR_ALL_DO for (int i = XHW_PIN_DO_FIRST; i <= XHW_PIN_DO_END; i++)
#define HIGH 1
#define LOW 0
#define READ_OUT 1
static struct {int isLoopbackEnable_Digital;} xHwTest;
static int clears, initCalls;
static void IOEXP_WriteIOclear(void){clears++;}
static void IOEXP_Init(void){initCalls++;}
static void vTaskDelay(U32 t){}
static void digitalWrite(int ch,int v){ioWrites++;}
static void IOEXP_GetDigitalOutput_str(char*s,int n){s[0]=0;}
static void __newLine(void){}
'''
code += ns['function'](ns['read']('App/Task/_10_XCommand_Core.c'), 'CMD_Handle_DO')
code += r'''
static tsXParsedData args(int n,int a,int b){tsXParsedData p={0};p.ParamCount=n;
 p.Params[0].type=p.Params[1].type=PARAM_TYPE_INT;p.Params[0].value._int=a;p.Params[1].value._int=b;return p;}
static void clearCounts(void){ioWrites=clears=initCalls=0;response[0]=0;}
int main(void){
 tsXParsedData p;
 reset();clearCounts();p=args(2,3,1);CMD_Handle_DO(&p,0);
 CHECK("idle DO ch,val writes the output",ioWrites==1 && strcmp(response,"BUSY"));
 reset();xSL.isError=1;clearCounts();p=args(2,3,1);CMD_Handle_DO(&p,0);
 CHECK("DO stays available in a stopped fault state for diagnosis",ioWrites==1);
 const char*names[]={"running LR","paused LR","maintenance"};
 for(int mode=0;mode<3;mode++){
  reset();
  if(mode<2){xSL.isHomed=1;xSL.CDecapping_Sensor.CT_Detect_Sensor=1;CDecap_TryLongRun(0);CDecap_Action_Statemachine(xAT);}
  if(mode==1){xAT=ACTION_PAUSE;CDecap_Action_Statemachine(xAT);CDecap_Action_Statemachine(ACTION_NONE);}
  if(mode==2) CDecap_BeginMaintenance();
  xHwTest.isLoopbackEnable_Digital=1;
  int cases[][3]={{0,0,0},{1,3,0},{1,99,0},{1,100,0},{2,3,1}};
  bool blocked=true;
  for(int c=0;c<5;c++){
   clearCounts();p=args(cases[c][0],cases[c][1],cases[c][2]);CMD_Handle_DO(&p,0);
   blocked &= !strcmp(response,"BUSY") && ioWrites==0 && clears==0 && initCalls==0;
  }
  char name[80];snprintf(name,sizeof name,"DO is BUSY with no output or SPI access during %s",names[mode]);
  CHECK(name,blocked && xHwTest.isLoopbackEnable_Digital==1);
  if(mode==2) CDecap_EndMaintenance();
 }
 return failures?1:0;
}
'''
zig = os.environ.get('ZIG_EXE')
if not zig: sys.exit('Set ZIG_EXE to a Zig C compiler (tested with Zig 0.13.0).')
with tempfile.TemporaryDirectory(prefix='vial-do-guard-') as work:
    source = pathlib.Path(work) / 'test.c'; exe = source.with_suffix('.exe')
    source.write_text(code, encoding='utf-8')
    subprocess.run([zig, 'cc', '-std=c11', '-Wno-format', str(source), '-o', str(exe)], check=True)
    sys.exit(subprocess.run([str(exe)]).returncode)
