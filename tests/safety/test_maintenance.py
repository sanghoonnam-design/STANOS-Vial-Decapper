import pathlib,sys,subprocess,os,shutil,tempfile
b=pathlib.Path(__file__).resolve().parent
ns={'__file__':str(b/'test_motion.py')}
exec((b/'test_motion.py').read_text(encoding='utf-8').split('# COMPILE_AND_RUN')[0],ns)
code=ns['code'].split('static int failures;')[0]
code+=r'''
static int timerStops,timerStarts,saves,httpStatus,ipChanges;
static bool saveOK=true;
static bool loadOK=true,lockAtTimerStart;
static int ramUpdates;
int gEEPROM;
bool gZCapUpPosSavePending; S32 gZCapUpPosPendingValue;
#define XTimer_Stop() (timerStops++)
#define IS_USB(c) ((c)==0)
#define IS_CMD_HELP(p) ((p)->ParamCount==1 && (p)->Params[0].value._int=='?')
#define __newLine() ((void)0)
#define XTimer_Start() (timerStarts++,lockAtTimerStart=CDecap_IsMaintenanceActive())
U32 SWRTC_GetTime_YYMMDDHH(void){return 0;}
void EEPROMPL_SaveToEEPROM(void){saves++;}
bool EEPROMPL_SaveVerified(void){saves++;return saveOK;}
bool EEPROMPL_LoadFromEEPROM(void*p){return loadOK;}
void SYSPL_UpdateSystemParams(void){ramUpdates++;}
struct {struct {int ip[4];int portNum;} network;} xSystemInfo;
void SystemInfo_Network_SetIP(void*n,U08*ip){ipChanges++;}
void XBuffer_AddInt(void*b,int v,int c){}
void SYSPL_FactorySetting(void){saves++;}
typedef struct {void *resp;} http_request_ctx_t;
void http_response_set(void*r,int status,const char*t,const char*b,int n){httpStatus=status;}
'''
read=ns['read'];fn=ns['function']
core=read('App/Task/_10_XCommand_Core.c');http=read('App/HW_Device/httpServer/user/http_post.c')
code+=fn(core,'CMD_BeginMaintenance')+fn(core,'CMD_Handle_SAVEE')+fn(core,'CMD_Handle_SetGetIP')+fn(core,'CMD_Handle_LOADE')
code+=fn(http,'http_begin_maintenance')+fn(http,'http_post_factorySet')
code+=r'''
static int failures;
#define CHECK(test,condition) do{if(!(condition)){printf("FAIL %s\n",test); failures++;}else printf("PASS %s\n",test);}while(0)
int main(void){tsXParsedData cmd={0};http_request_ctx_t ctx={0};
 CDecap_Init();xSL.isBusy=1;CMD_Handle_SAVEE(&cmd,0);
 CHECK("busy SAVEE never stops timer or writes",timerStops==0 && saves==0);
 timerStops=saves=0;xSL.isBusy=0;xAT=ACTION_HOME;CMD_Handle_SAVEE(&cmd,0);
 CHECK("queued motion blocks SAVEE",timerStops==0 && saves==0);
 timerStops=saves=0;xAT=ACTION_NONE;xSL.isBusy=1;http_post_factorySet(&ctx);
 CHECK("busy HTTP factory returns conflict without timer stop",httpStatus==409 && timerStops==0 && saves==0);
 timerStops=saves=0;xSL.isBusy=0;xAT=ACTION_NONE;CMD_Handle_SAVEE(&cmd,0);
 CHECK("idle save succeeds and releases maintenance",saves==1 && !CDecap_IsMaintenanceActive());
 CDecap_BeginMaintenance();
 CHECK("maintenance atomically excludes new motion",!CDecap_TryRequestAction(ACTION_HOME));CDecap_EndMaintenance();
 timerStops=timerStarts=0;cmd.ParamCount=4;for(int n=0;n<4;n++){cmd.Params[n].type=PARAM_TYPE_INT;cmd.Params[n].value._int=1;}cmd.Params[3].value._int=256;
 CMD_Handle_SetGetIP(&cmd,0);
 CHECK("invalid IP never stops timer",timerStops==0 && timerStarts==0 && !CDecap_IsMaintenanceActive());
 cmd.Params[3].value._int=100;xSL.isBusy=1;CMD_Handle_SetGetIP(&cmd,0);
 CHECK("busy IP setter cannot stop timer",timerStops==0 && ipChanges==0);
 xSL.isBusy=0;CMD_Handle_SetGetIP(&cmd,0);
 CHECK("valid IP restores timer and releases lock",timerStops==1 && timerStarts==1 && ipChanges==1 && !CDecap_IsMaintenanceActive());
 cmd.ParamCount=1;cmd.Params[0].value._int=1;saveOK=false;response[0]=0;CMD_Handle_SAVEE(&cmd,0);
 CHECK("failed save reports failure and releases lock",strstr(response,"FAIL") && !CDecap_IsMaintenanceActive() && timerStops==timerStarts);
 saveOK=true;gZCapUpPosSavePending=true;gZCapUpPosPendingValue=999;xPL.Decapper.ZCap_UpPos=123;response[0]=0;CMD_Handle_SAVEE(&cmd,0);
 CHECK("explicit save ignores other transport RPOS candidate",xPL.Decapper.ZCap_UpPos==123 && !strcmp(response,"OK"));
 cmd.ParamCount=0;lockAtTimerStart=false;xSL.isHomed=true;gZCapUpPosSavePending=true;CMD_Handle_LOADE(&cmd,0);
 CHECK("LOAD applies RAM and invalidates home without releasing lock early",ramUpdates==1 && !xSL.isHomed && lockAtTimerStart && !CDecap_IsMaintenanceActive() && !gZCapUpPosSavePending);
 loadOK=false;xSL.isHomed=true;CMD_Handle_LOADE(&cmd,0);
 CHECK("failed LOAD leaves active RAM and home unchanged",ramUpdates==1 && xSL.isHomed && !strcmp(response,"FAIL") && timerStops==timerStarts && !CDecap_IsMaintenanceActive());
 return failures?1:0;
}
'''
zig=os.environ.get('ZIG_EXE') or shutil.which('zig')
if not zig:sys.exit('Set ZIG_EXE to a Zig C compiler (tested with Zig 0.13.0).')
with tempfile.TemporaryDirectory(prefix='vial-maintenance-') as work:
    p=pathlib.Path(work)/'maintenance_test.c';p.write_text(code,encoding='utf-8');exe=p.with_suffix('.exe')
    r=subprocess.run([zig,'cc','-std=c11','-Wno-format','-o',str(exe),str(p)])
    result=r.returncode if r.returncode else subprocess.run([str(exe)]).returncode
sys.exit(result)
