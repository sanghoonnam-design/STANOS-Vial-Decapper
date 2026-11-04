"""Run the real UART command task and parser with a simulated byte queue."""
import os,pathlib,subprocess,tempfile
base=pathlib.Path(__file__).resolve().parent
scope={'__file__':str(base/'test_network.py')}
exec((base/'test_network.py').read_text(encoding='utf-8').split('# API to reset')[0],scope)
serial=scope['read']('App/Task/_10_XSerialCMD_Process.c')
serial=scope['strip'](serial[:serial.index('int Init_SerialCommand(')])
code=scope['prefix']+scope['strip'](scope['parser'])
code+=r'''
#define VOID void
#define FOREVER while(!finished)
#define COMM_RS232C 0
#define CR_LF_TIMEOUT_MS 5
#define pdMS_TO_TICKS(n) (n)
#define __TASK_TRIGGER_START_Using(...) ((void)0)
#define __TASK_TRIGGER_END_Using(...) ((void)0)
typedef unsigned TickType_t;typedef unsigned short U16;
typedef struct {int(*available)(void);int(*read)(void);} tsXSerial_cpu;
typedef struct {unsigned used;unsigned char data[1280];} tsXQueue;
static char incoming[2048];static unsigned cursor,length;static bool finished;
int available(void){return cursor<length;}int readByte(void){return incoming[cursor++];}
unsigned xQueue_GetSize(tsXQueue*q){return q->used;}
bool xQueue_Push(tsXQueue*q,char c){if(q->used>=sizeof(q->data))return false;q->data[q->used++]=c;return true;}
bool xQueue_Pop(tsXQueue*q,unsigned char*c){if(!q->used)return false;*c=q->data[0];memmove(q->data,q->data+1,--q->used);return true;}
void xQueue_Clear(tsXQueue*q){q->used=0;}
unsigned xTaskGetTickCount(void){return 0;}
void vTaskDelay(unsigned n){if(cursor>=length)finished=true;}
'''
code+=serial
code+=r'''
void Serial_ResetOnInvalidCommand(teXParsingErrorCode e,uint8_t pos){xQueue_Clear(&CommandQueue);while(available()){if(readByte()=='\n')break;}}
static void run(const char*data,unsigned size){memcpy(incoming,data,size);length=size;cursor=0;finished=false;received=0;_xSerial.available=available;_xSerial.read=readByte;TASK_SerialCommandLoop(0);}
int main(void){
 run("HOME\r\nSTOP\r\n",12);
 CHECK("UART CRLF executes real parser and consecutive commands",received==2 && !strcmp(commands[0],"HOME") && !strcmp(commands[1],"STOP"));
 char large[900];memset(large,'X',700);memcpy(large+700,"\r\nHOME\r\n",8);run(large,708);
 CHECK("oversized UART frame cannot overflow parser buffer and next frame recovers",received==1 && !strcmp(commands[0],"HOME"));
 return failures?1:0;
}
'''
with tempfile.TemporaryDirectory(prefix='vial-uart-') as work:
    source=pathlib.Path(work)/'test.c';exe=source.with_suffix('.exe')
    source.write_text(code,encoding='utf-8')
    subprocess.run([os.environ['ZIG_EXE'],'cc','-std=c11',str(source),'-o',str(exe)],check=True)
    raise SystemExit(subprocess.run([str(exe)],timeout=10).returncode)
