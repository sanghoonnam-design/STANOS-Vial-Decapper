"""Compile actual parser/framing code against local hardware boundaries."""
import os, pathlib, re, subprocess, tempfile
root = pathlib.Path(__file__).resolve().parents[2]
def read(path):
    return (root / path).read_text(encoding="utf-8-sig")
def strip(text):
    return re.sub(r'^\s*#include[^\n]*', '', text, flags=re.M)
parser = read("App/Common/XParser.c").split("void xParser_HandleError(")[0]
network = read("App/Task/_10_XNetworkMsg_Process.c")
network = network[network.index("typedef struct") if "typedef struct" in network else network.index("#if 0"):]
prefix = r"""
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#define U08 uint8_t
#define NO 0
#define BOOL bool
#define UPCASE(c) toupper((unsigned char)(c))
#define xprintf(...) ((void)0)
#define COMM_TCP 1
#define __5sec 500
#define _INFO 0
#define _WARNING 1
#define ERROR_CODE_NONE 0
#define ERROR_CODE_INVALID_COMMAND 1
static int gTriggerCount=1000, received, failures;
static struct {int severity;} gCurrentError;
static char commands[16][20];
void SetErrorCode(int e,const char *f,int l){}
"""
prefix += strip(read("App/Common/XParser.h"))
prefix += r"""
void xParser_HandleError(const char *f,teXParsingErrorCode e,U08 n){}
static void xParser_PrintParsedData(const tsXParsedData *p){}
void Handle_command(const tsXParsedData *p,U08 comm){strcpy(commands[received++],p->Command);}
#define CHECK(n,c) do{bool passed=(c);printf("%s %s\n",passed?"PASS":"FAIL",n);if(!passed)failures++;}while(0)
"""
# API to reset connection state will be implemented; absence is a behavior failure.
reset = "Network_ResetReceive(0);" if "void Network_ResetReceive(" in network else ""
main = r"""
int main(void){
 received=0;
 Network_MsgProcess(0,5,"HOME\r"); Network_MsgProcess(0,1,"\n");
 CHECK("split CRLF executes once",received==1 && !strcmp(commands[0],"HOME"));
 received=0;
 Network_MsgProcess(0,3,"MOV");
 Network_MsgProcess(1,6,"STOP\r\n");
 Network_MsgProcess(0,11,"E R Z 100\r\n");
 CHECK("socket buffers never mix",received==2 && !strcmp(commands[0],"STOP") && !strcmp(commands[1],"MOVE"));
 received=0;
 char large[600];memset(large,'X',sizeof large);
 Network_MsgProcess(0,sizeof large,large);
 Network_MsgProcess(0,8,"\r\nHOME\r\n");
 CHECK("oversized frame recovery",received==1 && !strcmp(commands[0],"HOME"));
 received=0;Network_MsgProcess(0,4,"HOME");
 RESET
 Network_MsgProcess(0,2,"\r\n");
 CHECK("new connection discards old tail",received==0);
 tsXParsedData p={0};
 CHECK("incomplete parser input rejected",xParser_ProcessReceivedData("HOME",&p,1)!=PARSER_ERR_SUCCESS);
 CHECK("repeated spaces accepted",xParser_ProcessReceivedData("MOVE  R   Z  123\r\n",&p,1)==PARSER_ERR_SUCCESS && p.ParamCount==3 && p.Params[2].value._int==123);
 CHECK("integer overflow rejected",xParser_ProcessReceivedData("MOVE R Z 2147483648\r\n",&p,1)!=PARSER_ERR_SUCCESS);
 CHECK("explicit positive integer accepted",xParser_ProcessReceivedData("MOVE A Z +123\r\n",&p,1)==PARSER_ERR_SUCCESS && p.Params[2].value._int==123);
 CHECK("finite scientific float accepted",xParser_ProcessReceivedData("PL 35,1e-05\r\n",&p,1)==PARSER_ERR_SUCCESS && p.Params[1].type==PARAM_TYPE_FLOAT && p.Params[1].value._float>0);
 return failures?1:0;
}
""".replace("RESET", reset)
with tempfile.TemporaryDirectory(prefix="vial-framing-") as work:
    source=pathlib.Path(work)/"test.c"; exe=source.with_suffix(".exe")
    source.write_text(prefix+strip(parser)+network+main,encoding="utf-8")
    subprocess.run([os.environ["ZIG_EXE"],"cc","-std=c11",str(source),"-o",str(exe)],check=True)
    raise SystemExit(subprocess.run([str(exe)]).returncode)
