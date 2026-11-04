"""USB CLI Enter normalization reserves CRLF and rejects an oversized line."""
import os,pathlib,re,subprocess,tempfile
base=pathlib.Path(__file__).resolve().parent
scope={'__file__':str(base/'test_network.py')}
exec((base/'test_network.py').read_text(encoding='utf-8').split('# API to reset')[0],scope)
debug=scope['read']('App/Task/XDebug.c')
match=re.search(r'static bool CLI_FinalizeInput\(void\)\s*\{',debug)
if match:
    start=match.start();end=match.end();depth=1
    while depth:
        depth+=(debug[end]=='{')-(debug[end]=='}');end+=1
    helper=debug[start:end]
else:helper='static bool CLI_FinalizeInput(void){return false;}'
code=scope['prefix']+scope['strip'](scope['parser'])
code+='typedef uint16_t U16;static char inputBuffer[50];static U16 inputIndex;static bool sInputOverflow;\n'+helper
code+=r'''
int main(void){tsXParsedData p;
 memcpy(inputBuffer,"HOME",4);inputIndex=4;
 CHECK("USB Enter produces a complete parser frame",CLI_FinalizeInput() && !strcmp(inputBuffer,"HOME\r\n") && xParser_ProcessReceivedData(inputBuffer,&p,0)==PARSER_ERR_SUCCESS);
 inputIndex=48;memset(inputBuffer,'X',48);
 CHECK("USB Enter rejects insufficient terminator space",!CLI_FinalizeInput() && inputIndex==0);
 memcpy(inputBuffer,"HOME",4);inputIndex=4;sInputOverflow=true;
 CHECK("USB oversized command is never truncated into an executable prefix",!CLI_FinalizeInput() && inputIndex==0 && !sInputOverflow);
 return failures?1:0;}
'''
with tempfile.TemporaryDirectory(prefix='vial-cli-') as work:
    source=pathlib.Path(work)/'test.c';exe=source.with_suffix('.exe');source.write_text(code,encoding='utf-8')
    subprocess.run([os.environ['ZIG_EXE'],'cc','-std=c11',str(source),'-o',str(exe)],check=True)
    raise SystemExit(subprocess.run([str(exe)]).returncode)
