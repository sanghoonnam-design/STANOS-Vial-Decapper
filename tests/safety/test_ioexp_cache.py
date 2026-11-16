from pathlib import Path
import os
import re
import subprocess
import sys
import tempfile

base = Path(__file__).resolve().parent
root = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else base.parents[1]
io_source = (root / "Drivers/BSP/IOEXP/IOEXP.c").read_text(encoding="utf-8-sig")
header = (root / "App/HW_Device/CDecap.h").read_text(encoding="utf-8-sig")
decap = (root / "App/HW_Device/CDecap.c").read_text(encoding="utf-8-sig")

def function(text, name):
    matches = list(re.finditer(r"(?m)^(?:static )?(?:void|U8|U16)\s+" + name + r"\([^;]*?\)\s*\{", text))
    assert matches, name
    match = matches[-1]
    index, depth = match.end(), 1
    while depth:
        depth += (text[index] == "{") - (text[index] == "}")
        index += 1
    return text[match.start():index] + "\n"

code = r'''
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t U8;
typedef uint16_t U16;
#define READ_IN 0
#define READ_OUT 1
#define IO_CH0 0
#define REG_GPIO 0x12
#define GPIO_PIN_SET 1
static U16 hwIn, hwOut, gInCache, gOutCache;
static int inReads, outReads;
static U16 ReadReg_In(U8 channel,U8 reg){inReads++;return hwIn;}
static U16 ReadReg_Out(U8 channel,U8 reg){outReads++;return hwOut;}
static void WriteReg_Out(U8 channel,U8 reg,U16 value){hwOut=value;}
'''
names = ("Z_H_Limit_Sensor","Z_L_Limit_Sensor","Y_F_Limit_Sensor","Y_B_Limit_Sensor",
         "CT_Cap_Grip_Detect_Open","CT_Cap_Grip_Detect_Close",
         "CT_Body_Grip_Detect_Open","CT_Body_Grip_Detect_Close","CT_Detect_Sensor")
code += "static struct { struct { " + "".join("U8 "+name+";" for name in names) + "} CDecapping_Sensor; } xSL;\n"
for define in ("Z_H_SENSOR_PIN","Z_L_SENSOR_PIN","Y_F_SENSOR_PIN","Y_B_SENSOR_PIN",
               "CT_CAP_GRIP_OPEN_SENSOR_PIN","CT_CAP_GRIP_CLOSE_SENSOR_PIN",
               "CT_BODY_GRIP_OPEN_SENSOR_PIN","CT_BODY_GRIP_CLOSE_SENSOR_PIN","CT_DETECT_OPEN_SENSOR_PIN"):
    code += re.search(r"(?m)^#define "+define+r"\s+\(\d+U\)",header)[0]+"\n"
for name in ("IOEXP_ReadIObit","IOEXP_ReadIObit_Cached","IOEXP_UpdateCache","IOEXP_WriteIO","IOEXP_WriteIObit"):
    code += function(io_source,name)
code += "static U8 (*digitalRead)(U8,U8)=IOEXP_ReadIObit_Cached;\n"
code += function(decap,"CDecap_Sensor_Update")
code += r'''
static int failures;
#define CHECK(name,condition) do{bool ok=(condition);printf("%s %s\n",ok?"PASS":"FAIL",name);failures+=!ok;}while(0)
int main(void){
    bool equal=true;
    for(unsigned value=0;value<65536;value++){
        hwIn=(U16)value;hwOut=(U16)~value;
        IOEXP_UpdateCache();
        for(U8 bit=0;bit<16;bit++){
            if(IOEXP_ReadIObit(READ_IN,bit)!=IOEXP_ReadIObit_Cached(READ_IN,bit))equal=false;
            if(IOEXP_ReadIObit(READ_OUT,bit)!=IOEXP_ReadIObit_Cached(READ_OUT,bit))equal=false;
        }
    }
    CHECK("cached and direct IN/OUT values match for all 65536 masks and 16 channels",equal);
    hwIn=0x015a;hwOut=0xffff;inReads=outReads=0;
    IOEXP_UpdateCache();
    hwIn=0x00a5; /* Simulate inputs changing after this control period's sample. */
    CDecap_Sensor_Update();
    const unsigned char expected[9]={0,1,0,1,1,0,1,0,1};
    printf("SAMPLE reads: IN=%d OUT=%d total=%d\n",inReads,outReads,inReads+outReads);
    CHECK("Decapper adds no input transactions after UpdateCache",inReads==1 && outReads==1);
    CHECK("all nine sensors use the same cached sample, including CT bit 8",
          sizeof xSL.CDecapping_Sensor==sizeof expected &&
          memcmp(&xSL.CDecapping_Sensor,expected,sizeof expected)==0);
    IOEXP_UpdateCache();
    CDecap_Sensor_Update();
    const unsigned char next[9]={1,0,1,0,0,1,0,1,0};
    CHECK("the next UpdateCache refreshes all sensor fields",
          memcmp(&xSL.CDecapping_Sensor,next,sizeof next)==0);
    IOEXP_WriteIO(0);
    bool outputOk=true;
    for(U8 bit=0;bit<16;bit++){
        IOEXP_WriteIObit(bit,1);
        outputOk &= IOEXP_ReadIObit_Cached(READ_OUT,bit)==1;
        IOEXP_WriteIObit(bit,0);
        outputOk &= IOEXP_ReadIObit_Cached(READ_OUT,bit)==0;
    }
    CHECK("successful output writes update the cache with existing active-low semantics",outputOk);
    return failures?1:0;
}
'''
with tempfile.TemporaryDirectory(prefix="decapper-ioexp-cache-") as directory:
    source = Path(directory) / "ioexp_cache.c"
    exe = source.with_suffix(".exe")
    source.write_text(code,encoding="utf-8")
    subprocess.run([os.environ["ZIG_EXE"],"cc","-std=c11",str(source),"-o",str(exe)],check=True,timeout=60)
    raise SystemExit(subprocess.run([str(exe)],timeout=20).returncode)
