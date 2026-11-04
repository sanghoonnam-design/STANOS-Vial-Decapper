"""Run actual EEPROM CRC/readback functions with an in-memory I2C boundary."""
import os,pathlib,re,subprocess,tempfile
base=pathlib.Path(__file__).resolve().parent
ns={"__file__":str(base/"test_motion.py")}
exec((base/"test_motion.py").read_text(encoding="utf-8").split("# COMPILE_AND_RUN")[0],ns)
text=ns["read"]("App/System/XEEPROMParam.c")
code=r'''
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
typedef uint8_t U8,U08;typedef uint16_t U16;typedef uint32_t U32;
#define YES 1
#define NO 0
#define EEPROM_FACTORY_SETTING_KEY 0xABCDA5A5u
#define EEPROM_APP_PARAM_BASE_ADDRESS 0
#define EEPROM_BLOCK_SIZE 128
#define EEPROM_MAX_BLOCKS 8
#define xprintf(...) ((void)0)
#define ERR_MSG_SEND(...) ((void)0)
#pragma pack(push,1)
typedef struct {struct {U32 FactorySetConfirm_Key;} header;U08 payload[12];U16 crc;} tsEEPROM_Config;
#pragma pack(pop)
static U08 isEEPROM_OK, memory[1024];static bool dropMain,dropBackup;
U16 CRC_Calculate(U8*data,U32 size){U16 sum=0;for(U32 n=0;n<size;n++)sum+=data[n];return sum;}
void EEPROM_WriteBytes(U16 address,U8*data,U16 size){
 if((dropMain && address<128)||(dropBackup && address>=128 && address<256))return;
 memcpy(memory+address,data,size);
}
void EEPROM_ReadBytes(U16 address,U8*data,U16 size){memcpy(data,memory+address,size);}
'''
for name in ["IsFactorySetting","ValidateEEPROMData","EEPROM_WriteBlock"]:
    code+=ns["function"](text,name)
code+=r'''
static int failures;
#define CHECK(n,c) do{printf("%s %s\n",(c)?"PASS":"FAIL",n);if(!(c))failures++;}while(0)
int main(void){
 tsEEPROM_Config config={0};config.header.FactorySetConfirm_Key=EEPROM_FACTORY_SETTING_KEY;config.payload[0]=17;
 config.crc=CRC_Calculate((U8*)&config,sizeof config-2);
 CHECK("valid CRC accepted",ValidateEEPROMData(&config));
 config.payload[0]++;
 CHECK("changed payload fails even after previous success",!ValidateEEPROMData(&config));
 config.payload[0]--;config.header.FactorySetConfirm_Key=0;
 CHECK("factory key mismatch never inherits valid flag",!ValidateEEPROMData(&config));
 config.header.FactorySetConfirm_Key=EEPROM_FACTORY_SETTING_KEY;
 dropBackup=true;
 CHECK("backup write failure reported",!EEPROM_WriteBlock(0,&config));
 dropBackup=false;dropMain=true;
 CHECK("main write failure reported",!EEPROM_WriteBlock(0,&config));
 dropMain=false;
 CHECK("both copies read back correctly",EEPROM_WriteBlock(0,&config));
 return failures?1:0;
}
'''
with tempfile.TemporaryDirectory(prefix="vial-storage-") as work:
 source=pathlib.Path(work)/"test.c";exe=source.with_suffix(".exe");source.write_text(code,encoding="utf-8")
 subprocess.run([os.environ["ZIG_EXE"],"cc","-std=c11",str(source),"-o",str(exe)],check=True)
 raise SystemExit(subprocess.run([str(exe)]).returncode)
