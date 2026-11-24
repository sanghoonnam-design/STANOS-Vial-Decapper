"""Exercise EEPROM readback and per-axis persistence against simulated I2C storage."""
import os
import pathlib
import subprocess
import tempfile

base = pathlib.Path(__file__).resolve().parent
helper = base / "test_motion.py"
ns = {"__file__": str(helper)}
exec(helper.read_text(encoding="utf-8").split("# COMPILE_AND_RUN")[0], ns)
code = ns["prefix"]
code += r'''
typedef struct {U08 ip[4],subnet[4],gw[4];U16 portNum;} Network_t;
'''
code += ns["stripped"](ns["read"]("App/System/XEEPROMParam.h"))
code += r'''
tsEEPROM_Config gEEPROM;
U08 isEEPROM_OK;
static unsigned applyCount;
U32 HAL_GetTick(void){return 0;}
void SYSPL_DefaultSetting(void){xPL=gEEPROM.sysInfo.xPL;applyCount++;}
static U8 memory[EEPROM_BLOCK_SIZE * EEPROM_MAX_BLOCKS];
static int failBlock=-1;
static unsigned writes;
U16 CRC_Calculate(U8 *data,U32 size){
 U16 crc=0xFFFF;
 for (U32 i=0;i<size;i++){
  crc^=(U16)data[i]<<8;
  for (int bit=0;bit<8;bit++) crc=(crc&0x8000)?(crc<<1)^0x1021:crc<<1;
 }
 return crc;
}
void EEPROM_WriteBytes(U16 address,U8 *data,U16 size){
 writes++;
 if (address/EEPROM_BLOCK_SIZE!=failBlock) memcpy(memory+address,data,size);
}
void EEPROM_ReadBytes(U16 address,U8 *data,U16 size){
 memcpy(data,memory+address,size);
}
'''
storage = ns["read"]("App/System/XEEPROMParam.c")
for name in ("IsFactorySetting", "ValidateEEPROMData", "EEPROM_ReadBlock",
             "FindLatestValidBlock", "EEPROM_VerifyBlock", "EEPROM_WriteBlock",
             "EEPROMPL_SaveDecapperTarget", "EEPROMPL_RestoreFromEEPROMBackup",
             "EEPROMPL_LoadFromEEPROM"):
    code += ns["function"](storage, name)
for name in ("CDecap_GetLimits", "CDecap_IsValidTargetFor"):
    code += ns["function"](ns["c"], name)
code += r'''
static int failures;
#define CHECK(label,condition) do{ \
 bool pass=(condition);printf("%s %s\n",pass?"PASS":"FAIL",label);if(!pass)failures++; \
}while(0)
static void updateCRC(tsEEPROM_Config *config){
 config->crc=CRC_Calculate((U8 *)config,sizeof(*config)-sizeof(config->crc));
}
static tsEEPROM_Config factoryImage(void){
 tsEEPROM_Config config={0};
 config.header.FactorySetConfirm_Key=EEPROM_FACTORY_SETTING_KEY;
 config.hwInfo.network.ip[3]=160;
 config.updateCount=19;
 config.sysInfo.xPL.Header.UpdateDate=20261111;
 config.sysInfo.xPL.Decapper.ZCap_UpPos=2000;
 config.sysInfo.xPL.Decapper.RDecapPos=111;
 config.sysInfo.xPL.Decapper.Limit_PosZ=160000;
 config.sysInfo.xPL.Decapper.Limit_PosR=100000;
 config.sysInfo.xPL.Decapper.RunCur[aZ]=1.1f;
 updateCRC(&config);
 return config;
}
static bool readLatest(tsEEPROM_Config *config){
 U08 index;
 return FindLatestValidBlock(&index) && EEPROM_ReadBlock(index,config);
}
static void seed(const tsEEPROM_Config *config){
 memset(memory,0,sizeof(memory));
 failBlock=-1;
 CHECK("seeded valid EEPROM and backup",EEPROM_WriteBlock(0,config));
 writes=0;
}
int main(void){
 tsEEPROM_Config initial=factoryImage(),latest,expected=initial;
 CHECK("valid CRC accepted",ValidateEEPROMData(&initial));
 expected.sysInfo.xPL.Decapper.ZCap_UpPos++;
 CHECK("changed payload fails even after previous success",!ValidateEEPROMData(&expected));
 expected.header.FactorySetConfirm_Key=0;updateCRC(&expected);
 CHECK("factory key mismatch never inherits valid flag",!ValidateEEPROMData(&expected));
 failBlock=1;
 CHECK("backup write failure reported",!EEPROM_WriteBlock(0,&initial));
 failBlock=0;
 CHECK("main write failure reported",!EEPROM_WriteBlock(0,&initial));
 seed(&initial);

 gEEPROM.sysInfo.xPL.Decapper.RDecapPos=88;
 xPL.Decapper.RDecapPos=999;
 xPL.Decapper.RunCur[aZ]=2.2f;
 CHECK("selective Z save is verified",EEPROMPL_SaveDecapperTarget(aZ,4321));
 expected=initial;expected.updateCount++;
 expected.sysInfo.xPL.Decapper.ZCap_UpPos=4321;updateCRC(&expected);
 CHECK("EEPROM matches original record except selected Z, count and CRC",
       readLatest(&latest) && !memcmp(&latest,&expected,sizeof(latest)));
 CHECK("selective EEPROM save never applies the stored image to RAM",
       xPL.Decapper.RDecapPos==999 && xPL.Decapper.RunCur[aZ]==2.2f);
 CHECK("selective R save accepts negative rotation",EEPROMPL_SaveDecapperTarget(aR,-3200));
 expected.updateCount++;
 expected.sysInfo.xPL.Decapper.RDecapPos=-3200;updateCRC(&expected);
 CHECK("EEPROM matches the stored image with only R changed",
       readLatest(&latest) && !memcmp(&latest,&expected,sizeof(latest)));
 writes=0;
 CHECK("limits of the stored configuration are enforced before writes",
       !EEPROMPL_SaveDecapperTarget(aR,8388608) && writes==0);

 seed(&initial);
 tsEEPROM_Config torn=initial;
 torn.updateCount++;
 memcpy(memory+6*EEPROM_BLOCK_SIZE,&torn,sizeof(torn));
 CHECK("bad CRC on a newer record does not hide the last valid record",
       readLatest(&latest) && !memcmp(&latest,&initial,sizeof(latest)));
 failBlock=2;
 CHECK("failed backup leaves the original record recoverable",
       !EEPROMPL_SaveDecapperTarget(aZ,5000) &&
       readLatest(&latest) && !memcmp(&latest,&initial,sizeof(latest)));
 failBlock=1;
 CHECK("verified backup commits the selection when the primary write fails",
       EEPROMPL_SaveDecapperTarget(aZ,5000) &&
       readLatest(&latest) && latest.sysInfo.xPL.Decapper.ZCap_UpPos==5000 &&
       latest.sysInfo.xPL.Decapper.RDecapPos==111);

 initial.updateCount=UINT16_MAX;updateCRC(&initial);seed(&initial);
 CHECK("latest-record selection follows the counter through wraparound",
       EEPROMPL_SaveDecapperTarget(aR,3200) &&
       readLatest(&latest) && latest.updateCount==0 && latest.sysInfo.xPL.Decapper.RDecapPos==3200);
 seed(&initial);
 tsEEPROM_Config loaded={0};
 memset(&gEEPROM,0,sizeof(gEEPROM));
 xPL.Decapper.ZCap_UpPos=123;
 CHECK("LOAD applies the validated record even with a separate output buffer",
       EEPROMPL_LoadFromEEPROM(&loaded) && applyCount==1 &&
       !memcmp(&loaded,&initial,sizeof(loaded)) && !memcmp(&gEEPROM,&initial,sizeof(gEEPROM)) &&
       xPL.Decapper.ZCap_UpPos==initial.sysInfo.xPL.Decapper.ZCap_UpPos);
 memset(memory,0,sizeof(memory));
 CHECK("LOAD without a valid record leaves caller data and active settings intact",
       !EEPROMPL_LoadFromEEPROM(&loaded) && applyCount==1 &&
       !memcmp(&loaded,&initial,sizeof(loaded)) && !memcmp(&gEEPROM,&initial,sizeof(gEEPROM)) &&
       xPL.Decapper.ZCap_UpPos==initial.sysInfo.xPL.Decapper.ZCap_UpPos);
 return failures?1:0;
}
'''
with tempfile.TemporaryDirectory(prefix="vial-storage-") as work:
    source = pathlib.Path(work) / "test.c"
    exe = source.with_suffix(".exe")
    source.write_text(code, encoding="utf-8")
    subprocess.run([os.environ["ZIG_EXE"], "cc", "-std=c11", str(source), "-o", str(exe)], check=True)
    raise SystemExit(subprocess.run([str(exe)]).returncode)
