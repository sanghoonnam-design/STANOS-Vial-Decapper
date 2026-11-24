/*******************************************************************************
 * XErrorCode.h
 *
 *  Created on: 2024.10.15
 *      Author: RND. Kang PilSoon.
 ******************************************************************************/
#ifndef XERRORCODE_H_
#define XERRORCODE_H_
#include "XGlobal.h"

/* 에러 심각도 정의 */
typedef enum
{
    /* 상위와의 소통용 */ //
    _INFO,                // 정보 전달용
    _WARNING,             // 경고용
    _CRITICAL,            // 에러 알람용
                          //
    /* 디버깅용 */        //
    _INTERNAL             // 내부 에러.

} teErrorSeverity;

/* 에러 코드 정의 */
typedef enum
{
    ERROR_CODE_NONE /*                     */ = 0,
    ERROR_CODE_DEFAULT /*                  */ = 0,

    ERROR_CODE_INVALID_COMMAND /*          */ = 2100,
    ERROR_CODE_INVALID_ARGUMENT /*         */ = 2110,

    ERROR_CODE_DECAP_TIMEOUT = 2800,
    ERROR_CODE_DECAP_LIMIT = 2810,
    ERROR_CODE_DECAP_STATE = 2820,
    ERROR_CODE_DECAP_Z_HOME_NOT_REACHED = 2850,
    ERROR_CODE_DECAP_Z_HOME_NOT_RELEASED = 2851,
    ERROR_CODE_DECAP_Y_FORWARD_NOT_REACHED = 2860,
    ERROR_CODE_DECAP_Y_BACK_NOT_REACHED = 2861,
    ERROR_CODE_DECAP_CAP_OPEN_NOT_CONFIRMED = 2870,
    ERROR_CODE_DECAP_CAP_GRIP_SENSOR_FAULT = 2871,   /* 동작 중 Cap 그리퍼 Open/Close 동시 감지(11) */
    ERROR_CODE_DECAP_BODY_HOLD_NOT_CONFIRMED = 2880,
    ERROR_CODE_DECAP_BODY_GRIP_SENSOR_FAULT = 2881,  /* 동작 중 Body 그리퍼 Open/Close 동시 감지(11) */
    ERROR_CODE_DECAP_Z_HIGH_LIMIT = 2890,
    ERROR_CODE_DECAP_Z_LOW_LIMIT = 2891,
    ERROR_CODE_DECAP_Z_LIMIT_CONFLICT = 2892,

} teErrorCode;

#define ERROR_CODE_MAKE(__base, __moduleId) ((__base) + (__moduleId))
#define MODULE_ID_NONE (0x0)

/* 에러 정의를 위한 구조체 */
typedef struct
{
    teErrorCode ErrorCode;    // errorcode: integer
    teErrorSeverity severity; //
    const char *message;      //
} tsErrorEntry;

/* 현재 에러를 다루는 구조체 */
typedef struct
{
    U08 moduleId;             // 모듈 몇 번째인가 (NONE 가능)
    teErrorCode Code;         // errorcode: integer type
    char strCode[8];          // errorcode: char type
    teErrorSeverity severity; // 에러 종류: 정보(INFO), 경고(WARNING), 에러(ERROR) 등...
    const char *message;      // 에러 description
                              //
    const char *funcName;     // 에러 발생 함수 이름
    int line;                 // 에러 발생 라인
} tsCurrentError;

extern tsCurrentError gCurrentError;

//==============================================================================
/** ERROR API  */
//==============================================================================
void InitErrorCode(void);

void ClearError(void);
int IsError(void);                         // 리턴: 에러상태 여부
int IsError_Critical(void);                // 리턴: 치명적 에러상태 여부
int IsCurrentError(teErrorCode errorCode); // 리턴: 현재 에러가 지정한 에러코드인지 여부

// Setters
void SetErrorCode(teErrorCode errorCode, const char *funcName, int line);                   // 공통, 단독모듈인 경우
void SetErrorCode_Id(teErrorCode errorCode, U08 module_Id, const char *funcName, int line); // 하나의 제어기에 여러 모듈이 붙은경우

// Getters
int GetErrorCode_int(void);
const char *GetErrorCode_char(void);                      // current error code in char format (e.g. "E1402")
const char *GetErrorMessage(void);                        // current error message (description)
const char *GetErrorMessageByCode(teErrorCode errorCode); // error message (description) for a given error code
teErrorSeverity GetErrorSeverity(void);
const char *GetErrorSeverity_char(teErrorSeverity sev);

// Debug / Monitor
void ErrorMonitor(void);
void ErrorMonitor_LED(void);
void xPrintError(int errorCode); // for debug

#endif /* XERRORCODE_H_ */