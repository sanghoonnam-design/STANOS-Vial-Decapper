/*
 * CDCap.h
 *
 *  Created on: 2026. 6. 18.
 *      Author: RND
 */

#ifndef   __CDECAP_H__
#define   __CDECAP_H__

#include <stdbool.h>
#include <stdint.h>
#include "XGlobal.h"   // U08, U32, S32, F32 등 기본 타입
#include "TMC2660.h"   // STEP_CH_MAX, sMotionStatus_t, TMC2660_GetMotorRun
#include "_01_XSystemManagement.h"

#ifdef __CDCAP_C__
	#define CDCEAP_EXT
#else
	#define CDECAP_EXT extern
#endif

// 모터 파라미터 관련 (pk223)
#define T_MOTOR_ACC				300000
#define T_MOTOR_VEL				20000
/*
#define MOTOR_ACC (800000)
#define MOTOR_SPD_SAFE (70000) // 모터 속도 안전 마진
#define MOTOR_SPD_XRECOV_SAFE (40000)
 * */
#define MOTOR_ACC				300000
#define MOTOR_SPD_SAFE			20000
#define MOTOR_SPD_XRECOV_SAFE 	40000
#define R_MOTOR_POS				20000

#define ZDECAP_ACC				300000
#define ZDECAP_VEL				10000

//TEST 하여 수정
/* 2026-11-20: Z 좌표는 아래로 갈수록 +(Z_COORD_SIGN 참고). 기존 -12000/-24000과 같은 실제 위치다. */
#define ZCAP_CAP_UP_POS				14000
#define ZCAP_CAP_SIDE_POS			28000
#define ZCAP_ORIGIN_POSITION	0

#define RDECAP_ACC				300000
#define RDECAP_VEL				100000
/* 2026-11-20: 4바퀴(921600) 기준 이동 약 9.9초, 모터 약 900 rpm(1바퀴 230400) */
/* 2026-11-20 실측(사용자): R 출력축 1바퀴 = 230400 pulse(6400 × 36, 32 마이크로스텝 기준).
 * MM2PULSE_R(115200)은 16 마이크로스텝 가정이라 1바퀴 값으로 쓰지 않는다. */
#define R_PULSE_PER_REV			(230400)
#define RDECAP_POS				(R_PULSE_PER_REV * 5) /* CAP Up↔Side 동안 5바퀴 = 1152000 (공장 기본값, 실기 값은 PL 57) */

#define MOTOR_SPD_HOME_FAST (20000) // 모터 속도 빠르게
#define MOTOR_SPD_HOME_SLOW (3000)  // 모터 속도 느리게

/* 실제 기구물의 가동 범위에 따른 mm 수정*/
#define Z_TOTAL_mm 50
#define Z_Limmit_mm (MM2PULSE_Z * Z_TOTAL_mm) //

#define R_TOTAL_mm 50
#define R_Limmit_mm (MM2PULSE_R * R_TOTAL_mm) //

// 1.8도 기준 1CYCLE 3200 --> PKp223 0.05도 *36
#define PKP223	36
#define PK266	1
#define PK235	1

#define MM2PULSE_Z (3200 * PK235) //1mm 기준
#define MM2PULSE_R (6400 * PKP223) //1mm 기준

//티칭 포인트==============================================================================
#define CDECAPPOS_Z_CAP_TOP 5 //ex 32
#define CDECAPPOS_Z_MOVE_TOP	ZCAP_CAP_UP_POS

#define CDECAPPOS_Z_CAP_SIDE 5 //ex 32
#define CDECAPPOS_Z_MOVE_SIDE 	ZCAP_CAP_SIDE_POS

//==============================================================================

// 축 설정
#define TOP (-1)
#define BOTTOM (1)
#define FRONT (-1)
#define BACK (1)

#define ZH (2)  // (미사용, 옛 표기) Z 센서 핀은 Z_H_SENSOR_PIN(Z+ 아래)/Z_L_SENSOR_PIN(Z− 위) 참고
#define ZL (3)  // (미사용, 옛 표기)

//Master
/* 모터 채널(TMC429/TMC2660 ch 번호). 2026-11-19 실기 배선 확인(사용자): ch0 = Rotate, ch1 = Z.
 * 축별 배열 파라미터(SelMaxCur/RunCur/StopCurRate/SwNegLimit/SwPosLimit)도 이 번호를 첨자로 쓰므로,
 * 이 값을 바꾸기 전에 EEPROM에 저장된 두 축 값은 서로 뒤바뀌어 읽힌다(HOME 전에 PL로 확인). */
#define aZ (1) // Z축 모터 채널
#define aR (0) // Rotate

/* Z 좌표 방향(2026-11-20 사용자 요청): 다른 과제·IO 리스트(Z− = 2번 위쪽 HOME 센서, Z+ = 1번 아래쪽)와 맞춰
 * HOME은 − 방향(위)으로 탐색하고, 작업 좌표(PL 52/53/54·GPOS·RPOS·MOVE)는 아래로 갈수록 + 이다.
 * TMC429의 방향 반전(IF_CONFIG inv_dir)은 세 채널 공통이라 R 회전까지 바뀌므로, Z만 펌웨어에서 부호를 바꾼다.
 * 앱 좌표 = Z_COORD_SIGN × TMC429 좌표. 실제 모터 움직임(배선·드라이버 방향)은 바뀌지 않는다.
 * tests/safety 는 mock 좌표를 앱 좌표로 쓰므로 test_motion.py 가 (1)로 덮어쓴다. */
#ifndef Z_COORD_SIGN
#define Z_COORD_SIGN (-1)
#endif

#define DECAPPING (1)
#define CAPPING (2)

#define DEBUG_SL_SIZE (10)
#define DEBUG_CD_SIZE (10)
#define DEBUG_PL_SIZE (10)

#define SEC_TO_TICKS(sec) ((U32)((sec) * 1000.0f / (float)FW_TICK_MS))
#define _TO_TICKS(ms) ((U32)((ms) * 1000.0f / (float)FW_TICK_MS))
#define FW_TICK_MS (10)
#define MS_TO_TICKS(ms) ((U32)(((uint64_t)(ms) + FW_TICK_MS - 1U) / FW_TICK_MS))

/* 공압 대기 시간. 단위는 ms이며 실제 제어 주기(10ms) 단위로 올림한다. */
#define BODY_GRIP_WAIT_MS        (1000U)
/* 자동 DECAP/CAP/LR에서 Y Back 센서가 감지된 뒤 Body를 잡기 전까지 기다리는 시간(ms). 2026-11-19 사용자 요청. */
#define Y_BACK_SETTLE_MS         (1000U)
#define CAP_GRIP_WAIT_MS         (1000U)
#define CAP_UNGRIP_WAIT_MS       (1000U)
/* HOME 시작 시 Cap·Body 해제 출력 후 Z를 움직이기 전까지 기다리는 시간(ms). 2026-11-20 사용자 요청. */
#define HOME_GRIP_RELEASE_WAIT_MS (1500U)
/* CAP에서 Z가 Up 위치에 도착한 뒤 R 회전·Z 하강을 시작하기 전 멈추는 시간(ms). */
#define CAP_UP_DWELL_MS          (1000U)
/* LR에서 DECAP 완료 후 CAP 시작 전, CAP 완료 후 다음 DECAP 시작 전 쉬는 시간(ms). */
#define LR_INTERVAL_MS           (5000U) /* 2026-11-20 사용자 요청: 1초 → 5초(Forward에 내려놓자마자 다시 가져가지 않게) */
/* LR 각 절반(DECAP/CAP) 시작 시 CT(바이알)가 감지될 때까지 기다리는 최대 시간(ms). 2026-11-20 사용자 요청.
 * Y Forward 센서는 실린더 끝 직전에 켜질 수 있어 바이알이 CT 앞에 자리 잡기 전에 읽히는 경우가 있었다.
 * 감지되면 바로 진행하고, 이 시간 동안 끝내 미감지면 E2820으로 정지한다(대기 중 모터·공압 출력 변화 없음). */
#define CT_DETECT_WAIT_MS        (3000U)
/* 그리퍼 Open/Close가 동시에 감지(11)된 상태가 이 시간 이상 이어지면 대기 중에도 E2871/E2881로 오류 정지한다(순간 노이즈 무시). */
#define GRIP_SENSOR_FAULT_MS     (50U)
/* 최소 대기 후 센서 완료를 추가로 기다리는 제한 시간이다. */
#define GRIP_SENSOR_TIMEOUT_MS   (5000U)

#define HOMING_TIMEOUT_MS		(5000U)
#define CAPPING_TIMEOUT_MS		(5000U)
#define DECAPPING_TIMEOUT_MS	(5000U)
#define READY_TIMEOUT_MS   		(5000U)

/* HOME 0점은 High(위, Z−) 센서 감지 위치에서 이 거리만큼 아래(+방향)로 물러난 곳이다(센서 위치 = −Z_HOME_OFFSET_PULSE). 평소 Z=0 복귀 때 리미트 센서를 치지 않게 한다.
 * 1000 pulse = 약 0.31 mm(3200 pulse/mm 기준). HOME은 이 위치에서 High가 해제(1)됐는지 확인하므로,
 * 센서 히스테리시스가 더 크면 E2851이 난다 → 그때는 이 값을 키운다. 값을 바꾸면 Z 티칭 좌표가 그만큼 밀린다. */
#define Z_HOME_OFFSET_PULSE     (1000)
/* HOME 저속 후퇴·재접근의 최대 이동 거리(센서 히스테리시스 여유). 허용 시간 계산에만 쓴다. */
#define Z_HOME_BACKOFF_MAX_PULSE (MM2PULSE_Z * 2)


#define IS_STARTED(idx) (sStartedFlags & (1UL << (idx)))
#define SET_STARTED(idx) (sStartedFlags |= (1UL << (idx)))
#define CLR_STARTED(idx) (sStartedFlags &= ~(1UL << (idx)))

#define IS_DONE(idx) (sDoneFlags & (1UL << (idx)))
#define SET_DONE(idx) (sDoneFlags |= (1UL << (idx)))
#define CLR_DONE(idx) (sDoneFlags &= ~(1UL << (idx)))

/*Sensor Parameter*/
/* STANOS 과제 배선: 바이알 감지가 0번으로 오고 나머지는 한 칸씩 밀린다(일반 Decapper는 Z High=0 ... 바이알 감지=8).
 * 2026-11-19 사용자 요청: HOME 중 Z High/Low가 뒤바뀐 것으로 보여 두 핀을 교환(기존 High=1, Low=2).
 * 2026-11-20 사용자 요청: 그리퍼 OFF 시 Close가 1로 읽혀 Cap·Body의 Open/Close 핀을 교환(기존 Cap 5/6, Body 7/8).
 * 2026-11-20 사용자 요청: Z 핀을 IO 리스트대로 둔다. H = Z+ = 1번(아래쪽), L = Z− = 2번(위쪽, HOME 센서).
 *   HOME은 위(− 방향)로 가며 L(Z−) 센서를 찾는다(CDecap_Homing). 실제 배선·동작은 이전(위쪽 2번을 HOME)과 같다. */
#define CT_DETECT_OPEN_SENSOR_PIN      (0U)
#define Z_H_SENSOR_PIN                 (1U)   /* Z+ : 아래쪽 리미트 */
#define Z_L_SENSOR_PIN                 (2U)   /* Z− : 위쪽 리미트 = HOME 센서 */
#define Y_F_SENSOR_PIN                 (3U)
#define Y_B_SENSOR_PIN                 (4U)
#define CT_CAP_GRIP_OPEN_SENSOR_PIN    (6U)
#define CT_CAP_GRIP_CLOSE_SENSOR_PIN   (5U)
#define CT_BODY_GRIP_OPEN_SENSOR_PIN   (8U)
#define CT_BODY_GRIP_CLOSE_SENSOR_PIN  (7U)

/* 센서 감지 레벨: IO 확장칩 raw 입력이 이 값이면 "감지"로 본다. CDecap_Sensor_Update()가 raw를 내부 논리값으로 바꾸며
 * 내부(xSL, GSENSOR) 규칙은 그대로다: Z/Y 리미트는 0 = 감지, 그리퍼 Open/Close·CT는 1 = 감지. raw 값은 DI 명령으로 본다.
 * 2026-11-19 실기(사용자 확인): Cap/Body Open/Close는 감지 시 0(열려 있는데 닫힘으로 읽힘), CT는 감지 시 1(정상).
 * Z High/Low는 감지 시 1(사용자 요청으로 다시 1): 앞서 1로 바꿨을 때 HOME 시작 즉시 E2892(입력 1,1)가 나서 0으로 되돌린 이력이 있다.
 * 같은 증상이 나면 DI로 Z High/Low raw 값을 먼저 확인한다. Y Forward/Back은 0(실기 미확인). */
#define Z_LIMIT_ACTIVE_LEVEL           (1U)
#define Y_LIMIT_ACTIVE_LEVEL           (0U)
#define GRIP_SENSOR_ACTIVE_LEVEL       (0U)
#define CT_DETECT_ACTIVE_LEVEL         (1U)

/*Y축 공압기*/
#define Y_PIN_F_CONTROLLER		(0)
#define Y_PIN_B_CONTROLLER		(1)

/* CT 공압 그리퍼 출력 (DO 채널 번호 = 비트 번호 + 1)
 * STANOS 과제 배선: Body = 비트 2/3, Cap = 비트 4/5 (일반 Decapper는 Cap 2/3, Body 6/7). */
#define AIR_CT_BODY_GRIP_PIN     (2) // DO3: GRIP
#define AIR_CT_BODY_UNGRIP_PIN   (3) // DO4: UNGRIP
#define AIR_CT_CAP_GRIP_PIN      (4) // DO5: GRIP
#define AIR_CT_CAP_UNGRIP_PIN    (5) // DO6: UNGRIP

typedef void (*FSM_Handler)(void);

/*ACTION */
typedef enum{
    ACTION_NONE = 0,
	/*User Command*/
    ACTION_STOP,
	ACTION_HOME,

	ACTION_DECAP,
	ACTION_CAP,

	ACTION_PAUSE,
	ACTION_RESUME,

	/*Debug Command*/
	ACTION_ORIGIN,

	ACTION_RMOVEZ,
	ACTION_AMOVEZ,
	ACTION_RROTATE,
	ACTION_AROTATE,

	ACTION_Y_F_MOVE,
	ACTION_Y_B_MOVE,

	ACTION_UDECAP,
	ACTION_UCAP,
	ACTION_LONGRUN,

	ACTION_BODY_GRIP,
	ACTION_CAP_GRIP,

    ACTION_MAX 		// 항상 마지막에 추가
} teXActionType;

typedef enum{
    DFSM_IDLE = 0,

	/*User Command*/
	DFSM_STOP,
	DFSM_HOME,

	DFSM_DECAP,
    DFSM_CAP,

	DFSM_PAUSE,
	DFSM_RESUME,

	/*Debug Command*/
	DFSM_ORIGIN,

	DFSM_RROTATE,
	DFSM_AROTATE,

    DFSM_RMOVEZ,
    DFSM_AMOVEZ,

	DFSM_Y_F_MOVE,
	DFSM_Y_B_MOVE,

	DFSM_UDECAP,
	DFSM_UCAP,
	DFSM_LONGRUN,

	DFSM_CT_BODY_GRIP,
	DFSM_CT_CAP_GRIP,

	DFSM_ABNORMAL,
    DFSM_END_OK,
    DFSM_PHASE_MAX
} teFSM_Decapper;

typedef enum{
    CDECAP_HOMING_NONE = 0,
	CDECAP_HOMING_START,
	CDECAP_HOMING_H_SEARCH,
	CDECAP_HOMING_H_REACH,
	CDECAP_HOMING_H_WAIT,
	CDECAP_HOMING_L_SEARCH,
	CDECAP_HOMING_L_REACH_OFF,
	CDECAP_HOMING_L_WAIT,
	CDECAP_HOMING_S_SEARCH,
	CDECAP_HOMING_S_REACH,
	CDECAP_HOMING_S_WAIT,
	CDECAP_HOMING_Z_DONE,
	CDECAP_HOMING_Y_HOME,
	CDECAP_HOMING_Y_WAIT,
	CDECAP_HOMING_COMPLETE,
	CDECAP_HOMING_BODY_RELEASE,	/* 15: Y 전진 전 Body 해제 대기(GERD 번호 호환을 위해 뒤에 추가) */
	CDECAP_HOMING_OFFSET_MOVE,	/* 16: High 감지 위치에서 Z_HOME_OFFSET_PULSE 만큼 물러나는 이동 발행 */
	CDECAP_HOMING_OFFSET_WAIT,	/* 17: 물러남 완료와 High 해제 확인 */
	CDECAP_HOMING_GRIP_RELEASE,	/* 18: Z 이동 전 Cap·Body 해제 후 HOME_GRIP_RELEASE_WAIT_MS 대기 */
}CDecap_HomingStep_t;

typedef struct{
	CDecap_HomingStep_t Step;
    int Dir;

    U32 H_Acc;
    S32 H_Vel;
    U32 L_Acc;
    S32 L_Vel;

} sHomingSequence_t;

typedef struct{
    struct{
        int HomingRun;
        int HomingComplete;
        int Run;
        int Dir;
        int PositiveLimit;
        int NegativeLimit;
        int HomeSensor;
    } Bit;
} sDriveStatus_t;


typedef struct{
    int isSWLimit;
} SystemInfo_t;

typedef enum{
    LONGRUN_DECAP = 0,
    LONGRUN_CAP
} teLongRunPhase;

typedef struct{
    S32 Motor_CurPos;
    S32 Motor_TargetPos[2];

    teLongRunPhase phaseDecapCap;
    U32 LongRunCount;
    U08 SpeedPercent;

    SystemInfo_t SystemInfo;

    int debug[DEBUG_CD_SIZE];
    S32 chMotor;

} tsXCD_Decapper;

typedef struct{
    F32 RunCur[2];
    int SelMaxCur[2];
    int StopCurRate[2];
    int StepResolution;
    //int PowerEnable[2];

    //Z,Y,R Limit POS 지정
    S32 Limit_PosZ;
    S32 Limit_PosR;

    //하드웨어적 최대 이동 위치
    S32 SwNegLimit[STEP_CH_MAX];
    S32 SwPosLimit[STEP_CH_MAX];
    U8 SoftLimitEnable;

    //Cap_Decap시 필요한 PL --> DB에 상세히 기록
    U32 ZDecapAcc;
    S32 ZDecapVel;
    S32 ZCap_UpPos;
    S32 ZCap_SidePos;
    S32 ZCap_Origin_Position;

    U32 RDecapAcc;
    S32 RDecapVel;
    S32 RDecapPos; // 부호 있는 DECAP 절대목표 pulse; CAP은 HOME 기준 R=0으로 복귀

    int ForDebug[DEBUG_PL_SIZE];

} tsXPL_Decapper;

typedef struct{

    bool isGripCap;  // Cap Open/Close 입력이 모두 0이면 보유
    bool isGripBody; // Forward에서 마지막으로 확인한 Vial 유무

    unsigned char Z_HL_isError;
    unsigned char Y_FB_isError;
    unsigned char CT_Cap_Grip_isError;
    unsigned char CT_Body_Grip_isError;

    //int ForDebug[DEBUG_SL_SIZE];

    //sMotionStatus_t MotionStatus[STEP_CH_MAX];

} tsXSL_Decapper;

typedef struct{

	void (*Senser_Update)(void);
    void (*Status_Update)(void);

    void (*CheckSensorValidity)(void);

    void (*Action_Fnc)(teXActionType actionType);       //
} tsxCDcap;

typedef enum{
    SUB_IDLE = 0,
    SUB_zRMOVE,
	SUB_rRMOVE,
	SUB_zAMOVE,
	SUB_rAMOVE,
	SUB_yFMOVE,
	SUB_yBMOVE,
	SUB_HOMEZ,
	SUB_HOMEY,
	SUB_sMOVE,
	SUB_BODY_GRIP_WAIT,
	SUB_CAPPING,
	SUB_DECAPPING,
    SUB_ACTION_COUNT,
	SUB_CAP_UNGRIP_WAIT,
	SUB_LR_INTERVAL,	/* LR DECAP/CAP 사이 대기(LR_INTERVAL_MS), PAUSE 시간 보정 대상 */
	SUB_LR_CT_WAIT,		/* LR 절반 시작 시 CT 감지 대기(CT_DETECT_WAIT_MS), PAUSE 시간 보정 대상 */
	SUB_MAX
} teDecapperSubIndex;

typedef enum{
    MOVE_IDLE = 0,
    MOVING,
    MOVE_DONE
} move_Step_t;

typedef enum{
	Recovery__IDLE = 0,
	Recovery_Z,
	Recovery_Y,
	Recovery_COMPLETE,
	Recovery__DONE,
	Recovery_Body_Release   /* 5: Z 복귀 후 Body 해제 → Y 전진 전 대기(GERD 번호 호환을 위해 뒤에 추가) */
} Recovery_Step_t;

typedef enum{
	CDecapping_Idle = 0,
	CDecapping_Body_Grip,
	CDecapping_Move_Pos,

	CDecapping_Capping_Ready,
	CDecapping_Decapping_Ready,

	CDecapping_Cap_Capgrip,

	CDecapping_sData_Capping,
	CDecapping_sData_Decapping,

	CDecapping_Move_PrevR,
	CDecapping_Move_PrevZ,
	CDecapping_Move_PrevY,
	CDecapping_Complete_Wait,
	CDecapping_Done,

	/* GERD step 번호 호환을 위해 기존 값 뒤에 추가 */
	CDecapping_Body_Release,      /* 13: 작업 후 Body 해제 → Y 전진 전 대기 */
	CDecapping_Body_PreRelease,   /* 14: 시작 시 Body 해제 확인 → Y 후진 전 대기(오류로 잡힌 채 남은 경우) */
	CDecapping_Cap_UpWait,        /* 15: CAP에서 Z Up 위치 도착 후 회전·하강 전 대기(CAP_UP_DWELL_MS) */
	CDecapping_Y_Back_Settle,     /* 16: Y Back 센서 감지 후 Body 잡기 전 대기(Y_BACK_SETTLE_MS) */

} CDecapping_Statmachin_state;

typedef struct{
	unsigned char Z_H_Limit_Sensor;
	unsigned char Z_L_Limit_Sensor;
	unsigned char Y_F_Limit_Sensor;
	unsigned char Y_B_Limit_Sensor;
	unsigned char CT_Body_Grip_Detect_Open;
	unsigned char CT_Body_Grip_Detect_Close;
	unsigned char CT_Cap_Grip_Detect_Open;
	unsigned char CT_Cap_Grip_Detect_Close;
	unsigned char CT_Detect_Sensor;
} xSLecapping_Sensor;

typedef enum{
	Ready_Idle= 0,
	Ready_Y_B,
	Ready_Done,
}Ready_Statemachin_state;

typedef struct{
	unsigned char Motor_Run;
} xSLecapping_MotorRun;

typedef struct{
	unsigned char Cap;
	unsigned char Body;
} xCDecapping_Grip;

typedef enum{
    PAUSE_Y_NONE = 0,
    PAUSE_Y_FORWARD,
    PAUSE_Y_BACK
} Pause_Y_Direction_t;

typedef struct{
    bool isPaused;
    bool resumeZ;
    bool resumeR;
    bool resumeY;

    S32 zTargetPos;
    S32 rTargetPos;

    U32 zAcc;
    S32 zVel;

    U32 rAcc;
    S32 rVel;

    Pause_Y_Direction_t yDirection;
    teFSM_Decapper savedPhase;
    U32 pauseStartTick;

} Pause_Context_t;

extern tsxCDcap xCDecap;

//Initialize
/**
 * @brief 완전히 대기 중일 때만 Decapper 오류와 공통 오류 코드를 해제한다.
 * 동작·대기 명령·유지보수가 있으면 아무것도 바꾸지 않는다. HOME 완료 상태를 새로 만들지는 않는다.
 */
void CDecap_Error_Clear(void);
/**
 * @brief STOP을 독립 플래그와 xAT에 기록하여 다음 제어 주기에 우선 처리하게 한다.
 * 임계 구역에서 기록하므로 뒤따르는 PAUSE/RESUME가 xAT를 바꿔도 STOP 요청은 남는다.
 * 이 함수 자체가 하드웨어 정지 완료를 기다리지는 않는다.
 */
void CDecap_RequestStop(void);
/**
 * @brief 유지보수·STOP·대기 액션이 없고 FSM/Busy 및 샘플된 Z/R 구동 상태가 모두 대기인지 반환한다.
 * 오류 유무와는 별도 판정이다. 오류 상태에서도 완전히 멈췄으면 CLER를 허용하기 위해 true가 될 수 있다.
 */
bool CDecap_IsIdle(void);
/**
 * @brief 저장·초기화 등 유지보수 작업이 일반 동작을 차단 중인지 반환한다.
 */
bool CDecap_IsMaintenanceActive(void);
/**
 * @brief 임계 구역에서 대기 여부 확인과 유지보수 잠금 획득을 함께 수행한다.
 * true일 때만 작업을 시작하며 성공한 호출은 모든 종료 경로에서 EndMaintenance와 짝을 맞춘다.
 * false이면 잠금을 얻지 못했으므로 타이머 정지나 설정 변경을 하면 안 된다.
 */
bool CDecap_BeginMaintenance(void);
/**
 * @brief 획득한 유지보수 잠금을 임계 구역에서 해제한다.
 * 주기 타이머를 정지한 호출부는 타이머를 복구한 뒤 이 함수를 호출한다.
 */
void CDecap_EndMaintenance(void);
/**
 * @brief 대기·오류 없음 확인과 일반 액션 등록을 임계 구역에서 함께 처리한다.
 * true는 요청 접수이며 완료가 아니다. false면 기존 요청을 유지한다.
 * 호출부가 유효한 일반 액션을 전달해야 한다. STOP은 RequestStop으로 별도 요청한다.
 */
bool CDecap_TryRequestAction(teXActionType action);
/** @brief DECAP/CAP/UDECAP/UCAP/LR처럼 HOME 완료(isHomed)가 필요한 동작이면 true. */
bool CDecap_ActionNeedsHome(teXActionType action);
/** @brief 현재 Phase에서 허용되는 PAUSE(true) 또는 RESUME(false) 요청만 접수한다. */
bool CDecap_TryRequestPause(bool pause);
/**
 * @brief 유지 중인 Decapper 오류를 xSL 및 공통 오류 조회 값과 동기화한다.
 * isError가 있고 저장된 원인이 없으면 상태 오류 2820을 사용한다. 모션 출력은 변경하지 않는다.
 */
void CDecap_ReportError(void);
/** @brief 첫 Decapper 오류의 상세 문구를 반환한다. 기록이 없으면 NULL이다. */
const char *CDecap_GetFaultDetail(void);
/**
 * @brief Decapper 콜백, 상태 머신, Z/R 드라이버와 공압 출력을 초기화한다.
 * PL을 읽은 뒤 호출한다. 전류·분해능·리미트를 적용하고 위치 카운터를 0으로 설정한다.
 * 이때의 0은 기구 원점 확정이 아니므로 isHomed는 false이며 HOME이 별도로 필요하다.
 */
void CDecap_Init(void);
//Update SigDate============================================
/**
 * @brief IO 확장기의 Z/Y 리미트, 그리퍼, CT 감지 입력을 xSL에 복사한다.
 * 리미트는 후속 시퀀스에서 0을 도착으로 해석한다. 이 함수 자체는 출력을 변경하지 않는다.
 */
void CDecap_Sensor_Update(void);
/**
 * @brief 현재 헤더에만 남아 있는 미구현 선언이다. 이 프로젝트에서 정의나 호출은 없으며 센서 갱신은 등록된 콜백을 사용한다.
 */
void CDecap_M_S_Detect(void);
/**
 * @brief TMC2660에서 Z/R 모터의 구동 여부를 읽어 xSL에 반영한다.
 * 이름의 Y와 달리 실제 대상은 aZ와 aR이다. 공압 Y축 완료는 리미트 센서로 판단한다.
 */
void YZ_Motor_Status_Update(void);
//Diagnose==================================================
/**
 * @brief Cap/Vial 보유 상태를 갱신하고 센서 조합 이상을 오류 플래그에 누적한다.
 * Z/Y 리미트가 모두 0이거나 그리퍼 Open/Close가 모두 1이면 해당 오류를 설정한다.
 * 정상 입력으로 돌아와도 자동 해제하지 않는다. 이 센서 오류를 정지에 연결하는 FSM 인터록은 현재 주석 처리되어 있다.
 */
void CDecapping_CheckSensorValidity(void);
//Application===============================================
/**
 * @brief 애플리케이션 제어 주기마다 요청을 해석하고 현재 FSM 핸들러를 한 번 실행한다.
 * STOP을 최우선 처리하고 유지보수·오류 중에는 일반 동작을 막는다. 일반 요청은 IDLE에서만 시작한다.
 * PAUSE/RESUME 조건과 이동 범위를 검사한다. HOME은 좌표 확립 중이므로 현재 위치 범위 감시에서 제외한다.
 * 센서 조합 오류를 정지에 연결하는 블록은 기구 확인 전까지 주석 상태로 유지되어 있다.
 */
void CDecap_Action_Statemachine(teXActionType actionType);
/**
 * @brief 새 이동 명령에 적용할 전체 속도 비율을 저장한다.
 * percent는 1~100만 허용하며 범위 밖이면 기존 값을 유지한다. 진행 중인 드라이버 명령을 재발행하지 않는다.
 */
void CDecap_SetSpeedPercent(U08 percent);
/**
 * @brief 현재 xCD에 저장된 전체 속도 비율(%)을 반환한다.
 */
U08 CDecap_GetSpeedPercent(void);
bool CDecap_IsValidTarget(U08 axis, S32 position);
/* 축의 현재 위치를 앱 좌표로 읽는다(Z는 Z_COORD_SIGN 적용). 호출부가 xMutex_DecapperMotion을 잡고 부른다. */
S32 CDecap_GetAxisPosition(U08 axis);
/** @brief 지정한 Decapper 설정의 기본·소프트 리미트로 목표 좌표를 검증한다. */
bool CDecap_IsValidTargetFor(const tsXPL_Decapper *parameters, U08 axis, S32 position);
int CDecap_GetPhase(void);
bool CDecap_IsPaused(void);
U32 CDecap_GetLongRunLimit(void);
bool CDecap_TryLongRun(U32 limit);
/** @brief 보내지 않은 LR 사이클 완료 횟수가 있으면 *count에 넣고 true(한 번만). 네트워크 태스크 전용. */
bool CDecap_TakeLongRunReport(U32 *count);
U32 CDecap_GetOperationId(void);
int CDecap_GetOperationResult(void);
#endif /* APP_HW_DEVICE_CDCAP_H_ */
