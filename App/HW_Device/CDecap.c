/*
 * CDecap.c
 *
 *  Created on: 2026. 6. 18.
 *      Author: RND
 */
/*
 * 읽는 순서: 명령 핸들러 -> xAT 요청 -> TASK_ApplicationControl -> 본 FSM.
 * xPL은 설정, xCD는 요청/제어 데이터, xSL은 관측 상태이다.
 * Z/R은 모터 채널 0/1의 pulse 좌표를 사용하고 Y는 공압/리미트 센서로 제어한다.
 * 센서 갱신 및 진단은 별도 태스크에서 수행하므로 FSM은 마지막 갱신값을 읽는다.
 * 각 시퀀스는 반복 호출로 진행하며 함수 반환 자체가 기구 동작 완료를 뜻하지 않는다.
 */
#include <stdio.h>
#include <math.h>

#include <CDecap.h>
#include "Drive.h"
#include "TMC2660.h"
#include "XDebug.h"
#include "XSystemInfo.h"
#include "XSystem_DB.h"
#include "_01_XSystemManagement.h"
#include "_04_XDiagnose.h"
#include "TMC429.h"
#include "stm32f7xx_hal.h"
#include "_02_XUpdateSIGData.h"

tsxCDcap xCDecap;
sHomingSequence_t HomingSeq;

unsigned char CDecapping_Step = 0;

static teFSM_Decapper sPhase = DFSM_IDLE; 		/* 현재 FSM Phase */

static U32 sStartedFlags = 0; 					/* 비트마스크: 시작 플래그 */
static U32 sDoneFlags = 0; 						/* 비트마스크: 완료 플래그 */

static const U32 Wait_1s = SEC_TO_TICKS(1);
static U08 sSubPhase[SUB_MAX] = { 0 }; 			/* 서브페이즈 (step 값) */
static U32 previousTime[SUB_MAX] = { 0 }; 		/* 서브페이즈별 타이머 */

static Recovery_Step_t Origin_Step = Recovery__IDLE;
static U32 Step_StartTick = 0;

static Pause_Context_t PauseContext;
static volatile bool sStopRequested = false;
static volatile bool sMaintenance = false;
static U32 sLongRunLimit = 0U; /* zero retains legacy unlimited LR */
static bool sLongRunInterval;  /* LR 절반 사이 LR_INTERVAL_MS 대기 중 */
static bool sLongRunCtWait;    /* LR 절반 시작 시 CT 감지 대기 중(CT_DETECT_WAIT_MS) */
/* LR 사이클 완료 알림: 제어 태스크가 기록하고 네트워크 태스크가 CDecap_TakeLongRunReport()로 가져간다. */
static volatile U32 sLongRunReportCount;
static volatile bool sLongRunReportPending;
static U32 sOperationId = 0U;
static int sOperationResult = 0; /* 0:none, 1:accepted/running, 2:finished, 3:stopped, 4:fault */
static teErrorCode sFaultCode = ERROR_CODE_NONE;
static char sFaultDetail[160];
/* 명령 발행 시의 절대 목표를 보존한다. 정지 신호만으로 완료 처리하지 않는다. */
static S32 sMoveTarget[STEP_CH_MAX];
static bool sMovePending[STEP_CH_MAX];
static S32 sZMoveDirection;
/* 축별 이동 허용 시간(ms). 거리·속도·가속도로 계산하며 고정 타임아웃보다 짧아지지 않는다. */
static U32 sMoveTimeoutMs[STEP_CH_MAX];
static bool sResuming; /* RESUME 재발행 중에는 정지 전 허용 시간을 줄이지 않는다. */
/* 대기/일시정지에 들어온 뒤 해당 축을 유지 전류로 내렸는지. 정지·종료·PAUSE 시점과 동작 중에 지운다. */
static bool sHoldApplied[STEP_CH_MAX];

static bool CDecap_IsMoveComplete(U08 ch);
static bool CDecap_HasReachedTarget(U08 ch);

static void CDecap_Fault(teErrorCode code, const char *condition);
static bool CDecap_GetLimits(const tsXPL_Decapper *parameters, U08 ch, S32 *lower, S32 *upper);
static bool CDecap_ValidateTarget(U08 ch, int64_t target);
static bool CDecap_CheckMotionLimits(void);
static bool CDecap_CheckZLimit(int64_t displacement);
static bool CDecap_CheckZMove(int64_t current, int64_t target);
static bool CDecap_CheckZTravelLimit(void);


static void CDecap_StartTimeout(void);
static U32 CDecap_EstimateMoveMs(int64_t distance, S32 velocity, U32 acc, U32 marginPercent);
static void CDecap_SetMoveTimeout(U08 ch, int64_t distance, S32 velocity, U32 acc);
static U32 CDecap_MotionTimeout(U32 baseMs);
static U32 CDecap_HomeSearchTimeout(S32 distance, S32 internalVelocity);
static bool CDecap_CheckTimeout(U32 timeoutMs, const char *condition);
static bool CDecap_CheckSensorTimeout(U32 timeoutMs, teErrorCode code, const char *condition);
static bool CDecap_WaitForGripper(U08 index, U32 minimumMs, bool sensorReady);
static bool CDecap_WaitBodyRelease(void);
static bool CDecap_CheckGripSensorFault(void);

static void CDecap_Idle(void);

static void Air_Y_Forward(void);
static void Air_Y_Back(void);

static bool CD_ABSMove(unsigned char ch, unsigned int Acc, unsigned int Vel, S32 Pos);
static S32 CDecap_ScaleVelocity(S32 velocity);

/* 자동 동작 액션 */
static void CDecap_Homing(void);
static void Decapper_StopAll(void);

static void CDecap_Capping(void);
static void CDecap_Decapping(void);

static void CDecap_Pause(void);
static void CDecap_Resume(void);

/* 수동 제어 액션 */
static void CDecap_Stop(unsigned char ch);

static void CDecap_Origin(void);

static void CDecap_Absmove_Z(void);
static void CDecap_Relmove_Z(void);
static void CDecap_RRotate(void);
static void CDecap_ARotate(void);

static void Air_Y_Forward_Move(void);
static void Air_Y_Back_Move(void);
static void Air_Y_StopOutput(void);
static void Air_Y_Stop(void);

static void UCDecap_Capping(void);
static void UCDecap_Decapping(void);
static void Decap_cap_LR(void);

typedef enum{
    CT_GRIP_RELEASE = OFF,
    CT_GRIP_ENGAGE = ON,
    CT_GRIP_STANDBY
} teCTGripOutput;

static void Air_CTBody_Gripper(teCTGripOutput mode);
static void Air_CTCap_Gripper(teCTGripOutput mode);

/* FSM 종료 및 비정상 처리 보조 함수 */
static void FSM_End_ok(void);
static void FSM_Abnormal(void);
static void ResetAllSubPhases(void);
static void ResetPauseContext(void);

/* 초기화 */
/**
 * @brief Decapper 콜백, 상태 머신, Z/R 드라이버와 공압 출력을 초기화한다.
 * PL을 읽은 뒤 호출한다. 전류·분해능·리미트를 적용하고 위치 카운터를 0으로 설정한다.
 * 이때의 0은 기구 원점 확정이 아니므로 isHomed는 false이며 HOME이 별도로 필요하다.
 */
void CDecap_Init(void){
    sLongRunLimit = 0U;
    sOperationResult = 0;
    sStopRequested = false;
    /* Keep an existing maintenance lock through idle LOAD reconfiguration. */
    sFaultCode = ERROR_CODE_NONE;
    sFaultDetail[0] = '\0';
	sMotionLimit_t MotionLimit;
	sMotionSwLimitPos_t MotionSwLimitPos;

	memset((char*) &xCDecap, 0x00, sizeof(xCDecap));
	memset((char*) &HomingSeq, 0x00, sizeof(HomingSeq));
	xCD.Decapper.SpeedPercent = 100U;

	/* 상태 갱신 콜백 등록 */
	xCDecap.Senser_Update /*            */= CDecap_Sensor_Update;
	xCDecap.Status_Update /*            */= YZ_Motor_Status_Update;

	/* 센서 진단 콜백 등록 */
	xCDecap.CheckSensorValidity /*      */=	CDecapping_CheckSensorValidity;
	/* 디캐퍼 상태 머신 콜백 등록 */
	xCDecap.Action_Fnc /*             	*/= CDecap_Action_Statemachine;
	/* 상태 머신 컨텍스트 초기화 */
	sPhase = DFSM_IDLE;

	/* Z/R 모터의 리미트, 전류, 분해능 설정 */
	MotionLimit.EnableNegLimit = false;
	MotionLimit.PolarityNegLimit = false;
	MotionLimit.EnablePosLimit = false;
	MotionLimit.PolarityPosLimit = false;
	MotionLimit.EnableSoftLimit = (bool) xPL.Decapper.SoftLimitEnable;

	for (U08 ch = 0; ch < STEP_CH_MAX; ch++){
		/* Migrate the old factory +/- limits, stored as identical positive values. */
        S32 limit = (ch == aZ) ? xPL.Decapper.Limit_PosZ : xPL.Decapper.Limit_PosR;
        if (limit > 0 && xPL.Decapper.SwNegLimit[ch] == limit &&
            xPL.Decapper.SwPosLimit[ch] == limit){
            xPL.Decapper.SwNegLimit[ch] = -limit;
        }
        MotionSwLimitPos.SwNegLimit = xPL.Decapper.SwNegLimit[ch];
		MotionSwLimitPos.SwPosLimit = xPL.Decapper.SwPosLimit[ch];

		Drive_SelMaxCurrent(ch, xPL.Decapper.SelMaxCur[ch]);
		Drive_SetCurrent(ch, xPL.Decapper.RunCur[ch], xPL.Decapper.StopCurRate[ch]);

		Drive_SetResoultion(ch, xPL.Decapper.StepResolution);

		Drive_SetHwLimit(ch, MotionLimit);
		Drive_SetSwLimitPos(ch, MotionSwLimitPos);
		Drive_PowerEnable(ch, ENABLE);

		TMC429_SetPosition(ch, 0); /* HOMING에서 기준 위치를 다시 설정한다. */
	}
	/* 공압 그리퍼 초기 상태 */
	Air_CTBody_Gripper(CT_GRIP_RELEASE);
	Air_CTCap_Gripper(CT_GRIP_RELEASE);

    ResetAllSubPhases();
    sPhase = DFSM_IDLE;
    xSL.isHomed = false;
    ResetPauseContext();
}
/* 센서 및 모터 상태 갱신 */
/* 리미트, 그리퍼, CT 감지 센서 값을 갱신한다. */
/**
 * @brief UpdateSIG에서 갱신한 IO 캐시의 Z/Y 리미트, 그리퍼, CT 감지 입력을 xSL 논리값으로 바꿔 저장한다.
 * raw 입력이 센서별 감지 레벨(*_ACTIVE_LEVEL, CDecap.h)과 같으면 감지다.
 * 리미트는 0 = 감지(후속 시퀀스가 0을 도착으로 해석), 그리퍼 Open/Close·CT는 1 = 감지로 저장한다.
 * 이 함수 자체는 출력을 변경하지 않는다.
 */
void CDecap_Sensor_Update(void) {
	//Z축 SENSOR 확인 (HIGH/LOW) : 0 = 감지
	xSL.CDecapping_Sensor.Z_H_Limit_Sensor = (digitalRead(READ_IN, Z_H_SENSOR_PIN) == Z_LIMIT_ACTIVE_LEVEL) ? 0 : 1;
	xSL.CDecapping_Sensor.Z_L_Limit_Sensor = (digitalRead(READ_IN, Z_L_SENSOR_PIN) == Z_LIMIT_ACTIVE_LEVEL) ? 0 : 1;
	//Y축 센서 확인 (Forward/Back) : 0 = 감지
	xSL.CDecapping_Sensor.Y_F_Limit_Sensor = (digitalRead(READ_IN, Y_F_SENSOR_PIN) == Y_LIMIT_ACTIVE_LEVEL) ? 0 : 1;
	xSL.CDecapping_Sensor.Y_B_Limit_Sensor = (digitalRead(READ_IN, Y_B_SENSOR_PIN) == Y_LIMIT_ACTIVE_LEVEL) ? 0 : 1;
	//CT Body를 잡는지 확인 (OPEN/CLOSE) : 1 = 감지
	xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open = (digitalRead(READ_IN, CT_BODY_GRIP_OPEN_SENSOR_PIN) == GRIP_SENSOR_ACTIVE_LEVEL) ? 0 : 1;
	xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Close = (digitalRead(READ_IN, CT_BODY_GRIP_CLOSE_SENSOR_PIN) == GRIP_SENSOR_ACTIVE_LEVEL) ? 0 : 1;
		//CT Cap을 잡는지 확인 (OPEN/CLOSE) : 1 = 감지
	xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Open = (digitalRead(READ_IN, CT_CAP_GRIP_OPEN_SENSOR_PIN) == GRIP_SENSOR_ACTIVE_LEVEL) ? 0 : 1;
	xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Close = (digitalRead(READ_IN, CT_CAP_GRIP_CLOSE_SENSOR_PIN) == GRIP_SENSOR_ACTIVE_LEVEL) ? 0 : 1;
		//CT가 있는지 확인 : 1 = 감지
	xSL.CDecapping_Sensor.CT_Detect_Sensor = (digitalRead(READ_IN, CT_DETECT_OPEN_SENSOR_PIN) == CT_DETECT_ACTIVE_LEVEL) ? 1 : 0;
}
/* Z축과 회전축의 구동 상태를 갱신한다. */
/**
 * @brief TMC2660에서 Z/R 모터의 구동 여부를 읽어 xSL에 반영한다.
 * 이름의 Y와 달리 실제 대상은 aZ와 aR이다. 공압 Y축 완료는 리미트 센서로 판단한다.
 */
void YZ_Motor_Status_Update(void){
	/* Z축 및 회전축 모터의 동작 여부 */
	xSL.xZ_Motor_Run.Motor_Run = (TMC2660_GetMotorRun(aZ) == 1) ? 1 : 0;
	xSL.xR_Motor_Run.Motor_Run = (TMC2660_GetMotorRun(aR) == 1) ? 1 : 0;
}
/* 센서 진단 */
/**
 * @brief 그리퍼 보유 상태를 갱신하고 센서 조합의 이상을 오류 플래그에 누적한다.
 * Cap은 Open/Close가 모두 0이면 보유로 판단한다. Vial은 Forward에서만 확인한다.
 * Z/Y 리미트가 모두 0이거나 그리퍼 Open/Close가 모두 1이면 해당 오류를 설정한다.
 * 정상 입력으로 돌아와도 자동 해제하지 않는다. 이 센서 오류를 정지에 연결하는 FSM 인터록은 현재 주석 처리되어 있다.
 */
void CDecapping_CheckSensorValidity(void){
	// Open/Close 센서가 모두 꺼져 있으면 Cap을 보유한 상태다.
	xSL.Decapper.isGripCap = !xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Open
			&& !xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Close;

	// Forward에서만 Vial을 확인한다. 이동 중이나 Back에서는 마지막 판정을 유지한다.
	if (!xSL.CDecapping_Sensor.Y_F_Limit_Sensor && xSL.CDecapping_Sensor.Y_B_Limit_Sensor){
		xSL.Decapper.isGripBody = (xSL.CDecapping_Sensor.CT_Detect_Sensor == 1U);
	}

	//Z_limit Sensor가 둘다 꺼지는 경우 기기 문제
	if((!xSL.CDecapping_Sensor.Z_H_Limit_Sensor) && (!xSL.CDecapping_Sensor.Z_L_Limit_Sensor)){
		//센서 두개다 들어올수없음 ERROR
		xSL.Decapper.Z_HL_isError = true;
	}
	// Y Forward/Back 리미트가 동시에 도착을 표시하면 센서 이상이다.
	if((!xSL.CDecapping_Sensor.Y_F_Limit_Sensor) && (!xSL.CDecapping_Sensor.Y_B_Limit_Sensor)){
		//센서 두개다 들어올수없음 ERROR
		xSL.Decapper.Y_FB_isError = true;
	}

	//MASTER의 경우 CT_Body 잡는 센서가 둘다 불이 들어올 수 없음 Open/Close 둘중 1개가 들어오거나 뚜껑이 있는 경우 둘다 0
	if(xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open && xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Close){
		xSL.Decapper.CT_Body_Grip_isError = true;
	}

	//MASTER의 경우 CT_CAP 잡는 센서가 둘다 불이 들어올 수 없음 Open/Close 둘중 1개가 들어오거나 뚜껑이 있는 경우 둘다 0
	if(xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Open && xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Close){
		xSL.Decapper.CT_Cap_Grip_isError = true;
	}
}
/* 하드웨어 출력 보조 함수 */
/** @brief 잡기/풀기 출력을 10/01로 설정한다. 대기 출력은 00이다. */
static void Air_CT_SetGripOutput(U08 gripPin, U08 ungripPin, teCTGripOutput mode){
	/* 방향을 바꿀 때는 반대쪽 출력을 먼저 끈다. */
	if (mode == CT_GRIP_ENGAGE){
		IOEXP_WriteIObit(ungripPin, OFF);
		IOEXP_WriteIObit(gripPin, ON);
	}
	else{
		IOEXP_WriteIObit(gripPin, OFF);
		IOEXP_WriteIObit(ungripPin, mode == CT_GRIP_RELEASE ? ON : OFF);
	}
}

/** @brief Body 그리퍼 출력만 변경한다. 센서나 도착 상태는 확인하지 않는다. */
static void Air_CTBody_Gripper(teCTGripOutput mode){
	Air_CT_SetGripOutput(AIR_CT_BODY_GRIP_PIN, AIR_CT_BODY_UNGRIP_PIN, mode);
}

/** @brief Cap 그리퍼 출력만 변경한다. 센서나 도착 상태는 확인하지 않는다. */
static void Air_CTCap_Gripper(teCTGripOutput mode){
	Air_CT_SetGripOutput(AIR_CT_CAP_GRIP_PIN, AIR_CT_CAP_UNGRIP_PIN, mode);
}

/* 전체 속도 비율(1~100%)을 저장한다. */
/**
 * @brief 새 이동 명령에 적용할 전체 속도 비율을 저장한다.
 * percent는 1~100만 허용하며 범위 밖이면 기존 값을 유지한다. 진행 중인 드라이버 명령을 재발행하지 않는다.
 */
void CDecap_SetSpeedPercent(U08 percent){
	if ((percent >= 1U) && (percent <= 100U)){
		xCD.Decapper.SpeedPercent = percent;
	}
}

/**
 * @brief 현재 xCD에 저장된 전체 속도 비율(%)을 반환한다.
 */
U08 CDecap_GetSpeedPercent(void){
	return xCD.Decapper.SpeedPercent;
}

/* 방향을 유지한 채 설정된 전체 속도 비율을 적용한다. */
/**
 * @brief 입력 속도에 전체 속도 비율을 적용한 정수 속도를 반환한다.
 * 부호를 보존하며 0이 아닌 입력이 정수 나눗셈으로 0이 되면 방향에 맞춰 최소 ±1을 반환한다.
 */
static S32 CDecap_ScaleVelocity(S32 velocity){
	S32 scaled = (velocity * (S32)xCD.Decapper.SpeedPercent) / 100;

	if ((velocity > 0) && (scaled == 0)) return 1;
	if ((velocity < 0) && (scaled == 0)) return -1;

	return scaled;
}

/**
 * @brief 앱 좌표 ↔ TMC429 좌표 변환(위치·상대 이동량·속도 부호 공통). Z만 Z_COORD_SIGN을 곱하며,
 *        부호 반전이라 양방향 변환에 같은 함수를 쓴다(CDecap.h Z_COORD_SIGN 설명 참고, 2026-11-20).
 */
static S32 CDecap_AxisToDriver(U08 axis, S32 value){
	return (axis == aZ) ? (S32)(Z_COORD_SIGN * value) : value;
}

/** @brief 축의 현재 위치를 앱 좌표로 읽는다. 호출부가 xMutex_DecapperMotion을 잡고 부른다. */
S32 CDecap_GetAxisPosition(U08 axis){
	return CDecap_AxisToDriver(axis, TMC429_GetPosition(axis));
}
/* 공압 Y축을 Forward 방향으로 구동한다. */
/**
 * @brief Y 공압을 Forward 방향으로 출력하고 재개에 사용할 방향을 저장한다.
 * F 출력 ON/B 출력 OFF이며 도착 센서 확인은 호출 시퀀스가 담당한다.
 */
static void Air_Y_Forward(void){
	PauseContext.yDirection = PAUSE_Y_FORWARD;
	IOEXP_WriteIObit(Y_PIN_F_CONTROLLER, ON);
	IOEXP_WriteIObit(Y_PIN_B_CONTROLLER, OFF);
}
/* 공압 Y축을 Back 방향으로 구동한다. */
/**
 * @brief Y 공압을 Back 방향으로 출력하고 재개에 사용할 방향을 저장한다.
 * F 출력 OFF/B 출력 ON이며 도착 센서 확인은 호출 시퀀스가 담당한다.
 */
static void Air_Y_Back(void){
	PauseContext.yDirection = PAUSE_Y_BACK;
	IOEXP_WriteIObit(Y_PIN_F_CONTROLLER, OFF);
	IOEXP_WriteIObit(Y_PIN_B_CONTROLLER, ON);
}
/* Y축 출력만 정지한다. PAUSE 중에는 저장된 방향을 유지한다. */
/**
 * @brief Y 공압의 F/B 출력을 모두 OFF로 설정한다.
 * PAUSE에서 재개 방향을 보존해야 하므로 저장된 yDirection은 지우지 않는다.
 */
static void Air_Y_StopOutput(void){
	IOEXP_WriteIObit(Y_PIN_F_CONTROLLER, OFF);
	IOEXP_WriteIObit(Y_PIN_B_CONTROLLER, OFF);
}
/* 공압 Y축을 정지하고 저장된 방향을 초기화한다. */
/**
 * @brief Y 공압 정지 출력을 적용하고 저장된 이동 방향도 지운다.
 * 일반 도착 처리와 STOP에서 사용한다. PAUSE는 방향 보존용 StopOutput을 사용한다.
 */
static void Air_Y_Stop(void){
	Air_Y_StopOutput();
	PauseContext.yDirection = PAUSE_Y_NONE;
}
/** @brief 수동 Y 이동을 시작하고 해당 방향 리미트까지 기다린다. */
static void Air_Y_Move(teDecapperSubIndex index, bool forward){
	if (!IS_STARTED(index)){
		SET_STARTED(index);
		CDecap_StartTimeout();
		sSubPhase[index] = MOVING;
	}

	switch (sSubPhase[index]){
	case MOVING:
		if (forward) Air_Y_Forward();
		else Air_Y_Back();
		sSubPhase[index] = MOVE_DONE;
		break;
	case MOVE_DONE: {
		bool limitActive = forward ? !xSL.CDecapping_Sensor.Y_F_Limit_Sensor
		                           : !xSL.CDecapping_Sensor.Y_B_Limit_Sensor;
		if (limitActive){
			Air_Y_Stop();
			sSubPhase[index] = MOVE_IDLE;
			SET_DONE(index);
		}
		else if (CDecap_CheckSensorTimeout(READY_TIMEOUT_MS,
		         forward ? ERROR_CODE_DECAP_Y_FORWARD_NOT_REACHED : ERROR_CODE_DECAP_Y_BACK_NOT_REACHED,
		         forward ? "Y Forward limit not reached" : "Y Back limit not reached")){
			return;
		}
		break;
	}
	default:
		break;
	}

	if (IS_DONE(index)) sPhase = DFSM_END_OK;
}

/** @brief READY 0의 Y Forward 단계를 진행한다. */
static void Air_Y_Forward_Move(void){
	Air_Y_Move(SUB_yFMOVE, true);
}

/** @brief READY 1의 Y Back 단계를 진행한다. */
static void Air_Y_Back_Move(void){
	Air_Y_Move(SUB_yBMOVE, false);
}

/* 현재 단계의 타임아웃 측정을 시작한다. */
/**
 * @brief 현재 단계의 공용 타임아웃 시작 시각을 gTriggerCount로 기록한다.
 * 10 ms tick 기준이며 단계가 바뀔 때 다시 호출하여 대기 시간을 새로 측정한다.
 */
static void CDecap_StartTimeout(void){
	Step_StartTick = gTriggerCount;
}

/**
 * @brief 이동 거리(pulse)·속도(pulse/s)·가속도(pulse/s²)로 이동 시간을 추정하고 여유율을 적용한다.
 * 가감속 구간은 v/a로 근사하며 결과에 1 s를 더한다. 속도 0은 1 pulse/s로 취급한다.
 */
static U32 CDecap_EstimateMoveMs(int64_t distance, S32 velocity, U32 acc, U32 marginPercent){
    uint64_t v = velocity < 0 ? (uint64_t)(-(int64_t)velocity) : (uint64_t)velocity;
    uint64_t d = distance < 0 ? (uint64_t)(-distance) : (uint64_t)distance;
    if (v == 0U) v = 1U;
    uint64_t ms = d * 1000U / v + (acc ? v * 1000U / acc : 0U);
    ms = ms * marginPercent / 100U + 1000U;
    return ms > UINT32_MAX ? UINT32_MAX : (U32)ms;
}

/** @brief 발행한 Z/R 이동의 허용 시간을 기록한다. 예상 시간의 2배 + 1 s이다. */
static void CDecap_SetMoveTimeout(U08 ch, int64_t distance, S32 velocity, U32 acc){
    U32 ms = CDecap_EstimateMoveMs(distance, velocity, acc, 200U);
    if (sResuming && sMoveTimeoutMs[ch] > ms) return;
    sMoveTimeoutMs[ch] = ms;
}

/** @brief 기본 타임아웃과 아직 도착하지 않은 Z/R 이동의 허용 시간 중 큰 값을 반환한다. */
static U32 CDecap_MotionTimeout(U32 baseMs){
    U32 timeout = baseMs;
    for (U08 ch = 0; ch < STEP_CH_MAX; ch++){
        if (sMovePending[ch] && sMoveTimeoutMs[ch] > timeout) timeout = sMoveTimeoutMs[ch];
    }
    return timeout;
}

/**
 * @brief HOME 센서 탐색의 허용 시간을 최대 탐색 거리와 TMC429 내부 속도 단위로 계산한다.
 * 예상 시간의 1.25배 + 1 s이며 HOMING_TIMEOUT_MS보다 짧아지지 않는다.
 */
static U32 CDecap_HomeSearchTimeout(S32 distance, S32 internalVelocity){
    if (internalVelocity == 0 || distance <= 0) return HOMING_TIMEOUT_MS;
    U32 ms = CDecap_EstimateMoveMs(distance, internalVelocity * (S32)STEP_PULSE_RATE_R, MOTOR_ACC, 125U);
    return ms > HOMING_TIMEOUT_MS ? ms : HOMING_TIMEOUT_MS;
}
/* 타임아웃 시 모든 동작을 멈추고 비정상 상태로 전환한다. */
/**
 * @brief timeout_ms를 10 ms tick으로 환산하여 현재 단계의 경과 시간을 검사한다.
 * 시간 초과이면 2800 오류를 유지하고 전체 모션 정지 후 1, 아직 남았으면 0을 반환한다.
 * 1을 받은 시퀀스는 즉시 return하여 이후 출력을 실행하지 않아야 한다.
 */
static bool CDecap_CheckTimeout(U32 timeoutMs, const char *condition){
    if (gTriggerCount - Step_StartTick < timeoutMs / 10) return false;
    CDecap_Fault(ERROR_CODE_DECAP_TIMEOUT, condition);
    return true;
}

/* 센서 대기의 기존 시간 제한을 사용하고 입력값을 함께 기록한다.
 * 2026-11-20: GERD 표기는 GSENSOR와 같이 리밋 1 = 감지로 기록한다(내부 값 반전, 판정 무관). */
static bool CDecap_CheckSensorTimeout(U32 timeoutMs, teErrorCode code, const char *condition){
    if (gTriggerCount - Step_StartTick < timeoutMs / 10) return false;

    char detail[100];
    if (code == ERROR_CODE_DECAP_Z_HOME_NOT_REACHED || code == ERROR_CODE_DECAP_Z_HOME_NOT_RELEASED){
        /* HOME 센서는 L(Z−, 위쪽 2번)이다(2026-11-20). */
        snprintf(detail, sizeof(detail), "%s (H=%u L=%u expected L=%u)", condition,
                 (unsigned)!xSL.CDecapping_Sensor.Z_H_Limit_Sensor,
                 (unsigned)!xSL.CDecapping_Sensor.Z_L_Limit_Sensor,
                 (unsigned)(code == ERROR_CODE_DECAP_Z_HOME_NOT_REACHED));
    }
    else {
        snprintf(detail, sizeof(detail), "%s (F=%u B=%u expected %s=1)", condition,
                 (unsigned)!xSL.CDecapping_Sensor.Y_F_Limit_Sensor,
                 (unsigned)!xSL.CDecapping_Sensor.Y_B_Limit_Sensor,
                 code == ERROR_CODE_DECAP_Y_FORWARD_NOT_REACHED ? "F" : "B");
    }
    CDecap_Fault(code, detail);
    return true;
}

/**
 * @brief 출력 후 최소 대기 시간과 완료 센서를 확인한다.
 * 최소 대기 후에도 완료되지 않으면 GRIP_SENSOR_TIMEOUT_MS 동안 추가 대기하고 오류 정지한다.
 * previousTime을 사용하므로 PAUSE/RESUME의 대기 시간 보정도 함께 적용된다.
 */
static bool CDecap_WaitForGripper(U08 index, U32 minimumMs, bool sensorReady){
    U32 elapsed = gTriggerCount - previousTime[index];
    U32 minimumTicks = MS_TO_TICKS(minimumMs);

    if (elapsed < minimumTicks) return false;
    if (sensorReady) return true;
    if (elapsed - minimumTicks >= MS_TO_TICKS(GRIP_SENSOR_TIMEOUT_MS)){
        bool body = index == SUB_BODY_GRIP_WAIT;
        unsigned open = body ? xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open :
                               xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Open;
        unsigned close = body ? xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Close :
                                xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Close;
        char detail[88];
        snprintf(detail, sizeof(detail), "%s not confirmed (Open=%u Close=%u expected %u/0)",
                 body ? "Body hold" : "Cap release", open, close, (unsigned)!body);
        CDecap_Fault(body ? ERROR_CODE_DECAP_BODY_HOLD_NOT_CONFIRMED :
                            ERROR_CODE_DECAP_CAP_OPEN_NOT_CONFIRMED, detail);
    }
    return false;
}

/* 래치된 디캐퍼 오류 상태를 초기화한다. */
/**
 * @brief 완전히 대기 중일 때만 Decapper 오류와 공통 오류 코드를 해제한다.
 * 동작·대기 명령·유지보수가 있으면 아무것도 바꾸지 않는다. HOME 완료 상태를 새로 만들지는 않는다.
 */
void CDecap_Error_Clear(void){
    if (!CDecap_IsIdle()) return;
    sFaultCode = ERROR_CODE_NONE;
    sFaultDetail[0] = '\0';
    xSL.errorCode = ERROR_CODE_NONE;
    ClearError();
	xSL.isError = false;
	xSL.Decapper.CT_Body_Grip_isError = false;
	xSL.Decapper.CT_Cap_Grip_isError = false;
	xSL.Decapper.Y_FB_isError = false;
	xSL.Decapper.Z_HL_isError = false;
}

/**
 * @brief 수동 Z/R 이동을 한 번 발행하고 정지·목표 도착·최소 대기를 확인한다.
 * 상대 pulse는 현재 좌표와 64비트로 합산해 범위를 검사한다.
 */
static void CDecap_ManualMotorMove(teDecapperSubIndex index, U08 axis, bool relative, S32 position){
	if (!IS_STARTED(index)){
		SET_STARTED(index);
		previousTime[index] = gTriggerCount;
		sSubPhase[index] = MOVING;

		int64_t current = CDecap_GetAxisPosition(axis);
		int64_t target = relative ? current + position : position;
		if (!CDecap_ValidateTarget(axis, target)) return;
		if (axis == aZ){
			if (!CDecap_CheckZMove(current, target)) return;
			sZMoveDirection = (target > current) - (target < current);
		}
		sMoveTarget[axis] = (S32)target;
		sMovePending[axis] = true;
		CDecap_StartTimeout();

		/* 2026-11-19 사용자 요청: 수동 이동도 축별 PL 속도·가속도를 쓴다(기존 고정값 T_MOTOR_ACC/T_MOTOR_VEL 대신).
		 * Z = ZDecapAcc/ZDecapVel, R = RDecapAcc/RDecapVel. 자동 DECAP/CAP/LR과 같은 값이며 PL로 축별 조정한다. */
		sMotionCommand_t command = {
			.Axis = axis,
			.Mode = relative ? MODE_REL : MODE_ABS,
			.Acc = axis == aZ ? xPL.Decapper.ZDecapAcc : xPL.Decapper.RDecapAcc,
			.Vel = CDecap_ScaleVelocity(axis == aZ ? xPL.Decapper.ZDecapVel : xPL.Decapper.RDecapVel),
			.Pos = CDecap_AxisToDriver(axis, position), /* 절대 좌표·상대 이동량 모두 드라이버 부호로 */
		};
		CDecap_SetMoveTimeout(axis, target - current, command.Vel, command.Acc);
		if (relative) Drive_RelMove(&command);
		else Drive_AbsMove(&command);
	}

	switch (sSubPhase[index]){
	case MOVING:
		if (CDecap_CheckTimeout(CDecap_MotionTimeout(READY_TIMEOUT_MS),
		        axis == aZ ? "Z stop or target not confirmed" : "R stop or target not confirmed")) return;
		if (CDecap_IsMoveComplete(axis) && (gTriggerCount - previousTime[index]) > Wait_1s){
			sSubPhase[index] = MOVE_DONE;
		}
		break;
	case MOVE_DONE:
		CDecap_Stop(axis);
		sSubPhase[index] = MOVE_IDLE;
		SET_DONE(index);
		break;
	default:
		break;
	}

	if (IS_DONE(index)) sPhase = DFSM_END_OK;
}

/** @brief MOVE R Z에서 받은 Z축 상대좌표를 실행한다. */
static void CDecap_Relmove_Z(void){
	CDecap_ManualMotorMove(SUB_zRMOVE, aZ, true, xCD.Decapper.Motor_TargetPos[aZ]);
}

/** @brief MOVE A Z에서 받은 Z축 절대좌표를 실행한다. */
static void CDecap_Absmove_Z(void){
	CDecap_ManualMotorMove(SUB_zAMOVE, aZ, false, xCD.Decapper.Motor_TargetPos[aZ]);
}

/** @brief MOVE R R에서 받은 R축 상대좌표를 실행한다. */
static void CDecap_RRotate(void){
	CDecap_ManualMotorMove(SUB_rRMOVE, aR, true, xCD.Decapper.Motor_TargetPos[aR]);
}

/** @brief MOVE A R에서 받은 R축 절대좌표를 실행한다. */
static void CDecap_ARotate(void){
	CDecap_ManualMotorMove(SUB_rAMOVE, aR, false, xCD.Decapper.Motor_TargetPos[aR]);
}

/* 동작하지 않는 대기 상태 */
/**
 * @brief FSM의 대기 상태 핸들러이다. 새 요청이 없을 때 출력을 변경하지 않는다.
 */
static void CDecap_Idle(void){}
/* 지정 축 스테퍼 모터 정지 */
/**
 * @brief ch 축에 감속 정지 명령을 보낸다.
 * 정지 완료를 기다리는 함수는 아니며 상위 상태 정리는 호출부가 담당한다.
 * @note 감속 중에 유지 전류로 내리면 토크가 줄어 탈조할 수 있으므로 여기서는 전류를 바꾸지 않는다.
 *       유지 전류 전환은 CDecap_HoldCurrentWhenStopped()가 정지를 확인한 뒤에 한다.
 */
static void CDecap_Stop(unsigned char ch){
	sMotionCommand_t MotionCmd;
	MotionCmd.Axis = ch;
	MotionCmd.Mode = MODE_STOP;
	MotionCmd.Acc = MOTOR_ACC;
	MotionCmd.Vel = MOTOR_SPD_SAFE;
	MotionCmd.Pos = 0;
	Drive_Stop(&MotionCmd);
}

/**
 * @brief FSM이 대기(정상 종료·STOP·오류 정지 뒤) 또는 일시정지 상태일 때, 멈춘 축만 유지 전류로 내린다.
 * 제어 주기마다 모터 상태를 읽은 직후 호출한다. 구동 중(감속 포함)인 축은 구동 전류를 유지한다.
 * TMC2660_SetHoldCurrent()는 이미 유지 전류면 SPI를 보내지 않으므로 반복 호출해도 통신 부담이 없다.
 * 동작 중 단계 사이의 짧은 정지(대기 시간)에서는 전류를 내리지 않는다.
 * @note TMC2660의 정지 신호(StandStill)는 첫 스텝이 나가기 전에도 정지로 보이므로, 이동 명령 직후
 *       정지시킨 경우를 놓치지 않도록 TMC429 실제 속도(VACTUAL)가 0인지도 확인한다.
 *       속도 SPI 읽기는 대기에 들어온 뒤 전류를 내릴 때까지만 하고, 내린 뒤에는 다시 읽지 않는다.
 */
static void CDecap_HoldCurrentWhenStopped(void){
	if (sPhase != DFSM_IDLE && !PauseContext.isPaused){
		/* 동작 중: 다음 대기에서 다시 확인하도록 표시만 지운다. */
		memset(sHoldApplied, 0, sizeof(sHoldApplied));
		return;
	}
	for (U08 ch = 0; ch < STEP_CH_MAX; ch++){
		bool running = ch == aZ ? xSL.xZ_Motor_Run.Motor_Run : xSL.xR_Motor_Run.Motor_Run;
		if (sHoldApplied[ch] || running || TMC429_GetVelocity(ch) != 0) continue;
		TMC2660_SetHoldCurrent(ch);
		sHoldApplied[ch] = true;
	}
}
/* 모든 모션을 정지하고 FSM 컨텍스트를 초기화한다. */
/**
 * @brief Z/R에 정지 명령을 보내고 Y 공압을 정지한 뒤 동작 문맥을 초기화한다.
 * xAT와 Busy를 해제하고 IDLE로 복귀한다. 그리퍼 출력과 기존 오류는 유지한다.
 */
static void Decapper_StopAll(void){
    if (sOperationResult == 1) sOperationResult = 3;
	for (U08 ch = 0; ch < STEP_CH_MAX; ch++){
		CDecap_Stop(ch);
	}
	Air_Y_Stop();
	sPhase = DFSM_IDLE;
	xSL.isBusy = NO;
	CDecapping_Step = CDecapping_Idle;
	xCD.Decapper.phaseDecapCap = LONGRUN_DECAP;
	xAT = ACTION_NONE;
	ResetAllSubPhases();
	ResetPauseContext();
}
/* 수동 Cap 그리퍼 제어 */
/**
 * @brief 저장된 Cap 그리퍼 수동 값을 출력한다.
 * 출력을 기록하면 END_OK로 전환하며 실제 그리퍼 도착 센서를 기다리지는 않는다.
 */
static void CT_Cap_Grip(void){
	Air_CTCap_Gripper(xCD.CT.Cap);
	sPhase = DFSM_END_OK;
}
/* 수동 Body 그리퍼 제어 */
/**
 * @brief 저장된 Body 그리퍼 수동 값을 출력한다.
 * 출력을 기록하면 END_OK로 전환하며 실제 그리퍼 도착 센서를 기다리지는 않는다.
 */
static void CT_Body_Grip(void){
	Air_CTBody_Gripper(xCD.CT.Body);
	sPhase = DFSM_END_OK;
}
/**
 * @brief pulse/s² 가속도를 TMC429 AMAX 내부 단위로 환산한다(Drive_AbsMove와 같은 식: ÷A_MAX_BASE).
 * TMC429_VelMove/MotorStop은 내부 단위를 받고 드라이버가 0x7FF로 자르므로, pulse/s² 값을 그대로
 * 넘기면 의도와 무관한 가속도가 된다(300000 → 992, 정상 환산값은 약 80). 0이면 가감속이 되지 않아 1 이상으로 둔다.
 */
static U32 CDecap_ToDriverAcc(U32 acc){
	U32 internal = (U32)((F32)acc / (F32)A_MAX_BASE);
	if (internal > MAX_ACC) internal = MAX_ACC;
	if (internal < 1U) internal = 1U;
	return internal;
}

/* ---------------------------------------------------------------------------------------------
 * CAP/DECAP Z·R 동시 이동 도착 시간 맞추기 (2026-11-20 사용자 요청)
 * Z가 Up↔Side를 움직이는 동안 R이 RDecapPos(예: 4바퀴 = 4 × R_PULSE_PER_REV = 921600)를 돌고 같이 끝나야 한다.
 * R(PL RDecapVel/RDecapAcc)을 기준으로 이 구간의 Z 속도·가속도만 계산해 낮춘다. Z 접근·복귀·수동 MOVE는
 * PL ZDecapVel/ZDecapAcc를 그대로 쓴다.
 * TMC429 단위(TMC429 데이터시트 속도·가속도 식, fCLK 16 MHz, PULSE_DIV=1, RAMP_DIV=6, TMC429.h):
 *   속도 1 = fCLK/(2^PULSE_DIV·2048·32) ≈ 122 pulse/s(STEP_PULSE_RATE_R)
 *   가속도 1 = fCLK²/2^(PULSE_DIV+RAMP_DIV+29) ≈ 3725 pulse/s²(A_MAX_BASE)
 * Z와 R은 같은 TMC429라 클럭 오차는 두 축에 똑같이 걸려 도착 시간 비율에 영향이 없다.
 * Z 속도 단위가 굵어서(약 122 pulse/s) 생기는 반올림 오차는 R 속도를 PL 값 이하로 미세 조정해 흡수한다.
 * ------------------------------------------------------------------------------------------- */
/* TMC429_SetAMax()는 AMAX 9 미만에서 PMUL/PDIV 쌍을 찾지 못한다(RAMP_DIV-PULSE_DIV=5 기준 계산). */
#define CDECAP_SYNC_MIN_AMAX   (9)

/** @brief 사다리꼴(최고속도 미도달 시 삼각형) 이동 시간[s]. */
static F32 CDecap_ProfileSec(F32 dist, F32 vel, F32 acc){
	if (vel <= 0.0f || acc <= 0.0f) return 0.0f;
	if (dist >= vel * vel / acc) return dist / vel + vel / acc;
	return 2.0f * sqrtf(dist / acc);
}

/** @brief 거리 dist를 sec초에 끝내는 사다리꼴 최고속도(작은 근). 불가능하면 -1. */
static F32 CDecap_VelocityForSec(F32 dist, F32 sec, F32 acc){
	F32 ta = sec * acc;
	F32 disc = ta * ta - 4.0f * dist * acc;
	if (disc < 0.0f) return -1.0f;
	return (ta - sqrtf(disc)) * 0.5f;
}

/**
 * @brief TMC429 속도 내부값을 CD_ABSMove()에 넘길 pulse/s로 되돌린다.
 * CD_ABSMove가 전체 속도 비율(SpeedPercent)을 다시 곱하고 Drive_AbsMove가 ÷STEP_PULSE_RATE_R로 자르므로,
 * 비율 적용 후 값이 해당 내부값 구간의 가운데(+½단위)가 되도록 역산한다.
 */
static S32 CDecap_InternalToVel(S32 internal){
	S32 sp = (S32)xCD.Decapper.SpeedPercent;
	if (sp < 1 || sp > 100) sp = 100;
	int64_t scaled = (int64_t)internal * STEP_PULSE_RATE_R + STEP_PULSE_RATE_R / 2;
	return (S32)((scaled * 100 + sp - 1) / sp);
}

/**
 * @brief CAP/DECAP의 Z·R 동시 이동에서 두 축이 같이 끝나도록 Z(필요하면 R) 속도·가속도를 계산한다.
 * @param zTarget,rTarget 이번 이동 절대 목표
 * @param zAcc,zVel  입력: PL Z 값(상한), 출력: 이번 이동에 쓸 Z 값
 * @param rVel       입력: PL R 속도(상한), 출력: 이번 이동에 쓸 R 속도. R 가속도는 PL 그대로 쓴다.
 * @note Z는 PL Z 속도·가속도보다 빨라지지 않고, R은 PL R 속도보다 빨라지지 않는다.
 *       Z가 PL 최고속도로도 R보다 오래 걸리면(R이 너무 빠름) Z는 PL 값 그대로, R만 늦춰 맞춘다.
 *       한 축만 움직이면 아무것도 바꾸지 않는다.
 */
static void CDecap_SyncZWithR(S32 zTarget, S32 rTarget, U32 *zAcc, S32 *zVel, U32 rAcc, S32 *rVel){
	const F32 vUnit = (F32)STEP_PULSE_RATE_R;
	const F32 aUnit = (F32)A_MAX_BASE;
	F32 dz = fabsf((F32)((int64_t)zTarget - xCD.Decapper.Motor_CurPos));
	F32 dr = fabsf((F32)((int64_t)rTarget - CDecap_GetAxisPosition(aR)));
	if (dz < 1.0f || dr < 1.0f || *zVel <= 0 || *rVel <= 0) return;

	/* Drive_AbsMove와 같은 방식으로 PL 값을 TMC429 내부값으로 바꾼다(속도 비율 적용 후). */
	S32 vr = CDecap_ScaleVelocity(*rVel) / STEP_PULSE_RATE_R;
	S32 vzMax = CDecap_ScaleVelocity(*zVel) / STEP_PULSE_RATE_R;
	S32 ar = (S32)(rAcc / A_MAX_BASE);
	S32 azMax = (S32)(*zAcc / A_MAX_BASE);
	if (vr < 1) vr = 1;
	if (vr > MAX_VELOCITY) vr = MAX_VELOCITY;
	if (vzMax < 1) vzMax = 1;
	if (vzMax > MAX_VELOCITY) vzMax = MAX_VELOCITY;
	if (ar < 1) ar = 1;
	if (ar > MAX_ACC) ar = MAX_ACC;
	if (azMax > MAX_ACC) azMax = MAX_ACC;
	if (azMax < CDECAP_SYNC_MIN_AMAX) azMax = CDECAP_SYNC_MIN_AMAX;

	/* Z 가속도는 R과 같은 비율(거리비)이 이상적이지만 TMC429 최소값(9)과 PL Z 가속도 사이로 제한한다. */
	S32 az = (S32)lroundf((F32)ar * dz / dr);
	if (az < CDECAP_SYNC_MIN_AMAX) az = CDECAP_SYNC_MIN_AMAX;
	if (az > azMax) az = azMax;

	/* 1) R 시간에 맞는 Z 속도를 구해 내림(Z는 같거나 조금 느림) → 2) 그 Z 시간에 맞게 R 속도를 다시 구한다. */
	F32 tr = CDecap_ProfileSec(dr, (F32)vr * vUnit, (F32)ar * aUnit);
	F32 vzReal = CDecap_VelocityForSec(dz, tr, (F32)az * aUnit);
	S32 vz = (vzReal < 0.0f) ? vzMax : (S32)(vzReal / vUnit);
	if (vz < 1) vz = 1;
	if (vz > vzMax) vz = vzMax;

	F32 tz = CDecap_ProfileSec(dz, (F32)vz * vUnit, (F32)az * aUnit);
	F32 vrReal = CDecap_VelocityForSec(dr, tz, (F32)ar * aUnit);
	S32 vrNew = (vrReal < 0.0f) ? vr : (S32)lroundf(vrReal / vUnit);
	if (vrNew < 1) vrNew = 1;
	if (vrNew > vr) vrNew = vr;

	*zVel = CDecap_InternalToVel(vz);
	*zAcc = (U32)az * A_MAX_BASE + A_MAX_BASE / 2U;
	*rVel = CDecap_InternalToVel(vrNew);
}

/* Z축을 홈 기준으로 설정한 뒤 공압 Y축을 Forward 리미트로 복귀한다. */
/**
 * @brief Cap·Body 해제 후 1.5초 대기 → Z 고속 탐색 → 저속 후퇴 → 저속 재접근
 *        → 감지 위치에서 Z_HOME_OFFSET_PULSE 물러남(그곳이 좌표 0) → Body 해제 → Y Forward 복귀를 진행한다.
 * 각 호출은 현재 단계만 수행하며 센서 대기는 타임아웃으로 보호한다.
 * 이미 isHomed이면 종료한다. R축 원점 탐색은 이 함수에 포함되어 있지 않다.
 * @note Z 속도 이동은 TMC429를 직접 쓰므로 여기서 구동 전류로 올리고(TMC2660_SetRunCurrent),
 *       Z 정지를 확인한 뒤 유지 전류로 내린다. 가속도는 CDecap_ToDriverAcc()로 환산해 넘긴다.
 * @note Y를 움직이기 전에 Body를 먼저 푼다(오류 정지 후 잡힌 채 남아 있을 수 있다).
 * @note 2026-11-20 사용자 요청: Z를 움직이기 전에 Cap·Body를 모두 풀고 HOME_GRIP_RELEASE_WAIT_MS(1.5초) 기다린다.
 *       전원이 내려갔다 올라온 뒤 바이알·뚜껑을 잡은 채 HOME하면 Z가 들고 올라가고 Y도 움직이기 때문이다.
 *       센서 확인 없이 시간으로만 기다린다(대기 중 R·Z·Y는 움직이지 않는다).
 */
static void CDecap_Homing(void){
	sHomingSequence_t*   Homing;
	Homing = &HomingSeq;

	if (!IS_STARTED(SUB_HOMEZ)) {
		SET_STARTED(SUB_HOMEZ);
		if((CDECAP_HOMING_NONE < HomingSeq.Step) && (HomingSeq.Step < CDECAP_HOMING_COMPLETE)) { return; }
		/* 매 HOME 요청마다 기준을 다시 잡는다. R은 센서 탐색 없이 정지 위치를 0으로 정한다. */
		xSL.isHomed = false;
		CDecap_Stop(aR); /* 속도 모드로 정지시켜 과거 절대목표의 재이동을 방지한다. */
		/* 모터 속도는 기구 테스트 후 조정한다. */
		/* TMC429_VelMove()/MotorStop()은 실제 pulse/s, pulse/s²가 아닌 내부 단위를 받는다. */
		/* 앱 좌표 기준 고속 탐색은 − 방향(위, High/Z− 센서 쪽), 저속 후퇴는 + 방향(아래)이다.
		 * H_Vel/L_Vel은 TMC429에 바로 넘기는 값이라 CDecap_AxisToDriver()로 드라이버 부호로 바꿔 저장한다. */
		HomingSeq.H_Acc = CDecap_ToDriverAcc(MOTOR_ACC);
		HomingSeq.H_Vel = CDecap_AxisToDriver(aZ, -CDecap_ScaleVelocity(MOTOR_SPD_HOME_FAST / STEP_PULSE_RATE_R));
		HomingSeq.L_Acc = CDecap_ToDriverAcc(MOTOR_ACC);
		HomingSeq.L_Vel = CDecap_AxisToDriver(aZ, CDecap_ScaleVelocity(MOTOR_SPD_HOME_SLOW / STEP_PULSE_RATE_R));
		HomingSeq.Step = CDECAP_HOMING_START;
	}


	switch(Homing->Step){
	    case CDECAP_HOMING_START:
	        /* Z를 움직이기 전에 Cap·Body를 모두 푼다(2026-11-20 사용자 요청). */
	        Air_CTCap_Gripper(CT_GRIP_RELEASE);
	        Air_CTBody_Gripper(CT_GRIP_RELEASE);
	        previousTime[SUB_BODY_GRIP_WAIT] = gTriggerCount;
	        Homing->Step = CDECAP_HOMING_GRIP_RELEASE;
	        break;
	    case CDECAP_HOMING_GRIP_RELEASE:
	        /* 해제 출력 후 HOME_GRIP_RELEASE_WAIT_MS 동안 기다린 뒤 Z 고속 탐색을 시작한다. */
	        if ((gTriggerCount - previousTime[SUB_BODY_GRIP_WAIT]) < MS_TO_TICKS(HOME_GRIP_RELEASE_WAIT_MS)) { return; }
	    	Homing->Step = CDECAP_HOMING_H_SEARCH;
	        break;
	    case CDECAP_HOMING_H_SEARCH:
	        CDecap_StartTimeout();
	        if (xSL.CDecapping_Sensor.Z_L_Limit_Sensor){
	            TMC2660_SetRunCurrent(aZ);
	            TMC429_VelMove(aZ, Homing->H_Acc, Homing->H_Vel);
	            Homing->Step = CDECAP_HOMING_H_REACH;
	        }
	        else {
	            TMC429_MotorStop(aZ, Homing->H_Acc);
	            Homing->Step = CDECAP_HOMING_H_WAIT;
	        }
	        break;
	    case CDECAP_HOMING_H_REACH:
	        /* 시작 위치를 모르므로 Z 기본 범위 전체를 최대 탐색 거리로 본다. */
	        if (CDecap_CheckSensorTimeout(CDecap_HomeSearchTimeout(xPL.Decapper.Limit_PosZ, Homing->H_Vel), ERROR_CODE_DECAP_Z_HOME_NOT_REACHED, "Z home sensor not reached during fast search")){return;}
	    	if(xSL.CDecapping_Sensor.Z_L_Limit_Sensor){ return; }
	    	TMC429_MotorStop(aZ, Homing->H_Acc);
	    	CDecap_StartTimeout();
	    	Homing->Step = CDECAP_HOMING_H_WAIT;
	        break;
	    case CDECAP_HOMING_H_WAIT:
            if (CDecap_CheckTimeout(HOMING_TIMEOUT_MS, "Z stop not confirmed after fast search")){return;}
	        if(xSL.xZ_Motor_Run.Motor_Run) { return; }
	        Homing->Step = CDECAP_HOMING_L_SEARCH;
	        break;
	    case CDECAP_HOMING_L_SEARCH:
	    	/* Home sensor를 해제하기 위해 반대 방향으로 저속 후퇴 */
	    	TMC2660_SetRunCurrent(aZ);
	    	TMC429_VelMove(aZ, Homing->L_Acc, Homing->L_Vel);
	    	CDecap_StartTimeout();
	        Homing->Step = CDECAP_HOMING_L_REACH_OFF;
	        break;
	    case CDECAP_HOMING_L_REACH_OFF:
	        /* 현재 프로젝트의 기준: 0 = Home 감지, 1 = 센서 해제 */
	        if (CDecap_CheckSensorTimeout(CDecap_HomeSearchTimeout(Z_HOME_BACKOFF_MAX_PULSE, Homing->L_Vel), ERROR_CODE_DECAP_Z_HOME_NOT_RELEASED, "Z home sensor did not release")){return;}
	        if(!xSL.CDecapping_Sensor.Z_L_Limit_Sensor) { return; }
	        TMC429_MotorStop(aZ, Homing->L_Acc);
	        CDecap_StartTimeout();
	        Homing->Step = CDECAP_HOMING_L_WAIT;
	        break;
	    case CDECAP_HOMING_L_WAIT:
            if (CDecap_CheckTimeout(HOMING_TIMEOUT_MS, "Z stop not confirmed after backing off")){return;}
	        if(xSL.xZ_Motor_Run.Motor_Run) { return; }
	        Homing->Step = CDECAP_HOMING_S_SEARCH;
	        break;
	    case CDECAP_HOMING_S_SEARCH:
	        /* 해제 위치에서 원래 방향으로 저속 재접근 */
	        CDecap_StartTimeout();
	        if (xSL.CDecapping_Sensor.Z_L_Limit_Sensor){
	            TMC2660_SetRunCurrent(aZ);
	            TMC429_VelMove(aZ, Homing->L_Acc, -Homing->L_Vel);
	            Homing->Step = CDECAP_HOMING_S_REACH;
	        }
	        else {
	            TMC429_MotorStop(aZ, Homing->L_Acc);
	            Homing->Step = CDECAP_HOMING_S_WAIT;
	        }
	        break;
	    case CDECAP_HOMING_S_REACH:
	        /* Home 센서가 다시 감지될 때까지 이동 */
            if (CDecap_CheckSensorTimeout(CDecap_HomeSearchTimeout(Z_HOME_BACKOFF_MAX_PULSE, Homing->L_Vel), ERROR_CODE_DECAP_Z_HOME_NOT_REACHED, "Z home sensor not reached during slow search")){return;}
	        if(xSL.CDecapping_Sensor.Z_L_Limit_Sensor) { return; }
	        TMC429_MotorStop(aZ, Homing->L_Acc);
	        CDecap_StartTimeout();
	        Homing->Step = CDECAP_HOMING_S_WAIT;
	        break;
	    case CDECAP_HOMING_S_WAIT:
            if (CDecap_CheckTimeout(HOMING_TIMEOUT_MS, "Z stop not confirmed after slow search")){return;}
	        if(xSL.xZ_Motor_Run.Motor_Run) { return; }
	        Homing->Step = CDECAP_HOMING_Z_DONE;
	        break;
	    case CDECAP_HOMING_Z_DONE:
	        /* 재접근으로 찾은 HOME 센서(L, Z−) 감지 위치를 앱 좌표 −Z_HOME_OFFSET_PULSE 로 정한다. 0점은 그만큼 아래(+)다.
	         * 이동 발행은 다음 주기에 한다(Motor_CurPos가 새 좌표로 갱신된 뒤). */
	        TMC429_SetPosition(aZ, CDecap_AxisToDriver(aZ, -Z_HOME_OFFSET_PULSE));
	        Homing->Step = CDECAP_HOMING_OFFSET_MOVE;
	        break;
	    case CDECAP_HOMING_OFFSET_MOVE:
	        /* 센서에서 물러나 0점으로 이동한다. 저속 탐색과 같은 속도·가속도를 쓴다(구동 전류는 Drive_AbsMove가 올린다). */
	        if (!CD_ABSMove(aZ, MOTOR_ACC, MOTOR_SPD_HOME_SLOW, 0)) return;
	        CDecap_StartTimeout();
	        Homing->Step = CDECAP_HOMING_OFFSET_WAIT;
	        break;
	    case CDECAP_HOMING_OFFSET_WAIT:
	        if (CDecap_CheckTimeout(CDecap_MotionTimeout(HOMING_TIMEOUT_MS), "Z stop or target not confirmed after home offset")){return;}
	        if (!CDecap_IsMoveComplete(aZ)) { return; }
	        /* 0점에서는 HOME 센서(L, Z−·위쪽 2번)가 해제돼 있어야 한다. 아직 감지 중이면 오프셋이 센서 히스테리시스보다 작다(대기 없이 바로 오류). */
	        if (!xSL.CDecapping_Sensor.Z_L_Limit_Sensor &&
	            CDecap_CheckSensorTimeout(0U, ERROR_CODE_DECAP_Z_HOME_NOT_RELEASED, "Z home sensor still active after home offset")){return;}
	        /* Z 정지를 확인했으므로 유지 전류로 내린다. */
	        TMC2660_SetHoldCurrent(aZ);
	        Homing->Step = CDECAP_HOMING_Y_HOME;
	        break;
	    case CDECAP_HOMING_Y_HOME:
	        /* Y를 움직이기 전에 Body를 먼저 푼다. 열림 확인 또는 BODY_GRIP_WAIT_MS 뒤에 진행한다. */
	        Air_CTBody_Gripper(CT_GRIP_RELEASE);
	        previousTime[SUB_BODY_GRIP_WAIT] = gTriggerCount;
	        Homing->Step = CDECAP_HOMING_BODY_RELEASE;
	        break;
	    case CDECAP_HOMING_BODY_RELEASE:
	        if (!CDecap_WaitBodyRelease()) { return; }
		Air_Y_Forward();
	    	CDecap_StartTimeout();
	        Homing->Step = CDECAP_HOMING_Y_WAIT;
	        break;
	    case CDECAP_HOMING_Y_WAIT:
            if (CDecap_CheckSensorTimeout(HOMING_TIMEOUT_MS, ERROR_CODE_DECAP_Y_FORWARD_NOT_REACHED, "Y Forward limit not reached")){return;}
		if(!xSL.CDecapping_Sensor.Y_F_Limit_Sensor){
	    		Air_Y_Stop();
            CDecap_StartTimeout();
	    		Homing->Step = CDECAP_HOMING_COMPLETE;
	    	}
	        break;
	    case CDECAP_HOMING_COMPLETE:
            /* R 원점 센서는 없으므로 정지 확인 후 현재 각도를 소프트웨어 원점으로 삼는다. */
            if (xSL.xR_Motor_Run.Motor_Run || TMC429_GetVelocity(aR) != 0){
                CDecap_CheckTimeout(HOMING_TIMEOUT_MS, "R stop not confirmed before zeroing");
                return;
            }
            if (!CDecap_ValidateTarget(aR, 0)) return;
            TMC429_SetPosition(aR, 0);
            xCD.Decapper.Motor_TargetPos[aR] = 0;
            sMoveTarget[aR] = 0;
            sMovePending[aR] = false;
	    	Homing->Step = CDECAP_HOMING_NONE;
	    	xSL.isHomed = true;
	    	xprintf("Homing Complete.\n");
	    	SET_DONE(SUB_HOMEZ);
	    	SET_DONE(SUB_HOMEY);
	    	sPhase = DFSM_END_OK; /* 중앙 종료 처리로 전환 */
	    	Homing->Step = CDECAP_HOMING_NONE;
		    break;
	}
}

/* Pause/Resume에서 재사용할 Z/R 절대 이동 명령을 저장하고 실행한다. */
/**
 * @brief 자동 시퀀스용 절대 이동을 발행하고 PAUSE 재개용 목표·가속도·속도를 저장한다.
 * ch는 Z/R, Pos는 pulse이다. 목표 검사 실패 시 오류 정지하고 false를 반환한다.
 * true는 명령 발행을 뜻하며 이동 완료가 아니다.
 */
static bool CD_ABSMove(unsigned char ch, unsigned int Acc, unsigned int Vel, S32 Pos){
	sMotionCommand_t MotionCmd;
	if (!CDecap_ValidateTarget(ch, Pos)) return false;
    int64_t current = ch == aZ ? xCD.Decapper.Motor_CurPos : CDecap_GetAxisPosition(ch);
    if(ch == aZ){
        int64_t displacement = (int64_t)Pos - current;
        if (!CDecap_CheckZMove(xCD.Decapper.Motor_CurPos, Pos)) return false;
        sZMoveDirection = (displacement > 0) - (displacement < 0);
        PauseContext.zTargetPos = Pos;
        PauseContext.zAcc = Acc;
        PauseContext.zVel = Vel;
    }
    else if(ch == aR){
        PauseContext.rTargetPos = Pos;
        PauseContext.rAcc = Acc;
        PauseContext.rVel = Vel;
    }
    sMoveTarget[ch] = Pos;
    sMovePending[ch] = true;

	MotionCmd.Axis = ch;
	MotionCmd.Mode = MODE_ABS;
	MotionCmd.Acc = Acc;
	MotionCmd.Vel = CDecap_ScaleVelocity((S32)Vel);
	MotionCmd.Pos = CDecap_AxisToDriver(ch, Pos); /* 목표·검사·재개 정보는 앱 좌표, 드라이버에만 변환해 넘긴다 */
	CDecap_SetMoveTimeout(ch, (int64_t)Pos - current, MotionCmd.Vel, Acc);
	Drive_AbsMove(&MotionCmd);
    return true;
}
/* 진행 중인 Z/R/Y 동작 정보를 보존한 채 모션 출력을 정지한다. */
/**
 * @brief 이동 중인 Z/R 여부와 Y 방향·정지 시작 시각을 보존한 뒤 출력을 정지한다.
 * 이미 일시정지했으면 재처리하지 않는다. 상위 FSM은 CAP/DECAP/LONGRUN에만 이 기능을 허용한다.
 */
static void CDecap_Pause(void){
	if (PauseContext.isPaused){
		return;
	}

	/* 구동 신호가 아직 0이어도 미도착 명령은 재개 대상으로 보존한다. */
	PauseContext.resumeZ = sMovePending[aZ] && !CDecap_IsMoveComplete(aZ);
	PauseContext.resumeR = sMovePending[aR] && !CDecap_IsMoveComplete(aR);
	PauseContext.resumeY = (PauseContext.yDirection != PAUSE_Y_NONE);
	PauseContext.pauseStartTick = gTriggerCount;

	for (U08 ch = 0; ch < STEP_CH_MAX; ch++){
		CDecap_Stop(ch);
	}
	Air_Y_StopOutput();
	memset(sHoldApplied, 0, sizeof(sHoldApplied));
	PauseContext.isPaused = true;
}

/* 저장된 Z/R 목표 위치와 Y 방향을 복원하여 동작을 재개한다. */
/**
 * @brief 저장한 Z/R 절대 목표와 Y 방향을 재발행하고 이전 FSM 단계로 돌아간다.
 * 일시정지 시간만큼 타임아웃 및 단계 대기 기준 시각을 보정한다.
 * 재개 목표가 현재 제한에 맞지 않으면 오류 정지하므로 호출 도중 종료될 수 있다.
 */
static void CDecap_Resume(void){
	teFSM_Decapper savedPhase = PauseContext.savedPhase;
	Pause_Y_Direction_t savedYDirection = PauseContext.yDirection;
	bool resumeZ = PauseContext.resumeZ;
	bool resumeR = PauseContext.resumeR;
	bool resumeY = PauseContext.resumeY;
	U32 pausedTicks = gTriggerCount - PauseContext.pauseStartTick;

	/* PAUSE 시간은 기존 timeout/dwell 경과시간에서 제외한다. */
	Step_StartTick += pausedTicks;
	for (U08 idx = 0; idx < SUB_MAX; idx++){
		previousTime[idx] += pausedTicks;
	}

	PauseContext.isPaused = false;
	PauseContext.resumeZ = false;
	PauseContext.resumeR = false;
	PauseContext.resumeY = false;
	PauseContext.savedPhase = DFSM_IDLE;
	PauseContext.pauseStartTick = 0U;

	/* 정지 전 경과시간이 단계 시간에 남으므로 재발행이 허용 시간을 줄이지 않게 한다. */
	sResuming = true;
	bool reissued = (!resumeZ || CD_ABSMove(aZ,PauseContext.zAcc,PauseContext.zVel,PauseContext.zTargetPos)) &&
	                (!resumeR || CD_ABSMove(aR,PauseContext.rAcc,PauseContext.rVel,PauseContext.rTargetPos));
	sResuming = false;
	if (!reissued) return;

	if(resumeY && savedYDirection == PAUSE_Y_FORWARD){
		Air_Y_Forward();
	}

	if(resumeY && savedYDirection == PAUSE_Y_BACK){
		Air_Y_Back();
	}

	sPhase = savedPhase;
}

/* HOMING 완료 후 Z를 0 pulse, Y를 High 리미트로 복귀한다. */
/**
 * @brief HOME 완료 상태에서 Z를 0 pulse로 이동하고, Body를 푼 뒤 Y를 Forward로 복귀시킨다.
 * 새 원점 탐색이 아닌 기존 좌표계의 복귀 동작이다. 미원점 상태면 이동 없이 END_OK로 종료한다.
 * @note 오류 정지 후 Body가 잡힌 채 남아 있을 수 있으므로 Y를 움직이기 전에 항상 Body를 푼다
 *       (열림 확인 또는 BODY_GRIP_WAIT_MS 경과).
 */
static void CDecap_Origin(void){
    switch(Origin_Step){
        case Recovery__IDLE:
            if (!xSL.isHomed){
                Origin_Step = Recovery__IDLE;
                sPhase = DFSM_END_OK;
                break;
            }
            /* Z축을 pulse 0 위치로 절대 이동 */
            if (!CD_ABSMove(aZ,xPL.Decapper.ZDecapAcc, xPL.Decapper.ZDecapVel, 0)) return;
            CDecap_StartTimeout();
            Origin_Step = Recovery_Z;
            break;

        case Recovery_Z:
            if(CDecap_CheckTimeout(CDecap_MotionTimeout(HOMING_TIMEOUT_MS), "Z origin stop or target not confirmed")){ return; }

            /* Z축 이동 완료 확인 후, Y를 움직이기 전에 Body를 먼저 푼다. */
            if(CDecap_IsMoveComplete(aZ)){
                Air_CTBody_Gripper(CT_GRIP_RELEASE);
                previousTime[SUB_BODY_GRIP_WAIT] = gTriggerCount;
                Origin_Step = Recovery_Body_Release;
            }
            break;

        case Recovery_Body_Release:
            if(!CDecap_WaitBodyRelease()) return;
            /* Y축을 Forward 방향으로 이동 */
            Air_Y_Forward();
            CDecap_StartTimeout();
            Origin_Step = Recovery_Y;
            break;

        case Recovery_Y:
            if(CDecap_CheckSensorTimeout(HOMING_TIMEOUT_MS, ERROR_CODE_DECAP_Y_FORWARD_NOT_REACHED, "Y Forward limit not reached")){ return; }

            /* Y_F 센서는 Active Low: 0이면 Forward 위치 도착 */
            if(!xSL.CDecapping_Sensor.Y_F_Limit_Sensor){
                Air_Y_Stop();
                Origin_Step = Recovery_COMPLETE;
            }
            break;

        case Recovery_COMPLETE:
            Origin_Step = Recovery__IDLE;
            sPhase = DFSM_END_OK;
            break;

        default:
            ERR_MSG_SEND("Invalid Origin Step: %d", Origin_Step);
            Air_Y_Stop();
            Origin_Step = Recovery__IDLE;
            sPhase = DFSM_END_OK;
            break;
    }
}

/**
 * @brief Body 해제 출력 뒤 Y를 움직여도 되는지 판정한다.
 * Open=1/Close=0(열림)이 보이면 바로, 아니면 BODY_GRIP_WAIT_MS 경과 후 true다.
 * 해제 실패를 오류로 정지하는 코드는 아직 없으므로 시간 경과로 진행한다. previousTime이라 PAUSE 보정 대상이다.
 */
static bool CDecap_WaitBodyRelease(void){
	bool opened = xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open &&
	              !xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Close;
	return opened || (gTriggerCount - previousTime[SUB_BODY_GRIP_WAIT]) >= MS_TO_TICKS(BODY_GRIP_WAIT_MS);
}

/**
 * @brief 그리퍼 Open/Close가 동시에 감지(논리값 1/1)되면 센서 고장으로 오류 정지한다.
 * 정상 조합: 비어 있으면 풀림 Open=1/Close=0 또는 닫힘 Open=0/Close=1, 뚜껑·바이알을 물고 있으면 0/0. 11은 어떤 상태에서도 나올 수 없다.
 * GRIP_SENSOR_FAULT_MS 이상 이어질 때만 오류로 본다(순간 노이즈 무시). 대기 중에도 검사하며, 정지(STOP) 처리 중만 건너뛴다.
 * 유지보수 중이거나 오류가 이미 래치된 동안에는 호출부(상태머신)가 이 검사 전에 반환하므로 같은 오류를 반복하지 않는다.
 * @return 이상 없으면 true, 오류 정지를 수행했으면 false(호출부는 이번 주기 처리를 중단한다).
 * @note 2026-11-20 사용자 요청. Body는 E2881, Cap은 E2871. CLR 후에도 11이면 다시 오류가 난다(센서 점검 필요).
 */
static bool CDecap_CheckGripSensorFault(void){
	static U32 bodySince = 0U, capSince = 0U;
	static bool bodyActive = false, capActive = false;
	bool checking = sPhase != DFSM_STOP;
	bool body11 = checking && xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open &&
	              xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Close;
	bool cap11 = checking && xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Open &&
	             xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Close;

	if (!body11) bodyActive = false;
	else if (!bodyActive){ bodyActive = true; bodySince = gTriggerCount; }
	if (!cap11) capActive = false;
	else if (!capActive){ capActive = true; capSince = gTriggerCount; }

	if (bodyActive && (gTriggerCount - bodySince) >= MS_TO_TICKS(GRIP_SENSOR_FAULT_MS)){
		bodyActive = false;
		CDecap_Fault(ERROR_CODE_DECAP_BODY_GRIP_SENSOR_FAULT, "Body grip sensor fault (Open=1 Close=1)");
		return false;
	}
	if (capActive && (gTriggerCount - capSince) >= MS_TO_TICKS(GRIP_SENSOR_FAULT_MS)){
		capActive = false;
		CDecap_Fault(ERROR_CODE_DECAP_CAP_GRIP_SENSOR_FAULT, "Cap grip sensor fault (Open=1 Close=1)");
		return false;
	}
	return true;
}

/**
 * @brief 체결·분리의 공통 단계를 진행한다. 자동 운전은 Vial 고정과 Y 이송을 추가한다.
 * 자동 순서: CT 확인 → (Body 해제 확인) → Y 후진(Vial 당겨오기) → Y Back 감지 후 1초 대기 → Body 잡기·확인 → Z/R 분리·체결
 *            → Z 복귀 → Body 풀기·대기 → Y 전진 → 완료. Z/R은 Y Back + Body 고정 상태에서만 움직인다.
 * 유닛 운전(UDECAP/UCAP)은 Y/Body 단계 없이 Z/R/Cap 그리퍼만 사용한다.
 * @note 단계 enum 값은 GERD step 번호로 노출되므로 기존 값을 유지하고 Body_Release만 뒤에 추가했다.
 */
static void CDecap_RunCapSequence(bool capping, bool transferVial){
	U32 timeout = capping ? CAPPING_TIMEOUT_MS : DECAPPING_TIMEOUT_MS;
	CDecapping_Statmachin_state ready = capping ? CDecapping_Capping_Ready : CDecapping_Decapping_Ready;

	switch (CDecapping_Step){
	case CDecapping_Idle:
		if (!transferVial){
			CDecapping_Step = ready;
		}
		else if (xSL.CDecapping_Sensor.CT_Detect_Sensor && xSL.isHomed){
			xprintf("There is CT \n");
			/* Y를 움직이기 전에 Body가 풀려 있어야 한다. 오류 정지 후에는 잡힌 채 남아 있을 수 있다. */
			Air_CTBody_Gripper(CT_GRIP_RELEASE);
			previousTime[SUB_BODY_GRIP_WAIT] = gTriggerCount;
			CDecapping_Step = CDecapping_Body_PreRelease;
		}
		else {
			xprintf("There is no CT.\n");
			sPhase = DFSM_END_OK;
		}
		break;

	case CDecapping_Body_PreRelease:
		if (!CDecap_WaitBodyRelease()) return;
		/* Vial을 Y 후진으로 작업 위치까지 당겨온다. */
		Air_Y_Back();
		CDecap_StartTimeout();
		CDecapping_Step = CDecapping_Move_Pos;
		break;

	case CDecapping_Move_Pos:
		/* Y_B 센서는 Active Low: 0이면 Back(작업 위치) 도착 */
		if (CDecap_CheckSensorTimeout(timeout, ERROR_CODE_DECAP_Y_BACK_NOT_REACHED, "Y Back limit not reached") || xSL.CDecapping_Sensor.Y_B_Limit_Sensor) return;
		Air_Y_Stop();
		/* Y Back 감지 시점부터 Y_BACK_SETTLE_MS 기다린 뒤 Body를 잡는다(2026-11-19 사용자 요청). */
		previousTime[SUB_BODY_GRIP_WAIT] = gTriggerCount;
		CDecapping_Step = CDecapping_Y_Back_Settle;
		break;

	case CDecapping_Y_Back_Settle:
		if ((gTriggerCount - previousTime[SUB_BODY_GRIP_WAIT]) < MS_TO_TICKS(Y_BACK_SETTLE_MS)) return;
		CDecapping_Step = CDecapping_Body_Grip;
		break;

	case CDecapping_Body_Grip:
		/* 작업 위치에서 Body를 잡아 Vial을 고정한다. 확인은 다음 단계에서 한다. */
		Air_CTBody_Gripper(CT_GRIP_ENGAGE);
		previousTime[SUB_BODY_GRIP_WAIT] = gTriggerCount;
		CDecapping_Step = ready;
		break;

	case CDecapping_Capping_Ready:
	case CDecapping_Decapping_Ready: {
		/* Body 고정(Open=0, Close=0)을 확인한 뒤에만 Z/R 작업을 시작한다. */
		if (transferVial && !CDecap_WaitForGripper(SUB_BODY_GRIP_WAIT, BODY_GRIP_WAIT_MS,
				!xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open &&
				!xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Close)) return;
		S32 approach = capping ? xPL.Decapper.ZCap_UpPos : xPL.Decapper.ZCap_SidePos;
		if (!CD_ABSMove(aZ, xPL.Decapper.ZDecapAcc, xPL.Decapper.ZDecapVel, approach)) return;
		CDecap_StartTimeout();
		CDecapping_Step = capping ? CDecapping_sData_Capping : CDecapping_Cap_Capgrip;
		break;
	}

	case CDecapping_Cap_Capgrip:
		if (CDecap_CheckTimeout(CDecap_MotionTimeout(timeout),"Z approach stop or target not confirmed") || !CDecap_IsMoveComplete(aZ)) return;
		Air_CTCap_Gripper(CT_GRIP_ENGAGE);
		previousTime[SUB_CAPPING] = gTriggerCount;
		CDecapping_Step = CDecapping_sData_Decapping;
		break;

	case CDecapping_sData_Capping:
		/* CAP: Z가 Up 위치에 도착하면 바로 돌지 않고 CAP_UP_DWELL_MS 동안 멈춘다. */
		if (CDecap_CheckTimeout(CDecap_MotionTimeout(timeout),"Z approach stop or target not confirmed") || !CDecap_IsMoveComplete(aZ)) return;
		previousTime[SUB_CAPPING] = gTriggerCount;
		CDecapping_Step = CDecapping_Cap_UpWait;
		break;

	case CDecapping_Cap_UpWait:
	case CDecapping_sData_Decapping: {
		/* CAP은 Up 도착 후 대기, DECAP은 Cap 잡기 후 대기. 둘 다 previousTime이라 PAUSE 보정 대상이다. */
		U32 waitMs = capping ? CAP_UP_DWELL_MS : CAP_GRIP_WAIT_MS;
		if ((gTriggerCount - previousTime[SUB_CAPPING]) < MS_TO_TICKS(waitMs)) return;

		S32 zTarget = capping ? xPL.Decapper.ZCap_SidePos : xPL.Decapper.ZCap_UpPos;
		S32 rTarget = capping ? 0 : xPL.Decapper.RDecapPos;
		/* 두 축의 목표가 모두 유효한 경우에만 R/Z 이동을 시작한다. */
		if (!CDecap_ValidateTarget(aZ, zTarget) ||
		    !CDecap_CheckZMove(xCD.Decapper.Motor_CurPos, zTarget)) return;
		/* R이 RDecapPos(4바퀴)를 도는 동안 Z가 Up↔Side를 같이 끝내도록 이 구간의 속도만 맞춘다(2026-11-20). */
		U32 zAcc = xPL.Decapper.ZDecapAcc;
		S32 zVel = xPL.Decapper.ZDecapVel;
		S32 rVel = xPL.Decapper.RDecapVel;
		CDecap_SyncZWithR(zTarget, rTarget, &zAcc, &zVel, xPL.Decapper.RDecapAcc, &rVel);
		if (!CD_ABSMove(aR, xPL.Decapper.RDecapAcc, (unsigned int)rVel, rTarget)) return;
		if (!CD_ABSMove(aZ, zAcc, (unsigned int)zVel, zTarget)) return;
		CDecap_StartTimeout();
		CDecapping_Step = CDecapping_Move_PrevR;
		break;
	}

	case CDecapping_Move_PrevR:
		if (gTriggerCount - Step_StartTick >= CDecap_MotionTimeout(timeout) / 10){
			bool zReached = CDecap_HasReachedTarget(aZ);
			bool rReached = CDecap_HasReachedTarget(aR);
			const char *condition = zReached && !rReached ? "R stop or target not confirmed" :
			                        !zReached && rReached ? "Z stop or target not confirmed" :
			                        "Z/R move exceeded the time limit";
			CDecap_Fault(ERROR_CODE_DECAP_TIMEOUT, condition);
			return;
		}
		if (!CDecap_IsMoveComplete(aZ) || !CDecap_IsMoveComplete(aR)) return;
		if (capping){
			Air_CTCap_Gripper(CT_GRIP_RELEASE);
			previousTime[SUB_CAP_UNGRIP_WAIT] = gTriggerCount;
		}
		CDecapping_Step = CDecapping_Move_PrevZ;
		break;

	case CDecapping_Move_PrevZ:
		if (capping && !CDecap_WaitForGripper(SUB_CAP_UNGRIP_WAIT, CAP_UNGRIP_WAIT_MS,
				xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Open &&
				!xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Close)) return;
		if (!CD_ABSMove(aZ, xPL.Decapper.ZDecapAcc, xPL.Decapper.ZDecapVel,
				xPL.Decapper.ZCap_Origin_Position)) return;
		CDecap_StartTimeout();
		CDecapping_Step = capping || transferVial ? CDecapping_Move_PrevY : CDecapping_Done;
		break;

	case CDecapping_Move_PrevY:
		if (CDecap_CheckTimeout(CDecap_MotionTimeout(timeout),"Z return stop or target not confirmed") || !CDecap_IsMoveComplete(aZ)) return;
		if (transferVial){
			/* Z 복귀 후 Body를 먼저 풀고, 해제 시간을 기다린 뒤 Y를 움직인다. */
			Air_CTBody_Gripper(CT_GRIP_RELEASE);
			previousTime[SUB_BODY_GRIP_WAIT] = gTriggerCount;
			CDecapping_Step = CDecapping_Body_Release;
		}
		else {
			CDecapping_Step = CDecapping_Done;
		}
		break;

	case CDecapping_Body_Release:
		if (!CDecap_WaitBodyRelease()) return;
		Air_Y_Forward();
		CDecap_StartTimeout();
		CDecapping_Step = CDecapping_Complete_Wait;
		break;

	case CDecapping_Complete_Wait:
		if (CDecap_CheckSensorTimeout(timeout, ERROR_CODE_DECAP_Y_FORWARD_NOT_REACHED, "Y Forward limit not reached") || xSL.CDecapping_Sensor.Y_F_Limit_Sensor) return;
		Air_Y_Stop();
		CDecapping_Step = CDecapping_Done;
		break;

	case CDecapping_Done:
		/* 유닛 DECAP은 이 단계에서 마지막 Z 복귀를 확인한다. */
		if (!capping && !transferVial &&
				(CDecap_CheckTimeout(CDecap_MotionTimeout(timeout), "Z return stop or target not confirmed") || !CDecap_IsMoveComplete(aZ))) return;
		CDecapping_Step = CDecapping_Idle;
		sPhase = DFSM_END_OK;
		break;
	}
}

/** @brief Vial을 이송하고 뚜껑을 체결한다. */
static void CDecap_Capping(void){
	CDecap_RunCapSequence(true, true);
}

/** @brief Vial을 이송하고 뚜껑을 분리한다. */
static void CDecap_Decapping(void){
	CDecap_RunCapSequence(false, true);
}

/** @brief Z/R과 Cap 그리퍼만 사용하여 유닛 체결 시험을 실행한다. */
static void UCDecap_Capping(void){
	CDecap_RunCapSequence(true, false);
}

/** @brief Z/R과 Cap 그리퍼만 사용하여 유닛 분리 시험을 실행한다. */
static void UCDecap_Decapping(void){
	CDecap_RunCapSequence(false, false);
}

/* STOP 전까지 자동 Decapping과 Capping을 반복하고, Capping 완료마다 횟수를 증가시킨다. */
/**
 * @brief 자동 DECAP과 CAP을 번갈아 반복하고 CAP 완료마다 LongRunCount를 증가시킨다.
 * 각 절반은 단독 DECAP/CAP과 같은 전체 시퀀스(Body 잡기, Y 후진, Z/R 분리·체결, Y 전진, Body 풀기)다.
 * LR n은 n번째 CAP 완료에서 끝나고, 0(무제한)은 STOP이나 오류가 반복을 종료한다.
 * @note 단독 DECAP/CAP은 CT 미감지면 동작 없이 END_OK로 끝난다. LR에서는 그대로 두면 기구는 멈춘 채
 *       횟수만 증가하므로, 각 절반 시작 시 CT가 없으면 E2820(상태 오류)으로 정지한다.
 * @note 절반(DECAP 또는 CAP)이 끝날 때마다 LR_INTERVAL_MS 동안 출력·모터 변화 없이 쉰 뒤 다음 절반을 시작한다.
 *       대기는 previousTime이라 PAUSE 시간이 보정되고, 마지막 CAP(LR n 완료) 뒤에는 쉬지 않고 끝난다.
 */
static void Decap_cap_LR(void){
	if (sLongRunInterval){
		if ((gTriggerCount - previousTime[SUB_LR_INTERVAL]) < MS_TO_TICKS(LR_INTERVAL_MS)) return;
		sLongRunInterval = false;
	}
	/* 2026-11-20: CT를 한 번만 읽지 않고 CT_DETECT_WAIT_MS까지 감지를 기다린다(바이알이 자리 잡기 전에 읽히는 문제). */
	if (CDecapping_Step == CDecapping_Idle && !xSL.CDecapping_Sensor.CT_Detect_Sensor){
		if (!sLongRunCtWait){
			sLongRunCtWait = true;
			previousTime[SUB_LR_CT_WAIT] = gTriggerCount;
			return;
		}
		if ((gTriggerCount - previousTime[SUB_LR_CT_WAIT]) < MS_TO_TICKS(CT_DETECT_WAIT_MS)) return;
		sLongRunCtWait = false;
		CDecap_Fault(ERROR_CODE_DECAP_STATE, "CT vial not detected");
		return;
	}
	sLongRunCtWait = false;
	switch (xCD.Decapper.phaseDecapCap){
		case LONGRUN_DECAP:
			CDecap_Decapping();
			if (sPhase == DFSM_END_OK){
				sPhase = DFSM_LONGRUN;
				xCD.Decapper.phaseDecapCap = LONGRUN_CAP;
				sLongRunInterval = true;
				previousTime[SUB_LR_INTERVAL] = gTriggerCount;
			}
			break;

		case LONGRUN_CAP:
			CDecap_Capping();
			if (sPhase == DFSM_END_OK){
				xCD.Decapper.LongRunCount++;
				/* 완료 횟수 알림(TCP는 네트워크 태스크가 전송, USB 콘솔은 여기서 출력) */
				taskENTER_CRITICAL();
				sLongRunReportCount = xCD.Decapper.LongRunCount;
				sLongRunReportPending = true;
				taskEXIT_CRITICAL();
				xprintf("LRCOUNT %lu\n", (unsigned long)xCD.Decapper.LongRunCount);
				if (sLongRunLimit && xCD.Decapper.LongRunCount >= sLongRunLimit) {
					return; /* retain END_OK to finish the complete cycle */
				}
				sPhase = DFSM_LONGRUN;
				xCD.Decapper.phaseDecapCap = LONGRUN_DECAP;
				sLongRunInterval = true;
				previousTime[SUB_LR_INTERVAL] = gTriggerCount;
			}
			break;

		default:
			CDecapping_Step = CDecapping_Idle;
			xCD.Decapper.phaseDecapCap = LONGRUN_DECAP;
			sPhase = DFSM_LONGRUN;
			break;
	}
}

bool CDecap_IsValidTargetFor(const tsXPL_Decapper *parameters, U08 axis, S32 position){
    S32 lower, upper;
    return CDecap_GetLimits(parameters, axis, &lower, &upper) && position >= lower && position <= upper;
}

bool CDecap_IsValidTarget(U08 axis, S32 position){
    return CDecap_IsValidTargetFor(&xPL.Decapper, axis, position);
}
int CDecap_GetPhase(void) { return (int)sPhase; }
U32 CDecap_GetOperationId(void) { return sOperationId; }
int CDecap_GetOperationResult(void) { return sOperationResult; }
bool CDecap_IsPaused(void) { return PauseContext.isPaused; }
U32 CDecap_GetLongRunLimit(void) { return sLongRunLimit; }

/**
 * @brief 아직 보내지 않은 LR 사이클 완료 횟수가 있으면 꺼내고 true를 반환한다(한 번만 반환).
 * 네트워크 태스크가 호출해 "LRCOUNT n"을 보낸다. 여러 사이클이 밀렸으면 마지막 횟수만 남는다.
 */
bool CDecap_TakeLongRunReport(U32 *count){
    bool pending;
    taskENTER_CRITICAL();
    pending = sLongRunReportPending;
    if (pending){
        if (count) *count = sLongRunReportCount;
        sLongRunReportPending = false;
    }
    taskEXIT_CRITICAL();
    return pending;
}
/**
 * @brief HOME으로 확정한 Z/R 절대 좌표(ZCap_*, RDecapPos)를 쓰는 체결·분리 동작인지 판정한다.
 * 미원점 상태의 0은 전원 투입 위치라 기구 위치와 맞지 않으므로 이 동작들은 HOME 뒤에만 허용한다.
 */
bool CDecap_ActionNeedsHome(teXActionType action){
    return action == ACTION_DECAP || action == ACTION_CAP || action == ACTION_UDECAP ||
           action == ACTION_UCAP || action == ACTION_LONGRUN;
}

/* LR도 TryRequestAction의 HOME 조건을 그대로 따른다. */
bool CDecap_TryLongRun(U32 limit) {
    bool accepted;
    taskENTER_CRITICAL();
    accepted = CDecap_TryRequestAction(ACTION_LONGRUN);
    if (accepted) sLongRunLimit = limit;
    taskEXIT_CRITICAL();
    return accepted;
}

/* ACTION → Phase 매핑 테이블 */
static const teFSM_Decapper ActionToPhase[ACTION_MAX] = {
		[ACTION_NONE] /*     	*/ = DFSM_IDLE,
		/* 자동 동작 */
		[ACTION_STOP] /*     	*/ = DFSM_STOP,
		[ACTION_HOME] /*     	*/ = DFSM_HOME,
    	[ACTION_DECAP] /*    	*/ = DFSM_DECAP,
		[ACTION_CAP] /*      	*/ = DFSM_CAP,

		[ACTION_PAUSE] /*     	*/ = DFSM_PAUSE,
		[ACTION_RESUME] /*     	*/ = DFSM_RESUME,

		/* 수동 동작 */
		[ACTION_ORIGIN] /*   	*/ = DFSM_ORIGIN,

		[ACTION_RMOVEZ] /*   	*/ = DFSM_RMOVEZ,
		[ACTION_AMOVEZ] /*   	*/ = DFSM_AMOVEZ,
		[ACTION_RROTATE] /*   	*/ = DFSM_RROTATE,
		[ACTION_AROTATE] /*   	*/ = DFSM_AROTATE,

		[ACTION_Y_F_MOVE] /*   	*/ = DFSM_Y_F_MOVE,
		[ACTION_Y_B_MOVE] /*   	*/ = DFSM_Y_B_MOVE,


		[ACTION_UDECAP] /*   	*/ = DFSM_UDECAP,
		[ACTION_UCAP] /*   		*/ = DFSM_UCAP,
		[ACTION_LONGRUN] /*   	*/ = DFSM_LONGRUN,

		[ACTION_BODY_GRIP] /*  	*/ = DFSM_CT_BODY_GRIP,
		[ACTION_CAP_GRIP] /*   	*/ = DFSM_CT_CAP_GRIP,
};

/* FSM 핸들러 테이블 */
static FSM_Handler FSM_Table[DFSM_PHASE_MAX] = {
		[DFSM_IDLE] /*      	*/ = CDecap_Idle,
		/* 자동 동작 */
		[DFSM_STOP] /*      	*/ = Decapper_StopAll,
		[DFSM_HOME] /*      	*/ = CDecap_Homing,

		[DFSM_DECAP] /*    		*/ = CDecap_Decapping,
    	[DFSM_CAP] /*       	*/ = CDecap_Capping,

		[DFSM_PAUSE] /*    		*/ = CDecap_Pause,
    	[DFSM_RESUME] /*       	*/ = CDecap_Resume,

		/* 수동 동작 */
		[DFSM_ORIGIN] /*    	*/ = CDecap_Origin,

    	[DFSM_RROTATE] /*    	*/ = CDecap_RRotate,
		[DFSM_AROTATE] /*    	*/ = CDecap_ARotate,

		[DFSM_RMOVEZ] /*    	*/ = CDecap_Relmove_Z,
		[DFSM_AMOVEZ] /*    	*/ = CDecap_Absmove_Z,

		[DFSM_Y_F_MOVE] /*    	*/ = Air_Y_Forward_Move,
		[DFSM_Y_B_MOVE] /*    	*/ = Air_Y_Back_Move,

		[DFSM_UDECAP] /*    	*/ = UCDecap_Decapping,
		[DFSM_UCAP] /*    		*/ = UCDecap_Capping,
		[DFSM_LONGRUN] /*    	*/ = Decap_cap_LR,

		[DFSM_CT_CAP_GRIP] /*   */ = CT_Cap_Grip,
		[DFSM_CT_BODY_GRIP] /*  */ = CT_Body_Grip,

    	[DFSM_ABNORMAL] /*  	*/ = FSM_Abnormal,
		[DFSM_END_OK] /*    	*/ = FSM_End_ok,
};

static bool CDecap_CanPause(void){
    return sPhase == DFSM_CAP || sPhase == DFSM_DECAP || sPhase == DFSM_LONGRUN;
}

static bool CDecap_CanResume(void){
    return sPhase == DFSM_PAUSE && PauseContext.isPaused;
}

/* 요청된 액션에 따라 디캐퍼 FSM을 한 주기 실행한다. */
/**
 * @brief 애플리케이션 제어 주기마다 요청을 해석하고 현재 FSM 핸들러를 한 번 실행한다.
 * STOP을 최우선 처리하고 유지보수·오류 중에는 일반 동작을 막는다. 일반 요청은 IDLE에서만 시작한다.
 * PAUSE/RESUME 조건과 이동 범위를 검사한다. HOME은 좌표 확립 중이므로 현재 위치 범위 감시에서 제외한다.
 * 센서 조합 오류를 정지에 연결하는 블록은 기구 확인 전까지 주석 상태로 유지되어 있다.
 */
void CDecap_Action_Statemachine(teXActionType actionType) {
	/* 제어 주기의 모터 상태를 읽는다. STOP과 유지보수에는 읽기를 생략한다. */
	if (!sStopRequested && actionType != ACTION_STOP && !sMaintenance) {
		xCDecap.Status_Update();
		xCD.Decapper.Motor_CurPos = CDecap_GetAxisPosition(aZ); /* 앱 좌표(아래 = +) */
		/* 방금 읽은 구동 상태로 판단한다. 오류 상태에서도 적용되도록 오류 분기보다 앞에 둔다. */
		CDecap_HoldCurrentWhenStopped();
	}

	/* STOP은 현재 상태와 관계없이 즉시 처리한다. */
	if (sStopRequested || actionType == ACTION_STOP) {
        taskENTER_CRITICAL();
        sStopRequested = false;
        taskEXIT_CRITICAL();
		sPhase = DFSM_STOP;
	}
	else if (sMaintenance){
        return;
    }
    else if (xSL.isError || sFaultCode != ERROR_CODE_NONE){
        xAT = ACTION_NONE;
        CDecap_ReportError();
        return;
    }
    else if (actionType == ACTION_PAUSE) {
		if (CDecap_CanPause()){
			PauseContext.savedPhase = sPhase;
			sPhase = DFSM_PAUSE;
		}
		xAT = ACTION_NONE;
	}
	else if (actionType == ACTION_RESUME) {
		if (CDecap_CanResume()){
			sPhase = DFSM_RESUME;
		}
		xAT = ACTION_NONE;
	}

	/* IDLE 상태에서만 일반 액션을 FSM Phase로 전환한다. */
	else if (sPhase == DFSM_IDLE && actionType != ACTION_NONE) {
		if ((int) actionType > 0 && (int) actionType < ACTION_MAX) {
			if (actionType == ACTION_LONGRUN) {
				xCD.Decapper.phaseDecapCap = LONGRUN_DECAP;
				xCD.Decapper.LongRunCount = 0U;
				CDecapping_Step = CDecapping_Idle;
				sLongRunInterval = false; /* 첫 DECAP은 대기 없이 시작 */
				sLongRunCtWait = false;
				sLongRunReportPending = false; /* 이전 LR의 보내지 못한 알림은 버린다 */
			}
			sPhase = ActionToPhase[actionType];
			xSL.isBusy = YES;
		} else {
			ERR_MSG_SEND("Invalid actionType: %d", actionType);
			xAT = ACTION_NONE;
			return;
		}
	}

	/* [1] Phase 유효성 검증 */
	if (sPhase < 0 || sPhase >= DFSM_PHASE_MAX){
		ERR_MSG_SEND("Invalid Phase=%d (0x%08X). Forcing ABNORMAL.", sPhase, (unsigned)sPhase);
		sPhase = DFSM_ABNORMAL;
	}

//	/* 래치된 센서 오류가 있으면 비정상 처리 상태로 전환한다. */
//	if(xSL.Decapper.Z_HL_isError || xSL.Decapper.Y_FB_isError
//		|| xSL.Decapper.CT_Cap_Grip_isError || xSL.Decapper.CT_Body_Grip_isError){
//		xSL.isBusy = NO;
//		sPhase = DFSM_ABNORMAL;
//	}

    /* 그리퍼 Open/Close 동시 감지(11)는 대기·동작과 무관하게 센서 고장으로 보고 즉시 오류 정지한다(2026-11-20 사용자 요청). */
    if (!CDecap_CheckGripSensorFault()) return;

    if (sPhase != DFSM_STOP && !CDecap_CheckZTravelLimit()) return;

    /* HOME establishes the coordinate reference before pulse limits apply. */
    if (sPhase != DFSM_STOP && sPhase != DFSM_HOME && sPhase != DFSM_ABNORMAL &&
        !CDecap_CheckMotionLimits()) return;

	/* 현재 Phase의 핸들러를 실행한다. */
	if (FSM_Table[sPhase]) {
		FSM_Table[sPhase]();
	}
}
/* 액션 완료 후 상태를 정리하고 IDLE로 복귀한다. */
/**
 * @brief 완료된 액션의 Busy·요청·하위 단계·일시정지 문맥을 정리하여 IDLE로 복귀한다.
 */
static void FSM_End_ok(void) {
    sOperationResult = 2;
	/* 정상 종료: IDLE 복귀 */
	sPhase = DFSM_IDLE;
	CDecapping_Step = CDecapping_Idle;
	xSL.isBusy = NO;
	xAT = ACTION_NONE;
	ResetAllSubPhases();
	ResetPauseContext();
}
/* 오류 상태를 보고하고 모든 디캐퍼 모션을 정지한다. */
/**
 * @brief 센서 오류 플래그를 진단 출력하고 상태 오류 2820으로 전체 모션을 정지한다.
 * 오류 해제는 자동으로 하지 않으며 대기 상태에서 별도 CLER가 필요하다.
 */
static void FSM_Abnormal(void){
    ERR_MSG_SEND("H_L Limit Sensor	%d",xSL.Decapper.Z_HL_isError);
    ERR_MSG_SEND("CT_Body Sensor 	%d",xSL.Decapper.CT_Body_Grip_isError);
    ERR_MSG_SEND("CT_Cap Sensor 	%d",xSL.Decapper.CT_Cap_Grip_isError);
    ERR_MSG_SEND("Y Limit Sensor 	%d",xSL.Decapper.Y_FB_isError);

    CDecap_Fault(ERROR_CODE_DECAP_STATE, "Abnormal Decapper state");
}
/* 모든 하위 단계와 타이머를 초기화한다. */
/**
 * @brief 하위 단계 배열·시각·시작/완료 비트와 HOME/ORG 진행 단계를 초기화한다.
 * 하드웨어 출력 변경 없이 다음 동작이 처음 단계에서 시작하도록 한다.
 */
static void ResetAllSubPhases(void) {
    sZMoveDirection = 0;
    sLongRunInterval = false;
    sLongRunCtWait = false;
    memset(sHoldApplied, 0, sizeof(sHoldApplied)); /* 대기 진입 후 정지를 다시 확인하고 전류를 내린다 */
    memset(sMoveTimeoutMs, 0, sizeof(sMoveTimeoutMs));
    memset(sMovePending, 0, sizeof(sMovePending));
    memset(sMoveTarget, 0, sizeof(sMoveTarget));
	memset(sSubPhase, 0, sizeof(sSubPhase));
	memset(previousTime, 0, sizeof(previousTime));
	sStartedFlags = 0;
	sDoneFlags = 0;
	HomingSeq.Step = CDECAP_HOMING_NONE;
	Origin_Step = Recovery__IDLE;
}

/* Pause/Resume에 사용한 저장 컨텍스트를 초기화한다. */
/**
 * @brief 일시정지 시 저장한 축 목표·방향·시각과 재개 플래그를 모두 초기화한다.
 */
static void ResetPauseContext(void) {
	memset(&PauseContext, 0, sizeof(PauseContext));
	PauseContext.savedPhase = DFSM_IDLE;
	PauseContext.yDirection = PAUSE_Y_NONE;
}


/* The STOP latch is independent of the single action mailbox. */
/**
 * @brief STOP을 독립 플래그와 xAT에 기록하여 다음 제어 주기에 우선 처리하게 한다.
 * 임계 구역에서 기록하므로 뒤따르는 PAUSE/RESUME가 xAT를 바꿔도 STOP 요청은 남는다.
 * 이 함수 자체가 하드웨어 정지 완료를 기다리지는 않는다.
 */
void CDecap_RequestStop(void){
    taskENTER_CRITICAL();
    sStopRequested = true;
    xAT = ACTION_STOP;
    taskEXIT_CRITICAL();
}

/**
 * @brief 유지보수·STOP·대기 액션이 없고 FSM/Busy 및 샘플된 Z/R 구동 상태가 모두 대기인지 반환한다.
 * 오류 유무와는 별도 판정이다. 오류 상태에서도 완전히 멈췄으면 CLER를 허용하기 위해 true가 될 수 있다.
 */
bool CDecap_IsIdle(void){
    return !sMaintenance && !sStopRequested && sPhase == DFSM_IDLE &&
           xSL.isBusy == NO && xAT == ACTION_NONE &&
           !xSL.xZ_Motor_Run.Motor_Run && !xSL.xR_Motor_Run.Motor_Run;
}

/**
 * @brief 저장·초기화 등 유지보수 작업이 일반 동작을 차단 중인지 반환한다.
 */
bool CDecap_IsMaintenanceActive(void){
    return sMaintenance;
}

/**
 * @brief 임계 구역에서 대기 여부 확인과 유지보수 잠금 획득을 함께 수행한다.
 * true일 때만 작업을 시작하며 성공한 호출은 모든 종료 경로에서 EndMaintenance와 짝을 맞춘다.
 * false이면 잠금을 얻지 못했으므로 타이머 정지나 설정 변경을 하면 안 된다.
 */
bool CDecap_BeginMaintenance(void){
    bool allowed;
    taskENTER_CRITICAL();
    allowed = CDecap_IsIdle();
    if (allowed) sMaintenance = true;
    taskEXIT_CRITICAL();
    return allowed;
}

/**
 * @brief 획득한 유지보수 잠금을 임계 구역에서 해제한다.
 * 주기 타이머를 정지한 호출부는 타이머를 복구한 뒤 이 함수를 호출한다.
 */
void CDecap_EndMaintenance(void){
    taskENTER_CRITICAL();
    sMaintenance = false;
    taskEXIT_CRITICAL();
}

/**
 * @brief 대기·오류 없음 확인과 일반 액션 등록을 임계 구역에서 함께 처리한다.
 * true는 요청 접수이며 완료가 아니다. false면 기존 요청을 유지한다.
 * 호출부가 유효한 일반 액션을 전달해야 한다. STOP은 RequestStop으로 별도 요청한다.
 * @note CDecap_ActionNeedsHome() 동작은 미원점(isHomed=0)이면 접수하지 않는다.
 */
bool CDecap_TryRequestAction(teXActionType action){
    bool allowed;
    taskENTER_CRITICAL();
    allowed = CDecap_IsIdle() && !xSL.isError && sFaultCode == ERROR_CODE_NONE &&
              (xSL.isHomed || !CDecap_ActionNeedsHome(action));
    if (allowed) {
        xAT = action;
        ++sOperationId;
        sOperationResult = 1;
    }
    taskEXIT_CRITICAL();
    return allowed;
}

bool CDecap_TryRequestPause(bool pause){
    taskENTER_CRITICAL();
    bool accepted = !sStopRequested && !xSL.isError &&
                    (pause ? CDecap_CanPause() : CDecap_CanResume());
    if (accepted) xAT = pause ? ACTION_PAUSE : ACTION_RESUME;
    taskEXIT_CRITICAL();
    return accepted;
}

/**
 * @brief 유지 중인 Decapper 오류를 xSL 및 공통 오류 조회 값과 동기화한다.
 * isError가 있고 저장된 원인이 없으면 상태 오류 2820을 사용한다. 모션 출력은 변경하지 않는다.
 */
void CDecap_ReportError(void){
    if (xSL.isError){
        teErrorCode code = sFaultCode != ERROR_CODE_NONE ? sFaultCode : ERROR_CODE_DECAP_STATE;
        xSL.errorCode = code;
        if (GetErrorCode_int() != code) SetErrorCode(code, __func__, __LINE__);
    }
}

/**
 * @brief 첫 오류 원인을 보존하고 isError를 설정한 뒤 전체 모션을 정지한다.
 * 후속 오류로 최초 원인을 덮지 않으며 ReportError로 외부 조회 값도 맞춘다.
 */
static void CDecap_Fault(teErrorCode code, const char *condition){
    static const char *const phaseNames[DFSM_PHASE_MAX] = {
        [DFSM_IDLE] = "IDLE",
        [DFSM_HOME] = "HOME",
        [DFSM_DECAP] = "DECAP",
        [DFSM_CAP] = "CAP",
        [DFSM_UDECAP] = "UDECAP",
        [DFSM_UCAP] = "UCAP",
        [DFSM_ORIGIN] = "ORG",
        [DFSM_RMOVEZ] = "MOVE R Z",
        [DFSM_AMOVEZ] = "MOVE A Z",
        [DFSM_RROTATE] = "MOVE R R",
        [DFSM_AROTATE] = "MOVE A R",
        [DFSM_Y_F_MOVE] = "READY 0",
        [DFSM_Y_B_MOVE] = "READY 1",
        [DFSM_PAUSE] = "PAUSE",
        [DFSM_RESUME] = "RESUME",
        [DFSM_ABNORMAL] = "ABNORMAL",
    };
    bool firstFault = sFaultCode == ERROR_CODE_NONE;
    teFSM_Decapper phase = sPhase;
    unsigned step = phase == DFSM_HOME ? HomingSeq.Step :
                    phase == DFSM_ORIGIN ? Origin_Step : CDecapping_Step;
    unsigned long completed = xCD.Decapper.LongRunCount;
    const char *lrHalf = "UNKNOWN";
    if (xCD.Decapper.phaseDecapCap == LONGRUN_CAP) lrHalf = "CAP";
    else if (xCD.Decapper.phaseDecapCap == LONGRUN_DECAP) lrHalf = "DECAP";

    sOperationResult = 4;
    if (firstFault) sFaultCode = code;
    xSL.isError = YES;
    xSL.errorCode = sFaultCode;
    Decapper_StopAll();

    if (firstFault){
        if (phase == DFSM_LONGRUN){
            snprintf(sFaultDetail, sizeof(sFaultDetail), "LR/%s step=%u completed=%lu - %s",
                     lrHalf, step, completed, condition);
        }
        else if ((unsigned)phase < DFSM_PHASE_MAX && phaseNames[phase]){
            snprintf(sFaultDetail, sizeof(sFaultDetail), "%s step=%u - %s",
                     phaseNames[phase], step, condition);
        }
        else {
            snprintf(sFaultDetail, sizeof(sFaultDetail), "phase=%u - %s", (unsigned)phase, condition);
        }
    }
    CDecap_ReportError();
}

/** @brief 정지 전에 기록한 첫 오류의 동작과 조건을 반환한다. */
const char *CDecap_GetFaultDetail(void){
    return xSL.isError && sFaultDetail[0] ? sFaultDetail : NULL;
}

/**
 * @brief ch 축의 기본 ±Limit_Pos 범위와 활성화된 소프트 리미트의 교집합을 구한다.
 * 성공 시 lower/upper에 pulse 경계를 쓰고 true를 반환한다. 포인터는 호출부가 유효하게 제공해야 한다.
 * 축·기본 범위·소프트 범위가 잘못되거나 교집합이 없으면 false이며 이 함수 자체는 오류를 설정하지 않는다.
 * @note R(회전)축은 리미트를 쓰지 않는다(2026-11-19 사용자 요청: 회전축이라 범위 제한이 필요 없음).
 *       Limit_PosR/SwNegLimit[R]/SwPosLimit[R] 값과 무관하게 TMC429 24비트 좌표 범위(±8388607)만 허용한다.
 */
static bool CDecap_GetLimits(const tsXPL_Decapper *parameters, U08 ch, S32 *lower, S32 *upper){
    if (ch >= STEP_CH_MAX) return false;
    if (ch == aR){
        *lower = -8388607;
        *upper = 8388607;
        return true;
    }
    S32 limit = parameters->Limit_PosZ;
    if (limit <= 0 || limit > 8388607) return false;
    *lower = -limit;
    *upper = limit;
    if (parameters->SoftLimitEnable){
        S32 neg = parameters->SwNegLimit[ch];
        S32 pos = parameters->SwPosLimit[ch];
        if (neg >= pos) return false;
        if (neg > *lower) *lower = neg;
        if (pos < *upper) *upper = pos;
    }
#if (Z_COORD_SIGN < 0)
    /* 2026-11-20: Z 좌표가 아래 = + 이므로 원점 확정 후 Z=0(High 센서보다 Z_HOME_OFFSET_PULSE 아래)보다 위(음수)는
     * 갈 이유가 없다. 부호 변경 전 EEPROM 값(예: PL 52 = -12000)이 남아 있으면 센서 너머 위쪽 목표가 되므로
     * 이동 전에 E2810(RANGE)으로 막는다. HOME 중(isHomed=0)의 -1000→0 오프셋 이동에는 적용하지 않는다. */
    if (xSL.isHomed && *lower < 0) *lower = 0;
#endif
    return *lower < *upper;
}

/**
 * @brief 64비트 목표 pulse가 유효한 축 범위 안인지 검사하며 양 끝 경계는 허용한다.
 * 실패 시 2810 오류와 전체 정지를 수행하고 false를 반환하므로 호출부는 후속 이동을 중단해야 한다.
 */
static bool CDecap_ValidatePosition(U08 ch, int64_t position, const char *kind){
    const char *axis = ch == aZ ? "Z" : ch == aR ? "R" : "Unknown axis";
    S32 lower, upper;
    char condition[80];
    if (!CDecap_GetLimits(&xPL.Decapper, ch, &lower, &upper)){
        snprintf(condition, sizeof(condition), "%s limits invalid", axis);
    }
    else if (position < INT32_MIN || position > INT32_MAX){
        snprintf(condition, sizeof(condition), "%s %s outside 32-bit range", axis, kind);
    }
    else if (position < lower || position > upper){
        snprintf(condition, sizeof(condition), "%s %s %ld outside [%ld,%ld] pulse",
                 axis, kind, (long)position, (long)lower, (long)upper);
    }
    else {
        return true;
    }
    CDecap_Fault(ERROR_CODE_DECAP_LIMIT, condition);
    return false;
}

static bool CDecap_ValidateTarget(U08 ch, int64_t target){
    return CDecap_ValidatePosition(ch, target, "target");
}

/* 2026-11-20 IO 리스트 기준: H(High) = Z+ = 1번 아래쪽(E2890), L(Low) = Z− = 2번 위쪽 = HOME 센서(E2891).
 * 리미트 입력(내부값)은 0일 때 감지 상태다. HOME은 − 방향(위)으로 L을 찾는다.
 * 0점은 L 감지 위치에서 Z_HOME_OFFSET_PULSE 아래이므로 평소 이동에서 L(위쪽)이 켜지면 항상 이상이다(예외 없음).
 * 2026-11-19 사용자 요청: 원점 확정 전(isHomed=0, HOME 진행 중 포함)에는 리미트를 검사하지 않는다.
 * 원점 확정 후에는 이동 방향과 무관하게 High/Low 중 하나라도 감지되면 오류 정지한다(복구는 CLR 후 HOME).
 * displacement는 호출부 호환을 위해 남겨 둔 인자이며 판정에 쓰지 않는다. */
static bool CDecap_CheckZLimit(int64_t displacement){
    (void)displacement;
    if (!xSL.isHomed) return true;
    bool high = !xSL.CDecapping_Sensor.Z_H_Limit_Sensor;
    bool low = !xSL.CDecapping_Sensor.Z_L_Limit_Sensor;
    if (high && low){
        CDecap_Fault(ERROR_CODE_DECAP_Z_LIMIT_CONFLICT, "Z High and Low limits both active (H=1 L=1)");
    }
    else if (high){
        CDecap_Fault(ERROR_CODE_DECAP_Z_HIGH_LIMIT, "Z travel blocked by High limit (H=1 L=0)");
    }
    else if (low){
        CDecap_Fault(ERROR_CODE_DECAP_Z_LOW_LIMIT, "Z travel blocked by Low limit (H=0 L=1)");
    }
    else {
        return true;
    }
    return false;
}

/** @brief 현재 위치에서 목표까지의 Z 이동을 리미트 입력으로 검사한다. */
static bool CDecap_CheckZMove(int64_t current, int64_t target){
    return CDecap_CheckZLimit(target - current);
}

/* 새 발행 전 검사에 더해 이미 시작한 Z 이동도 제어 주기마다 확인한다. */
static bool CDecap_CheckZTravelLimit(void){
    /* HOME 중에는 리미트를 검사하지 않는다(2026-11-19 사용자 요청). High 탐색·재접근의 정지는 HOME 시퀀스가,
     * 그 밖의 이상은 HOME 단계별 타임아웃이 처리한다. 재HOME 첫 주기에는 isHomed가 아직 1일 수 있어 여기서 먼저 거른다. */
    if (sPhase == DFSM_HOME) return true;
    if (!sMovePending[aZ] || !sZMoveDirection) return true;
    return CDecap_CheckZLimit(sZMoveDirection);
}

/**
 * @brief 샘플된 구동 상태가 ON인 Z/R 축의 현재 TMC429 위치를 범위 검사한다.
 * 모두 유효하면 true, 범위를 벗어나 오류 정지를 수행하면 false이다. HOME 제외 여부는 호출부가 결정한다.
 */
static bool CDecap_CheckMotionLimits(void){
    for (U08 ch = 0; ch < STEP_CH_MAX; ch++){
        if ((ch == aZ ? xSL.xZ_Motor_Run.Motor_Run : xSL.xR_Motor_Run.Motor_Run) &&
            !CDecap_ValidatePosition(ch, CDecap_GetAxisPosition(ch), "position")) return false;
    }
    return true;
}

/**
 * @brief 명령한 축의 정지 상태와 절대 목표 도착을 모두 확인한다.
 * 아직 시작하지 않은 정지 상태는 목표가 다르면 완료가 아니다. 0 pulse 이동은 바로 완료될 수 있다.
 * TMC429 명령 좌표의 정확한 일치를 확인하며 실제 기구 탈조를 검출하는 함수는 아니다.
 */
static bool CDecap_IsMoveComplete(U08 ch){
    if (!CDecap_HasReachedTarget(ch)) return false;
    sMovePending[ch] = false;
    return true;
}

/* 시간 초과 진단에서도 대기 중인 이동 상태를 건드리지 않고 확인한다. */
static bool CDecap_HasReachedTarget(U08 ch){
    bool running = ch == aZ ? xSL.xZ_Motor_Run.Motor_Run : xSL.xR_Motor_Run.Motor_Run;
    return !running && (!sMovePending[ch] || CDecap_GetAxisPosition(ch) == sMoveTarget[ch]);
}
