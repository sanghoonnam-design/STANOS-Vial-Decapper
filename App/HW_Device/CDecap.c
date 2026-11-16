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
static U32 sOperationId = 0U;
static int sOperationResult = 0; /* 0:none, 1:accepted/running, 2:finished, 3:stopped, 4:fault */
static teErrorCode sFaultCode = ERROR_CODE_NONE;
static char sFaultDetail[160];
/* 명령 발행 시의 절대 목표를 보존한다. 정지 신호만으로 완료 처리하지 않는다. */
static S32 sMoveTarget[STEP_CH_MAX];
static bool sMovePending[STEP_CH_MAX];

static bool CDecap_IsMoveComplete(U08 ch);
static bool CDecap_HasReachedTarget(U08 ch);

static void CDecap_Fault(teErrorCode code, const char *condition);
static bool CDecap_GetLimits(const tsXPL_Decapper *parameters, U08 ch, S32 *lower, S32 *upper);
static bool CDecap_ValidateTarget(U08 ch, int64_t target);
static bool CDecap_CheckMotionLimits(void);


static void CDecap_StartTimeout(void);
static bool CDecap_CheckTimeout(U32 timeoutMs, const char *condition);
static bool CDecap_CheckSensorTimeout(U32 timeoutMs, teErrorCode code, const char *condition);
static bool CDecap_WaitForGripper(U08 index, U32 minimumMs, bool sensorReady);

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
static void UCDecap_CDecap_Longrun(void);

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
 * @brief UpdateSIG에서 갱신한 IO 캐시의 Z/Y 리미트, 그리퍼, CT 감지 입력을 xSL에 복사한다.
 * 리미트는 후속 시퀀스에서 0을 도착으로 해석한다. 이 함수 자체는 출력을 변경하지 않는다.
 */
void CDecap_Sensor_Update(void) {
	//Z축 SENSOR 확인 (HIGH/LOW)
	xSL.CDecapping_Sensor.Z_H_Limit_Sensor = (digitalRead(READ_IN, Z_H_SENSOR_PIN) == GPIO_PIN_SET) ? 1 : 0;
	xSL.CDecapping_Sensor.Z_L_Limit_Sensor = (digitalRead(READ_IN, Z_L_SENSOR_PIN) == GPIO_PIN_SET) ? 1 : 0;
	//Y축 센서 확인 (Forward/Back)
	xSL.CDecapping_Sensor.Y_F_Limit_Sensor = (digitalRead(READ_IN, Y_F_SENSOR_PIN) == GPIO_PIN_SET) ? 1 : 0;
	xSL.CDecapping_Sensor.Y_B_Limit_Sensor = (digitalRead(READ_IN, Y_B_SENSOR_PIN) == GPIO_PIN_SET) ? 1 : 0;
	//CT Body를 잡는지 확인 (OPEN/CLOSE)
	xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open = (digitalRead(READ_IN, CT_BODY_GRIP_OPEN_SENSOR_PIN) == GPIO_PIN_SET) ? 1 : 0;
	xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Close = (digitalRead(READ_IN, CT_BODY_GRIP_CLOSE_SENSOR_PIN) == GPIO_PIN_SET) ? 1 : 0;
		//CT Cap을 잡는지 확인 (OPEN/CLOSE)
	xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Open = (digitalRead(READ_IN, CT_CAP_GRIP_OPEN_SENSOR_PIN) == GPIO_PIN_SET) ? 1 : 0;
	xSL.CDecapping_Sensor.CT_Cap_Grip_Detect_Close = (digitalRead(READ_IN, CT_CAP_GRIP_CLOSE_SENSOR_PIN) == GPIO_PIN_SET) ? 1 : 0;
		//CT가 있는지 확인
	xSL.CDecapping_Sensor.CT_Detect_Sensor = (digitalRead(READ_IN, CT_DETECT_OPEN_SENSOR_PIN) == GPIO_PIN_SET) ? 1 : 0;
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

/* 센서 대기의 기존 시간 제한을 사용하고 입력값을 함께 기록한다. */
static bool CDecap_CheckSensorTimeout(U32 timeoutMs, teErrorCode code, const char *condition){
    if (gTriggerCount - Step_StartTick < timeoutMs / 10) return false;

    char detail[100];
    if (code == ERROR_CODE_DECAP_Z_HOME_NOT_REACHED || code == ERROR_CODE_DECAP_Z_HOME_NOT_RELEASED){
        snprintf(detail, sizeof(detail), "%s (H=%u L=%u expected H=%u)", condition,
                 (unsigned)xSL.CDecapping_Sensor.Z_H_Limit_Sensor,
                 (unsigned)xSL.CDecapping_Sensor.Z_L_Limit_Sensor,
                 (unsigned)(code == ERROR_CODE_DECAP_Z_HOME_NOT_RELEASED));
    }
    else {
        snprintf(detail, sizeof(detail), "%s (F=%u B=%u expected %s=0)", condition,
                 (unsigned)xSL.CDecapping_Sensor.Y_F_Limit_Sensor,
                 (unsigned)xSL.CDecapping_Sensor.Y_B_Limit_Sensor,
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

		int64_t target = position;
		if (relative) target += TMC429_GetPosition(axis);
		if (!CDecap_ValidateTarget(axis, target)) return;
		sMoveTarget[axis] = (S32)target;
		sMovePending[axis] = true;
		CDecap_StartTimeout();

		sMotionCommand_t command = {
			.Axis = axis,
			.Mode = relative ? MODE_REL : MODE_ABS,
			.Acc = T_MOTOR_ACC,
			.Vel = CDecap_ScaleVelocity(T_MOTOR_VEL),
			.Pos = position,
		};
		if (relative) Drive_RelMove(&command);
		else Drive_AbsMove(&command);
	}

	switch (sSubPhase[index]){
	case MOVING:
		if (CDecap_CheckTimeout(READY_TIMEOUT_MS,
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

/** @brief MOVE R Z에서 받은 Z축 증분을 실행한다. */
static void CDecap_Relmove_Z(void){
	CDecap_ManualMotorMove(SUB_zRMOVE, aZ, true, xCD.Decapper.Motor_TargetPos[aZ]);
}

/** @brief MOVE A Z에서 받은 Z축 절대좌표를 실행한다. */
static void CDecap_Absmove_Z(void){
	CDecap_ManualMotorMove(SUB_zAMOVE, aZ, false, xCD.Decapper.Motor_TargetPos[aZ]);
}

/** @brief MOVE R R에서 받은 R축 증분을 실행한다. */
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
 * @brief ch 축에 감속 정지 명령을 보내고 유지 전류 설정을 적용한다.
 * 정지 완료를 기다리는 함수는 아니며 상위 상태 정리는 호출부가 담당한다.
 */
static void CDecap_Stop(unsigned char ch){
	sMotionCommand_t MotionCmd;
	MotionCmd.Axis = ch;
	MotionCmd.Mode = MODE_STOP;
	MotionCmd.Acc = MOTOR_ACC;
	MotionCmd.Vel = MOTOR_SPD_SAFE;
	MotionCmd.Pos = 0;
	Drive_Stop(&MotionCmd);
	TMC2660_SetHoldCurrent(ch);
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
/* Z축을 홈 기준으로 설정한 뒤 공압 Y축을 Forward 리미트로 복귀한다. */
/**
 * @brief Z 고속 탐색 → 저속 후퇴 → 저속 재접근 → 좌표 0 설정 → Y Forward 복귀를 진행한다.
 * 각 호출은 현재 단계만 수행하며 센서 대기는 타임아웃으로 보호한다.
 * 이미 isHomed이면 종료한다. R축 원점 탐색은 이 함수에 포함되어 있지 않다.
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
		HomingSeq.H_Acc = MOTOR_ACC;
		/* TMC429_VelMove()는 실제 pulse/s가 아닌 내부 속도 단위를 받음 */
		HomingSeq.H_Vel = CDecap_ScaleVelocity(MOTOR_SPD_HOME_FAST / STEP_PULSE_RATE_R);
		HomingSeq.L_Acc = MOTOR_ACC;
		HomingSeq.L_Vel = -CDecap_ScaleVelocity(MOTOR_SPD_HOME_SLOW / STEP_PULSE_RATE_R);
		HomingSeq.Step = CDECAP_HOMING_START;
	}


	switch(Homing->Step){
	    case CDECAP_HOMING_START:
	    	Homing->Step = CDECAP_HOMING_H_SEARCH;
	        break;
	    case CDECAP_HOMING_H_SEARCH:
	        TMC429_VelMove(aZ, Homing->H_Acc, Homing->H_Vel);
	        CDecap_StartTimeout();
	        Homing->Step = CDECAP_HOMING_H_REACH;
	        break;
	    case CDECAP_HOMING_H_REACH:
	        if (CDecap_CheckSensorTimeout(HOMING_TIMEOUT_MS, ERROR_CODE_DECAP_Z_HOME_NOT_REACHED, "Z home sensor not reached during fast search")){return;}
	    	if(xSL.CDecapping_Sensor.Z_H_Limit_Sensor){ return; }
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
	    	TMC429_VelMove(aZ, Homing->L_Acc, Homing->L_Vel);
	    	CDecap_StartTimeout();
	        Homing->Step = CDECAP_HOMING_L_REACH_OFF;
	        break;
	    case CDECAP_HOMING_L_REACH_OFF:
	        /* 현재 프로젝트의 기준: 0 = Home 감지, 1 = 센서 해제 */
	        if (CDecap_CheckSensorTimeout(HOMING_TIMEOUT_MS, ERROR_CODE_DECAP_Z_HOME_NOT_RELEASED, "Z home sensor did not release")){return;}
	        if(!xSL.CDecapping_Sensor.Z_H_Limit_Sensor) { return; }
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
	        TMC429_VelMove(aZ, Homing->L_Acc, -Homing->L_Vel);
	        CDecap_StartTimeout();
	        Homing->Step = CDECAP_HOMING_S_REACH;
	        break;
	    case CDECAP_HOMING_S_REACH:
	        /* Home 센서가 다시 감지될 때까지 이동 */
            if (CDecap_CheckSensorTimeout(HOMING_TIMEOUT_MS, ERROR_CODE_DECAP_Z_HOME_NOT_REACHED, "Z home sensor not reached during slow search")){return;}
	        if(xSL.CDecapping_Sensor.Z_H_Limit_Sensor) { return; }
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
	        TMC429_SetPosition(aZ, 0);
	        Homing->Step = CDECAP_HOMING_Y_HOME;
	        break;
	    case CDECAP_HOMING_Y_HOME:
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
    sMoveTarget[ch] = Pos;
    sMovePending[ch] = true;

    if(ch == aZ){
        PauseContext.zTargetPos = Pos;
        PauseContext.zAcc = Acc;
        PauseContext.zVel = Vel;
    }
    else if(ch == aR){
        PauseContext.rTargetPos = Pos;
        PauseContext.rAcc = Acc;
        PauseContext.rVel = Vel;
    }

	MotionCmd.Axis = ch;
	MotionCmd.Mode = MODE_ABS;
	MotionCmd.Acc = Acc;
	MotionCmd.Vel = CDecap_ScaleVelocity((S32)Vel);
	MotionCmd.Pos = Pos;
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

	if(resumeZ){
		if (!CD_ABSMove(aZ,PauseContext.zAcc,PauseContext.zVel,PauseContext.zTargetPos)) return;
	}

	if(resumeR){
		if (!CD_ABSMove(aR,PauseContext.rAcc,PauseContext.rVel,PauseContext.rTargetPos)) return;
	}

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
 * @brief HOME 완료 상태에서 Z를 0 pulse로 이동한 뒤 Y를 High로 복귀시킨다.
 * 새 원점 탐색이 아닌 기존 좌표계의 복귀 동작이다. 미원점 상태면 이동 없이 END_OK로 종료한다.
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
            if(CDecap_CheckTimeout(HOMING_TIMEOUT_MS, "Z origin stop or target not confirmed")){ return; }

            /* Z축 이동 완료 확인 */
            if(CDecap_IsMoveComplete(aZ)){
                /* Y축을 H 방향으로 이동 */
                Air_Y_Forward();
                CDecap_StartTimeout();
                Origin_Step = Recovery_Y;
            }
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
 * @brief 체결·분리의 공통 단계를 진행한다. 자동 운전은 Vial 고정과 Y 이송을 추가한다.
 * 유닛 운전과 Longrun에서도 단계 번호, Cap 대기, Z/R 정지 및 도착 확인을 유지한다.
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
			CDecapping_Step = CDecapping_Body_Grip;
		}
		else {
			xprintf("There is no CT.\n");
			sPhase = DFSM_END_OK;
		}
		break;

	case CDecapping_Body_Grip:
		Air_CTBody_Gripper(CT_GRIP_ENGAGE);
		previousTime[SUB_BODY_GRIP_WAIT] = gTriggerCount;
		CDecapping_Step = CDecapping_Move_Pos;
		break;

	case CDecapping_Move_Pos:
		if (!CDecap_WaitForGripper(SUB_BODY_GRIP_WAIT, BODY_GRIP_WAIT_MS,
				!xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Open &&
				!xSL.CDecapping_Sensor.CT_Body_Grip_Detect_Close)) return;
		Air_Y_Back();
		CDecap_StartTimeout();
		CDecapping_Step = ready;
		break;

	case CDecapping_Capping_Ready:
	case CDecapping_Decapping_Ready: {
		if (transferVial){
			if (CDecap_CheckSensorTimeout(timeout, ERROR_CODE_DECAP_Y_BACK_NOT_REACHED, "Y Back limit not reached") || xSL.CDecapping_Sensor.Y_B_Limit_Sensor) return;
			Air_Y_Stop();
		}
		S32 approach = capping ? xPL.Decapper.ZCap_UpPos : xPL.Decapper.ZCap_SidePos;
		if (!CD_ABSMove(aZ, xPL.Decapper.ZDecapAcc, xPL.Decapper.ZDecapVel, approach)) return;
		CDecap_StartTimeout();
		CDecapping_Step = capping ? CDecapping_sData_Capping : CDecapping_Cap_Capgrip;
		break;
	}

	case CDecapping_Cap_Capgrip:
		if (CDecap_CheckTimeout(timeout, "Z approach stop or target not confirmed") || !CDecap_IsMoveComplete(aZ)) return;
		Air_CTCap_Gripper(CT_GRIP_ENGAGE);
		previousTime[SUB_CAPPING] = gTriggerCount;
		CDecapping_Step = CDecapping_sData_Decapping;
		break;

	case CDecapping_sData_Capping:
	case CDecapping_sData_Decapping: {
		if (capping){
			if (CDecap_CheckTimeout(timeout, "Z approach stop or target not confirmed") || !CDecap_IsMoveComplete(aZ)) return;
		}
		else if ((gTriggerCount - previousTime[SUB_CAPPING]) < MS_TO_TICKS(CAP_GRIP_WAIT_MS)){
			return;
		}

		S32 zTarget = capping ? xPL.Decapper.ZCap_SidePos : xPL.Decapper.ZCap_UpPos;
		S32 rTarget = capping ? 0 : xPL.Decapper.RDecapPos;
		/* 두 축의 목표가 모두 유효한 경우에만 R/Z 이동을 시작한다. */
		if (!CDecap_ValidateTarget(aZ, zTarget)) return;
		if (!CD_ABSMove(aR, xPL.Decapper.RDecapAcc, xPL.Decapper.RDecapVel, rTarget)) return;
		if (!CD_ABSMove(aZ, xPL.Decapper.ZDecapAcc, xPL.Decapper.ZDecapVel, zTarget)) return;
		CDecap_StartTimeout();
		CDecapping_Step = CDecapping_Move_PrevR;
		break;
	}

	case CDecapping_Move_PrevR:
		if (gTriggerCount - Step_StartTick >= timeout / 10){
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
		if (CDecap_CheckTimeout(timeout, "Z return stop or target not confirmed") || !CDecap_IsMoveComplete(aZ)) return;
		if (transferVial){
			Air_Y_Forward();
			CDecap_StartTimeout();
			CDecapping_Step = CDecapping_Complete_Wait;
		}
		else {
			CDecapping_Step = CDecapping_Done;
		}
		break;

	case CDecapping_Complete_Wait:
		if (CDecap_CheckSensorTimeout(timeout, ERROR_CODE_DECAP_Y_FORWARD_NOT_REACHED, "Y Forward limit not reached") || xSL.CDecapping_Sensor.Y_F_Limit_Sensor) return;
		Air_Y_Stop();
		Air_CTBody_Gripper(CT_GRIP_RELEASE);
		CDecapping_Step = CDecapping_Done;
		break;

	case CDecapping_Done:
		/* 유닛 DECAP은 이 단계에서 마지막 Z 복귀를 확인한다. */
		if (!capping && !transferVial &&
				(CDecap_CheckTimeout(timeout, "Z return stop or target not confirmed") || !CDecap_IsMoveComplete(aZ))) return;
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

/* STOP 전까지 Decapping과 Capping을 반복하고, Capping 완료마다 횟수를 증가시킨다. */
/**
 * @brief 유닛 DECAP과 CAP을 번갈아 반복하고 CAP 완료마다 LongRunCount를 증가시킨다.
 * 고정 반복 횟수는 없으며 STOP이나 오류가 반복을 종료한다. 자동 시퀀스의 Y/Body 동작은 포함하지 않는다.
 */
static void UCDecap_CDecap_Longrun(void){
	switch (xCD.Decapper.phaseDecapCap){
		case LONGRUN_DECAP:
			UCDecap_Decapping();
			if (sPhase == DFSM_END_OK){
				sPhase = DFSM_LONGRUN;
				xCD.Decapper.phaseDecapCap = LONGRUN_CAP;
			}
			break;

		case LONGRUN_CAP:
			UCDecap_Capping();
			if (sPhase == DFSM_END_OK){
				xCD.Decapper.LongRunCount++;
				if (sLongRunLimit && xCD.Decapper.LongRunCount >= sLongRunLimit) {
					return; /* retain END_OK to finish the complete cycle */
				}
				sPhase = DFSM_LONGRUN;
				xCD.Decapper.phaseDecapCap = LONGRUN_DECAP;
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
		[DFSM_LONGRUN] /*    	*/ = UCDecap_CDecap_Longrun,

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
		xCD.Decapper.Motor_CurPos = TMC429_GetPosition(aZ);
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

    /* HOME establishes the coordinate reference; sensor interlock remains deferred. */
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
 */
bool CDecap_TryRequestAction(teXActionType action){
    bool allowed;
    taskENTER_CRITICAL();
    allowed = CDecap_IsIdle() && !xSL.isError && sFaultCode == ERROR_CODE_NONE;
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
 */
static bool CDecap_GetLimits(const tsXPL_Decapper *parameters, U08 ch, S32 *lower, S32 *upper){
    if (ch >= STEP_CH_MAX) return false;
    S32 limit = ch == aZ ? parameters->Limit_PosZ : parameters->Limit_PosR;
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

/**
 * @brief 샘플된 구동 상태가 ON인 Z/R 축의 현재 TMC429 위치를 범위 검사한다.
 * 모두 유효하면 true, 범위를 벗어나 오류 정지를 수행하면 false이다. HOME 제외 여부는 호출부가 결정한다.
 */
static bool CDecap_CheckMotionLimits(void){
    for (U08 ch = 0; ch < STEP_CH_MAX; ch++){
        if ((ch == aZ ? xSL.xZ_Motor_Run.Motor_Run : xSL.xR_Motor_Run.Motor_Run) &&
            !CDecap_ValidatePosition(ch, TMC429_GetPosition(ch), "position")) return false;
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
    return !running && (!sMovePending[ch] || TMC429_GetPosition(ch) == sMoveTarget[ch]);
}
