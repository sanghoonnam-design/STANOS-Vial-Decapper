/*******************************************************************************
 * XParser.c
 *
 *  Created on: 2024.10.14
 *      Author: RND. Kang PilSoon.
 *
 ******************************************************************************/
#include "XParser.h"
#include <errno.h>
#include <limits.h>
#include <math.h>
#include "XDebug.h"

tsXParsedData xParsedData_RS232C;  // Serial Command 파싱된 데이터
tsXParsedData xParsedData_Network; // Network Command 파싱된 데이터
tsXParsedData xParsedData_USB;     // USB Command 파싱된 데이터

static void xParser_PrintParsedData(const tsXParsedData *data); // 파싱된 데이터를 출력하는 함수 (디버깅용)

U08 DB_isPrintParsingDataEnabled = NO; // 디버깅 코드: 파싱데이터를 display 하는 조건

// 노이즈 필터링 : 유효한 문자인지 확인하는 함수
static int xParser_IsValidChar(char c)
{
    return isalnum((unsigned char)c) || c == ' ' || c == ',' || c == '.' || c == '-' || c == '+' || c == '_' ||
           c == '\r' || c == '\n' || c == '?' || c == 'h' || c == 'H' ||
           c == 's' || c == 'S';
}

static teXParsingErrorCode xParser_ParseCommand(const char *buffer, tsXParsedData *data) {
    const char *p = buffer;
    while (*p == ' ') ++p;
    int length = 0;
    while (*p && *p != ' ') {
        if (length >= MAX_CMD_LEN-1) return PARSER_ERR_INVALID_CMD;
        /* UPCASE는 인자를 여러 번 평가하는 매크로(project.h)라 *p++를 직접 넘기면 p가 두 번 증가해 글자를 건너뛴다. */
        char ch = *p++;
        data->Command[length++] = UPCASE(ch);
    }
    data->Command[length] = '\0';
    if (!length) return PARSER_ERR_INVALID_CMD;
    data->ParamCount = 0;
    while (*p) {
        while (*p == ' ' || *p == ',') ++p;
        if (!*p) break;
        if (data->ParamCount >= MAX_PARAMS) return PARSER_ERR_INVALID_CMD;
        char token[64]; int n = 0;
        while (*p && *p != ' ' && *p != ',') {
            if (n >= (int)sizeof(token)-1) return PARSER_ERR_INVALID_CMD;
            token[n++] = *p++;
        }
        token[n] = '\0';
        tsParam *param = &data->Params[data->ParamCount++];
        if (n == 1 && (isalpha((unsigned char)token[0]) || token[0] == '?')) {
            param->type = PARAM_TYPE_INT;
            param->value._int = UPCASE(token[0]);
        } else {
            char *end;
            errno = 0;
            if (strchr(token, '.') || strchr(token, 'e') || strchr(token, 'E')) {
                float value = strtof(token, &end);
                if (*end || errno == ERANGE || !isfinite(value)) return PARSER_ERR_INVALID_CMD;
                param->type = PARAM_TYPE_FLOAT;
                param->value._float = value;
            } else {
                long value = strtol(token, &end, 10);
                if (*end || errno == ERANGE || value < INT_MIN || value > INT_MAX) return PARSER_ERR_INVALID_CMD;
                param->type = PARAM_TYPE_INT;
                param->value._int = (int)value;
            }
        }
    }
    return PARSER_ERR_SUCCESS;
}

// 시리얼 데이터를 처리하고 노이즈 필터링하는 함수: 2중 처리
teXParsingErrorCode xParser_ProcessReceivedData(const char *rxBuffer, tsXParsedData *parsedData, bool useTCP) {
    (void)useTCP;
    if (!rxBuffer || !parsedData) return PARSER_ERR_INVALID_CMD;
    size_t length = strlen(rxBuffer);
    if (length < 3 || length >= BUFFER_SIZE_RX_MSG ||
        rxBuffer[length-2] != '\r' || rxBuffer[length-1] != '\n') return PARSER_ERR_INVALID_CMD;
    for (size_t i=0; i<length-2; ++i)
        if (!xParser_IsValidChar(rxBuffer[i]) || rxBuffer[i]=='\r' || rxBuffer[i]=='\n')
            return PARSER_ERR_NOISE_DETECTED;
    memset(parsedData, 0, sizeof(*parsedData));
    memcpy(parsedData->RawBuffer, rxBuffer, length-2);
    teXParsingErrorCode result=xParser_ParseCommand(parsedData->RawBuffer, parsedData);
    if(result==PARSER_ERR_SUCCESS && DB_isPrintParsingDataEnabled)xParser_PrintParsedData(parsedData);
    return result;
}

void xParser_HandleError(const char *functionName, teXParsingErrorCode errorCode, U08 position)
{
    vTaskDelay(100);

    switch (errorCode)
    {
    case PARSER_ERR_INVALID_CMD:
        ERR_MSG_SEND("%s(%d): Invalid command format.", functionName, position);
        break;
    case PARSER_ERR_NOISE_DETECTED:
        ERR_MSG_SEND("%s(%d): Noise detected in input.", functionName, position);
        break;
    case PARSER_ERR_TIMEOUT:
        ERR_MSG_SEND("%s(%d): Time out.", functionName, position);
        break;
    case PARSER_ERR_HELP_COMMAND_EXPLANATION: // not used.
        // printf("Info: Help requested for a specific command.\n");
        break;
    case PARSER_ERR_HELP_COMMAND_LIST_CALLED: // not used.
        // printf("Info: Help command list has been called.\n");
        break;
    default:
        ERR_MSG_SEND("%s(%d): Unknown error.", functionName, position);
        break;
    }
}

// 파싱된 데이터를 출력하는 함수 (디버깅용)
static void xParser_PrintParsedData(const tsXParsedData *data)
{
    xprintf("\tCommand     : %s", data->Command);
    xprintf("\tParam.Count : %d", data->ParamCount);
    char paramBuffer[256];
    int offset = 0;

    for (int i = 0; i < data->ParamCount; i++)
    {
        if (data->Params[i].type == PARAM_TYPE_INT)
        {
            offset += snprintf(paramBuffer + offset, sizeof(paramBuffer) - offset,
                               "%s%d", (i == 0 ? "" : ","), data->Params[i].value._int);
        }
        else if (data->Params[i].type == PARAM_TYPE_FLOAT)
        {
            offset += snprintf(paramBuffer + offset, sizeof(paramBuffer) - offset,
                               "%s%7.3f", (i == 0 ? "" : ","), data->Params[i].value._float);
        }

        // 버퍼 오버플로우 방지
        if (offset >= sizeof(paramBuffer) - 1)
        {
            break;
        }
    }

    xprintf("\tParameters  : %s\r\n", paramBuffer);
}
