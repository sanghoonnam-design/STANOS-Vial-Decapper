/*******************************************************************************
 * XNetworkMsg_Process.c
 *
 *  Created on: 2025.06.20
 *      Author: RND. Kang PilSoon.
 ******************************************************************************/
#include "_10_XNetworkMsg_Process.h"
#include "_10_XCommand_Core.h"
#include "XParser.h"
#include "XErrorCode.h"

/*******************************************************************************
 ** @brief TCP transmit message handling
 *******************************************************************************/
void TCP_Send(const char *msg)
{
    size_t len = strlen(msg);

    // 8000 포트로 메시지 전송
    size_t sent = xMessageBufferSend(xMessageBuffer_TCP, msg, len, 0);

    if (sent != len)
    {
        ERR_MSG_SEND_N("[TCP_Send: 8000] Message buffer full or send failed. Sent %u / %u bytes.", sent, len);
    }

    // TODO: GSTA 같은 응답은 생략 , 8001로 보내지 말것. SD 같이 메모리 고려

    // 8001 포트로 디버깅 메시지 전송
    // 폭주함.
    // Network_TCP_DebugSend((uint8_t *)msg, len);
}

BOOL Network_TCP_DebugSend(const uint8_t *msg, size_t len)
{
    size_t sent;

    // SD
    // 초기화가 되어 있으면,
    // SD q 에 입력

    // TCP
    if (xMessageBuffer_TCP_Debug == NULL ||
        xNet.isConnected_TCP_Debug == DISCONNECTED)
        return FALSE;

    sent = xMessageBufferSend(xMessageBuffer_TCP_Debug, msg, len, 0);

    if (sent != len)
    {
        ERR_MSG_SEND_N("[TCP_Send: 8002] Buffer full or send failed (%u / %u)",
                       (unsigned int)sent,
                       (unsigned int)len);
        return FALSE;
    }

    return TRUE;
}

/*******************************************************************************
** @brief This function is a function of parsing and processing data received
**        through TCP communication.
*******************************************************************************/

typedef struct {
    char data[BUFFER_SIZE_RX_MSG];
    unsigned int used;
    bool discarding;
    char previous;
} tsCommandFrame;
static tsCommandFrame sFrames[8];
void Network_ResetReceive(int socketId) {
    if (socketId >= 0 && socketId < 8) memset(&sFrames[socketId], 0, sizeof(sFrames[0]));
}

void Network_MsgProcess(int MsgId, int MsgLength, char *pData) {
    if (MsgId < 0 || MsgId >= 8 || !pData || MsgLength <= 0) return;
    if (gTriggerCount < __5sec) {
        Network_ResetReceive(MsgId);
        return;
    }
    tsCommandFrame *frame = &sFrames[MsgId];
    for (int i = 0; i < MsgLength; ++i) {
        char ch = pData[i];
        bool end = frame->previous == '\r' && ch == '\n';
        if (!frame->discarding) {
            if (frame->used >= sizeof(frame->data) - 1) {
                frame->used = 0;
                frame->discarding = true;
                SetErrorCode(ERROR_CODE_INVALID_COMMAND, __func__, __LINE__);
            } else frame->data[frame->used++] = ch;
        }
        frame->previous = ch;
        if (!end) continue;
        if (!frame->discarding && frame->used > 2) {
            frame->data[frame->used] = '\0';
            tsXParsedData parsed;
            teXParsingErrorCode result = xParser_ProcessReceivedData(frame->data, &parsed, COMM_TCP);
            if (result == PARSER_ERR_SUCCESS) Handle_command(&parsed, COMM_TCP);
            else SetErrorCode(ERROR_CODE_INVALID_COMMAND, __func__, __LINE__);
        }
        Network_ResetReceive(MsgId);
    }
}
