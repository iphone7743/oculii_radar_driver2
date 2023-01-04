/*
 * CANTP.h
 *
 *  Created on: Sep 19, 2019
 *      Author: Aymeric
 */

#ifndef DEV_CANTP_H_
#define DEV_CANTP_H_

#ifdef _WIN32
    #include <windows.h>
    #include "CAN_WINDOWS/PCANBasic.h"
#else
    #include <PCANBasic.h>
#endif



/* Should have the same value than into offr_config.h */
#define CAN_TP_PACKET_SIZE 64U

#define CAN_TP_HEADER_SIZE 3U /* 1 Byte  - Frame info \
                               * 2 Bytes - Remaining Data Size*/

#define CAN_TP_PAYLOAD_SIZE (CAN_TP_PACKET_SIZE - CAN_TP_HEADER_SIZE)

#define CAN_TP_FIRST_FRAME_MASK 0x80U
#define CAN_TP_FRAME_ID_MASK 0x7FU

/// @cond DEV
/*! CanTP_Obj_t type to define a CANFD socket
 */
typedef struct CanTP_Obj_t
{
    BYTE waitNewFrame;                  /*!< Flag                                                     */
    BYTE frameInfoId;                   /*!< Frame Id value from FrameInfo field                      */
    unsigned int nextRemainingDataSize; /*!< Expected remaining data size value in the next CAN frame */
    unsigned int dataSize;              /*!< Number of bytes written in the buffer                    */
    unsigned int frameBuffSize;         /*!< Buffer size                                              */
    BYTE *frameBuff;                    /*!< Pointer to the buffer where to store the decode result   */
} canData;

/*! extern function to initialize CANFD
 * @param dataObj is a canData pointer which is the camera ip
 * @param frameBuff is a pointer to the data read buffer
 * @param frameBuffSize is a unsigned integer which indicates data read buffer size
 * @return None
 */
extern void canTpInit(canData *dataObj, BYTE *frameBuff, unsigned int frameBuffSize);
extern int canTpDecode(canData *dataObj, const TPCANMsgFD canMsg);
extern int canTpUpdateBuffer(canData *dataObj, const TPCANMsgFD canMsg);
/// @endcond

#endif /* DEV_CANTP_H_ */
