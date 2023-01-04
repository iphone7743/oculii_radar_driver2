#ifndef _CAN_H_
#define _CAN_H_

#ifdef _WIN32
    #include <windows.h>
    #include <tchar.h>
#endif

#include <iostream>
#include <string.h>
#include <unordered_map>
#include <vector>

#include "CAN/CANTP.h"

/* Should have the same value than into offr_config.h */
#define CAN_TX_ID_DET_TRACK_LIST 0xC1U

#define MAX_NBR_DETECTION_OBJ 2000
#define MAX_NBR_TRACKER_OBJ 200

#define CAN_TP_FRAME_BUFFER_SIZE (48 + MAX_NBR_DETECTION_OBJ * 16 + MAX_NBR_TRACKER_OBJ * 32)

/* Information offset in output frame */
#define OFFSET_FRAME_NBR_BYTE_0 8
#define OFFSET_FRAME_NBR_BYTE_1 9
#define OFFSET_FRAME_NBR_BYTE_2 10
#define OFFSET_FRAME_NBR_BYTE_3 11

#define OFFSET_SW_VERSION_BYTE_0 12
#define OFFSET_SW_VERSION_BYTE_1 13
#define OFFSET_SW_VERSION_BYTE_2 14
#define OFFSET_SW_VERSION_BYTE_3 15

#define OFFSET_NBR_DETECTION_BYTE_0 16
#define OFFSET_NBR_DETECTION_BYTE_1 17

#define OFFSET_NBR_TRACK_BYTE_0 18
#define OFFSET_NBR_TRACK_BYTE_1 19

class CAN
{
public:
    /// @cond DEV
    void canInit(int radID);
    void canReceive(void);
    void canExit(void);

    canData canDataObj;
    typedef struct CanFdRx_Obj_t
    {
        TPCANHandle canHdl;
        char bitRateParam[256];
        
        #ifdef _WIN32
            HANDLE rxEventHdl;
        #else
            fd_set fds; /* Event Handle to manage CAN reception via events */
            int fd;
        #endif
        
        unsigned int numFrameRequested;
        unsigned int numFrameDone;
        unsigned int numFrameSaved;
        unsigned int numFrameDropped;
        //ofstream* pFile;

    } CanFdRx_Obj;
    CanFdRx_Obj canFdRxObj;

private:
    int canReadMessage(void);
    void canProcessCANStatusError(const TPCANStatus errorStatus);
    int canProcessRxMessage(const TPCANMsgFD canMsg);
    void canProcessCompleteCANTPFrame(void);

    BYTE canFrameBuff_[CAN_TP_FRAME_BUFFER_SIZE];
    int sensorID = -1;

    unsigned int lutCanDataSize_[16];

    typedef union u16BigEndian_t
    {
        BYTE u8[2];
        unsigned short int u16;
    } u16BigEndian_;

    typedef union u32BigEndian_t
    {
        BYTE u8[4];
        unsigned int u32;
    } u32BigEndian_;
    /// @endcond
};

#endif /* DEV_CANFD_RECEIVER_H_ */
