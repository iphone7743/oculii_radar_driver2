#ifndef _IMU_ERROR__
#define _IMU_ERROR__

/*! Oculii SDK namespace
 */
namespace oculii
{
    /*! ERROR_CODE enumeration flags for exception handling
     */
    enum class ImuErrorCode
    {
        SUCCESS, /*!< General success flag */
        FAILED, /*!< General failure flag */
        FAILED_TO_INIT_THREAD, /*!< Failed to initilize a thread */
        FAILED_TO_RECEIVE_DATA, /*!< Failed to reveive data */
        INVALID_PARAMETER, /*!< Function arguments error */
        GPSIMU_GRAB_ERROR, /*!< No new GPS/IMU package when calling get GPS/IMU function */
        INVALID_HEADER_FORMAT, /*!< Config file header format is invalid */
        INVALID_RADAR_FORMAT, /*!< Config file radar format is invalid */
        DATA_SIZE_MISMATCH /*!< Data structure sizes mismatch */
    };
}

#endif
