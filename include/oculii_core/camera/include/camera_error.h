#ifndef _CAMERA_ERROR__
#define _CAMERA_ERROR__

/*! Oculii SDK namespace
 */
namespace oculii
{
    /*! ERROR_CODE enumeration flags for exception handling
     */
    enum class CameraErrorCode
    {
        SUCCESS, /*!< General success flag */
        FAILED, /*!< General failure flag */
        FAILED_TO_INIT_THREAD, /*!< Failed to initilize a thread */
        FAILED_TO_RECEIVE_DATA, /*!< Failed to reveive data */
        INVALID_PARAMETER, /*!< Function arguments error */
        IMG_GRAB_ERROR, /*!< No new image when calling get image function */
        SENSOR_CONNECTION_ERROR, /*!< Cannot connect to the desired sesnor */
        FILE_PATH_DOESNT_EXIST, /*!< File path doesn't exist */
        INVALID_CAMERA_FORMAT, /*!< Config file camera format is invalid */
        DATA_SIZE_MISMATCH /*!< Data structure sizes mismatch */
    };
}

#endif
