#ifndef _CORE_ERROR_
#define _CORE_ERROR__

/*! Oculii SDK namespace
 */
namespace oculii
{
    /*! ERROR_CODE enumeration flags for exception handling
     */
    enum ERROR_CODE
    {
        SUCCESS, /*!< General success flag */
        FAILED, /*!< General failure flag */
        FAILED_TO_INIT_THREAD, /*!< Failed to initilize a thread */
        FAILED_TO_RECEIVE_DATA, /*!< Failed to reveive data */
        PCL_GRAB_ERROR, /*!< No new detection package when calling get pointcloud function */
        TRK_GRAB_ERROR, /*!< No new tracker package when calling get tracker function */
        GPSIMU_GRAB_ERROR, /*!< No new GPS/IMU package when calling get GPS/IMU function */
        IMG_GRAB_ERROR /*!< No new image when calling get image function */
    };
}

#endif
