#ifndef _RADAR_ERROR__
#define _RADAR_ERROR__

#include <string>
/*! Oculii SDK namespace
 */
namespace oculii
{
    /*! ERROR_CODE enumeration flags for exception handling
     */
    enum class RadarErrorCode
    {
        SUCCESS, /*!< General success flag */
        FAILED, /*!< General failure flag */
        FAILED_TO_INIT_THREAD, /*!< Failed to initilize a thread */
        FAILED_TO_RECEIVE_DATA, /*!< Failed to reveive data */
        INVALID_PARAMETER, /*!< Function arguments error */
        PCL_GRAB_ERROR, /*!< No new detection package when calling get pointcloud function */
        TRK_GRAB_ERROR, /*!< No new tracker package when calling get tracker function */
        FILE_HANDLING_ERROR, /*!< Update File not consistent or incorrect */
        SENSOR_CONNECTION_ERROR, /*!< Cannot connect to the desired sesnor */
        SENSOR_FLASH_ERROR, /*!< Error occured while flashing sensor */
        EAGLE_TRIGGER_ERROR, /*!< Eagle sensor couldnt be triggered */
        FILE_PATH_DOESNT_EXIST, /*!< File path doesn't exist */
        INVALID_HEADER_FORMAT, /*!< Config file header format is invalid */
        INVALID_RADAR_FORMAT, /*!< Config file radar format is invalid */
        INVALID_CAMERA_FORMAT, /*!< Config file camera format is invalid */
        DATA_SIZE_MISMATCH, /*!< Data structure sizes mismatch */
        DATA_IN_QUEUE, /*!< Data is feeding into queue */
        INVALID_RANGE_ACCURACY, /*!< Range accuracy in the packet is invalid */
        INVALID_RADAR_ID, /*!< The radar ID is invalid */
        INVALID_CONDITIONS_FOR_ENHANCED_PCL, /*!< Cannot get enhanced pcl with currenct conditions */
    };

    static std::string ErrorToString(RadarErrorCode error)
    {
        switch(error)
        {
            case RadarErrorCode::SUCCESS:
                return "SUCCESS";
            case RadarErrorCode::FAILED:
                return "FAILED";
            case RadarErrorCode::FAILED_TO_INIT_THREAD:
                return "Failed to Initialize thread";
            case RadarErrorCode::FAILED_TO_RECEIVE_DATA:
                return "Failed to Receive Data";
            case RadarErrorCode::INVALID_PARAMETER:
                return "Invalid Parameter";
            case RadarErrorCode::PCL_GRAB_ERROR:
                return "Point Cloud Grab error";
            case RadarErrorCode::TRK_GRAB_ERROR:
                return "Track Grab Error";
            case RadarErrorCode::FILE_HANDLING_ERROR:
                return "File Handling Error";
            case RadarErrorCode::SENSOR_CONNECTION_ERROR:
                return "Sensor Connection Error";
            case RadarErrorCode::SENSOR_FLASH_ERROR:
                return "Sensor Flash Error";
            case RadarErrorCode::EAGLE_TRIGGER_ERROR:
                return "Eagle Trigger Error";
            case RadarErrorCode::FILE_PATH_DOESNT_EXIST:
                return "File Path Doesn't Exist";
            case RadarErrorCode::INVALID_HEADER_FORMAT:
                return "Invalid Config File Header Format";
            case RadarErrorCode::INVALID_RADAR_FORMAT:
                return "Invalid Config File Radar Format";
            case RadarErrorCode::INVALID_CAMERA_FORMAT:
                return "Invalid Config File Camera Format";
            case RadarErrorCode::DATA_SIZE_MISMATCH:
                return "Data structure size mismatch";
            case RadarErrorCode::DATA_IN_QUEUE:
                return "Data is feeding into queue";
            case RadarErrorCode::INVALID_RANGE_ACCURACY:
                return "Range accuracy in the packet is invalid";
            case RadarErrorCode::INVALID_RADAR_ID:
                return "Invalid Radar Id";
            case RadarErrorCode::INVALID_CONDITIONS_FOR_ENHANCED_PCL:
                return "Invalid Conditions for Enhanced PCL";
            default:
                return "Unknown error code";

        }
    };
}

#endif
