#ifndef _CAMERA_DEFS_
#define _CAMERA_DEFS_

#include <string>



/*! Oculii SDK namespace
 */
namespace oculii
{

    /*! ImageData struct for storing camera images
     */
    // struct ImageData
    // {
    //     std::string timestamp; /*!< Timestamp for the frame */
    //     cv::Mat data; /*!< Image in cv::Mat format */
    // };

    struct CamCalibrationTransMatrix
    {
        float mountYaw; /*!< Mounting yaw angle */
        float mountPitch; /*!< Mounting pitch angle */
        float mountRoll; /*!< Mounting roll angle */
        float xOff; /*!< Mounting offset on x axis */
        float yOff; /*!< Mounting offset on y axis */
        float zOff; /*!< Mounting offset on z axis */
    };

};

#endif
