#ifndef _IMU_DEFS_
#define _IMU_DEFS_

#include <string>
#include <vector>

/*! Oculii SDK namespace
 */
namespace oculii
{
    /*! ImuOnlyData struct for storing IMU data
     */
    struct ImuOnlyData
    {
        std::string timestamp; /*!< Timestamp for the packet */
        std::string rawStr; /*!< IMU packet in raw sting format */

        float sYaw = 0; /*!< IMU yaw angle */
        float sPit = 0; /*!< IMU pitch angle */
        float sRol = 0; /*!< IMU roll angle */
        float accX = 0; /*!< IMU x axis velocity */
        float accY = 0; /*!< IMU y axis velocity */
        float accZ = 0; /*!< IMU z axis velocity */
        bool isValid; /*!< If confident on the IMU data */
    };

    /*! OdomOnlyData struct for storing odometry data 
     */
    struct OdomOnlyData
    {
        std::string timestamp; /*!< Timestamp for the packet */
        std::string rawStr; /*!< Odometry packet in raw sting format */

        float sYaw = 0; /*!< Yaw angle velocity */
        float sPit = 0; /*!< Pitch angle velocity */
        float sRol = 0; /*!< Roll angle velocity */
        float yaw = 0; /*!< Absolute yaw angle */
        float pit = 0; /*!< Absolute pitch angle */
        float rol = 0; /*!< Absolute roll angle */
        float sX = 0; /*!< x axis velocity */
        float sY = 0; /*!< y axis velocity */
        float sZ = 0; /*!< z axis velocity */
        float x = 0; /*!< x position */
        float y = 0; /*!< y position */
        float z = 0; /*!< z position */
    };

    /*! GpsImuData struct for storing GPS and IMU data in the same struct (VN200)
     */
    struct GpsImuData
    {
        std::string timestamp; /*!< Timestamp for the packet */
        std::string rawStr; /*!< GPS/IMU packet in raw sting format */
        uint32_t frameNumber = 0; /*!< Frame number of the first sensor*/

        float yaw = 0; /*!< Absolute yaw angle */
        float pitch = 0; /*!< Absolute pitch angle */
        float roll = 0; /*!< Absolute roll angle */
        float lat = 0; /*!< Latitude */
        float lon = 0; /*!< Longitude */
        float alt = 0; /*!< Altitude */
        float speed = 0; /*!< Host speed */
        double gpstime = 0; /*!< GPS timestamp */
        bool isValid; /*!< If confident on the GPS/IMU data */
    };

    /*! CalibrationTransMatrix struct for storing imu mounting information 
     */
    struct ImuCalibrationTransMatrix
    {
        float mountYaw = 0; /*!< Mounting yaw angle */
        float mountPitch = 0; /*!< Mounting pitch angle */
        float mountRoll = 0; /*!< Mounting roll angle */
        float xOff = 0; /*!< Mounting offset on x axis */
        float yOff = 0; /*!< Mounting offset on y axis */
        float zOff = 0; /*!< Mounting offset on z axis */
    };

}

#endif
