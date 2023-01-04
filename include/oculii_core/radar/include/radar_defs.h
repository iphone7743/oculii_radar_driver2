#ifndef _RADAR_DEFS_
#define _RADAR_DEFS_

#include <vector>
#include <string>

/*! Oculii SDK namespace
 */
namespace oculii
{

    /*! Struct to store Radar frame accuracy */
    struct RadarFrameAccuracy
    {
        double rangeAccu = 0.0; /*!< Store the accuracy value of range data */
        double dopplerAccu = 0.0; /*!< Store the accuracy value of doppler data */
        double aziAccu = 0.0; /*!< Store the accuracy value of aziumith angle data */
        double eleAccu = 0.0; /*!< Store the accuracy value of elevation angle data */
    };

    /*! Struct to store Radar data */
    struct RadarDetectionData
    {
        uint16_t dotFlags = 0;                 /*!< Point status flag */
        uint16_t denoiseFlag = 0;              /*!< Mark of noise point */
        uint16_t historyFrameFlag = 0;         /*!< Mark of the last frame detections */
        uint16_t dopplerCorrectionFlag = 0;    /*!< Mark of doppler correction feature */
        RadarFrameAccuracy framAccuracy;       /*!< Frame accuracy */
        float recoveredSpeed = 0.0;            /*!< Target true speed segment along vehicle moving direction*/
        float range = 0.0;                     /*!< Detection range to radar center */
        float doppler = 0.0;                   /*!< Current doppler speed */
        float alpha = 0.0;                     /*!< Detection horizontal angle */
        float power = 0.0;                     /*!< Power received by radar */
        float beta = 0.0;                      /*!< Detection vertical angle */
        float x = 0.0;                         /*!< Detection x position in Oculii radar coordinate */
        float y = 0.0;                         /*!< Detection y position in Oculii radar coordinate */
        float z = 0.0;                         /*!< Detection z position in Oculii radar coordinate */
    };

    /*! RadarDetectionPacket struct for storing radar data packet for a frame
     */
    struct RadarDetectionPacket
    {
        uint64_t timestamp = 0;                   /*!< Timestamp for the receiving packet in sdk*/
        uint32_t ptpTimestamp = 0;                /*!< Timestamp for the ptp*/
        uint32_t frameNumber = 0;                 /*!< Frame number of the packet */
        uint32_t version = 0;                     /*!< Version number of the radar */
        uint16_t numObjOut = 0;                   /*!< Number of detections in current frame */
        uint16_t numTrkOut = 0;                   /*!< Number of tracks in current frame */
        uint16_t frameRate = 0;                   /*!< Frame rate of the radar */
        double DSPWorkload = 0;                   /*!< DSP workload of the radar */
        double ARMWorkload = 0;                   /*!< ARM workload of the radar */
        float ego = 0;                            /*!< Current host speed of the sensor */
        float hostAngle = 0;                      /*!< Current orientation of the host */
        std::vector<RadarDetectionData> data;     /*!< RadarDetectionData struct @see RadarDetectionData */
    };

    /*! RadarTrackerData struct for storing track data of a single track
     */
    struct RadarTrackerData
    {
        ////////////////////////////////////////////////////////////////////
        /////////////////////////old format attribute///////////////////////
        ////////////////////////////////////////////////////////////////////
        uint32_t id = 0;         /*!< Track ID */
        uint16_t trkFlags = 0;   /*!< Track status flag(32 bytes format) */
        uint16_t trkClass = 0;   /*!< Track classification result */
        uint16_t confidence = 0; /*!< Track confidence */

        ////////////////////////////////////////////////////////////////////
        /////////////////////////new format/////////////////////////////////
        ////////////////////////////////////////////////////////////////////
        uint32_t trkAge = 0;                    /*!< Track Age */
        uint16_t targetID = 0;                  /*!< Target ID */
        uint16_t trkID = 0;                     /*!< Track ID */
        float x = 0;                            /*!< Track x position in Oculii radar coordinate */
        float y = 0;                            /*!< Track y position in Oculii radar coordinate */
        uint16_t z = 0;                         /*!< Track z position in Oculii radar coordinate */
        float vx = 0;                           /*!< Track x direction velocity in Oculii radar coordinate */
        float vy = 0;                           /*!< Track y direction velocity in Oculii radar coordinate */
        float vz = 0;                           /*!< Track z direction velocity in Oculii radar coordinate */
        float trkPower = 0;                     /*!< Normalized Track Power */
        float targetHorizontalHeading = 0;      /*!< Target Horizontal Heading */
        float ax = 0;                           /*!< Horizontal Lateral Acceleration(ax) */
        float ay = 0;                           /*!< Vertical Acceleration(ay) */
        float az = 0;                           /*!< Horizontal Longitudinal Acceleration(az) */
        float stdDevX = 0;                      /*!< Standard deviation of Lateral Position(x) */
        float stdDevY = 0;                      /*!< Standard deviation of Vertical Position(y) */
        float stdDevZ = 0;                      /*!< Standard deviation of Longitudinal Position(z) */
        float stdDevVx = 0;                     /*!< Standard deviation of Lateral Velocity(vx) */
        float stdDevVy = 0;                     /*!< Standard deviation of Vertical Velocity(vy) */
        float stdDevVz = 0;                     /*!< Standard deviation of Longitudinal Velocity(vz) */
        float posCovCoefficient = 0;            /*!< Covariance correlation coefficient of Target Position */
        float velCovCoefficient = 0;            /*!< Covariance correlation coefficient of Target Velocity */
        float trkScore = 0;                     /*!< Track score */
        float objExistenceProb = 0;             /*!< Probability of object existence */
        float obstacleProb = 0;                 /*!< Obstacle Probability(overdrivable/underdrivable) */
        uint8_t trkAllFlags = 0;                /*!< Track Flags (for new format)*/
        float sx = 0;                           /*!< Target Width(sx)*/
        float sz = 0;                           /*!< Target Length(sz) */
        float sy = 0;                           /*!< Target Height(sy) */
        float stdDevSz = 0;                     /*!< Standard deviation of Target Length(sz) */
        float stdDevSy = 0;                     /*!< Standard deviation of Target Height(sy) */
        float stdDevSx = 0;                     /*!< Standard deviation of Target Width(sx) */
        uint8_t targetClass = 0;                /*!< Target Class */
        float targetClassificationConf = 0;     /*!< Target Classification Confidence */
        float radarCrossSecton = 0;             /*!< Radar Cross Section */
    };

    /*! RadarTrackerPacket struct for storing radar tracker packet for a frame 
     */
    struct RadarTrackerPacket
    {
        uint64_t timestamp = 0;                 /*!< Timestamp for the receiving packet in sdk*/
        uint32_t ptpTimestamp = 0;              /*!< Timestamp for the ptp*/
        uint32_t frameNumber = 0;           /*!< Frame number of the packet */
        uint32_t version = 0;                   /*!< Version number of the tracker */
        uint16_t numObjOut = 0;                 /*!< Number of detections in current frame */
        uint16_t numTrkOut = 0;                 /*!< Number of tracks in current frame */
        double DSPWorkload = 0;                 /*!< DSP workload of the radar */
        double ARMWorkload = 0;                 /*!< ARM workload of the radar */
        float ego = 0;                          /*!< Current host speed of the sensor */
        float hostAngle = 0;                    /*!< Current orientation of the host */
        std::vector<RadarTrackerData> data; /*!< RadarTrackerData struct @see RadarTrackerData */
    };


    /*! CalibrationTransMatrix struct for storing radar mounting information 
     */
    struct RadarCalibrationTransMatrix
    {
        float mountYaw; /*!< Mounting yaw angle */
        float mountPitch; /*!< Mounting pitch angle */
        float mountRoll; /*!< Mounting roll angle */
        float xOff; /*!< Mounting offset on x axis */
        float yOff; /*!< Mounting offset on y axis */
        float zOff; /*!< Mounting offset on z axis */
        
        RadarCalibrationTransMatrix()
        {
            mountYaw = 0; /*!< Mounting yaw angle */
            mountPitch = 0; /*!< Mounting pitch angle */
            mountRoll = 0; /*!< Mounting roll angle */
            xOff = 0; /*!< Mounting offset on x axis */
            yOff = 0; /*!< Mounting offset on y axis */
            zOff = 0; /*!< Mounting offset on z axis */
        }
        
        RadarCalibrationTransMatrix(float mountYaw, float mountPitch, float mountRoll, float xOff, float yOff, float zOff)
        {
            this->mountYaw = mountYaw; /*!< Mounting yaw angle */
            this->mountPitch = mountPitch; /*!< Mounting pitch angle */
            this->mountRoll = mountRoll; /*!< Mounting roll angle */
            this->xOff = xOff; /*!< Mounting offset on x axis */
            this->yOff = yOff; /*!< Mounting offset on y axis */
            this->zOff = zOff; /*!< Mounting offset on z axis */
        }
        
        RadarCalibrationTransMatrix(const RadarCalibrationTransMatrix& trans)
        {
            this->mountYaw = trans.mountYaw; /*!< Mounting yaw angle */
            this->mountPitch = trans.mountPitch; /*!< Mounting pitch angle */
            this->mountRoll = trans.mountRoll; /*!< Mounting roll angle */
            this->xOff = trans.xOff; /*!< Mounting offset on x axis */
            this->yOff = trans.yOff; /*!< Mounting offset on y axis */
            this->zOff = trans.zOff; /*!< Mounting offset on z axis */
        }
    };

    /*! ModeCommand enumeration for storing radar command alias
     */
    enum ModeCommand
    {
        SENSOR_MODE_0, /*!< Switch to radar mode 0 */
        SENSOR_MODE_1, /*!< Switch to radar mode 1 */
        SENSOR_MODE_2, /*!< Switch to radar mode 2 */
        SENSOR_MODE_3, /*!< Switch to radar mode 3 */
        SENSOR_STOP,   /*!< Pause the sensor data output */
        SENSOR_START,  /*!< Restart sensor data output */
        SENSOR_RESTART /*!< Pause and then restart sensor data output */
    };

    /*! Struct to store Footer Information
    */
    struct RadarFooter
    {
        float ego = 0.0; /*!< Ego Speed */
        float yaw = 0.0; /*!< Yaw */
    };

    /* Struct to store the parsing position of data frame header*/
    struct DataPacketPtrConsts
    {
        const int MAGIC_NUMBER = 0; /*!< magic num bit position */
        const int FRAME_NUMBER = 8; /*!< frame num bit position */
        const int VERSION_NUMBER = 12; /*!< version num bit position */
        const int NUMBER_OF_DETECTION = 16; /*!< detection num bit position */
        const int NUMBER_OF_TRACKS = 18; /*!< tracker num bit position */
        const int HOST_SPEED = 20; /*!< host speed bit position */
        const int HOST_ANGLE = 22; /*!< host angle bit position */
        const int RANGE_ACCU = 32; /*!< range accuracy bit position */
        const int DOPPLER_ACCU = 34; /*!< doppler accuracy bit position */
        const int AZIMUTH_ACCU = 36; /*!< azimuth accuracy bit position */
        const int ELEVATION_ACCU = 38; /*!< elevation accuracy bit position */
        const int DSP = 40; /*!< DSP workload bit position */
        const int ARM = 41; /*!< ARM workload bit position */
        const int HEADER_SIZE = 48; /*!< header size bit position */
    };

    /* Struct to store the parsing attribut byte size */
    struct DataPacketSizeConsts
    {
        const int MAGIC_NUMBER_SIZE = 8; /*!< magic num size in bytes */
        const int FRAME_NUMBER_SIZE = 4; /*!< frame num size in bytes */
        const int VERSION_NUMBER_SIZE = 4; /*!< version num size in bytes */
        const int NUMBER_OF_DETECTION_SIZE = 2; /*!< detection num size in bytes */
        const int NUMBER_OF_TRACKS_SIZE = 2; /*!< tracker num size in bytes */
        const int HOST_SPEED_SIZE = 2; /*!< host speed size in bytes */
        const int HOST_ANGLE_SIZE = 2; /*!< host angle size in bytes */
        const int RANGE_ACCU_SIZE = 2; /*!< range accuracy size in bytes */
        const int DOPPLER_ACCU_SIZE = 2; /*!< doppler accuracy size in bytes */
        const int AZIMUTH_ACCU_SIZE = 2; /*!< azimuth accuracy size in bytes  */
        const int ELEVATION_ACCU_SIZE = 2; /*!< elevation accuracy size in bytes  */
        const int DSP_SIZE = 1; /*!< DSP workload size in bytes  */
        const int ARM_SIZE = 1; /*!< ARM workload size in bytes  */
        const int DETECTION_STRUCTURE_SIZE = 8; /*!< detection structure size in bytes */
        const int FOOTER_STRUCTURE_SIZE = 32; /*!< footer structure size in bytes */
        const int FOOTER_EGO_SIZE = 4; /*!< footer structure ego size in bytes */
        const int FOOTER_YAW_SIZE = 4; /*!< footer structure yaw size in bytes */
        const int FOOTER_FRAME_RATE_SIZE = 2; /*!< footer structure frame rate size in bytes */
    };

    /* Struct to store the parsing position of detection data*/
    struct DetectionPacketConsts
    {
        const int RANGE = 0; /*!< range bit position */
        const int DOPPLER = 10; /*!< doppler bit position */
        const int ALPHA = 20; /*!< alpha bit position */
        const int BETA = 30; /*!< beta bit position */
        const int POWER = 40; /*!< power bit position */
        const int DOT_FLAGS = 59; /*!< dot flag bit position */
        const int DOPPLER_CORRECTION_FLAG = 61; /*!< doppler correction flag bit position */
        const int HISTORY_FRAME_FLAG = 62; /*!< history frame flag bit position */
        const int DENOISE_FLAG = 63; /*!< denoise flag bit position */
    };

    /* Struct to store the parsing position for yaw and ego */
    struct FooterPtrConsts
    {
        const int RANGE_ACCU = 24; /*!< range accuracy bit position */
        const int DOPPLER_ACCU = 22; /*!< doppler accuracy bit position */
        const int AZIMUTH_ACCU = 20; /*!< azimuth accuracy bit position */
        const int ELEVATION_ACCU = 18; /*!< elevation accuracy bit position */
        const int YAW_POS = 12; /*!< yaw bit position */
        const int EGO_POS = 8; /*!< ego speed bit position */
        const int FRAME_RATE = 2; /*!< frame rate bit position */
    };

    /* Struct to store the power filter threshold */
    struct PowerFilter
    {
        const float MIN_POW_THRESH = 11; /*!< min power threshold */
        const float MAX_POW_THRESH = 15; /*!< max power threshold */
        const float POW_THRESH = -1; /*!< power threshold */
    };

    /* Struct to store flags */
    struct Flags
    {
        bool POW_THRESH_FLAG = false; /*!< To set power filter. To be set along with the filter settings */
        bool MULTIMODE_FLAG =false; /*!< To set single mode/multimode */
        bool ENABLE_TRACKER_FLAG = true; /*!< To Enable Tracker */
        bool ENABLE_FOOTER_PARSING = true; /*!< To enable footer parsing */
        uint32_t FREQUENCY_FLAG=0; /*!< estimated frequency */
    };

    /**
    * Type of FileInfo to be read in update sensor
    */
    struct FileInfo
    {
        int fileLength = 0;
        char *fileBytes;
    };

    /**
    * Type of File to be either appended or segregated for update sensor
    * File Index : 0-MCU 1-Firmware 2-QSPI 3-FirmwareB 4-QSPIB
    */
    enum FileType
    {
        MCU,
        FIRMWARE,
        FIRMWAREB,
        QSPI,
        QSPIB
    };


};

#endif
