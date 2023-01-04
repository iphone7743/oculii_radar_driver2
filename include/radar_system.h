/*
/* *****************************************************************************
*  Name:    Kranthi Kumar M
*  Description:  Radar System (Application Layer) 
*
* Copyright (C) 2018 Oculii Corp. - http://www.oculii.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of the owner nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _RADAR_SYSTEM_
#define _RADAR_SYSTEM_
#include <opencv2/opencv.hpp>
#include "imu_error.h"
#include "imu_params.h"
#include "imu_defs.h"
#include "radar_error.h"
#include "radar_defs.h"
#include "radar_params.h"
#include <thread>
#include <mutex>

namespace oculii
{
    // Buffer structure for each radar to store pointCloud and tracker data with radar object pointer
    class RadarSystem
    {
        public:

            /*! Static method to return the instance of the Singleton object of RadarSystem
            * @param configPath, string path for the config file
            * @return radarSystem *, pointer to the instance of RadarSystem
            */
            static RadarSystem *GetRadarSystemInstance(std::string configPath);

            /*! Virtual destructor for RadarSystem class
            * @return None
            */
            ~RadarSystem(){};

            /*! Validate and start the system based on the config file
            * @return RadarErrorCode, refer to core_error.h for details
            */
            RadarErrorCode StartSystem ();

            /*! gets pointcloud of sensors as given by radar IDs. If given multiple IDs then will output combined pcl
            * @param pclIdToPacketsMap is a map between radarIDs and RadarDetectionPacket structure for the point cloud
            * @return RadarErrorCode, refer to core_error.h for details
            */
            RadarErrorCode GetPointcloud(std::map< int, RadarDetectionPacket > &pclIdToPacketsMap);

            /*! gets enhanced pointcloud by the radar IDs. If given multiple IDs then will output combined pcl
            * @param pclIdToPacketsMap is a map between radarIDs and RadarDetectionPacket structures which reserve for combined point cloud output
            * @return RadarErrorCode, refer to core_error.h for details
            */
            RadarErrorCode GetEnhancedPointcloud(std::map< int, std::vector<RadarDetectionPacket> > &pclIdToPacketsMap);

            /*! Copy current frame tracks in a frame to empty RadarTrackerPacket
            * @param trkIdToPacketsMap is a map between radarIDs and RadarTrackerPacket pointer for radar tracks 
            * @return RadarErrorCode, take a reference to the RadarErrorCode struct
            */
            RadarErrorCode GetTracker(std::map< int, RadarTrackerPacket > &trkIdToPacketsMap);

            /*! Combine all radar data to have tracker based on combined pcl
            * @param trkPackets is the output container of fused tracks
            * @return RadarErrorCode, take a reference to the RadarErrorCode struct
            */
            RadarErrorCode GetFusionTracker(std::map< int, RadarTrackerPacket > &trkIdToPacketsMap);
            
            /*! Go through switch mode command required process and send switch mode command
            * @param modeCmd is a vector of enum ModeCommand used to store mode type
            * @return RadarErrorCode, take a reference to the RadarErrorCode struct
            */
            std::vector<RadarErrorCode> SendModeSwitchCmd(const std::vector <int>& radarIDs, const std::vector <ModeCommand>& modeCmd);      

            /*! Go through handshaking process and send host yaw and speed info to the sensor
            * @param yaw is a vector of double to store the host yaw(degree clockwise)
            * @param speed is a vector of double to store the host speed(m/s)
            * @return RadarErrorCode, take a reference to the RadarErrorCode struct
            */
            std::vector<RadarErrorCode> SendHostInfo(const std::vector <int>& radarIDs, const std::vector <double>& yaw, const std::vector <double>& speed);

            /*! Go through handshaking process and send mounting angle to the sensor
            * @param radarIDs is a int vector of radarIDs
            * @param mountingAngles is a vector of type double to store the mounting angle of the radars
            * @return RadarErrorCode, take a reference to the RadarErrorCode struct
            */
            std::vector<RadarErrorCode> SendHostSetupCfg(const std::vector <int>& radarIDs, const std::vector <double>& moutingAngles);

            /*! Go through handshaking process and send PTP request to the sensor
            * @param radarIDs is a int vector of radarIDs
            * @param waitPTP is a vector of type bool to represent to wait for PTP sync or not
            * @param multiCast is vector of type bool to indicate multicast or unicast
            * @param ip is a vector of type string to store the ip configuration
            * @return RadarErrorCode, take a reference to the RadarErrorCode struct
            */
            std::vector<RadarErrorCode> SendPtpRequest(const std::vector<int>& radarIDs, const std::vector<bool>& waitPTP, const std::vector<bool>& multiCast, const std::vector<std::string>& ip);

            /*! Set Radar Receive State to true and start all of the threads to continuously update data
            * @return RadarErrorCode, take a reference to the RadarErrorCode struct
            */
            RadarErrorCode StartRadarReceive();

            /*! Set Radar Receive State to true and start desired threads to continuously update data
            * @param radarIDs is a int vector of radarIDs
            * @return RadarErrorCode, take a reference to the RadarErrorCode struct
            */
            RadarErrorCode StartRadarReceive(const std::vector <int>& radarIDs);
      
            /*! Set Radar Receive State to False stop
            * @param radarIDs is a int vector of radarIDs
            * @return None
            */
            RadarErrorCode PauseRadarReceive(const std::vector <int>& radarIDs);

            /*! Close the reading and parsing process and exist threads
            * @return None
            */
            void Close();

            /*! get GPS IMU data
            * @return GpsImuData struct, find a reference to the GpsImuData struct in defs.h
            */
            GpsImuData GetGpsImu();

            /*! set GPS IMU data
            * @param userGpsImu is a GpsImuData struct, Find a reference in defs.h
            * @return ImuErrorCode, find a reference to the ImuErrorCode struct 
            */
            ImuErrorCode SetGpsImu(GpsImuData userGpsImu);


            /*! get status of external ImuGps
            * @return boolean, true if user set true in config file, false otherwise
            */
            bool GetExternalImuStatus();

            /*! get if the sensor version is minimal gen 7
            * @return boolean, true if user set true in config file, false otherwise
            */
            bool GetIfMinGen7();

            /*! Check if current setting satisfies enhanced pcl output
            * @return bool, ture means meet the enhanced pcl condition
            */
            bool MeetEnhancedPclCondition();


            /*! Check if current setting satisfies fusion tracker output
            * @return bool, ture means meet the fusion tracker condition
            */
            bool MeetFusionTrackerCondition();

            /*! get if the visualizer is enabled in the configuration
            * @return boolean, true if user enabled visualizer, false otherwise
            */
            bool GetIfVisualizerEnabled();

            /*! get Frame Rate of system
            * @param radarID is the ID of the sensor chosen for the frame rate.
            * @return int, Frame rate of the system
            */
            int GetFrameRate(int radarID);

            /*! set the config to SLAM mode
            * @param isSlamMode is the switch for SLAM mode configs.
            */
            void SetSlamMode(bool isSlamMode = true);

            /*! get the sensor ports
            * @return std::vector<int>, ports of the radar sensors.
            */
            std::vector<int> GetSensorPorts();

            /*! get the sensor IDs
            * @return std::vector<int>, IDs of the radar sensors.
            */
            std::vector<int> GetSensorIDs();

            /*! Update the sensor with single file that contains bootloader, firmware and QSPI
            *   @param filePath is a string value which contains the path in which the update file is placed
            */
            RadarErrorCode UpdateSensor(std::string filePath, const std::vector <int>& radarIDs);
#ifdef OPENCV

            /*! Read Cam Data and write it to the saved path if enable saving, pic name is the timestamp
            * @return the cv::Mat object
            */
            cv::Mat GetOneImg();

            /*!
             * Do image queue and project input tracker to input image and the projected points will be on inputImg.
             * @param inputTrck is a RadarTrackerPacket instances which contains the current frame data provided.
             * @param inputImg is a cv::Mat to pass input image.
             * @param outputImg is a cv::Mat to pass output image and the projected points will be on this image.
             * @param stationaryTrckUv is std::vector<cv::Point2d> to output stationary projected points in uv domain.
             * @param approachTrckUv is std::vector<cv::Point2d> to output approaching projected points in uv domain.
             * @param awayTrckUv is std::vector<cv::Point2d> to output moving away projected points in uv domain.
             * @return the RadarErrorCode
             */
            RadarErrorCode GetTrkProj(const oculii::RadarTrackerPacket& inputTrck, const cv::Mat& inputImg, cv::Mat& outputImg, std::vector<cv::Point2d>& stationaryTrckUv, std::vector<cv::Point2d>& approachTrckUv, std::vector<cv::Point2d>& awayTrckUv);
#endif   
         

    };
};
#endif
