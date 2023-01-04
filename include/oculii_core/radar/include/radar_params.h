/*
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

#ifndef _RADAR_PARAMS_CLASS_
#define _RADAR_PARAMS_CLASS_

#ifndef CANFD
    #include <boost/asio.hpp>
#endif
#include <boost/filesystem.hpp>

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <exception>

#include <radar_defs.h>
/*! Oculii SDK namespace
 */
namespace oculii
{

    /*! RadarInitParams class helps to pass sensor configurations
     */
    class RadarInitParams
    {

    public:

        /*! Constructor for RadarInitParams class which creates a parameter container
         * @param port is an integer for the PC port
         * @param mcuPort is an integer for the radar port
         * @param radarIp is an string for the radar ip
         * @param isSave is a boolean value which is the switch for data saving
         * @param saveDir is a string for data save path, if isSave is set to false we can omit it
         * @return None
         */
        RadarInitParams(int port, int mcuPort, std::string radarIp, bool isSave = false, std::string saveDir = "");

        /*! User callable function to set the id
         * @param myRadId is an integer for the radar id
         * @return None
         */
        void SetRadId(int myRadId);

        /*! User callable function to set the port
         * @param myEthernetPort is an integer for the radar port
         * @return None
         */
        void SetRadPort(int myEthernetPort);

        /*! User callable function to set the mcu port
         * @param myEthernetPort is an integer for the mcu port
         * @return None
         */
        void SetMcuPort(int mcuPort);

        /*! User callable function to set the radar ip
         * @param myEthernetIp is an string for the radar
         * @return None
         */
        void SetRadarIp(std::string myEthernetIp);

        /*! User callable function to set the frame rate
         * @param myFrameRate is an integer for the radar framerate
         * @return None
         */
        void SetFrameRate(int myFrameRate);

        /*! User callable function to set if to save data
         * @param saveData is a boolean which is the switch for data saving
         * @return None
         */
        void EnableDataSave(bool saveData);

        /*! User callable function to update radar data save path
         * @param myDataSavePath is a string which is the radar data save path
         * @return None
         */
        void SetDataSavePath(std::string myDataSavePath);

        /*! User callable function to set radar calibration
         * @param yaw is a float value for radar mounting yaw angle
         * @param pitch is a float value for radar mounting pitch angle
         * @param roll is a float value for radar mounting roll angle
         * @param xoff is a float value for radar mounting offset on x axis
         * @param yoff is a float value for radar mounting offset on y axis
         * @param zoff is a float value for radar mounting offset on z axis
         * @return None
         */
        void SetRadMountingPose(float yaw, float pitch, float roll, float xoff, float yoff, float zoff);
        
        /*! User callable function to get the id
         * @return integer value which is the radar id
         */
        int GetRadId();

        /*! User callable function to get the port
         * @return integer value which is the radar port
         */
        int GetRadPort();

        /*! User callable function to get the mcu port
         * @return integer value which is the mcu port
         */
        int GetMcuPort();

        /*! User callable function to get the radar ip
         * @return string which is the radar ip
         */
        std::string GetRadarIp();

        /*! User callable function to get the frame rate
         * @return integer value which is the frame rate
         */
        int GetFrameRate();

        /*! User callable function to get the detection packet size
         * @return integer value which is the detection packet size
         */
        int GetPclPacketSize();

        /*! User callable function to get the tracker packet size
         * @return integer value which is the tracker packet size
         */
        int GetTrkPacketSize();

        /*! User callable function to check if save data
         * @return boolean value which is the indicator of data save
         */
        bool IsSavingData();

        /*! User callable function to check if use camera
         * @return boolean value which is the indicator of camera use
         */
        bool IsUsingCam();

        /*! User callable function to get data save path
         * @return string value which is the data save path
         */
        std::string GetDataSavePath();

        /*! User callable function to get radar mounting info
         * @return RadarCalibrationTransMatrix struct which contains radar mounting info
         */
        RadarCalibrationTransMatrix GetRadMountingPose();
        
        /*! User callable function to set online or offline data reading. By default sdk is running online
         */
        void SetOfflineReading();

        /*! User callable function to check if use offline data
        * @return boolean value which is the indicator of offline or not
        */
        bool IsOffline();

    private:
        /* contains id of the radar */
        int radId_;

        /*Contains the name of the ethernet port*/
        int radPort_;

        /*Contains the name of the mcu port*/
        int mcuPort_;

        /*Contains the frame rate that sensor is running at*/
        int frameRate_;

        /*Contains the pointcloud packet size sent from radar*/
        int pclPacketSize_;

        /*Contains the tracker packet size sent from radar*/
        int trkPacketSize_;

        /*Contains the radar ip address*/
        std::string radarIpAddr_;

        /*Indicate the data saving status*/
        bool savingData_;

        /*Indicate online or offline*/
        bool offlineReading_;

        /*Indicate to use camera or not*/
        bool camEnabled_;

        /*The local path for data saving*/
        std::string dataSavePath_;

        /*Mounting angle and position*/
        RadarCalibrationTransMatrix radMountingPose_;
    };

} // namespace oculii
#endif
