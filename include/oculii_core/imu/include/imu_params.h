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

#ifndef _IMU_PARAMS_CLASS_
#define _IMU_PARAMS_CLASS_

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <exception>
#include <boost/filesystem.hpp>
#include <boost/asio.hpp>
#include "imu_defs.h"

/*! Oculii SDK namespace
 */
namespace oculii
{

    /*! ImuInitParams class helps to pass sensor configurations
     */
    class ImuInitParams
    {

        public:

            /*! Constructor for ImuInitParams class which creates a parameter container
            * @param port is an integer for the imu port
            * @param isSave is a boolean value which is the switch for data saving
            * @param saveDir is a string for data save path, if isSave is set to false we can omit it
            * @return None
            */
            ImuInitParams(int port, bool isSave = false, std::string saveDir = "");

            /*! User callable function to set if to save data
            * @param saveData is a boolean which is the switch for data saving
            * @return None
            */
            void EnableDataSave(bool saveData);

            /*! User callable function to enable Ethernet imu/gps and set GPS port
            * @param port is a string which is the GPS/IMU Ethernet port
            * @return None
            */
            void EnableEthImuGps(int port);
            
            /*! User callable function to disable the use of Ethernet imu/gps
            * @return None
            */
            void DisableEthImuGps();

            /*! User callable function to update radar data save path
            * @param myDataSavePath is a string which is the radar data save path
            * @return None
            */
            void SetDataSavePath(std::string myDataSavePath);
            
            /*! User callable function to get the GPS/IMU port
            * @return integer value which is the GPS/IMU port
            */
            int GetEthImuGpsPort();

            /*! User callable function to get the GPS & IMU packet size
            * @return integer value which is the GPS & IMU packet size
            */
            int GetGpsImuPacketSize();

            /*! User callable function to get radar mounting info
            * @return ImuCalibrationTransMatrix struct which contains radar mounting info
            */
            ImuCalibrationTransMatrix GetImuMountingPose();

            /*! User callable function to check if save data
            * @return boolean value which is the indicator of data save
            */
            bool IsSavingData();
            
            /*! User callable function to check if use GPS/IMU
            * @return boolean value which is the indicator of GPS/IMU use
            */
            bool IsUsingEthImuGps();

            /*! User callable function to get data save path
            * @return string value which is the data save path
            */
            std::string GetDataSavePath();

            /*! User callable function to set radar calibration
            * @param yaw is a float value for radar mounting yaw angle
            * @param pitch is a float value for radar mounting pitch angle
            * @param roll is a float value for radar mounting roll angle
            * @param xoff is a float value for radar mounting offset on x axis
            * @param yoff is a float value for radar mounting offset on y axis
            * @param zoff is a float value for radar mounting offset on z axis
            * @return None
            */
            void SetImuMountingPose(float yaw, float pitch, float roll, float xoff, float yoff, float zoff);

        private:
            /*Contains the gps & imu packet size sent from radar*/
            int gpsImuPacketSize_;

            /*Indicate the data saving status*/
            bool savingData_;
            
            /*Indicate the imu status*/
            bool ethImuGpsEnabled_;
            int imuPort_;

            /*The local path for data saving*/
            std::string dataSavePath_;

            /*Mounting angle and position*/
            ImuCalibrationTransMatrix imuMountingPose_;
    };

} // namespace oculii
#endif
