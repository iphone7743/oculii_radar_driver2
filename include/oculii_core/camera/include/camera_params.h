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

#ifndef _CAMERA_PARAMS_CLASS_
#define _CAMERA_PARAMS_CLASS_

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
#include <opencv2/opencv.hpp>
#include <camera_defs.h>

/*! Oculii SDK namespace
 */
namespace oculii
{

    /*! CameraInitParams class helps to pass sensor configurations
     */
    class CameraInitParams
    {

    public:

        /*! Constructor for CameraInitParams class which creates a parameter container
         * @param isSave is a boolean value which is the switch for data saving
         * @param saveDir is a string for data save path, if isSave is set to false we can omit it
         * @return None
         */
        CameraInitParams(bool savingData_, std::string saveDir = "");


        /*! User callable function to set the ip camera image width
         * @param myImageWidth is an integer for the captured image width
         * @return None
         */
        void SetImageWidth(int myImageWidth);

        /*! User callable function to set the ip camera image height
         * @param myImageWidth is an integer for the captured image height
         * @return None
         */
        void SetImageHeight(int myImageHeight);

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

        
        /*! User callable function to set camera extrinsic calibration
         * @param yaw is a float value for camera mounting yaw angle
         * @param pitch is a float value for camera mounting pitch angle
         * @param roll is a float value for camera mounting roll angle
         * @param xoff is a float value for camera mounting offset on x axis
         * @param yoff is a float value for camera mounting offset on y axis
         * @param zoff is a float value for camera mounting offset on z axis
         * @return None
         */
        void SetCamMountingPose(float yaw, float pitch, float roll, float xoff, float yoff, float zoff);

        /*! User callable function to get the camera image width
         * @return integer value which is the camera image width
         */
        int GetImageWidth();

        /*! User callable function to get the camera image height
         * @return integer value which is the camera image height
         */
        int GetImageHeight();

        /*! User callable function to check if save data
         * @return boolean value which is the indicator of data save
         */
        bool IsSavingData();

        /*! User callable function to get data save path
         * @return string value which is the data save path
         */
        std::string GetDataSavePath();
        


        /*! User callable function to get intrinsicMat
         * @return cv::Mat which is the intrinsic matrix
         */
        cv::Mat GetIntrinsicMat();

        /*! User callable function to set intrinsicMat
         * @return void
         */
        void SetIntrinsicMat(uint32_t fx, uint32_t fy, uint32_t cx, uint32_t cy);


        /*! User callable function to get extrinsicMat
        * @return cv::Mat which is the extrinsic matrix
        */
        cv::Mat GetExtrinsicMat();

        /*! User callable function to set extrinsicMat
        * @return void
        */
        void SetExtrinsicMat(float yaw, float pitch, float roll, float x, float y, float z);









    private:

        /*Camera video stream width*/
        int imageWidth_;

        /*Camera video stream height*/
        int imageHeight_;

        /*Indicate the data saving status*/
        bool savingData_;

        /*Indicate the cam status*/
        bool camEnabled_;

        /*The local path for data saving*/
        std::string dataSavePath_;

        /*The intrinsicMat*/
        cv::Mat intrinsicMat_;

        /*The extrinsicMat*/
        cv::Mat extrinsicMat_;

    };

} // namespace oculii
#endif
