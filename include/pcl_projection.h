/*
/* *****************************************************************************
*  Name:    Tong Wu
*  Description:  Pcl project to image 
*
* Copyright (C) 2021 Oculii Corp. - http://www.oculii.com/
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

/* Header Guards */
#ifndef _PCL_PROJECTION_
#define _PCL_PROJECTION_

/**
 * \class Projection
 *
 * \brief 
 * This class helps the user to project pcl to image
 * \note 
 * \author Tong Wu
 * \version 1.0
 * \date 03/01/2021
 *
 * Contact: twu@oculii.com
 * Created on: March 01 2021
 *
 */

//Includes
#include "camera_params.h"
#include "radar_defs.h"
#include "radar_error.h"
#include <stdlib.h>
#include <math.h>
#include <opencv2/opencv.hpp>

namespace oculii
{
    class Projection
    {
        public:
            /*!
             * Constructor which initialize Projection class.
             * @param cameraInitParams is CameraInitParams type with camera info.
             * @param imgQueueSize is uint32_t which define the image delay queue size.
             * @param maxDrawingRadius is uint32_t which define the max radius for drawing obj on image.
             */
            Projection(CameraInitParams cameraInitParams, uint32_t imgQueueSize, uint32_t maxDrawingRadius);

            /*!
             * Destructor which destruct Projection class.
             */
            ~Projection();

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
    };
} // namespace oculii

#endif
