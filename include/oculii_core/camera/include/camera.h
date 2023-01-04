#ifndef _CAMERA_CLASS_
#define _CAMERA_CLASS_

#include "camera_error.h"
#include "camera_defs.h"
#include "camera_params.h"
#include <thread>
#include <mutex> 
#include <opencv2/opencv.hpp>

namespace oculii
{
    /*! ImageData struct for storing camera images
     */
    struct ImageData
    {
        std::string timestamp; /*!< Timestamp for the frame */
        cv::Mat data; /*!< Image in cv::Mat format */
    };
    class Camera
    {
        public:
            /*! Constructor with given configurations in Camera abstract class
            * @param id is an integer for the cam ID assigned
            * @param CameraInitParams is a CameraInitParams instance which contains all the configiguation paramers
            * @return None
            */
            Camera(int id, CameraInitParams CameraInitParams);

            /*! Virtual destructor for Camera class
            * @return None
            */
            virtual ~Camera();

            /*! Start to reading thread with parameters provided in CameraInitParams
            * @return CameraErrorCode, take a reference to the CameraErrorCode structe
            */
            CameraErrorCode Open( );
            
            /*! Close virtual function
            * @return None
            */
            virtual void Close() = 0;
            

            /*! Set framerate of the camera
            * @return None
            */
            void SetFrameRate(int framerate);

            /*! Get framerate of the camera
            * @return framerate of the rocorded camera
            */
            int GetFrameRate();
            
            /*! Start recording of the camera
            * @return None
            */
            void EnableRecording();

            /*! End recording of the camera
            * @return framerate of the rocorded camera
            */
            void EndRecording();

        protected:

            /* Get Current Time*/
            std::string GetCurFormattedTime();
            
            /*Camera status*/
            bool processData_;   

            /*If Camera enabled*/
            bool camEnabled_;

            /*filenames for image recording*/
            bool saveToFile_;
            std::string camRootDir_;
            std::string camDirName_;
            std::string camTimeStampFileName_;

            /*If current image buffer has data*/
            bool imgHasData_;

            /*Current camera image width & height*/
            int imgWidth_, imgHeight_;

            /*Frame rate*/
            int frameRate_;


            /*record data*/
            bool recordData_;
            
            /*Current image pointer*/
            ImageData *curImgPt_;
            ImageData *nextImgPt_;


            /*Index of img data when data recording*/
            uint64_t imgIndex_;         
            


        private:
            /*Read incoming Cam Data Thread*/
            virtual void ReadDataFromCam(  ) = 0;
            

        
            /*Read/Write Buffers*/
            ImageData imgPublishBuffers_[2];

            /*Current CameraReading id*/
            int camId_;

            /*Data reading & sorting threads*/
            std::thread camThread_;

    };


    class WebCamera : public Camera
    {
        public:
            /*! Constructor with given configurations in WebCamera class
            * @param id is an integer for the cam ID assigned
            * @param CameraCameraInitParams is a CameraCameraInitParams instance which contains all the configiguation paramers
            * @return None
            */
            WebCamera(int id, std::string cameraSource, CameraInitParams CameraInitParams);

            /*! Destructor for WebCamera class which terminate reading thread and clear all memory allocated
            * @return None
            */
            ~WebCamera();

            /*! Read Cam Data and write it to the saved path, pic name is the timestamp
            * @return the cv::Mat object
            */
            cv::Mat GetOneFrame();

            
        private:

            /*VideoCapture instance*/
            cv::VideoCapture vCap_;

            /*VideoCapture source*/
            std::string source_;
            
            /*! Read incoming Cam Data Thread
            * @return None
            */
            void ReadDataFromCam ( );


            /*! If camera is opened, this function will close the connection to the camera, close all saving files, and free the allocated memory
            * @return None
            */
            void Close();


    };

}


#endif
