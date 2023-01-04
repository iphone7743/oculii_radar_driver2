#include <boost/asio.hpp>
#include <iostream>
#include <fstream>
#include "radar_system.h"
#include <sys/types.h>
#ifdef OPENCV
#include <opencv2/opencv.hpp>
#endif 

#define IMAGE_SIZE 900
#define MAX_DIST 300
#define FOV_ANGLE_LIMIT 50
#define FRAME_RATE_IMU 28

#define USE_FUSION_TRACKER false

std::mutex mutexLock;
 
#ifdef _WIN32
std::string configPath = "..\\..\\config.xml";
#else
std::string configPath="../config.xml";
#endif

// ======================================== Create an instance of the system =======================================================
oculii::RadarSystem *mySystem_ = oculii::RadarSystem::GetRadarSystemInstance(configPath);
bool isExtImu_;
bool isVisualizerEnabled_;
std::atomic<bool> hasNewData_;
std::atomic<bool> bufferSwapped_;
std::map< int, std::vector<oculii::RadarDetectionPacket> >* enhancedPclPacketsReadPt_;
std::map< int, std::vector<oculii::RadarDetectionPacket> >* enhancedPclPacketsDispPt_;
std::map< int, std::vector<oculii::RadarDetectionPacket> > enhancedPclPacketsSwapBuffer_[2];
std::map< int, oculii::RadarDetectionPacket >* pclPacketsReadPt_;
std::map< int, oculii::RadarDetectionPacket >* pclPacketsDispPt_;
std::map< int, oculii::RadarDetectionPacket > pclPacketsSwapBuffer_[2];
std::map< int, oculii::RadarTrackerPacket >* trkPacketsReadPt_;
std::map< int, oculii::RadarTrackerPacket >* trkPacketsDispPt_;
std::map< int, oculii::RadarTrackerPacket > trkPacketsSwapBuffer_[2];

#ifdef OPENCV
cv::VideoWriter video("output.avi", cv::VideoWriter::fourcc('M','J','P','G'), 15, cv::Size(IMAGE_SIZE, IMAGE_SIZE));

// ======================================== Function to display data =======================================================
void DispFrame(unsigned int xMax, unsigned int zMax, float scale)
{
    const int powerThreshold = 0;

    while(true)
    {
        cv::Mat frame = cv::Mat::zeros(cv::Size(IMAGE_SIZE, IMAGE_SIZE), CV_8UC3);
        
        if(!hasNewData_)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
            continue;
        }

        // swap buffer
        auto tmpPt = trkPacketsReadPt_;
        trkPacketsReadPt_ = trkPacketsDispPt_;
        trkPacketsDispPt_ = tmpPt;
        
        if (mySystem_->MeetEnhancedPclCondition())
        {
            // swap buffer
            auto tmpPt = enhancedPclPacketsReadPt_;
            enhancedPclPacketsReadPt_ = enhancedPclPacketsDispPt_;
            enhancedPclPacketsDispPt_ = tmpPt;
            bufferSwapped_ = true;
            hasNewData_ = false;

            for(auto it = enhancedPclPacketsDispPt_->begin(); it != enhancedPclPacketsDispPt_->end(); ++it)
                for (auto& packet : it->second)
                    for (auto& det : packet.data)
                    {
                        if (det.power < powerThreshold)
                            continue;
                        cv::Point2d pt((xMax + det.x) * scale, (zMax - det.z) * scale);
                        switch (det.denoiseFlag)
                        {
                            case 0:
                                break;
                            case 1:
                                cv::circle(frame, pt, 1, cv::Scalar(0, 255, 255), -1, 8);
                                break;
                            case 2:
                                cv::circle(frame, pt, 2, cv::Scalar(0, 255, 0), -1, 8);
                                break;
                            case 3:
                                cv::circle(frame, pt, 2, cv::Scalar(0, 0, 255), -1, 8);
                                break;
                            default:;
                        }
                    }
        }
        else
        {
            // swap buffer
            auto tmpPt = pclPacketsReadPt_;
            pclPacketsReadPt_ = pclPacketsDispPt_;
            pclPacketsDispPt_ = tmpPt;
            bufferSwapped_ = true;
            hasNewData_ = false;

            for(auto it = pclPacketsDispPt_->begin(); it != pclPacketsDispPt_->end(); ++it)
                for (auto& det : it->second.data)
                {
                    if (det.power < powerThreshold)
                        continue;
                    cv::Point2d pt((xMax + det.x) * scale, (zMax - det.z) * scale);
                    switch (det.denoiseFlag)
                    {
                        case 0:
                            cv::circle(frame, pt, 1, cv::Scalar(0, 255, 255), -1, 8);
                        default:;
                    }
                }
        }

        for(auto it = trkPacketsDispPt_->begin(); it != trkPacketsDispPt_->end(); ++it)
            for (auto track : it->second.data)
            {
                cv::Point2d pt((xMax + track.x) * scale, (zMax - track.z) * scale);
                cv::circle(frame, pt, 5, cv::Scalar(152, 107, 252), 1, 8);
            }
        cv::rectangle(frame, cv::Point(IMAGE_SIZE / 2 - 3, IMAGE_SIZE / 2 - 4), cv::Point(IMAGE_SIZE / 2 + 3, IMAGE_SIZE / 2 + 4), cv::Scalar(0, 255, 0));
        //cv::line(frame, cv::Point(xMax * scale, zMax * scale), cv::Point(0, zMax * scale - xMax * scale / tan(boost::math::constants::pi<double>() / 3.0)), cv::Scalar(0, 0, 255), 2);
        //cv::line(frame, cv::Point(xMax * scale, zMax * scale), cv::Point(xMax * 2 * scale, zMax * scale - xMax * scale / tan(boost::math::constants::pi<double>() / 3.0)), cv::Scalar(0, 0, 255), 2);
        cv::imshow("Radar frame Visualizer", frame);
        video.write(frame);
        cv::waitKey(1);
    }
}
#endif 


// =================================Example  function for Imu input ==============================================
void ImuThread()
{
    oculii::GpsImuData tempImuData;

    while (true)
    {

        // ============================ Code to update Imu information to the system ====================================

        /*
        * User code to read and parse imu data through whichever external device they wish
        *
        *
        *
        *
        *
        *
        * user code ends
        */

        tempImuData.yaw = 0;
        tempImuData.pitch = 0;
        tempImuData.roll = 0;
        tempImuData.gpstime = 0;

        mutexLock.lock();
        mySystem_->SetGpsImu(tempImuData);
        mutexLock.unlock();

        std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }
}


// =========================================== Main Function ====================================================

int main()
{
    std::cout << "Testing application layer stuff" << std::endl;
    
    oculii::RadarErrorCode status = mySystem_->StartSystem();
    if(status == oculii::RadarErrorCode::SUCCESS)
        std::cout << "Start Radar System success! " << std::endl;                               // validate the config file and instantiate radar objects in the system
    else
    {
        std::cout << "Start Radar System fail with error " << oculii::ErrorToString(status) << std::endl;
        return 0;
    }
    
    uint32_t oldFrameNumber=0;

    isExtImu_ = mySystem_->GetExternalImuStatus();
    isVisualizerEnabled_ = mySystem_->GetIfVisualizerEnabled();

    // ============ Send Commands to Radar =========================
    //create vectors to uart command parameters for different radars
    std::vector<double> mountingAngles{ 1.0 };
    std::vector<bool> wait{ false };
    std::vector<bool> multicast{ false };
    std::vector<std::string> masterIps{ "192.168.2.20" };
    std::vector <oculii::ModeCommand> modes{ oculii::ModeCommand::SENSOR_MODE_0 };
    std::vector <double> hostYaw{ 0.0 };
    std::vector <double> hostSpeed{ 0.0 };
    std::vector<int> IDs { 1 };
    //mySystem_->UpdateSensor("", IDs);
    //mySystem_->SendHostSetupCfg(IDs, mountingAngles);
    //mySystem_->SendModeSwitchCmd(IDs, modes);
    //mySystem_->SendHostInfo(IDs, hostYaw, hostSpeed);
    //mySystem_->SendPtpRequest(IDs, wait, multicast, masterIps);
    std::cout << "Initialization done" << std::endl;

    //Start Imu thread if External Imu available
    if(isExtImu_)
    {
        // Demo usage for external IMU
        //std::thread th1(ImuThread);
    }

    mySystem_->StartRadarReceive();

    // Init swap buffers
    enhancedPclPacketsReadPt_ = &enhancedPclPacketsSwapBuffer_[0];
    enhancedPclPacketsDispPt_ = &enhancedPclPacketsSwapBuffer_[1];
    pclPacketsReadPt_ = &pclPacketsSwapBuffer_[0];
    pclPacketsDispPt_ = &pclPacketsSwapBuffer_[1];
    trkPacketsReadPt_ = &trkPacketsSwapBuffer_[0];
    trkPacketsDispPt_ = &trkPacketsSwapBuffer_[1];
    
#ifdef OPENCV
    std::thread visThread;
    if(isVisualizerEnabled_)
    {
        // Initialize synchronization flags
        hasNewData_ = false;
        bufferSwapped_ = true;
        // Visualizer thread
        visThread = std::thread(DispFrame, MAX_DIST, MAX_DIST, IMAGE_SIZE / (2.0 * MAX_DIST));
    }
#endif

    while (true)
    {
        if(!bufferSwapped_)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }

        if(USE_FUSION_TRACKER && mySystem_->MeetFusionTrackerCondition())
        {
            if (mySystem_->GetFusionTracker(*trkPacketsReadPt_) == oculii::RadarErrorCode::SUCCESS)
            {
                std::cout << "Fusion Trk FrameNum: " << trkPacketsReadPt_->begin()->second.frameNumber << " Num of trks: " << trkPacketsReadPt_->begin()->second.numTrkOut << std::endl;
            }
        }
        else
        {
            if (mySystem_->GetTracker(*trkPacketsReadPt_) == oculii::RadarErrorCode::SUCCESS)
            {
                std::cout << "Trk Timestamp: " << trkPacketsReadPt_->begin()->second.timestamp << " FrameNum: " << trkPacketsReadPt_->begin()->second.frameNumber << " Num of trks: " << trkPacketsReadPt_->begin()->second.numTrkOut << std::endl;
            }
        }

        // ================= Read enhanced point cloud in the presence of external IMU ==========================
        if (mySystem_->MeetEnhancedPclCondition())
        {

            if (mySystem_->GetEnhancedPointcloud(*enhancedPclPacketsReadPt_) == oculii::RadarErrorCode::SUCCESS)
            {
                std::cout << "Pcl Timestamp: " << enhancedPclPacketsReadPt_->begin()->second.back().timestamp << " FrameNum: " << enhancedPclPacketsReadPt_->begin()->second.back().frameNumber << " Num of objs: " << enhancedPclPacketsReadPt_->begin()->second.back().numObjOut << std::endl << std::endl;
                if(enhancedPclPacketsReadPt_->begin()->second.back().frameNumber - oldFrameNumber != 1)
                    std::cout << "======================== Frame missed =======================" << std::endl;
                oldFrameNumber = enhancedPclPacketsReadPt_->begin()->second.back().frameNumber;
            }
            
        }
        // ================= Read standard pcl packet ==========================
        else
        {
            if (mySystem_->GetPointcloud(*pclPacketsReadPt_) == oculii::RadarErrorCode::SUCCESS)
            {
                std::cout << "Pcl Timestamp: " << pclPacketsReadPt_->begin()->second.timestamp << " FrameNum: " << pclPacketsReadPt_->begin()->second.frameNumber << " Num of objs: " << pclPacketsReadPt_->begin()->second.numObjOut << std::endl << std::endl;
                if(pclPacketsReadPt_->begin()->second.frameNumber - oldFrameNumber != 1)
                    std::cout << "======================== Frame missed =======================" << std::endl;
                oldFrameNumber = pclPacketsReadPt_->begin()->second.frameNumber;
            }
        }
#ifdef OPENCV
        if(isVisualizerEnabled_)
        {
            bufferSwapped_ = false;
            hasNewData_ = true;
        }
#endif
    }

#ifdef OPENCV
    if(isVisualizerEnabled_)
        visThread.join();
#endif

    return 0;
}
