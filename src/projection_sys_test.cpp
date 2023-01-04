#include <boost/asio.hpp>

#include <iostream>

#include <fstream>

#include "radar_system.h"

#include <sys/types.h>
#ifdef OPENCV
#include <opencv2/opencv.hpp>
#endif 

#ifdef _WIN32
std::string configPath = "..\\..\\config.xml";
#else
std::string configPath = "../config.xml";
#endif

// ======================================== Create an instance of the system =======================================================
oculii::RadarSystem* mySystem = oculii::RadarSystem::GetRadarSystemInstance(configPath);
// =========================================== Main Function ====================================================

int main()
{
    std::cout << "Testing application layer stuff" << std::endl;

    oculii::RadarErrorCode status = mySystem->StartSystem();
    if (status == oculii::RadarErrorCode::SUCCESS)
        std::cout << "Start Radar System success! " << std::endl;                               // validate the config file and instantiate radar objects in the system
    else
    {
        std::cout << "Start Radar System fail with error " << int(status) << std::endl;
        return 0;
    }

    uint32_t oldFrameNumber = 0;

    //create vectors to receive pcl and tracker from different radars. we create to receive only one radar pcl and tracker here
    std::map< int, oculii::RadarTrackerPacket > trk;

    std::cout << "Initialization done" << std::endl;

    mySystem->StartRadarReceive();


    while (true)
    {
        // ================= Read Tracker ==========================
        if (mySystem->GetTracker(trk) == oculii::RadarErrorCode::SUCCESS)
        {
            std::cout << "Trk Timestamp: " << trk.begin()->second.timestamp << " FrameNum: " << trk.begin()->second.frameNumber << " Num of trks: " << trk.begin()->second.numTrkOut << std::endl;
        }
#ifdef OPENCV
        // ================= Read Image ==========================

         cv::Mat frame = mySystem->GetOneImg();

         // ================= Do projection (currently only support one radar and one camera projection) ==========================
         cv::Mat imgWithProjectedTrck;
         std::vector<cv::Point2d> stationaryTrckUv, approachTrckUv, awayTrckUv;
         if (mySystem->GetTrkProj(trk.begin()->second, frame, imgWithProjectedTrck, stationaryTrckUv, approachTrckUv, awayTrckUv) == oculii::RadarErrorCode::SUCCESS)
         {
             std::cout << "stationary tracker num: " << stationaryTrckUv.size() << " approaching tracker num:  " << approachTrckUv.size() << " leaving tracker num:  " << awayTrckUv.size() << std::endl;
             cv::imshow("Projection Visualizer", imgWithProjectedTrck);
             cv::waitKey(1);
         }
#endif

        std::this_thread::sleep_for(std::chrono::microseconds(int(1e6 / 40.0)));
    }


    return 0;


}
