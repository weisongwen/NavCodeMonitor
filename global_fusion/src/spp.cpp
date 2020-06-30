/*******************************************************
 * Copyright (C) 2019, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
 * 
 * This file is part of GraphGNSSLib.
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Weisong Wen (weisong.wen@connect.polyu.hk)
 *******************************************************/

// std inputs and outputs, fstream
#include <iostream>
#include <string>  
#include <fstream>
#include<sstream>
#include <stdlib.h>
#include <iomanip>

// math
#include <math.h>
//time 
#include <time.h>
//algorithm 
#include <algorithm>

// google eigen
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include<Eigen/Core>

// google implements commandline flags processing.
#include <gflags/gflags.h>
// google loging tools
#include <glog/logging.h>
// ros
#include <ros/ros.h>
/* Reference from NovAtel GNSS/INS */
// #include <novatel_msgs/INSPVAX.h> // novatel_msgs/INSPVAX
#include "gnss_tools.h"
#include <nlosExclusion/GNSS_Raw_Array.h>


#include <geometry_msgs/Point32.h>
#include <stdio.h>
#include <queue>
#include <map>
#include <queue>
#include <mutex>
#include <thread>
#include <nav_msgs/Odometry.h>

#include <nav_msgs/Path.h>


#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "tic_toc.h"

#include <signal.h>


using namespace std;

/* save result */
FILE* trajectory = fopen("../GraphGNSSLib/src/result/trajectory.csv", "w+");
FILE* posError = fopen("../GraphGNSSLib/src/result/error.csv", "w+");
bool app_stopped = false;
void sigint_handler(int sig){
	if(sig == SIGINT){
		// ctrl+c退出时执行的代码
		std::cout << "ctrl+c pressed!" << std::endl;
		app_stopped = true;
	}
  }
    

class gnssSinglePointPositioning
{
    ros::NodeHandle nh;

    // ros subscriber
    ros::Subscriber gnss_raw_sub;
    ros::Publisher pub_WLS, pub_FGO, GNSS_raw_pub;
    std::queue<nlosExclusion::GNSS_Raw_ArrayConstPtr> gnss_raw_buf;
    std::map<double, nlosExclusion::GNSS_Raw_Array> gnss_raw_map;

    std::mutex m_gnss_raw_mux;
    std::mutex optimize_mux;
    std::thread optimizationThread;

    GNSS_Tools m_GNSS_Tools; // utilities

    nav_msgs::Path fgo_path;

    int gnss_frame = 0;

    Eigen::Matrix<double, 3,1> ENU_ref;
    

    
public:
  gnssSinglePointPositioning()
  {
      gnss_raw_sub = nh.subscribe("/rtk_estimator/GNSS_data11", 500, &gnssSinglePointPositioning::gnss_raw_msg_callback, this); // call callback for gnss raw msg

    //   optimizationThread = std::thread(&gnssSinglePointPositioning::optimization, this);

      pub_WLS = nh.advertise<nav_msgs::Odometry>("WLS_spp", 100); // 
      pub_FGO = nh.advertise<nav_msgs::Odometry>("FGO_spp", 100); //  
      GNSS_raw_pub = nh.advertise<nlosExclusion::GNSS_Raw_Array>("/rtk_estimator/GNSS_data", 500);
      ENU_ref<< 114.177707462258, 22.2999154035483, 4.89580645161292; // 20190428 data
    //   ENU_ref<<-2414266.9200, 5386768.9870, 2407460.0310;
      signal(SIGINT, sigint_handler);
      getUbloxNMEASolution();


  }
  ~gnssSinglePointPositioning()
  {
    //   optimizationThread.detach();
  }


  
    /* get the GNSS raw measurements*/
  void getUbloxNMEASolution()
  {
    // while(1)
    {
      std::chrono::milliseconds dura(1000); // this thread sleep for any ms
      std::this_thread::sleep_for(dura);
      // load image list
      FILE* NMEAFile;
      nlosExclusion::GNSS_Raw_Array gnss_data;
      int last_gps_sec = 270152;
      int current_gps_sec = 270152;
      char line[1024];
      // std::string NMEAPath = "../ion_GNSS_2020/src/data/ublox_190331_084530.csv";
      std::string NMEAPath = "../NavCodeMonitor/src/data/GNSSData.csv";
      NMEAFile = std::fopen((NMEAPath).c_str() , "r");
      if(NMEAFile == NULL){
          printf("cannot find file: ublox NMEA File \n", NMEAPath.c_str());
          ROS_BREAK();
	    }

      if(1)
      {
         while ((fscanf(NMEAFile, "%[^\n]", line)) != EOF)
        {
            if (app_stopped){
			break;
		    }

          fgetc(NMEAFile);    // Reads in '\n' character and moves file
                            // stream past delimiting character
        //   printf("Line = %s \n", line);
          std::stringstream ss(line); // split into three string
          std::vector<string> result;
          while (ss.good())
          {
            string substr;
            getline(ss, substr, ',');
            result.push_back(substr);
            std::cout << std::setprecision(17);
          }

          nlosExclusion::GNSS_Raw sv_data;
          sv_data.GNSS_time =strtod((result[0]).c_str(), NULL);
          sv_data.pseudorange = strtod((result[1]).c_str(), NULL);
          sv_data.snr = strtod((result[2]).c_str(), NULL);
          sv_data.elevation = strtod((result[3]).c_str(), NULL);
          sv_data.azimuth = strtod((result[4]).c_str(), NULL);
          sv_data.err_tropo = strtod((result[5]).c_str(), NULL);
          sv_data.err_iono = strtod((result[6]).c_str(), NULL);
          sv_data.sat_clk_err = strtod((result[7]).c_str(), NULL);
          sv_data.sat_pos_x = strtod((result[8]).c_str(), NULL);
          sv_data.sat_pos_y = strtod((result[9]).c_str(), NULL);
          sv_data.sat_pos_z = strtod((result[10]).c_str(), NULL);
          sv_data.prn_satellites_index = strtod((result[11]).c_str(), NULL);
          gnss_data.GNSS_Raws.push_back(sv_data);

          current_gps_sec = int(sv_data.GNSS_time);

          if(current_gps_sec != last_gps_sec)
          {
              GNSS_raw_pub.publish(gnss_data);
              
              last_gps_sec = current_gps_sec;
              LOG(INFO) << "new GNSS data: " <<current_gps_sec;
              std::chrono::milliseconds dura(1000); // this thread sleep for any ms
              std::this_thread::sleep_for(dura);

              Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare_GPS(
                                            m_GNSS_Tools.getAllPositions(gnss_data),
                                            m_GNSS_Tools.getAllMeasurements(gnss_data),
                                            gnss_data);
            Eigen::Matrix<double ,3,1> ENU;
            // Eigen::Matrix<double, 3,1> ENU_ref;
            // ENU_ref<< 114.179000972, 22.3011535667, 0;
            // ENU = m_GNSS_Tools.ecef2enu(ENU_ref, eWLSSolutionECEF);
            std::cout<< " WLS solution -> "<< eWLSSolutionECEF<< std::endl ;
            // std::cout << "eWLSSolutionECEF"<<eWLSSolutionECEF<<std::endl;

            // nav_msgs::Odometry odometry;
            // // odometry.header = pose_msg->header;
            // odometry.header.frame_id = "map";
            // odometry.child_frame_id = "map";
            // odometry.pose.pose.position.x = ENU(0);
            // odometry.pose.pose.position.y = ENU(1);
            // odometry.pose.pose.position.z = ENU(2);
            // pub_WLS.publish(odometry);
            gnss_data.GNSS_Raws.clear();
          }
          

          std::cout << std::setprecision(17);
        //   m_NMEAVector[int(NMEAData_.Timestamp)] = NMEAData_;
          // std::cout<<"m_NMEAVector.size() = "<<m_NMEAVector.size()<<std::endl;
        }
      }
      std::fclose(NMEAFile);
    }
  }

   /**
   * @brief gnss raw msg callback
   * @param gnss raw msg
   * @return void
   @ 
   */
    void gnss_raw_msg_callback(const nlosExclusion::GNSS_Raw_ArrayConstPtr& msg)
    {
        m_gnss_raw_mux.lock();
        gnss_frame++;
        std::cout << "get gnss data -> " <<std::endl;
        if(msg->GNSS_Raws.size())
        {
            gnss_raw_buf.push(msg); 
            gnss_raw_map[gnss_frame] = *msg;
            Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare(
                                            m_GNSS_Tools.getAllPositions(*msg),
                                            m_GNSS_Tools.getAllMeasurements(*msg),
                                            *msg, "WLS");
            Eigen::Matrix<double ,3,1> ENU;
            // Eigen::Matrix<double, 3,1> ENU_ref;
            // ENU_ref<< 114.179000972, 22.3011535667, 0;
            ENU = m_GNSS_Tools.ecef2enu(m_GNSS_Tools.ecef2llh(ENU_ref), eWLSSolutionECEF);
            LOG(INFO) << "ENU WLS -> "<< std::endl << ENU;
            // std::cout << "eWLSSolutionECEF"<<eWLSSolutionECEF<<std::endl;

            nav_msgs::Odometry odometry;
            // odometry.header = pose_msg->header;
            odometry.header.frame_id = "map";
            odometry.child_frame_id = "map";
            odometry.pose.pose.position.x = ENU(0);
            odometry.pose.pose.position.y = ENU(1);
            odometry.pose.pose.position.z = ENU(2);
            pub_WLS.publish(odometry);
        }
        m_gnss_raw_mux.unlock();

    }

};

int main(int argc, char **argv)
{
    FLAGS_logtostderr = 1;  // output to console
    google::InitGoogleLogging(argv[0]); // init the google logging
    google::ParseCommandLineFlags(&argc, &argv, true); // parseCommandLineFlags 
    ros::init(argc, argv, "GNSS_SPP");  
    // ...
    gnssSinglePointPositioning gnssSinglePointPositioning;
    ros::spin();
    return 0;
}
