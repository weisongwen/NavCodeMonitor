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
#include <novatel_msgs/INSPVAX.h> // novatel_msgs/INSPVAX
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

/* save result */
FILE* trajectory = fopen("../GraphGNSSLib/src/result/trajectory.csv", "w+");
FILE* posError = fopen("../GraphGNSSLib/src/result/error.csv", "w+");

class gnssSinglePointPositioning
{
    ros::NodeHandle nh;

    // ros subscriber
    ros::Subscriber gnss_raw_sub;
    ros::Publisher pub_WLS, pub_FGO;
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
    // from Weisong
    struct pseudorangeFactor
    {
        pseudorangeFactor(std::string sat_sys, double s_g_x, double s_g_y, double s_g_z, double pseudorange, double var)
                    :sat_sys(sat_sys),s_g_x(s_g_x), s_g_y(s_g_y), s_g_z(s_g_z), pseudorange(pseudorange),var(var){}

        template <typename T>
        bool operator()(const T* state, T* residuals) const
        {
            T est_pseudorange; 
            T delta_x = pow((state[0] - s_g_x),2);
            T delta_y = pow((state[1] - s_g_y),2);
            T delta_z = pow((state[2] - s_g_z),2);
            est_pseudorange = sqrt(delta_x+ delta_y + delta_z);
            
            if(sat_sys == "GPS") 
            {
                est_pseudorange = est_pseudorange - state[3];
            }
            
            else if(sat_sys == "BeiDou") 
            {
                est_pseudorange = est_pseudorange - state[4];
            }

            residuals[0] = (est_pseudorange - T(pseudorange)) / T(var);

            return true;
        }

        double s_g_x, s_g_y, s_g_z, pseudorange, var;
        std::string sat_sys; // satellite system

    };

public:
  gnssSinglePointPositioning()
  {
      gnss_raw_sub = nh.subscribe("/rtk_estimator/GNSS_station", 500, &gnssSinglePointPositioning::gnss_raw_msg_callback, this); // call callback for gnss raw msg

      optimizationThread = std::thread(&gnssSinglePointPositioning::optimization, this);

      pub_WLS = nh.advertise<nav_msgs::Odometry>("WLS_spp", 100); // 
      pub_FGO = nh.advertise<nav_msgs::Odometry>("FGO_spp", 100); //  

    //   ENU_ref<< 114.179000972, 22.3011535667, 0; // 20190428 data
      ENU_ref<<-2414266.9200, 5386768.9870, 2407460.0310;


  }
  ~gnssSinglePointPositioning()
  {
      optimizationThread.detach();
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
        if(msg->GNSS_Raws.size())
        {
            gnss_raw_buf.push(msg); 
            gnss_raw_map[gnss_frame] = *msg;
            Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare(
                                            m_GNSS_Tools.getAllPositions(*msg),
                                            m_GNSS_Tools.getAllMeasurements(*msg),
                                            *msg, "LS");
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

    void optimization()
    {
        while(1)
        {
            // process gnss raw measurements
            // optimize_mux.lock();
            // std::cout << "size of gnss map  "<< gnss_raw_map.size() << std::endl;
            if(gnss_raw_map.size())
            {
                TicToc optimization_time;
                ceres::Problem problem;
                ceres::Solver::Options options;
                options.use_nonmonotonic_steps = true;
                options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
                options.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
                options.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
                // options.num_threads = 8;
                options.max_num_iterations = 100;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                // loss_function = new ceres::HuberLoss(1.0);
                loss_function = NULL;
                
                int length = gnss_raw_map.size();
                double state_array[length][5]; 

                std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter;
                iter = gnss_raw_map.begin();
                for(int i = 0;  i < length; i++,iter++) // initialize
                {
                    nlosExclusion::GNSS_Raw_Array gnss_data = (iter->second);
                    Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare(
                                                m_GNSS_Tools.getAllPositions(gnss_data),
                                                m_GNSS_Tools.getAllMeasurements(gnss_data),
                                                gnss_data, "LS");
                    // state_array[i][0] = eWLSSolutionECEF(0);
                    // state_array[i][1] = eWLSSolutionECEF(1);
                    // state_array[i][2] = eWLSSolutionECEF(2);
                    // state_array[i][3] = eWLSSolutionECEF(3);
                    // state_array[i][4] = eWLSSolutionECEF(4);

                    state_array[i][0] = 0;
                    state_array[i][1] = 0;
                    state_array[i][2] = 0;
                    state_array[i][3] = 0;
                    state_array[i][4] = 0;

                    problem.AddParameterBlock(state_array[i],5);
                }

                std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr;
                iter_pr = gnss_raw_map.begin();
                for(int m = 0;  m < length; m++,iter_pr++) // 
                {
                    nlosExclusion::GNSS_Raw_Array gnss_data = (iter_pr->second);
                    MatrixXd weight_matrix; //goGPS weighting
                    weight_matrix = m_GNSS_Tools.cofactorMatrixCal_WLS(gnss_data, "LS"); //goGPS
                    // std::cout << "weight_matrix-> "<<weight_matrix<<std::endl;
                    int sv_cnt = gnss_data.GNSS_Raws.size();
                    for(int i =0; i < sv_cnt; i++)
                    {
                        std::string sat_sys;
                        double s_g_x = 0, s_g_y = 0,s_g_z = 0, var = 1;
                        double pseudorange = 0;
                        if(m_GNSS_Tools.PRNisGPS(gnss_data.GNSS_Raws[i].prn_satellites_index)) sat_sys = "GPS";
                        if(m_GNSS_Tools.PRNisBeidou(gnss_data.GNSS_Raws[i].prn_satellites_index)) sat_sys = "BeiDou";

                        s_g_x = gnss_data.GNSS_Raws[i].sat_pos_x;
                        s_g_y = gnss_data.GNSS_Raws[i].sat_pos_y;
                        s_g_z = gnss_data.GNSS_Raws[i].sat_pos_z;

                        pseudorange = gnss_data.GNSS_Raws[i].pseudorange;
                        
                        ceres::CostFunction* ps_function = new ceres::AutoDiffCostFunction<pseudorangeFactor, 1 
                                                                , 5>(new 
                                                                pseudorangeFactor(sat_sys, s_g_x, s_g_y, s_g_z, pseudorange, sqrt(1/weight_matrix(i,i))));
                        problem.AddResidualBlock(ps_function, loss_function, state_array[m]);
                    }
                }

                ceres::Solve(options, &problem, &summary);
                  std::cout << summary.BriefReport() << "\n";
                //   std::cout << "state_array[i][0]"<<state_array[length-1][0]<<std::endl;
                //   std::cout << "state_array[i][1]"<<state_array[length-1][1]<<std::endl;
                //   std::cout << "state_array[i][2]"<<state_array[length-1][2]<<std::endl;
                Eigen::Matrix<double ,3,1> ENU;
                
                Eigen::Matrix<double, 3,1> state;
                
                state<< state_array[length-1][0], state_array[length-1][1], state_array[length-1][2];
                ENU = m_GNSS_Tools.ecef2enu(m_GNSS_Tools.ecef2llh(ENU_ref), state);
                // LOG(INFO) << "ENU- FGO-> "<< std::endl<< ENU;

                nav_msgs::Odometry odometry;
                // odometry.header = pose_msg->header;
                odometry.header.frame_id = "map";
                odometry.child_frame_id = "map";
                odometry.pose.pose.position.x = ENU(0);
                odometry.pose.pose.position.y = ENU(1);
                odometry.pose.pose.position.z = ENU(2);
                pub_FGO.publish(odometry);

                FILE* FGO_trajectory = fopen("../GraphGNSSLib/src/result/FGO_trajectory.csv", "w+");
                fgo_path.poses.clear();
                // for(int m = 0;  m < length; m++) // 
                // {
                //     state<< state_array[m][0], state_array[m][1], state_array[m][2];
                //     ENU = m_GNSS_Tools.ecef2enu(ENU_ref, state);
                //     fprintf(FGO_trajectory, "%d,%7.5f,%7.5f,%7.5f  \n", m, ENU(0),ENU(1),ENU(2));
                //     fflush(FGO_trajectory);
                //   //   FGO_trajectory->close();

                    
                // }

                // std::cout << "optimization_time-> "<<optimization_time.toc()<<std::endl;
            }
            std::chrono::milliseconds dura(1000); // this thread sleep for 100 ms
            std::this_thread::sleep_for(dura);
            // gnss_raw_map.clear();
            optimize_mux.unlock();
            
        }
    }

};

int main(int argc, char **argv)
{
    FLAGS_logtostderr = 1;  // output to console
    google::InitGoogleLogging(argv[0]); // init the google logging
    google::ParseCommandLineFlags(&argc, &argv, true); // parseCommandLineFlags 
    ros::init(argc, argv, "spp");  
    // ...
    gnssSinglePointPositioning gnssSinglePointPositioning;
    ros::spin();
    return 0;
}
