
#define D2R 3.1415926/180.0
#include <nlosExclusion/GNSS_Raw_Array.h>

// std inputs and outputs, fstream
#include <iostream>
#include <string>  
#include <fstream>
#include <sstream>
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

using namespace Eigen;
#define pi_ 3.1415926
#define D2R 3.1415926/180.0
#define minGPSCnt 4
#define minBeidouCnt 1

#define useEleVar 1

#define use_fixed_cov_ar 0

/**
   * @brief  weighted least square for signle point positioning
   * @param eAllSVPositions ((n,4) prn, sx, sy, sz, )     eAllSVPositions ((n,3) PRN CNO Pseudorange)
   * @return eWLSSolution 5 unknowns with two clock bias variables
   @ 
  */
  Eigen::MatrixXd WeightedLeastSquare_GPS(Eigen::MatrixXd eAllSVPositions, Eigen::MatrixXd eAllMeasurement, nlosExclusion::GNSS_Raw_Array GNSS_data){
  
    Eigen::MatrixXd eWLSSolution;
    eWLSSolution.resize(4, 1);

    MatrixXd weight_matrix = cofactorMatrixCal_WLS(GNSS_data, "WLS");

    /**after read the obs file, one measure is not right**/
    int validNumMeasure=0;
    std::vector<int> validMeasure;
    for (int idx = 0; idx < eAllMeasurement.rows(); idx++){
      for (int jdx = 0; jdx < eAllSVPositions.rows(); jdx++){
        if (int(eAllMeasurement(idx, 0)) == int(eAllSVPositions(jdx, 0))){
          validNumMeasure++;
          validMeasure.push_back(int(eAllMeasurement(idx, 0)));
        }
      }
    }

    Eigen::MatrixXd validMeasurement; // for WLS 
    validMeasurement.resize(validNumMeasure,eAllMeasurement.cols());
    for (int idx = 0; idx < eAllMeasurement.rows(); idx++){
      for (int jdx = 0; jdx < eAllSVPositions.rows(); jdx++){
        if (int(eAllMeasurement(idx, 0)) == int(eAllSVPositions(jdx, 0))){
          for (int kdx = 0; kdx < eAllMeasurement.cols(); kdx++){
            // std::cout<<"satellite prn -> "<<eAllMeasurement(idx, 0)<<"\n"<<std::endl;
            validMeasurement(idx, kdx) = eAllMeasurement(idx, kdx);
            
          }
        }
      }
    }


    int iNumSV = validMeasurement.rows();

    /*Find the received SV and Sort based on the order of Measurement matrix*/
    Eigen::MatrixXd eExistingSVPositions; // for WLS
    eExistingSVPositions.resize(iNumSV, eAllSVPositions.cols());

    for (int idx = 0; idx < validMeasurement.rows(); idx++){
      for (int jdx = 0; jdx < eAllSVPositions.rows(); jdx++){
        if (int(validMeasurement(idx, 0)) == int(eAllSVPositions(jdx, 0))){
          for (int kdx = 0; kdx < eAllSVPositions.cols(); kdx++){
            // std::cout<<"satellite prn -> "<<eAllMeasurement(idx, 0)<<"\n"<<std::endl;
            eExistingSVPositions(idx, kdx) = eAllSVPositions(jdx, kdx);
            
          }
        }
      }
    } 

    //Intialize the result by guessing.
    for (int idx = 0; idx < eWLSSolution.rows(); idx++){
      eWLSSolution(idx, 0) = 0;
    }
    
    // for the case of insufficient satellite
    if (iNumSV < 5){
      std::cout<<"satellite number is not enough" <<std::endl;
      return eWLSSolution;
    }

    bool bWLSConverge = false;

    int count = 0;
    while (!bWLSConverge)
    {
      Eigen::MatrixXd eH_Matrix;
      eH_Matrix.resize(iNumSV, eWLSSolution.rows());

      Eigen::MatrixXd eDeltaPr;
      eDeltaPr.resize(iNumSV, 1);

      Eigen::MatrixXd eDeltaPos;
      eDeltaPos.resize(eWLSSolution.rows(), 1);

      for (int idx = 0; idx < iNumSV; idx++){

        int prn = int(validMeasurement(idx, 0));
        double pr = validMeasurement(idx, 2);
        
        // Calculating Geometric Distance
        double rs[3], rr[3], e[3];
        double dGeoDistance;

        rs[0] = eExistingSVPositions(idx, 1);
        rs[1] = eExistingSVPositions(idx, 2);
        rs[2] = eExistingSVPositions(idx, 3);

        rr[0] = eWLSSolution(0);
        rr[1] = eWLSSolution(1);
        rr[2] = eWLSSolution(2);

        // dGeoDistance = geodist(rs, rr, e);
        dGeoDistance = sqrt(pow((rs[0] - rr[0]),2) + pow((rs[1] - rr[1]),2) +pow((rs[2] - rr[2]),2));
        
        double OMGE_ = 7.2921151467E-5;
        double CLIGHT_ = 299792458.0;
        dGeoDistance = dGeoDistance + OMGE_ * (rs[0]*rr[1]-rs[1]*rr[0])/CLIGHT_;

        // Making H matrix      
        eH_Matrix(idx, 0) = -(rs[0] - rr[0]) / dGeoDistance;
        eH_Matrix(idx, 1) = -(rs[1] - rr[1]) / dGeoDistance;
        eH_Matrix(idx, 2) = -(rs[2] - rr[2]) / dGeoDistance;

        if (PRNisGPS(prn)){
          eH_Matrix(idx, 3) = 1;
        }
        // Making delta pseudorange
        double rcv_clk_bias;
        if (PRNisGPS(prn)){
          rcv_clk_bias = eWLSSolution(3);       
        }
        // double sv_clk_bias = eExistingSVPositions(idx, 4) * CLIGHT;
        eDeltaPr(idx, 0) = pr - dGeoDistance + rcv_clk_bias;
        //printf("%2d - %f %f %f %f \n", prn, pr, dGeoDistance, eDeltaPr(idx, 0), rcv_clk_bias);
      }

      // Least Square Estimation 
      eDeltaPos = (eH_Matrix.transpose() * weight_matrix * eH_Matrix).inverse() * eH_Matrix.transpose() * weight_matrix * eDeltaPr;
      //eDeltaPos = eH_Matrix.householderQr().solve(eDeltaPr);


      eWLSSolution(0) += eDeltaPos(0);
      eWLSSolution(1) += eDeltaPos(1);
      eWLSSolution(2) += eDeltaPos(2);
      eWLSSolution(3) += eDeltaPos(3);

      for (int i = 0; i < 3; ++i){
        //printf("%f\n", fabs(eDeltaPos(i)));
        if (fabs(eDeltaPos(i)) >1e-4)
        {
          bWLSConverge = false;
        }
        else { 
          bWLSConverge = true;
        };
        
      }
      count += 1;
      if (count > 6)
      {
        bWLSConverge = true;
        std::cout<<" more than 6 times in iterations"<<std::endl;
      }
    }
    // printf("WLS -> (%11.2f,%11.2f,%11.2f)\n\n", eWLSSolution(0), eWLSSolution(1), eWLSSolution(2));
    std::cout << std::setprecision(12);

    return eWLSSolution;
  }


  /**
   * @brief covariance estimation
   * @param nlosExclusion::GNSS_Raw_Array GNSS_data
   * @return weight_matrix
   @ 
   */
  Eigen::MatrixXd cofactorMatrixCal_WLS(nlosExclusion::GNSS_Raw_Array GNSS_data, std::string method)
  {
    Eigen::Matrix<double,4,1> parameters;
    parameters<<50.0, 30.0, 30.0, 10.0; // loosely coupled 
    // parameters<<50.0, 30.0, 20.0, 30.0; // loosely coupled 
    double snr_1 = parameters(0); // T = 50
    double snr_A = parameters(1); // A = 30
    double snr_a = parameters(2);// a = 30
    double snr_0 = parameters(3); // F = 10

    VectorXd cofactor_;  // cofactor of satellite
    cofactor_.resize(GNSS_data.GNSS_Raws.size());
    for(int i = 0; i < GNSS_data.GNSS_Raws.size(); i++)
    {
      if( 1 )
      {
        double snr_R = GNSS_data.GNSS_Raws[i].snr;
        double elR = GNSS_data.GNSS_Raws[i].elevation;
        double q_R_1 = 1 / (pow(( sin(elR * 3.1415926/180.0 )),2));
        double q_R_2 = pow(10,(-(snr_R - snr_1) / snr_a));
        double q_R_3 = (((snr_A / (pow(10,(-(snr_0 - snr_1) / snr_a))) - 1) / (snr_0 - snr_1)) * (snr_R - snr_1) + 1);
        double q_R = q_R_1* (q_R_2 * q_R_3);
        cofactor_[i]=(1.0/float(q_R)); // uncertainty: cofactor_[i] larger, larger uncertainty
      }
    }
    // cout<<"cofactor_ -> "<<cofactor_<<endl;

    MatrixXd weight_matrix;
    weight_matrix.resize(GNSS_data.GNSS_Raws.size(),GNSS_data.GNSS_Raws.size());
    weight_matrix.setIdentity();
    for(int k = 0; k < weight_matrix.rows(); k++)
    {
      weight_matrix.row(k) = weight_matrix.row(k) * cofactor_(k);
    }
    if(method == "WLS")
    {
      return weight_matrix;
    }
    else
    {
      weight_matrix.setIdentity();
      return weight_matrix;
    }
  }
