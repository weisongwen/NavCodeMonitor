/*******************************************************
 * Copyright (C) 2019, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
 * 
 * This file is part of GraphGNSSLib.
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Weisong Wen (weisong.wen@connect.polyu.hk)
 *******************************************************/

#include <stdarg.h>
#include "../../RTKLIB/src/rtklib.h"
#include <ros/ros.h>
#include <stdio.h>
#include <assert.h>
#define ENACMP 1

extern void postposRegisterPub(ros::NodeHandle &n);
extern void rtkposRegisterPub(ros::NodeHandle &n);
extern void pntposRegisterPub(ros::NodeHandle &n);

/* read .obs, .nav files and perform RTK GNSS and save results to file */

// #include "utilities.h"
// #include "WeightLeastSquare.h"
// #include "SelectEphemeris.h"

/* rnx2rtkp main -------------------------------------------------------------*/

int main(int argc, char **argv)
{
	
    ros::init(argc, argv, "rtk_estimator");
	ros::NodeHandle nh("~");
    std::cout<<"......rtk_estimator node start......"<<std::endl;

    int n=0,i,stat;
	postposRegisterPub(nh);
	rtkposRegisterPub(nh);
	pntposRegisterPub(nh);

	/* processing time setting */
	double ti=0.0;						// processing interval  (s) (0:all)
	double tu=0.0;						// unit time (s) (0:all)
	gtime_t ts={0},te={0};
	ts.time=0;							// start time (0:all)
	te.time=0;							// end time (0:all)

	/* options */
	prcopt_t prcopt = prcopt_default;	// processing option
	solopt_t solopt = solopt_default;	// output solution option
	filopt_t filopt = {""};	            // file option

	prcopt.mode = PMODE_KINEMA;			// Kinematic RTK
	// prcopt.mode = PMODE_SINGLE;			// SPP
	prcopt.navsys=SYS_ALL;              // use all satellites system
	prcopt.nf=2;						// frequency (1:L1,2:L1+L2,3:L1+L2+L5) 
	prcopt.soltype=0;					// 0:forward,1:backward,2:combined
	prcopt.elmin=15.0*D2R;				// elevation mask (rad)	
	prcopt.tidecorr=0;					// earth tide correction (0:off,1-:on) 
	prcopt.posopt[4] = 0;               // use RAIM FDE (qmo)  1
	prcopt.tropopt=TROPOPT_SAAS;        // troposphere option: Saastamoinen model
	prcopt.ionoopt=IONOOPT_BRDC;		// ionosphere option: Broad cast
	prcopt.sateph=EPHOPT_BRDC;			// ephemeris option: broadcast ephemeris

	prcopt.modear=3;					// AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold)

	// prcopt.rb[0] = -2419080.3906;			// base position for relative mode {x,y,z} (ecef) (m)
	// prcopt.rb[1] = 5379846.0123;				// base position for relative mode {x,y,z} (ecef) (m)
	// prcopt.rb[2] = 2418100.6068;				// base position for relative mode {x,y,z} (ecef) (m)

	solopt.outopt=1;					// output processing options (0:no,1:yes)
	solopt.timef=0;						// time format (0:sssss.s,1:yyyy/mm/dd hh:mm:ss.s)
	solopt.timeu=3;						// time digits under decimal point
	solopt.sep[0]=',';					// field separator
	solopt.sstat=0;						// solution statistics level (0:off,1:states,2:residuals)
	solopt.trace=0;						// debug trace level (0:off,1-5:debug)
	solopt.sstat=0;						// get the solution file
	solopt.posf = SOLF_LLH;
	solopt.height = 0;

	char *rov="",*base="";
	char infile_[10][1024]={""}, *infile[10];
	char outfile[1024];

	/* set input files */
	for (i=0;i<10;i++) infile[i]=infile_[i];
	

	#if RTK_FGO
	/* HK TST data, 20200603, for GPS solution evaluation (static, ivan) */
	strcpy(infile[n++],"/home/wws/GraphGNSSLib/src/global_fusion/src/rtk_estimator/gps_solution_TST2/2020_06_03_TST_03.obs");
	strcpy(infile[n++],"/home/wws/GraphGNSSLib/src/global_fusion/src/rtk_estimator/gps_solution_TST2/hksc155d.20o");
	strcpy(infile[n++],"/home/wws/GraphGNSSLib/src/global_fusion/src/rtk_estimator/gps_solution_TST2/hksc155d.20b");
	strcpy(infile[n++],"/home/wws/GraphGNSSLib/src/global_fusion/src/rtk_estimator/gps_solution_TST2/hksc155d.20n");
	prcopt.rb[0] = -2414266.9197;			// base position for relative mode {x,y,z} (ecef) (m)
	prcopt.rb[1] = 5386768.9868;			// base position for relative mode {x,y,z} (ecef) (m)
	prcopt.rb[2] = 2407460.0314;			// base position for relative mode {x,y,z} (ecef) (m)
	#else

	/* HK TST data, 20190428, for GPS solution evaluation (dynamic, LOOP) */
	strcpy(infile[n++],"/home/wws/GraphGNSSLib/src/global_fusion/src/rtk_estimator/gps_solution_TST//COM3_190428_124409.obs");
	strcpy(infile[n++],"/home/wws/GraphGNSSLib/src/global_fusion/src/rtk_estimator/gps_solution_TST//hksc1180.19o");
	strcpy(infile[n++],"/home/wws/GraphGNSSLib/src/global_fusion/src/rtk_estimator/gps_solution_TST//hksc1180.19b");
	strcpy(infile[n++],"/home/wws/GraphGNSSLib/src/global_fusion/src/rtk_estimator/gps_solution_TST//hksc1180.19n");
	prcopt.rb[0] = -2414266.9197;			// base position for relative mode {x,y,z} (ecef) (m)
	prcopt.rb[1] = 5386768.9868;			// base position for relative mode {x,y,z} (ecef) (m)
	prcopt.rb[2] = 2407460.0314;			// base position for relative mode {x,y,z} (ecef) (m)
	#endif



	
	/* set output files */
	strcpy(outfile, "/home/wws/GraphGNSSLib/src/global_fusion/src/rtk_estimator/test.pos");

	// while(ros::ok())
	{
		/* post positioning */
		stat=postpos(ts,te,ti,tu,&prcopt,&solopt,&filopt,infile,n,outfile,rov,base);

		printf("\n");
		if(stat==0){
			printf("rtklib completed...\n");
		}
		else if(stat>0){
			printf("rtklib has rrrors...\n");
		}
		else if(stat==-1){
			printf("rtklib aborted...\n");
		}
		// system("pause");
	}
	

    ros::spin();
    return 0    ;
}

// /* dummy application functions for shared library ----------------------------*/
extern int showmsg(char *format,...) {
	va_list arg;
	char buff[1024];
	if (*format) {
		va_start(arg,format);
		vsprintf(buff,format,arg);
		va_end(arg);
		printf("%s\n",buff);
	}
	return 0;	
}
extern void settspan(gtime_t ts, gtime_t te) {}
extern void settime(gtime_t time) {}
