#ifndef READ_PARAM_H
#define READ_PARAM_H

#include <fstream>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

// 读取程序所需参数
class Param
{
public:
	Param()
	{
		int modelnum = 3;
		pP = Eigen::MatrixXd(6, 6);
		pP.fill(0.0);
		for (int i = 0; i < 6; ++i)
		{
			pP(i, i) = 1.;
		}

		pmodel = std::vector<std::string>(modelnum);
		pmodel[0] = "CTRV";
		pmodel[1] = "CTRA";
		pmodel[2] = "CV";

		pQ = std::vector<Eigen::MatrixXd>(modelnum);
		for (int k = 0; k < modelnum; ++k)
		{
			pQ[k] = Eigen::MatrixXd(6, 6);
			pQ[k].fill(0.0);
			for (int i = 0; i < 6; ++i)
			{
				pQ[k](i, i) = 1e-25;
			}
			pQ[k](1, 1) = 4e-2;
			pQ[k](2, 2) = 4e-3;
			pQ[k](3, 3) = 4e-3;
			pQ[k](4, 4) = 1e-15;
		}

		pR = std::vector<Eigen::MatrixXd>(modelnum);
		for (int k = 0; k < modelnum; ++k)
		{
			pR[k] = Eigen::MatrixXd(2, 2);
			pR[k].fill(0.0);
			pR[k](0, 0) = 1e-3;
			pR[k](1, 1) = 1e-3;
		}

		// std::cout<<"LLLLLLLLLLLLLLL"<<std::endl;

		pinteract = Eigen::MatrixXd(modelnum, modelnum);
		pinteract.fill(0.05);
		pinteract(0, 0) = 0.9;
		pinteract(1, 1) = 0.9;
		pinteract(2, 2) = 0.9;
		// pinteract(2,0) = 0.15;
		// pinteract(2,1) = 0.15;
	};

	void ReadFile(std::string &filepath)
	{
		// TODO make config file
	}



	Param operator=(Param &pcopy)
	{
		this->pstate_v = pcopy.pstate_v;
		this->pmea_v = pcopy.pmea_v;
		this->pIou_thresh = pcopy.pIou_thresh;
		this->pg_sigma = pcopy.pg_sigma;
		this->pdist_thresh = pcopy.pdist_thresh;
		this->pmodel = pcopy.pmodel;
		this->pP = pcopy.pP;
		this->pinteract = pcopy.pinteract;
		this->pQ = pcopy.pQ;
		this->pR = pcopy.pR;
		this->pd = pcopy.pd;
	}

	float pd = 1.0;

	int pstate_v = 6; // 6维状态
	int pmea_v = 2;

	float pIou_thresh = 0.1;
	float pg_sigma = 15; // for the Mahalanobis distdance threshold: pi*g*|s|
	float pdist_thresh = 1.5;

	float pi = 3.1415926;

	std::vector<std::string> pmodel;

	Eigen::MatrixXd pP;		   // 初始化协方差
	Eigen::MatrixXd pinteract; // 状态转移概率

	std::vector<Eigen::MatrixXd> pQ;
	std::vector<Eigen::MatrixXd> pR;

	// detect
	int min_pt = 10;  //最小聚类点数
	int max_pt = 1000;  //最大聚类点数
	std::vector<float> min_size = {0.15, 0.1, 0.4}; // 最小尺寸length,width,height
	std::vector<float> max_size = {1.8, 1.5, 1.8};  //最大尺寸
	int Horizon_SCAN = 1024;
	int N_SCAN = 64;
	int downsampleRate = 2; //和SLAM的downsampleRate一样

	// gridmap
	float z_min = -0.6;
	float z_max = 1.5;
	float scan_range = 25.0; //跟踪、局部地图范围
	float localresolution = 0.2;
	int globalmapsize = 300;
	float globalresolution = 0.2;
};
#endif
