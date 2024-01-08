/*
 * author: wx
 * date: 2020.12.13
 * reference:
 */
#include "imm_ukf.h"

void IMM_UKF::IMM_Initialization(Eigen::VectorXd& Z, float time, float velo, float angle){
	//TODO initial x_merge_ interact_pro_
	isinitialized = true;
	
	// filewrite.open("/home/wx/Desktop/tracking_code/trackernew/result.txt", std::ios::out|std::ios::app);

	Eigen::VectorXd x(n_x_);  //6维
	x.fill(0.0);
	for(int k=0; k<n_z_; ++k){   //量测
		x(k) = Z(k);
	}

	x(n_z_) = velo;
	x(n_z_+1) = angle;  //没有角速度和加速度初始化
	//初始化状态量
	for(int i=0; i<model_size; ++i){  //model_size=3 三种模型
		model_X_[i] = x;
	}
	// std::cout<<"############################## IMM initial ########################### "<<velo<<std::endl;

	for(int i=0; i<model_size; ++i){
		imm_ukf_[i].Initialization(model_X_[i],model_P_[i], time); //初始化三种UKF模型 初值都一样
	}
}

//输入交互
void IMM_UKF::InputInteract(){
	// std::cout<<"########## IMM InputInteract ##########\n"<<model_pro_<<std::endl;
	if(std::isnan(model_pro_(0)))
		std::abort();
	if(model_pro_.sum() !=0)
		model_pro_ /model_pro_.sum();  //归一化

	c_.fill(0.0);
	//the jth model
	for(int j=0; j<model_size; ++j){
		// 获取上一时刻的状态估计和状态协方差估计
		model_X_[j] = imm_ukf_[j].Get_state();
		model_P_[j] = imm_ukf_[j].Get_covariance();
		for(int i=0; i<model_size; ++i){
			c_(j) += interact_pro_(i,j)*model_pro_(i); //i转移到j的概率之和，即权重
		}
	}
	// std::cout<<"########## IMM InputInteract ##########"<<std::endl;
	for(int j=0; j<model_size; ++j){
		X_hat_[j].fill(0.);
		P_hat_[j].fill(0.);
		for(int i=0; i<model_size; ++i){
			float u =  ((interact_pro_(i,j)*model_pro_(i))/c_(j));  //权值
			X_hat_[j] += u * model_X_[i]; //problem  状态加权估计
		}
		for(int i=0; i<model_size; ++i){
			float u =  (interact_pro_(i,j)*model_pro_(i))/c_(j);		
			P_hat_[j] += (u * (model_P_[i] + (model_X_[i] - X_hat_[j])*(model_X_[i] - X_hat_[j]).transpose()));
		}
	}		
	
}	

//预测
void IMM_UKF::PredictionZmerge(float time){
	// std::cout<<"############################## IMM Prediction #############################"<<std::endl;
	InputInteract();  //输入交互

	std::vector<float> model_ita(model_size);  //干嘛的

	S_merge_ = Eigen::MatrixXd(n_z_,n_z_);
	S_merge_.fill(0.0);

	Zpre_merge_ = Eigen::VectorXd(n_z_);
	Zpre_merge_.fill(0.0);
	// std::cout<<"############################## IMM Prediction1 #############################"<<std::endl;

	for(int i=0; i<model_size; ++i){
		imm_ukf_[i].PredictionZ(X_hat_[i], P_hat_[i], time);  //三种模型分别预测
		//TODO get zmerge
		Zpre_merge_ += (model_pro_(i) * imm_ukf_[i].Get_PredictionZ());  //model_pro_(i)模型概率加权预测的量测
	}
	// std::cout<<"############################## IMM Prediction2 #############################"<<std::endl;

	for(int i=0; i<model_size; ++i){
		Eigen::MatrixXd S = imm_ukf_[i].Get_S();
		Eigen::VectorXd Zp = imm_ukf_[i].Get_PredictionZ();
		S_merge_ += ( model_pro_(i) * (S + (Zp - Zpre_merge_)*(Zp - Zpre_merge_).transpose()));
	}

	// std::cout<<"########## Zpre_merge_ ##########\n"<<Zpre_merge_<<"\n ########## Spre_merge_ ##########\n"<<S_merge_<<"\n"<<std::endl;

	// std::cout<<"#################### IMM Prediction END #############################"<<std::endl;
}

//模型以及概率更新		
void IMM_UKF::UpdateProbability(std::vector<Eigen::VectorXd>& Z, const Eigen::VectorXd& beta, const float& last_beta){
	// std::cout<<"#################### IMM UpdateProbability #############################"<<std::endl;

	std::vector<float> model_ita(model_size);

	for(int i=0; i<model_size; ++i){

		imm_ukf_[i].Update(Z, beta, last_beta);
	    //Zminus 是 IMM-UKF 算法中的一部分用于计算更新后的概率的中间变量，它表示预测值与观测值的差。
	   //在 IMM-UKF 算法中，对于每个模型，使用 UKF 进行预测和更新，从中得到预测值和观测值，然后计算 Zminus。
	   //在计算更新概率时，需要使用 Zminus 和测量噪声协方差矩阵 S 来计算中间变量 ita。
		Eigen::VectorXd Zminus = imm_ukf_[i].Get_Zminus();
		Eigen::MatrixXd S = imm_ukf_[i].Get_S();
       //当前模型下得到该观测值的概率密度函数值
		float ita = 1/(sqrt(2*pi_)*sqrt(fabs(S.determinant())))*exp(-0.5* Zminus.transpose() * S.inverse() * Zminus);
		model_ita[i] = ita;
	}
	//所有模型下的观测概率的和
	float c_temp = 0;	
	for(int i=0; i<model_size; ++i){
		c_temp += model_ita[i] * c_(i);
	}
    //更新模型概率
	for(int i=0; i<model_size; ++i){
		model_pro_(i) = (model_ita[i]*c_(i))/c_temp;  //概率
		if(model_pro_(i)<1e-4) model_pro_(i) = 1e-4;
	}


	// filewrite<<Z[0](0)<<" "<<Z[0](1)<<" ";
	MixProbability();

	// std::cout<<"########## Model probability ##########\n "<<model_pro_<<std::endl;
	// std::cout<<"############################## IMM UpdateProbability END #############################"<<std::endl;
}

//模型以及概率更新
void IMM_UKF::UpdateProbability(Eigen::VectorXd& Z){
	// std::cout<<"#################### IMM UpdateProbability #############################"<<std::endl;

	std::vector<float> model_ita(model_size);

	for(int i=0; i<model_size; ++i){

		imm_ukf_[i].Update(Z);

		Eigen::VectorXd Zminus = imm_ukf_[i].Get_Zminus(); //残差？
		Eigen::MatrixXd S = imm_ukf_[i].Get_S();  //协方差 
        //权重系数，表示在第 i 个模型下，测量值 Z 对系统状态的贡献  标准正态分布的概率密度函数
		float ita = 1/(sqrt(2*pi_)*sqrt(fabs(S.determinant())))*exp(-0.5* Zminus.transpose() * S.inverse() * Zminus);
		model_ita[i] = ita;  //概率
	}

	float c_temp = 0;
	for(int i=0; i<model_size; ++i){
		c_temp += model_ita[i] * c_(i);  //权重
	}
    //加权得后验概率
	for(int i=0; i<model_size; ++i){
		model_pro_(i) = (model_ita[i]*c_(i))/c_temp; //后验概率
		if(model_pro_(i)<1e-4) model_pro_(i) = 1e-4;
	}

	// filewrite<<Z(0)<<" "<<Z(1)<<" ";
	MixProbability();

	// std::cout<<"########## Model probability ##########\n "<<model_pro_<<std::endl;
	// std::cout<<"############################## IMM UpdateProbability END #############################"<<std::endl;
}
//输出交互 根据后验概率加权
void IMM_UKF::MixProbability(){
	// std::cout<<"########## IMM MixProbability #########"<<std::endl;
	x_merge_.fill(0.0);
	p_merge_.fill(0.0);
	for(int i=0; i<model_size; ++i){
		model_X_[i] = imm_ukf_[i].Get_state();//当前时刻更新结果
		model_P_[i] = imm_ukf_[i].Get_covariance();
		// std::cout<<" model_X_ \n"<<model_X_[i]<<"\n"<<std::endl;
		x_merge_ += model_X_[i] * model_pro_(i);
	}
	// std::cout<<"########## final merge X ##########\n"<< x_merge_<<std::endl;
	for(int i=0; i<model_size; ++i){
		p_merge_ += model_pro_(i) * (model_P_[i] + (model_X_[i] -x_merge_)* (model_X_[i] -x_merge_).transpose());
	}
	// filewrite<<track_id_<<" "<<x_merge_(0)<<" "<<x_merge_(1)<<" "<<x_merge_(2)<<" "<<x_merge_(3)<<" "<<model_pro_(0)<<" "<<model_pro_(1)<<" "<<model_pro_(2)<<"\n";
	// std::cout<<"final merge P\n"<< p_merge_<<std::endl;
}


void IMM_UKF::Process(std::vector<Eigen::VectorXd>& Z, const Eigen::VectorXd& beta, const float& last_beta, float& time){
	// std::cout<<"#################### IMM Process ####################"<<std::endl;
	if(!IsInitialized()){
		IMM_Initialization(Z[0], time,0,0);
	}else{
		PredictionZmerge(time);
		UpdateProbability(Z, beta, last_beta);
	}
}


Eigen::VectorXd IMM_UKF::getMixState(){
	return x_merge_;
}

Eigen::MatrixXd IMM_UKF::getMixCovariance(){

	return p_merge_;
}

Eigen::MatrixXd IMM_UKF::GetS(){
	return S_merge_;
}

Eigen::VectorXd IMM_UKF::GetZpre(){
	return Zpre_merge_;
}
