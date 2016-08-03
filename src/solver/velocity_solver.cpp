#include "expressiongraph_wbf/solver/velocity_solver.hpp"


using namespace wbf;
using namespace std;
using namespace KDL;

namespace wbf {


void velocity_solver::Prepare(){
	prepared=false;


	cnstr->Prepare();
	n_of_output=0;
	n_of_joints=0;

	if(!cnstr->getPriorityCardinality(constraintsPerPriority) ||
	   !cnstr->getQweights(Wq))
		throw constraintsNotPreparedException();



	if(constraintsPerPriority.size()>3)
		throw constraintsNotPreparedException() ;
	if(constraintsPerPriority[0]!=0){

		for (int i=0; i<constraintsPerPriority.size();i++)
			cout<<i<<" constraintsPerPriority\t"<<constraintsPerPriority[i]<<endl;
		throw constraintsNotPreparedException() ;
	}
	JPerPriority.resize(constraintsPerPriority.size());
	LBPerPriority.resize(constraintsPerPriority.size());
	UBPerPriority.resize(constraintsPerPriority.size());
	constraintsPerPriorityCheck.resize(constraintsPerPriority.size());

	for (int i=0;i<constraintsPerPriority.size();i++){
		if(constraintsPerPriority[i]!=0){
			n_of_output+=constraintsPerPriority[i];
			JPerPriority[i].resize(constraintsPerPriority[i],n_of_joints);
			LBPerPriority[i].resize(constraintsPerPriority[i]);
			UBPerPriority[i].resize(constraintsPerPriority[i]);
		}
	}
	n_of_joints=Wq.size();
	n_of_slack=constraintsPerPriority[2];
	Wy.resize(n_of_slack);
	n_of_variables=n_of_joints+n_of_slack;


	H         = Eigen::MatrixXd::Identity(n_of_variables,n_of_variables);
	A         = Eigen::MatrixXd::Zero(n_of_output,n_of_variables);
	lb        = Eigen::VectorXd::Constant(n_of_variables,-HUGE_VALUE);
	lbA       = Eigen::VectorXd::Constant(n_of_output,-HUGE_VALUE);
	ub        = Eigen::VectorXd::Constant(n_of_variables,HUGE_VALUE);
	ubA       = Eigen::VectorXd::Constant(n_of_output,HUGE_VALUE);
	g         = Eigen::VectorXd::Zero(n_of_variables);

	solution  =Eigen::VectorXd::Zero(n_of_variables);
	J.resize(n_of_output,n_of_joints);
	J_slack   =Eigen::MatrixXd::Zero(n_of_output,n_of_slack);
	J_slack.block(0,constraintsPerPriority[1],n_of_slack,n_of_slack)
		  =Eigen::MatrixXd::Identity(n_of_slack,n_of_slack);


	//in H we can already put the joint weights
	//

	H.block(0,0,n_of_joints,n_of_joints)=Eigen::MatrixXd(Wq.asDiagonal())*regularization_factor;


	//Jt.resize(n_of_joints,n_of_output);
	//lambda_des.resize(n_of_output);
	QP  = qpOASES::SQProblem(n_of_variables,n_of_output,qpOASES::HST_POSDEF);
	QP.setPrintLevel(qpOASES::PL_NONE);
	prepared=true;
	firsttime=true;
}

bool velocity_solver::Compute(const std::vector<double> &q_in, const std::vector<Rotation> &R_in, double time,
		Eigen::VectorXd &qdot_out,bool time_present){

	if (!prepared)
	{
		cout<<"not prepared"<<endl;
		return false;
	}
	if (qdot_out.size()!=n_of_joints) return -13;

	if (!cnstr->getPriorityCardinality(constraintsPerPriorityCheck))
		throw wrongQsizeException();
	if (constraintsPerPriorityCheck!=constraintsPerPriority)
		throw constraintsNotPreparedException() ;
	if (!cnstr->computeJacobianAndBounds(q_in,R_in,time,time_present))
		throw constraintsNotPreparedException() ;
	cnstr->getJacobian(JPerPriority);
	cnstr->getLowerBounds(LBPerPriority);
	cnstr->getUpperBounds(UBPerPriority);
	cnstr->getYweights(WyPerPriority);

	if(constraintsPerPriority[1]!=0){
		J.block(0,0,constraintsPerPriority[1],n_of_joints)=JPerPriority[1];
		lbA.block(0,0,constraintsPerPriority[1],1)=LBPerPriority[1];
		ubA.block(0,0,constraintsPerPriority[1],1)=UBPerPriority[1];
	}

	if(constraintsPerPriority[2]!=0){
		J.block(constraintsPerPriority[1],0,constraintsPerPriority[2],n_of_joints)=JPerPriority[2];
		lbA.block(constraintsPerPriority[1],0,constraintsPerPriority[2],1)=LBPerPriority[2];
		ubA.block(constraintsPerPriority[1],0,constraintsPerPriority[2],1)=UBPerPriority[2];
	}
	A.block(0,0,n_of_output,n_of_joints)=J;
	A.block(0,n_of_joints,n_of_output,n_of_slack)=J_slack;

	Wy=WyPerPriority[2];
	H.block(n_of_joints,n_of_joints,n_of_slack,n_of_slack)=Eigen::MatrixXd(Wy.asDiagonal());




	//compose A=[J|J_slack]
	//	cout<<"J\n"<<J<<endl;
	//	cout<<"J_slack\n"<<J_slack<<endl;
	//	cout<<"A init\n"<<A<<endl;
	//cout<<"time_present \t"<<time_present<<endl;
	//cout<<"time \t"<<time<<endl;
	//cout<<"A=[J|J_slack]\n"<<A<<endl;
	//cout<<"lbA\n"<<lbA<<endl;
	//cout<<"ubA\n"<<ubA<<endl;
	//cout<<"Wy\n"<<Wy.transpose()<<endl;
	//cout<<"H\n"<<H<<endl;
	//cout<<"nWSR\n"<<nWSR<<endl;
	//cout<<"cputime\n"<<cputime<<endl;


	int _nWSR=nWSR;
	double _cputime=cputime;
	//call QP
	int ret;
	if (firsttime) {
		ret=QP.init (H.data(), g.data(), A.data(),
				lb.data(), ub.data(), lbA.data(), ubA.data(),
				_nWSR, & _cputime);
	//	cout<<"firstime ret\n"<<ret<<endl;
		firsttime=false;
	} else {
		ret=QP.hotstart (H.data(), g.data(), A.data(),
				lb.data(), ub.data(), lbA.data(), ubA.data(),
				_nWSR, & _cputime);
	//	cout<<"hotstart ret\n"<<ret<<endl;
	}
	if (ret!=0) return ret;

	ret = QP.getPrimalSolution(solution.data());
	//cout<<"solution res\n"<<solution.transpose()<<endl;
	qdot_out=solution.block(0,0,n_of_joints,1);



	return true;
}

bool velocity_solver::Compute(const std::vector<double> &q_in, double time,
		Eigen::VectorXd &tau_out)
{
	std::vector<Rotation> R_in;
	return  Compute(q_in,R_in, time,tau_out,true);
}
bool velocity_solver::Compute(const std::vector<double> &q_in,
		Eigen::VectorXd &tau_out)
{
	std::vector<Rotation> R_in;
	return  Compute(q_in,R_in, 0.0,tau_out,false);
}
bool velocity_solver::Compute(const std::vector<double> &q_in,
		const std::vector<Rotation> &R_in,
		double time,
		Eigen::VectorXd &tau_out)
{

	return  Compute(q_in,R_in, time,tau_out,true);
}
bool velocity_solver::Compute(const std::vector<double> &q_in,
		const std::vector<Rotation> &R_in,
		Eigen::VectorXd &tau_out)
{

	return  Compute(q_in,R_in, 0.0,tau_out,false);
}
}
