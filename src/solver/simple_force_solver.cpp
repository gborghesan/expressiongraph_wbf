#include "expressiongraph_wbf/solver/simple_force_solver.hpp"


using namespace wbf;
using namespace std;
using namespace KDL;

namespace wbf {

int simple_force_solver::Prepare(){
	prepared=false;

	cnstr->Prepare();
	n_of_output=0;
	n_of_joints=0;

	if(!cnstr->getPriorityCardinality(constraintsPerPriority)) return -1;
	for (int i=0;i<constraintsPerPriority.size();i++){
		cout<<"START1constraintsPerPriority["<<i<<"]: "<<constraintsPerPriority[i]<<endl;
	}

	if(!cnstr->getQweights(Wq)) return -1;
	JPerPriority.resize(constraintsPerPriority.size());
	LBPerPriority.resize(constraintsPerPriority.size());
	constraintsPerPriorityCheck.resize(constraintsPerPriority.size());
	for (int i=0;i<constraintsPerPriority.size();i++){
		if(constraintsPerPriority[i]!=0){
			n_of_output+=constraintsPerPriority[i];
			JPerPriority[i].resize(constraintsPerPriority[i],n_of_joints);
			LBPerPriority[i].resize(constraintsPerPriority[i]);
		}
	}
	n_of_joints=Wq.size();
	J.resize(n_of_output,n_of_joints);
	Jt.resize(n_of_joints,n_of_output);
	lambda_des.resize(n_of_output);
	for (int i=0;i<constraintsPerPriority.size();i++){
		cout<<"END1constraintsPerPriority["<<i<<"]: "<<constraintsPerPriority[i]<<endl;
	}

	prepared=true;
	return 1;//ok
}


int simple_force_solver::Compute(
		const std::vector<double> &q_in,
		const std::vector<KDL::Rotation> &R_in,
		double time,
		Eigen::VectorXd &tau_out,
		bool time_present)
{
	if (!prepared)
	{
		cout<<"not prepared"<<endl;
		return -1;
	}
	if (tau_out.size()!=n_of_joints) return -13;
	if (!cnstr->getPriorityCardinality(constraintsPerPriorityCheck)) return -1;
	if (constraintsPerPriorityCheck!=constraintsPerPriority)
		return -14;
	int ret;
	for (int i=0;i<constraintsPerPriority.size();i++){
		cout<<"BEFOREconstraintsPerPriority["<<i<<"]: "<<constraintsPerPriority[i]<<endl;
	}
		ret=cnstr->computeJacobianAndBounds(q_in,R_in,time,time_present);
	if (ret!=0) return ret;
	cnstr->getJacobian(JPerPriority);
	cnstr->getLowerBounds(LBPerPriority);
	cnstr->getYweights(WyPerPriority);

	int starting_index=0;
	for (int i=0;i<constraintsPerPriority.size();i++){
		cout<<"constraintsPerPriority["<<i<<"]: "<<constraintsPerPriority[i]<<endl;
		int nCostraint=constraintsPerPriority[i];
		if(nCostraint!=0){
			J.block(0,starting_index,nCostraint,n_of_joints)=JPerPriority[i];
			lambda_des.block(0,starting_index,nCostraint,1)=LBPerPriority[i].cwiseProduct(WyPerPriority[i]);
			starting_index+=nCostraint;
		}
	}

	cout<<"J total\n"<<J<<endl;
	cout<<"lambda_des total\n"<<lambda_des.transpose()<<endl;
	Jt=J.transpose();
	tau_out=Jt*lambda_des;


	return 1;
}
int simple_force_solver::Compute(const std::vector<double> &q_in, double time,
		Eigen::VectorXd &tau_out)
{
	std::vector<KDL::Rotation> R_in;
	return  Compute(q_in,R_in, time,tau_out,true);
}
int simple_force_solver::Compute(const std::vector<double> &q_in,
		Eigen::VectorXd &tau_out)
{
	std::vector<KDL::Rotation> R_in;
	return  Compute(q_in,R_in, 0.0,tau_out,false);
}

int simple_force_solver::Compute(const std::vector<double> &q_in,
		const std::vector<KDL::Rotation> &R_in,
		double time,
		Eigen::VectorXd &tau_out)
{
	return  Compute(q_in,R_in, time,tau_out,true);
}
int simple_force_solver::Compute(const std::vector<double> &q_in,
		const std::vector<KDL::Rotation> &R_in,
		Eigen::VectorXd &tau_out)
{
	return  Compute(q_in,R_in, 0.0,tau_out,false);
}
}
