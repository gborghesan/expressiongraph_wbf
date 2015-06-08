#include "expressiongraph_wbf/solver/simple_force_solver.hpp"


using namespace wbf;
using namespace std;
using namespace KDL;

namespace wbf {
simple_force_solver::simple_force_solver(
		const std::vector<int>& _joint_indexes,
		int const _time_index){

	joint_indexes=_joint_indexes;
	time_index=_time_index;
	prepared=false;
	lambda1.resize(1);
	lambda3.resize(3);
	lambda6.resize(6);
	n_of_joints=-1;n_of_output=-1;
};
simple_force_solver::simple_force_solver(){
	prepared=false;
	lambda1.resize(1);
	lambda3.resize(3);
	lambda6.resize(6);
	n_of_joints=-1;n_of_output=-1;time_index=-1;
};
void simple_force_solver::setJointIndex(const std::vector<int>&_joint_indexes){
	joint_indexes=_joint_indexes;
};

void simple_force_solver::setTimeIndex(const int _time_index){time_index=_time_index;
};
bool simple_force_solver::addConstraint(const std::string& name,
		const constraint::Ptr &c){
	if (c_map.find(name)==c_map.end())
	{
		c_map[name]=c;
		prepared=false;
		return true;
	}
	return false;
};
bool simple_force_solver::addConstraint(const std::string& name,const constraint &c){
	constraint::Ptr cp(new constraint(c));
	return addConstraint(name,cp);
};
bool simple_force_solver::RemoveConstraint(const std::string &name){
	c_map_type::iterator it=c_map.find(name);
	if (it!=c_map.end())
	{
		c_map.erase(it);
		prepared=false;
		return true;
	}
	return false;
};
int simple_force_solver::Prepare(){
	prepared=false;
	for (unsigned int i=0;i<joint_indexes.size();i++)
		if(joint_indexes[i]==time_index || joint_indexes[i]<0)
			return -1;//index problem
	c_map_type::iterator it;
	n_of_output=0;
	n_of_joints=joint_indexes.size();
	for (it=c_map.begin();it!=c_map.end();it++)
	{
		bool ok=check_constraint_validity(it->second);
		if(!ok) return -2;//mismatch size of controller/size of space
		n_of_output+=it->second->ctrl->output_size();
	}
	J.resize(n_of_output,n_of_joints);
	J1.resize(1,n_of_joints);
	J3.resize(3,n_of_joints);
	J6.resize(6,n_of_joints);
	Jt.resize(n_of_joints,n_of_output);
	lambda_des.resize(n_of_output);
	prepared=true;
	return 1;//ok
};
int simple_force_solver::Compute(const std::vector<double> &q_in, double time,
		Eigen::VectorXd &tau_out,bool time_present){

	if (!prepared) return -11;
	if (q_in.size()!=n_of_joints) return -12;
	if (tau_out.size()!=n_of_joints) return -13;

	int i=0;
	c_map_type::iterator it;
	for (it=c_map.begin();it!=c_map.end();it++)
	{
		it->second->ctrl->update_expressions(q_in,joint_indexes);
		it->second->space->update_expressions(q_in,joint_indexes);
		if(time_index>-1&&time_present)
			it->second->ctrl->update_time(time,time_index);

		switch (it->second->ctrl->output_size()) {
		case 1:
			if(!it->second->space->compute_jacobian(J1,joint_indexes))
				return -14;
			if(!it->second->ctrl->compute_action(lambda1))
				return -15;
			J.row(i)=J1;
			lambda_des(i)=lambda1(0);
			i++;
			break;
		case 3:
			if(!it->second->space->compute_jacobian(J3,joint_indexes))
					return -34;
				if(!it->second->ctrl->compute_action(lambda3))
					return -35;
				J.row(i)=J3.row(0);
				J.row(i+1)=J3.row(1);
				J.row(i+2)=J3.row(2);
				lambda_des(i)=lambda3(0);
				lambda_des(i+1)=lambda3(1);
				lambda_des(i+2)=lambda3(2);
				i=i+3;
			break;
			if(!it->second->space->compute_jacobian(J6,joint_indexes))
					return -64;
				if(!it->second->ctrl->compute_action(lambda6))
					return -65;
				J.row(i)=J6.row(0);
				J.row(i+1)=J6.row(1);
				J.row(i+2)=J6.row(2);
				J.row(i+3)=J6.row(3);
				J.row(i+4)=J6.row(4);
				J.row(i+5)=J6.row(5);
				lambda_des(i)=lambda6(0);
				lambda_des(i+1)=lambda6(1);
				lambda_des(i+2)=lambda6(2);
				lambda_des(i+3)=lambda6(3);
				lambda_des(i+4)=lambda6(4);
				lambda_des(i+5)=lambda6(5);
				i=i+6;
				break;
		default:
			return -100;//size of output not yet implemented
			break;
		}
		Jt=J.transpose();
		tau_out=Jt*lambda_des;


	}

	return 1;
};
int simple_force_solver::Compute(const std::vector<double> &q_in, double time,
		Eigen::VectorXd &tau_out)
{
	return  Compute(q_in, time,tau_out,true);
};
int simple_force_solver::Compute(const std::vector<double> &q_in,
		Eigen::VectorXd &tau_out)
{
	return  Compute(q_in, 0.0,tau_out,false);
};

}
