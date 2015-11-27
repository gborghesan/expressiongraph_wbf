#include "expressiongraph_wbf/solver/simple_force_solver.hpp"


using namespace wbf;
using namespace std;
using namespace KDL;

namespace wbf {
simple_force_solver::simple_force_solver(
		const std::vector<int>& _joint_indexes,
		int const _time_index){

	joint_indexes_for_output=_joint_indexes;
	joint_indexes_input_scalar=_joint_indexes;
	time_index=_time_index;
	prepared=false;
	lambda1.resize(1);
	lambda3.resize(3);
	lambda6.resize(6);
	n_of_joints=-1;n_of_output=-1;
};
simple_force_solver::simple_force_solver(
		std::vector<int> _joint_indexes_for_output,
		std::vector<int> _joint_indexes_input_scalar,
		std::vector<int> _joint_indexes_input_rotation,
		const int _time_index)
{
	joint_indexes_for_output=_joint_indexes_for_output;
	joint_indexes_input_scalar=_joint_indexes_input_scalar;
	joint_indexes_input_rotation=_joint_indexes_input_rotation;
	time_index=_time_index;
	prepared=false;
	lambda1.resize(1);
	lambda3.resize(3);
	lambda6.resize(6);
	n_of_joints=-1;n_of_output=-1;
}
simple_force_solver::simple_force_solver(){
	prepared=false;
	lambda1.resize(1);
	lambda3.resize(3);
	lambda6.resize(6);
	n_of_joints=-1;n_of_output=-1;time_index=-1;
};
void simple_force_solver::setJointIndex(const std::vector<int>&_joint_indexes){
	joint_indexes_for_output=_joint_indexes;
	joint_indexes_input_scalar=_joint_indexes;
};
void simple_force_solver::setJointIndex(const std::vector<int>&indx_out,
		const std::vector<int>&indx_scalar,
		const std::vector<int>&indx_rot){
	joint_indexes_for_output=indx_out;
	joint_indexes_input_scalar=indx_scalar;
	joint_indexes_input_rotation=indx_rot;
}

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
	for (unsigned int i=0;i<joint_indexes_for_output.size();i++)
		if(joint_indexes_for_output[i]==time_index || joint_indexes_for_output[i]<0)
			return -1;//index problem
	c_map_type::iterator it;
	n_of_output=0;
	n_of_joints=joint_indexes_for_output.size();
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


int simple_force_solver::Compute(
		const std::vector<double> &q_in,
		const std::vector<KDL::Rotation> &R_in,
		double time,
		Eigen::VectorXd &tau_out,
		bool time_present)
{

	if (R_in.size()!=joint_indexes_input_rotation.size()) return -15;
	if (q_in.size()!=joint_indexes_input_scalar.size()) return -16;
	if (!prepared) return -11;
	if ((q_in.size()+R_in.size()*3)!=n_of_joints) return -12;
	if (tau_out.size()!=n_of_joints) return -13;

	int i=0;
	c_map_type::iterator it;
	for (it=c_map.begin();it!=c_map.end();it++)
	{

		it->second->ctrl->update_expressions(q_in,joint_indexes_input_scalar);
		it->second->space->update_expressions(q_in,joint_indexes_input_scalar);
		it->second->ctrl->update_expressions_rot(R_in,joint_indexes_input_rotation);
		it->second->space->update_expressions_rot(R_in,joint_indexes_input_rotation);


		if(time_index>-1&&time_present)
			it->second->ctrl->update_time(time,time_index);

		switch (it->second->ctrl->output_size()) {
		case 1:
			if(!it->second->space->compute_jacobian(J1,joint_indexes_for_output))
				return -14;
			if(!it->second->ctrl->compute_action(lambda1))
				return -15;
			J.row(i)=J1;
			lambda_des(i)=lambda1(0);
			i++;
			break;
		case 3:
			if(!it->second->space->compute_jacobian(J3,joint_indexes_for_output))
					return -34;
				if(!it->second->ctrl->compute_action(lambda3))
					return -35;

		//		cout<<"J before\n"<<J<<endl;
		//		cout<<"J3 \n"<<J3<<endl;
				J.block(0,i,3,n_of_joints)=J3;
		//		cout<<"J after\n"<<J<<endl;
		//		cout<<"lambda_des before\n"<<lambda_des.transpose()<<endl;
		//		cout<<"lambda3 \n"<<lambda3.transpose()<<endl;
				lambda_des.block(i,0,3,1)=lambda3;
		//		cout<<"lambda_des after\n"<<lambda_des.transpose()	<<endl;
				i=i+3;
			break;
			if(!it->second->space->compute_jacobian(J6,joint_indexes_for_output))
					return -64;
				if(!it->second->ctrl->compute_action(lambda6))
					return -65;

				J.block(0,i,6,n_of_joints)=J6;
				lambda_des.block(i,0,6,1)=lambda6;
				i=i+6;
				break;
		default:
			return -100;//size of output not yet implemented
			break;
		}
		//cout<<"J total\n"<<J<<endl;
		//cout<<"lambda_des total\n"<<lambda_des.transpose()<<endl;
		Jt=J.transpose();
		tau_out=Jt*lambda_des;


	}

	return 1;
};
int simple_force_solver::Compute(const std::vector<double> &q_in, double time,
		Eigen::VectorXd &tau_out)
{
	std::vector<KDL::Rotation> R_in;
	return  Compute(q_in,R_in, time,tau_out,true);
};
int simple_force_solver::Compute(const std::vector<double> &q_in,
		Eigen::VectorXd &tau_out)
{
	std::vector<KDL::Rotation> R_in;
	return  Compute(q_in,R_in, 0.0,tau_out,false);
};

int simple_force_solver::Compute(const std::vector<double> &q_in,
		const std::vector<KDL::Rotation> &R_in,
		double time,
		Eigen::VectorXd &tau_out)
{
	return  Compute(q_in,R_in, time,tau_out,true);
};
int simple_force_solver::Compute(const std::vector<double> &q_in,
		const std::vector<KDL::Rotation> &R_in,
		Eigen::VectorXd &tau_out)
{
	return  Compute(q_in,R_in, 0.0,tau_out,false);
};
}
