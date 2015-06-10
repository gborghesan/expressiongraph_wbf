#include "expressiongraph_wbf/solver/velocity_solver.hpp"


using namespace wbf;
using namespace std;
using namespace KDL;

namespace wbf {
velocity_solver::velocity_solver(
		const std::vector<int>& _joint_indexes,
					const int _time_index,
					double _max_cpu_time,
					double _regularization_factor,
					int _nWSR){

	joint_indexes=_joint_indexes;
	time_index=_time_index;
	prepared=false;
	ydotlb1.resize(1);	ydotlb3.resize(3);	ydotlb6.resize(6);
	ydotub1.resize(1);	ydotub3.resize(3);	ydotub6.resize(6);
	n_of_joints=-1;n_of_output=-1;
	n_of_slack=-1;
	n_of_variables=-1;
	firsttime=true;
	regularization_factor=_regularization_factor;
	nWSR  =_nWSR  ;
	cputime =_max_cpu_time;



};
velocity_solver::velocity_solver(){
	prepared=false;
	firsttime=true;
	ydotlb1.resize(1);	ydotlb3.resize(3);	ydotlb6.resize(6);
	ydotub1.resize(1);	ydotub3.resize(3);	ydotub6.resize(6);
	n_of_joints=-1;n_of_output=-1;time_index=-1;
	n_of_slack=-1;
	n_of_variables=-1;

	nWSR  =10  ;
	cputime =0.01;
	regularization_factor=0.001;

};

void velocity_solver::setJointIndex(const std::vector<int>&_joint_indexes){
	joint_indexes=_joint_indexes; prepared=false;
};

void velocity_solver::setTimeIndex(const int _time_index){time_index=_time_index;
};
bool velocity_solver::addConstraint(const std::string& name,
		const constraint::Ptr &c){
	if (c_map.find(name)==c_map.end())
	{
		c_map[name]=c;
		prepared=false;
		return true;
	}
	return false;
};
bool velocity_solver::addConstraint(const std::string& name,const constraint &c){
	constraint::Ptr cp(new constraint(c));
	return addConstraint(name,cp);
};
bool velocity_solver::RemoveConstraint(const std::string &name){
	c_map_type::iterator it=c_map.find(name);
	if (it!=c_map.end())
	{
		c_map.erase(it);
		prepared=false;
		return true;
	}
	return false;
};
int velocity_solver::Prepare(){
	prepared=false;



	for (unsigned int i=0;i<joint_indexes.size();i++)
		if(joint_indexes[i]==time_index || joint_indexes[i]<0)
			return -1;//index problem


	c_map_type::iterator it;
	n_of_output=0;
	n_of_slack=0;
	n_of_joints=joint_indexes.size();

	if(Wqdiag.size()!=n_of_joints)
		//in case is not initialized..
		Wqdiag=Eigen::VectorXd::Ones(n_of_joints);

	for (it=c_map.begin();it!=c_map.end();it++)
	{
		bool ok=check_constraint_validity(it->second);
		if(!ok) return -2;//mismatch size of controller/size of space
		n_of_output+=it->second->ctrl->output_size();
		if(it->second->priority_level==2)
			n_of_slack+=it->second->ctrl->output_size();
	}

	n_of_variables=n_of_slack+n_of_joints;
	//J.resize(n_of_output,n_of_joints);
	J1.resize(1,n_of_joints);
	J3.resize(3,n_of_joints);
	J6.resize(6,n_of_joints);
	J.resize(n_of_output,n_of_joints);




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

	//J slack can be computed now, as it is not going to change during execution
	//J Slack has a one when the constraint is 2nd priority.
	unsigned int i_y=0, i_s=0;
	for (it=c_map.begin();it!=c_map.end();it++)
	{
		if(i_s>=n_of_slack)
			cout<<"ERROR: I_S is:"<<i_s<<" n_of_slack is:"<<n_of_slack<<endl;
		if(i_y>=n_of_slack)
			cout<<"ERROR: I_Y is:"<<i_y<<" n_of_output is:"<<n_of_output<<endl;

		unsigned int size=it->second->ctrl->output_size();
		if(it->second->priority_level==2)
		{
			//add an I matrix of size "size" starting from (i_s,i_y)
			J_slack.block(i_y,i_s,size,size)
							=Eigen::MatrixXd::Identity(size,size);
			i_s+=size;
		}
		i_y+=size;
	}

	//in H we can already put the joint weights
	//

	H.block(0,0,n_of_joints,n_of_joints)=Wqdiag.asDiagonal()*regularization_factor;


	//Jt.resize(n_of_joints,n_of_output);
	//lambda_des.resize(n_of_output);
	QP  = qpOASES::SQProblem(n_of_variables,n_of_output,qpOASES::HST_POSDEF);
	QP.setPrintLevel(qpOASES::PL_NONE);
	prepared=true;
	firsttime=true;
	return 1;//ok
};
void velocity_solver::setQweights(const Eigen::VectorXd& _Wqdiag)
{	Wqdiag=_Wqdiag; prepared=false;}

int velocity_solver::Compute(const std::vector<double> &q_in, double time,
		Eigen::VectorXd &qdot_out,bool time_present){

	if (!prepared) return -11;
	if (q_in.size()!=n_of_joints) return -12;
	if (qdot_out.size()!=n_of_joints) return -13;

	int i=0;
	c_map_type::iterator it;
	for (it=c_map.begin();it!=c_map.end();it++)
	{
		it->second->ctrl->update_expressions(q_in,joint_indexes);
		it->second->ctrl_sec->update_expressions(q_in,joint_indexes);
		it->second->space->update_expressions(q_in,joint_indexes);

		if(time_index>-1&&time_present)
		{
			it->second->ctrl->update_time(time,time_index);
			it->second->ctrl_sec->update_time(time,time_index);
		}
		switch (it->second->ctrl->output_size()) {
		case 1:
			if(!it->second->space->compute_jacobian(J1,joint_indexes))
				return -14;
			if(!it->second->ctrl->compute_action(ydotlb1))
				return -15;
			if(!it->second->ctrl_sec->compute_action(ydotub1))
				return -16;
			J.row(i)=J1;
			lbA(i)=ydotlb1(0);
			ubA(i)=ydotub1(0);
			i++;
			break;
		case 3:
			if(!it->second->space->compute_jacobian(J3,joint_indexes))
				return -34;
			if(!it->second->ctrl->compute_action(ydotlb3))
				return -35;
			if(!it->second->ctrl_sec->compute_action(ydotub3))
				return -36;

			J.block(0,i,3,n_of_joints)=J3;
			lbA.block(i,0,3,1)=ydotlb3;
			ubA.block(i,0,3,1)=ydotub3;
			i=i+3;
			break;
		case 6:
			if(!it->second->space->compute_jacobian(J6,joint_indexes))
				return -64;
			if(!it->second->ctrl->compute_action(ydotlb6))
				return -65;
			if(!it->second->ctrl_sec->compute_action(ydotub6))
				return -66;
			J.block(0,i,6,n_of_joints)=J6;
			lbA.block(i,0,6,1)=ydotlb6;
			ubA.block(i,0,6,1)=ydotub6;
			i=i+6;
			break;
		default:
			return -100;//size of output not yet implemented
			break;
		}
	}
	//fill hessian with constraint space weights value
	unsigned int  i_s=n_of_joints;
	for (it=c_map.begin();it!=c_map.end();it++)
		if(it->second->priority_level==2)
		{
			;
			it->second->weight->setInputValue(time_index,time);
			double w=it->second->weight->value();

			unsigned int size=it->second->ctrl->output_size();
			//add an I matrix of size "size" starting from (i_y,i_y)

			H.block(i_s,i_s,size,size)=Eigen::MatrixXd::Identity(size,size)*w;
			i_s+=size;
		}




	//compose A=[J|J_slack]
	//	cout<<"J\n"<<J<<endl;
	//	cout<<"J_slack\n"<<J_slack<<endl;
	//	cout<<"A init\n"<<A<<endl;
	A.block(0,0,n_of_output,n_of_joints)=J;
	A.block(0,n_of_joints,n_of_output,n_of_slack)=J_slack;
	//	cout<<"A=[J|J_slack]\n"<<A<<endl;
	//	cout<<"lbA\n"<<lbA<<endl;
	//	cout<<"ubA\n"<<ubA<<endl;
	//	cout<<"H\n"<<H<<endl;
	//	cout<<"nWSR\n"<<nWSR<<endl;
	//	cout<<"cputime\n"<<cputime<<endl;
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



	return 1;
};

int velocity_solver::Compute(const std::vector<double> &q_in, double time,
		Eigen::VectorXd &tau_out)
{
	return  Compute(q_in, time,tau_out,true);
};
int velocity_solver::Compute(const std::vector<double> &q_in,
		Eigen::VectorXd &tau_out)
{
	return  Compute(q_in, 0.0,tau_out,false);
};

}
