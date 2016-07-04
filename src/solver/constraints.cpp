#include "expressiongraph_wbf/solver/constraints.hpp"


using namespace wbf;
using namespace std;
using namespace KDL;

namespace wbf {


constraints::constraints	(
		const std::vector<int>& _joint_indexes_for_output,
		const std::vector<int>& _joint_indexes_input_scalar,
		const std::vector<int>& _joint_indexes_input_rotation,
		const int _time_index){
	joint_indexes_for_output=     _joint_indexes_for_output;
	joint_indexes_input_scalar=   _joint_indexes_input_scalar;
	joint_indexes_input_rotation= _joint_indexes_input_rotation;
	timeIndex=_time_index;
	prepared=false;

}
constraints::constraints	(
		const std::vector<int>& _joint_indexes,
		const int _time_index){
	joint_indexes_for_output=_joint_indexes;
	joint_indexes_input_scalar=_joint_indexes;
	timeIndex=_time_index;
	prepared=false;


}
bool constraints::addConstraint(const string &name, const constraint &c){
	constraint::Ptr cp(new constraint(c));
	return addConstraint(name,cp);
}
bool constraints::addConstraint(const string &name, const constraint::Ptr &c){
	//step one, check validity
	if(!check_constraint_validity(c)) return false;
	if (c->priority_level<0) throw negativePriorityException();

	//step two, check that the constraint name is not present
	std::vector< c_map_type>::iterator vIt;

	for (vIt=constrainMapPerPriority.begin();vIt!=constrainMapPerPriority.end();vIt++){

		if (vIt->find(name)!=vIt->end()){
			return false;
		}
	}

	// step three, retrieve the level and check that the vector
	// of maps is long enough
	if (c->priority_level >= constrainMapPerPriority.size())
		constrainMapPerPriority.resize(c->priority_level+1);

	constrainMapPerPriority[c->priority_level][name]=c;
	prepared=false;
	return true;

}
bool constraints::RemoveConstraint(const string &name){
	std::vector< c_map_type>::iterator vIt;
	for (vIt=constrainMapPerPriority.begin();vIt!=constrainMapPerPriority.end();vIt++){
		c_map_type::iterator it=vIt->find(name);
		if (it!=vIt->end())
		{
			vIt->erase(it);
			prepared=false;
			return true;
		}

	}
	return false;
}

void constraints::Prepare(){

	prepared=false;
	//step 1: check some problems with index for inputs.
	//this function will throw an exception
	checkJointIndex();

	n_of_joints=joint_indexes_for_output.size();
	J1.resize(1,n_of_joints);
	J3.resize(3,n_of_joints);
	J6.resize(6,n_of_joints);
	y1l.resize(1);
	y3l.resize(3);
	y6l.resize(6);
	y1u.resize(1);
	y3u.resize(3);
	y6u.resize(6);

	unsigned int numberPriority=constrainMapPerPriority.size();
	constraintsPerPriority.resize(numberPriority);
	std::fill(constraintsPerPriority.begin(), constraintsPerPriority.end(), 0);

	std::vector< c_map_type>::iterator vIt;

	//compute the number of constaints for each level
	for (unsigned int currentPriority=0;currentPriority<numberPriority;currentPriority++){
		for (c_map_type::iterator it=constrainMapPerPriority[currentPriority].begin();
			 it!=constrainMapPerPriority[currentPriority].end();
			 it++){
			constraintsPerPriority[currentPriority]+=it->second->space->output_size();
		}

	}
	//resize the internal storage for jacobian and bounds
	JacobianPerPriority.resize(numberPriority);
	lowerBoundPerPriority.resize(numberPriority);
	upperBoundPerPriority.resize(numberPriority);
	WyPerPriority.resize(numberPriority);
	for (int i=0;i<numberPriority;i++ )
	{
		JacobianPerPriority[i].resize(constraintsPerPriority[i],n_of_joints);
		lowerBoundPerPriority[i].resize(constraintsPerPriority[i]);
		upperBoundPerPriority[i].resize(constraintsPerPriority[i]);
		WyPerPriority[i].resize(constraintsPerPriority[i]);
	}
	prepared=true;
}

int constraints::computeJacobianAndBounds(
		const std::vector<double> &q_in,
		const std::vector<Rotation> &R_in,
		double time,
		bool time_present)
{

	if(!prepared) return -14;
	if (R_in.size()!=joint_indexes_input_rotation.size()) return -15;
	if (q_in.size()!=joint_indexes_input_scalar.size()) return -16;
	if (!prepared) return -11;
	if ((q_in.size()+R_in.size()*3)!=n_of_joints) return -12;

	std::vector< c_map_type>::iterator vIt;
	for (vIt=constrainMapPerPriority.begin();vIt!=constrainMapPerPriority.end();vIt++)
	{
		int i=0;
		int currentPriority=vIt - constrainMapPerPriority.begin();

		c_map_type::iterator it;
		double w;
		for (it=vIt->begin();it!=vIt->end();it++)
		{
			it->second->ctrl->update_expressions(q_in,joint_indexes_input_scalar);
			it->second->ctrl_lower->update_expressions(q_in,joint_indexes_input_scalar);
			it->second->space->update_expressions(q_in,joint_indexes_input_scalar);
			it->second->weight->setInputValues(joint_indexes_input_scalar,q_in);

			it->second->ctrl->update_expressions_rot(R_in,joint_indexes_input_rotation);
			it->second->ctrl_lower->update_expressions_rot(R_in,joint_indexes_input_rotation);
			it->second->space->update_expressions_rot(R_in,joint_indexes_input_rotation);
			it->second->weight->setInputValues(joint_indexes_input_rotation,R_in);

			if(timeIndex<0&&time_present)
				return -9;

			if(time_present)
			{
				it->second->ctrl->update_time(time,timeIndex);
				it->second->ctrl_lower->update_time(time,timeIndex);
				it->second->weight->setInputValue(timeIndex,time);
			}

		}

		for (it=vIt->begin();it!=vIt->end();it++)
		{
			w=it->second->weight->value();
			switch (it->second->ctrl->output_size()) {
			case 1:
				if(!it->second->space->compute_jacobian(J1,joint_indexes_for_output))
					return -14;
				if(!it->second->ctrl->compute_action(y1u))
					return -15;
				if(!it->second->ctrl_lower->compute_action(y1l))
					return -16;
				JacobianPerPriority[currentPriority].row(i)=J1;
				lowerBoundPerPriority[currentPriority](i)=y1l(0);
				upperBoundPerPriority[currentPriority](i)=y1u(0);
				WyPerPriority[currentPriority](i)=w;
				i++;
				break;
			case 3:
				if(!it->second->space->compute_jacobian(J3,joint_indexes_for_output))
					return -34;
				if(!it->second->ctrl->compute_action(y3u))
					return -35;
				if(!it->second->ctrl_lower->compute_action(y3l))
					return -36;

				JacobianPerPriority[currentPriority].block(0,i,3,n_of_joints)=J3;
				lowerBoundPerPriority[currentPriority].block(i,0,3,1)=y3l;
				upperBoundPerPriority[currentPriority].block(i,0,3,1)=y3u;
				WyPerPriority[currentPriority](i)=w;
				WyPerPriority[currentPriority](i+1)=w;
				WyPerPriority[currentPriority](i+2)=w;
				i=i+3;
				break;
				/*case 6:
				if(!it->second->space->compute_jacobian(J6,joint_indexes_for_output))
					return -64;
				if(!it->second->ctrl->compute_action(ydotlb6))
					return -65;
				if(!it->second->ctrl_lower->compute_action(ydotub6))
					return -66;
				J.block(0,i,6,n_of_joints)=J6;
				lbA.block(i,0,6,1)=ydotlb6;
				ubA.block(i,0,6,1)=ydotub6;
				i=i+6;
				break;*/
			default:
				return -100;//size of output not yet implemented
				break;
			}
		}
	}
	return 0;
}


void constraints::setJointIndex(
		const std::vector<int>& _joint_indexes_for_output,
		const std::vector<int>& _joint_indexes_input_scalar,
		const std::vector<int>& _joint_indexes_input_rotation){
	joint_indexes_for_output=     _joint_indexes_for_output;
	joint_indexes_input_scalar=   _joint_indexes_input_scalar;
	joint_indexes_input_rotation= _joint_indexes_input_rotation;
}
void constraints::setJointIndex(const std::vector<int> &_joint_indexes){
	joint_indexes_for_output=_joint_indexes;
	joint_indexes_input_scalar=_joint_indexes;
}
void constraints::setQweights(const Eigen::VectorXd &_Wqdiag)
{
	Wqdiag=_Wqdiag; prepared=false;
}


void constraints::setTimeIndex(const int _timeIndex){
	timeIndex=_timeIndex;
}
void constraints::checkJointIndex(){
	bool someIndexEqualToTimeIndex=false;
	for (unsigned int i=0;i<joint_indexes_for_output.size();i++)
		if(joint_indexes_for_output[i]==timeIndex  )
			someIndexEqualToTimeIndex=true;
	for (unsigned int i=0;i<joint_indexes_input_scalar.size();i++)
		if(joint_indexes_input_scalar[i]==timeIndex  )
			someIndexEqualToTimeIndex=true;
	for (unsigned int i=0;i<joint_indexes_input_rotation.size();i++)
		if(joint_indexes_input_rotation[i]==timeIndex  )
			someIndexEqualToTimeIndex=true;

	bool someIndexIsNegative=false;
	for (unsigned int i=0;i<joint_indexes_for_output.size();i++)
		if( joint_indexes_for_output[i]<0)
			someIndexIsNegative=true;
	for (unsigned int i=0;i<joint_indexes_input_scalar.size();i++)
		if( joint_indexes_input_scalar[i]<0)
			someIndexIsNegative=true;
	for (unsigned int i=0;i<joint_indexes_input_rotation.size();i++)
		if( joint_indexes_input_rotation[i]<0)
			someIndexIsNegative=true;
	if (someIndexIsNegative) throw someIndexIsNegativeException();

	if(joint_indexes_for_output.size()!=
	   joint_indexes_input_rotation.size()+joint_indexes_input_scalar.size())
		throw JointIndexSizeException();

	if (Wqdiag.size()!=0 ){
		if( Wqdiag.size()!=joint_indexes_for_output.size())
			throw 	jointWeightException();
		if( Wqdiag.minCoeff() < 0.0)
			throw 	jointWeightException();}
	else
		Wqdiag=Eigen::VectorXd::Constant(joint_indexes_for_output.size(),1.0);
}


}//END NAMESPACE
