#include "expressiongraph_wbf/solver/space_description.hpp"
using namespace KDL;
namespace wbf {

//SCALAR SPACE
scalar_space::scalar_space(Expression<double>::Ptr _space_output)
{
	space_output=_space_output;
	type="scalar";
	size_of_output=1;
}
/*scalar_space& scalar_space::operator=(scalar_space const &s)
{
	space_output=s.space_output;
	type="scalar";
	size_of_output=1;
	type=s.type;
	return *this;
}*/
void scalar_space::update_expressions(const std::vector<double>&q,
		const std::vector<int> & q_index)
{
	space_output->setInputValues(q_index,q);
}
void scalar_space::update_expressions_rot(const std::vector<KDL::Rotation>&R,
			const std::vector<int> & q_index)
{
	space_output->setInputValues(q_index,R);
}
bool scalar_space::compute_jacobian(Eigen::MatrixXd& J_partial,
		const std::vector<int> i_of_v)
{
	if (J_partial.rows()!=size_of_output) return false;
	if (J_partial.cols()!=i_of_v.size()) return false;

	//force update by calling value;
	space_output->value();

	//computing the jacobian
	for(unsigned int i=0;i<i_of_v.size();i++)
	{
		J_partial(0,i)=space_output->derivative(i_of_v[i]);
	}
	//std::cout<<"x:\t"<<x<<"\txd:\t"<<xd<<"\tk:\t"<<k<<"\nres:\t"<<res[0]<<std::endl;
	return true;
}


//ROTATIONAL SPACE
rot_space::rot_space(Expression<Rotation>::Ptr _space_output,
		which_direction_type _which_direction,bool _base_frame)
{
	which_direction=_which_direction;
	space_output=_space_output;
	base_frame=_base_frame;
	switch (which_direction) {
	case FULL_ROTATION:
		type="Rot";
		size_of_output=3;
		break;
	case ROT_X:
		type="RotX";
		size_of_output=1;
		break;
	case ROT_Y:
		type="RotY";
		size_of_output=1;
		break;
	case ROT_Z:
		type="RotZ";
		size_of_output=1;
		break;
	default:
		type="ERROR";
	}

	if (!base_frame)
			type=type+"_own";
}

rot_space::rot_space(Expression<Rotation>::Ptr _space_output)
{
	which_direction=FULL_ROTATION;
	space_output=_space_output;
	base_frame=true;
		type="Rot";
		size_of_output=3;


};
rot_space::rot_space(Expression<Rotation>::Ptr _space_output,
		which_direction_type _which_direction)
{
	which_direction=_which_direction;
	space_output=_space_output;
	base_frame=true;
	switch (which_direction) {
	case FULL_ROTATION:
		type="Rot";
		size_of_output=3;
		break;
	case ROT_X:
		type="RotX";
		size_of_output=1;
		break;
	case ROT_Y:
		type="RotY";
		size_of_output=1;
		break;
	case ROT_Z:
		type="RotZ";
		size_of_output=1;
		break;
	default:
		type="ERROR";
	}
};
/*
rot_space& rot_space::operator=(rot_space const &s)
{
	space_output=s.space_output;
	which_direction=s.which_direction;
	size_of_output=s.size_of_output;
	base_frame=s.base_frame;
	type=s.type;
	return *this;
}
*/
void rot_space::update_expressions(const std::vector<double>&q,
		const std::vector<int> & q_index)
{
	space_output->setInputValues(q_index,q);
}
void rot_space::update_expressions_rot(const std::vector<KDL::Rotation>&R,
			const std::vector<int> & q_index)
{
	space_output->setInputValues(q_index,R);
}
bool rot_space::compute_jacobian(Eigen::MatrixXd& J_partial,
		const std::vector<int> i_of_v)
{
	if (J_partial.rows()!=size_of_output) return false;
	if (J_partial.cols()!=i_of_v.size()) return false;

	//force update by calling value;
	KDL::Rotation R=space_output->value();

	//computing the jacobian
	for(unsigned int i=0;i<i_of_v.size();i++)
	{
		KDL::Vector omega=space_output->derivative(i_of_v[i]);
		if (!base_frame){
			//apply rotation to J to express in current space
			omega=R*omega;
		}

		switch (which_direction)
		{
		case FULL_ROTATION:
			J_partial(0,i)=omega.x();
			J_partial(1,i)=omega.y();
			J_partial(2,i)=omega.z();
			break;
		case ROT_X:
			J_partial(0,i)=omega.x();
			break;
		case ROT_Y:
			J_partial(0,i)=omega.y();
			break;
		case ROT_Z:
			J_partial(0,i)=omega.z();
			break;
		}
	}
	//std::cout<<"x:\t"<<x<<"\txd:\t"<<xd<<"\tk:\t"<<k<<"\nres:\t"<<res[0]<<std::endl;
	return true;
}

//VECTOR SPACE
pos_space::pos_space(Expression<Vector>::Ptr _space_output,
		which_position_type _which_direction,
		Expression<Rotation>::Ptr _new_base)
{
	which_direction=_which_direction;
	space_output=_space_output;
	new_base=_new_base;
	switch (which_direction) {
	case FULL_POSITION:
		type="Pos";
		size_of_output=3;
		break;
	case POS_X:
		type="PosX";
		size_of_output=1;
		break;
	case POS_Y:
		type="PosY";
		size_of_output=1;
		break;
	case POS_Z:
		type="PosZ";
		size_of_output=1;
		break;
	}
	if (_new_base)
		type=type+"_own";

}
/*pos_space& pos_space::operator=(pos_space const &s)
{
	space_output=s.space_output;
	which_direction=s.which_direction;
	size_of_output=s.size_of_output;
	new_base=s.new_base;
	type=s.type;
	return *this;
}*/

void pos_space::update_expressions(const std::vector<double>&q,
		const std::vector<int> & q_index)
{
	space_output->setInputValues(q_index,q);
	if(new_base)
		new_base->setInputValues(q_index,q);
	//std::cout<<"OK"<<std::endl;
}
void pos_space::update_expressions_rot(const std::vector<KDL::Rotation>&R,
			const std::vector<int> & q_index)
{
	space_output->setInputValues(q_index,R);
	if(new_base)
		new_base->setInputValues(q_index,R);
}
bool pos_space::compute_jacobian(Eigen::MatrixXd& J_partial,
		const std::vector<int> i_of_v)
{
	if (J_partial.rows()!=size_of_output) return false;
	if (J_partial.cols()!=i_of_v.size()) return false;

	//force update by calling value;

	space_output->value();
	KDL::Rotation R;
	if(new_base)
		R=new_base->value();

	//computing the jacobian
	for(unsigned int i=0;i<i_of_v.size();i++)
	{
		KDL::Vector v_part=space_output->derivative(i_of_v[i]);

		if (new_base){
			//apply rotation to J to express in current space
			v_part=R*v_part;
		}

		switch (which_direction)
		{
		case FULL_POSITION:
			J_partial(0,i)=v_part.x();
			J_partial(1,i)=v_part.y();
			J_partial(2,i)=v_part.z();
			break;
		case POS_X:
			J_partial(0,i)=v_part.x();
			break;
		case POS_Y:
			J_partial(0,i)=v_part.y();
			break;
		case POS_Z:
			J_partial(0,i)=v_part.z();
			break;
		}
	}
	//std::cout<<"x:\t"<<x<<"\txd:\t"<<xd<<"\tk:\t"<<k<<"\nres:\t"<<res[0]<<std::endl;
	return true;
}


};
