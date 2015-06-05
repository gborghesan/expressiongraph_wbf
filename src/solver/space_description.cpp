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
void scalar_space::update_expressions(const std::vector<double>&q,
		const std::vector<int> & q_index)
{
	space_output->setInputValues(q_index,q);
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
		which_direction_type _which_direction)
{
	which_direction=_which_direction;
	space_output=_space_output;
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
	}

}
void rot_space::update_expressions(const std::vector<double>&q,
		const std::vector<int> & q_index)
{
	space_output->setInputValues(q_index,q);
	//std::cout<<"OK"<<std::endl;
}
bool rot_space::compute_jacobian(Eigen::MatrixXd& J_partial,
		const std::vector<int> i_of_v)
{
	if (J_partial.rows()!=size_of_output) return false;
	if (J_partial.cols()!=i_of_v.size()) return false;

	//force update by calling value;
	space_output->value();

	//computing the jacobian
	for(unsigned int i=0;i<i_of_v.size();i++)
	{
		KDL::Vector omega=space_output->derivative(i_of_v[i]);
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

};