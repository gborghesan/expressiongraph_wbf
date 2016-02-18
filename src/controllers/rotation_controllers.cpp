#include <expressiongraph_wbf/controllers/rotation_controllers.hpp>
using namespace KDL;
namespace wbf {

proportional_rotation_controller::proportional_rotation_controller(Expression<Rotation>::Ptr _p_meas,
		Expression<Rotation>::Ptr _p_des,
		Expression<double>::Ptr _K,
		which_direction_type _t
)
{
	p_des=_p_des;
	p_meas=_p_meas;
	K=_K;
	type="Rot";
	which_direction=_t;
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
void proportional_rotation_controller::update_expressions(
		const std::vector<double> & q_in,
		const std::vector<int> & q_index)
{
	p_meas->setInputValues(q_index,q_in);
	//std::cout<<"OK"<<std::endl;
}
void proportional_rotation_controller::update_expressions_rot(
		const std::vector<KDL::Rotation>&R,
				const std::vector<int> & q_index)
{
	p_meas->setInputValues(q_index,R);
}
void proportional_rotation_controller::update_time(double time, int time_index)
{
	p_des->setInputValue(time_index,time);
	K->setInputValue(time_index,time);
}

//TODO Check math of this function!!!!
bool proportional_rotation_controller::compute_action(Eigen::VectorXd &res)
{
	if (res.size()!=size_of_output) return false;
	Rotation w_R_m=p_meas->value();
	//std::cout<<"w_R_m:\t"<<w_R_m<<std::endl;
	Rotation w_R_d=p_des->value();
	//std::cout<<"w_R_d:\t"<<w_R_d<<std::endl;
	Rotation m_R_d=w_R_m.Inverse()*w_R_d;//Rotation from measured to desired
	//std::cout<<"m_R_d:\t"<<m_R_d<<std::endl;
	Vector   m_w_m=m_R_d.GetRot();//Omega expressed in m and applied in m
	//std::cout<<"m_w_m:\t"<<m_w_m<<std::endl;
	double k=K->value();
	Vector   torque=-m_w_m*k;
	//std::cout<<"torque:\t"<<torque<<std::endl;
	switch (which_direction)
	{
	case FULL_ROTATION:
		res[0]=torque.x();
		res[1]=torque.y();
		res[2]=torque.z();
		break;
	case ROT_X:
		res[0]=torque.x();
		break;
	case ROT_Y:
		res[0]=torque.y();
		break;
	case ROT_Z:
		res[0]=torque.z();
		break;
	}
	return true;
}
};
