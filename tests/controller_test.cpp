#include "expressiongraph_wbf/controllers/position_controller.hpp"
#include "expressiongraph_wbf/controllers/rotation_controller.hpp"
#include "kdl/frames_io.hpp"
using namespace wbf;
using namespace std;
int main()
{
	//robot in expression
	Expression<Vector>::Ptr L      = Constant(Vector(0,0,0.2));
	Expression<Vector>::Ptr L0=  Constant(Vector(0,0,0.0));
	Expression<Rotation>::Ptr R0=  Constant(Rotation(KDL::Rotation::Identity()));
	Expression<Frame>::Ptr w_T_l1  =       frame( rot_z(input(1)), L0)*frame( R0, L);
	Expression<Frame>::Ptr w_T_l2 = w_T_l1*frame( rot_x(input(2)), L0)*frame( R0, L);
	Expression<Frame>::Ptr w_T_l3 = w_T_l2*frame( rot_z(input(3)), L0)*frame( R0, L);
	Expression<Frame>::Ptr w_T_ee = w_T_l3*frame( rot_z(input(4)), L0)*frame( R0, L);
	//Expression<Frame>::Ptr w_T_ee = w_T_l4*frame( Constant(Rotation::Identity()), L);

	Expression<Rotation>::Ptr w_R_ee=rotation(w_T_ee);
	Expression<double>::Ptr w_x_ee=coord_x(origin( (w_T_ee)));
	Expression<double>::Ptr w_y_ee=coord_y(origin( (w_T_ee)));
	Expression<double>::Ptr w_z_ee=coord_z(origin( (w_T_ee)));

	std::vector<int> joint_indexes(4);
	joint_indexes[0]=1;
	joint_indexes[1]=2;
	joint_indexes[2]=3;
	joint_indexes[3]=4;
	std::vector<double> q(4);
	q[0] = 1;
	q[1] = 1.5;
	q[2] = -0.5;
	q[3] = 2;
	controller::Ptr ctrl(new position_controller(w_x_ee,Constant(0.0),Constant(0.3)));
	ctrl->update_expressions(q,joint_indexes);
	Eigen::VectorXd res(1);
	Eigen::VectorXd resR(3);
	ctrl->compute_action(res);
	std::cout<<"res:\t"<<res<<std::endl;


	controller::Ptr ctrlR(
			new rotation_controller(w_R_ee,
			Constant(KDL::Rotation::Identity()),
			Constant(1.0)));
	ctrlR->update_expressions(q,joint_indexes);
	ctrlR->compute_action(resR);
	std::cout<<"w_R_ee\n"<<w_R_ee->value()<<endl;

	std::cout<<"resR:\n"<<resR<<std::endl;
}
