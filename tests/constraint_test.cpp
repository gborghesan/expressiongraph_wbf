#include "expressiongraph_wbf/solver/constraints.hpp"
#include "expressiongraph_wbf/controllers/position_controller.hpp"



using namespace wbf;
using namespace std;
using namespace KDL;
int main()
{
	//set input values

	std::vector<double> inp(4);
	inp[0] = 1;
	inp[1] = 1.5;
	inp[2] = -0.5;
	inp[3] = 2;
	std::vector<int> joint_indexes(4);
	joint_indexes[0]=0;
	joint_indexes[1]=1;
	joint_indexes[2]=2;
	joint_indexes[3]=3;



	//robot in expression
	Expression<Vector>::Ptr L      = Constant(Vector(0,0,0.2));
	Expression<Vector>::Ptr L0=  Constant(Vector(0,0,0.0));
	Expression<Rotation>::Ptr R0=  Constant(Rotation(KDL::Rotation::Identity()));
	Expression<Frame>::Ptr w_T_l1  =       frame( rot_z(input(0)), L0)*frame( R0, L);
	Expression<Frame>::Ptr w_T_l2 = w_T_l1*frame( rot_x(input(1)), L0)*frame( R0, L);
	Expression<Frame>::Ptr w_T_l3 = w_T_l2*frame( rot_z(input(2)), L0)*frame( R0, L);
	Expression<Frame>::Ptr w_T_ee = w_T_l3*frame( rot_z(input(3)), L0)*frame( R0, L);

	Expression<Rotation>::Ptr w_R_ee=rotation(w_T_ee);
	Expression<double>::Ptr w_x_ee=coord_x(origin( (w_T_ee)));
	Expression<double>::Ptr w_y_ee=coord_y(origin( (w_T_ee)));
	Expression<double>::Ptr w_z_ee=coord_z(origin( (w_T_ee)));


	Expression<double>::Ptr scalar_des=Constant(0.1);
	Expression<double>::Ptr gain=Constant(3.1);

	controller::Ptr ctrl(new position_controller(w_x_ee,scalar_des,gain));
	space_description::Ptr space(new scalar_space(w_x_ee));
	space_description::Ptr spaceR(new rot_space(w_R_ee));

	constraint c(space,ctrl);
	constraint::Ptr cp(new constraint (space,ctrl));
	constraint::Ptr cRp(new constraint (spaceR,ctrl));
	std::cout<<"should be 1, is: "<<check_constraint_validity(c)<<std::endl;
	std::cout<<"should be 1, is: "<<check_constraint_validity(cp)<<std::endl;
	std::cout<<"should be 0, is: "<<check_constraint_validity(cRp)<<std::endl;
}
