#include <expressiongraph_wbf/controllers/scalar_controllers.hpp>
#include "expressiongraph_wbf/solver/simple_force_solver.hpp"

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>



using namespace wbf;
using namespace std;
using namespace KDL;
int main()
{
	//set input values

	KDL::Expression<KDL::Frame>::Ptr w_T_ee
	= frame( inputRot(4),
				KDL::vector(input(1),input(2),input(3)));

	std::vector<int> joint_indexes_out(6);
	std::vector<int> joint_indexes_scalar(3);
	std::vector<int> joint_indexes_rot(1);
	joint_indexes_out[0]=1; //position x
	joint_indexes_out[1]=2; //position y
	joint_indexes_out[2]=3; //position z
	joint_indexes_out[3]=4; //rotation
	joint_indexes_out[4]=5; //rotation
	joint_indexes_out[5]=6; //rotation
	joint_indexes_scalar[0]=1; //position x
	joint_indexes_scalar[1]=2; //position y
	joint_indexes_scalar[2]=3; //position z
	joint_indexes_rot[0]=4; //rotation
	// Calculate forward position kinematics







	//allows for changing set-points from outside.
	VariableType<Vector>::Ptr w_P_trocar;
	std::vector<int> ndx;
	int time_index=0;
	ndx.push_back(time_index);//index of time
	w_P_trocar = Variable<Vector>(ndx);
	w_P_trocar->setValue(Vector(0.1,0,0.3));//value of set-point
	w_P_trocar->setJacobian(time_index,Vector::Zero());//used for feed-forward

	KDL::Expression<KDL::Vector>::Ptr ee_P_trocar=inv(w_T_ee)*w_P_trocar;

//build  constraints
	const double K=1;
	space_description::Ptr space_x(new scalar_space(coord_x(ee_P_trocar)));
	space_description::Ptr space_y(new scalar_space(coord_y(ee_P_trocar)));

	Expression<double>::Ptr gain=Constant(K);
	controller::Ptr	ctrl_x =controller::Ptr(new proportional_scalar_controller(
				coord_x(ee_P_trocar),KDL::Constant(0.0),gain));
	controller::Ptr	ctrl_y =controller::Ptr(new proportional_scalar_controller(
				coord_y(ee_P_trocar),KDL::Constant(0.0),gain));

	constraint::Ptr c_x(new constraint (space_x,ctrl_x));
	constraint::Ptr c_y(new constraint (space_y,ctrl_y));




	simple_force_solver::Ptr solver(new simple_force_solver(
			joint_indexes_out,
			joint_indexes_scalar,
			joint_indexes_rot
			));

		solver->addConstraint("pos_x",c_x);
		solver->addConstraint("pos_y",c_y);

	//external inputs (setpoints)

		Eigen::VectorXd wrench_out(6);

	std::vector<double> pos(3);
	pos[0]=0;pos[1]=0;pos[2]=0;
	std::vector<KDL::Rotation> R(1);
	R[0]=Rotation::Identity();
	cout<<"Compute returned: "<<solver->Prepare()<<endl;
	int res=solver->Compute(pos,R,wrench_out);

	cout<<"computed wrench\n" <<wrench_out.transpose()<<endl;
	cout<<"Compute returned: "<<res<<endl;



}
