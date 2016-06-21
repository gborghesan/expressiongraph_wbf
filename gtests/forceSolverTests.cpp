#include <gtest/gtest.h>

#include <expressiongraph_wbf/controllers/scalar_controllers.hpp>
#include <expressiongraph_wbf/controllers/rotation_controllers.hpp>
#include "expressiongraph_wbf/solver/simple_force_solver.hpp"

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>

#include"testHelpFunctions.hpp"
using namespace wbf;
using namespace std;
using namespace KDL;

//run with ```catkin_make run_tests_expressiongraph_wbf```


TEST(forceSolverTests, basicTest)
{
	FOUR_DOF_ROBOT;
	FOUR_DOF_KDL_ROBOT;

	//allows for changing set-points from outside.
	VariableType<double>::Ptr scalar_des;
	std::vector<int> ndx;
	int time_index=0;
	ndx.push_back(time_index);//index of time
	scalar_des = Variable<double>(ndx);
	scalar_des->setValue(0.0);//value of set-point
	scalar_des->setJacobian(time_index,0.3);//used for feed-forward

	//build  constraints
	const double K=100;
	space_description::Ptr space_x(new scalar_space(w_x_ee));
	space_description::Ptr space_y(new scalar_space(w_y_ee));
	space_description::Ptr space_z(new scalar_space(w_z_ee));

	Expression<double>::Ptr gain=Constant(K);
	controller::Ptr ctrl_x(new proportional_scalar_controller(w_x_ee,scalar_des,gain));
	controller::Ptr ctrl_y(new proportional_scalar_controller(w_y_ee,scalar_des,gain));
	controller::Ptr ctrl_z(new proportional_scalar_controller(w_z_ee,scalar_des,gain));

	constraint::Ptr c_x(new constraint (space_x,ctrl_x));
	constraint::Ptr c_y(new constraint (space_y,ctrl_y));
	constraint::Ptr c_z(new constraint (space_z,ctrl_z));

	//	std::cout<<"\n======Expected values========:"<<endl;
	//cout<<"w_p_ee\n" <<cartpos.p<<endl;

	Eigen::VectorXd wrenchDesired(6);
	wrenchDesired(0)=-cartpos.p.x()*K;
	wrenchDesired(1)=-cartpos.p.y()*K;
	wrenchDesired(2)=-cartpos.p.z()*K;
	wrenchDesired(3)=wrenchDesired(4)=wrenchDesired(5)=0;
	//cout<<"desired wrench\n" <<wrench.transpose()<<endl;
	Eigen::VectorXd tauExpected(4),tauComputed(4), tauErrorExpectedComputed(4);
	tauExpected=JKDL.data.transpose()*wrenchDesired;
	//cout<<"expected tau\n" <<tau.transpose()<<endl;

	simple_force_solver::Ptr s(new simple_force_solver(joint_indexes));
	ASSERT_TRUE(s->addConstraint("pos_x",c_x));
	ASSERT_TRUE(s->addConstraint("pos_y",c_y));
	ASSERT_TRUE(s->addConstraint("pos_z",c_z));

	ASSERT_EQ(s->Prepare(),1);
	ASSERT_EQ(s->Compute(inp,tauComputed),1);

	tauErrorExpectedComputed=tauComputed-tauExpected;
	ASSERT_NEAR(tauErrorExpectedComputed.norm(), 0,0.000001);
	//	cout<<"Joint Torque is:\n "<<tauComputed.transpose()<<endl;

}
TEST(forceSolverTests, basicTestWithJointConstraint)
{
	FOUR_DOF_ROBOT;
	FOUR_DOF_KDL_ROBOT;

	//allows for changing set-points from outside.
	VariableType<double>::Ptr scalar_des;
	std::vector<int> ndx;
	int time_index=0;
	ndx.push_back(time_index);//index of time
	scalar_des = Variable<double>(ndx);
	scalar_des->setValue(0.0);//value of set-point
	scalar_des->setJacobian(time_index,0.3);//used for feed-forward

	double q4des=0.1;


	//build  constraints
	const double K=100;
	space_description::Ptr space_x(new scalar_space(w_x_ee));
	space_description::Ptr space_y(new scalar_space(w_y_ee));
	space_description::Ptr space_z(new scalar_space(w_z_ee));

	space_description::Ptr space_q4(new scalar_space(input(4)));
	Expression<double>::Ptr gain=Constant(K);
	controller::Ptr ctrl_x(new proportional_scalar_controller(w_x_ee,scalar_des,gain));
	controller::Ptr ctrl_y(new proportional_scalar_controller(w_y_ee,scalar_des,gain));
	controller::Ptr ctrl_z(new proportional_scalar_controller(w_z_ee,scalar_des,gain));
	controller::Ptr ctrl_q(new proportional_scalar_controller(input(4),Constant(q4des),gain));

	constraint::Ptr c_x(new constraint (space_x,ctrl_x));
	constraint::Ptr c_y(new constraint (space_y,ctrl_y));
	constraint::Ptr c_z(new constraint (space_z,ctrl_z));
	constraint::Ptr c_q(new constraint (space_q4,ctrl_q));

	//	std::cout<<"\n======Expected values========:"<<endl;
	//cout<<"w_p_ee\n" <<cartpos.p<<endl;

	Eigen::VectorXd wrenchDesired(6);
	wrenchDesired(0)=-cartpos.p.x()*K;
	wrenchDesired(1)=-cartpos.p.y()*K;
	wrenchDesired(2)=-cartpos.p.z()*K;
	wrenchDesired(3)=wrenchDesired(4)=wrenchDesired(5)=0;
	//cout<<"desired wrench\n" <<wrench.transpose()<<endl;
	Eigen::VectorXd tauExpected(4),tauComputed(4), tauErrorExpectedComputed(4);
	tauExpected=JKDL.data.transpose()*wrenchDesired;

	tauExpected(3)+=(q4des-inp[3])*K;


	simple_force_solver::Ptr s(new simple_force_solver(joint_indexes));
	ASSERT_TRUE(s->addConstraint("pos_x",c_x));
	ASSERT_TRUE(s->addConstraint("pos_y",c_y));
	ASSERT_TRUE(s->addConstraint("pos_z",c_z));
	ASSERT_TRUE(s->addConstraint("pos_q",c_q));

	ASSERT_EQ(s->Prepare(),1);
	ASSERT_EQ(s->Compute(inp,tauComputed),1);
	//cout<<"expected tau\n" <<tauExpected.transpose()<<endl;
	//cout<<"Joint Torque is:\n "<<tauComputed.transpose()<<endl;
	tauErrorExpectedComputed=tauComputed-tauExpected;
	ASSERT_NEAR(tauErrorExpectedComputed.norm(), 0,0.000001);


}
TEST(forceSolverTests, freeFloatingBase)
{
	FLOATING_BASE_ROBOT;



	//allows for changing set-points from outside.
	VariableType<Vector>::Ptr w_P_trocar;
	std::vector<int> ndx;
	int time_index=0;
	//ndx.push_back(time_index);//index of time
	w_P_trocar = Variable<Vector>(ndx);
	w_P_trocar->setValue(Vector(0.1,0,0.3));//value of set-point
	//w_P_trocar->setJacobian(time_index,Vector::Zero());//used for feed-forward

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

	ASSERT_TRUE(solver->addConstraint("pos_x",c_x));
	ASSERT_TRUE(solver->addConstraint("pos_y",c_y));

	ASSERT_EQ(solver->Prepare(),1);



	//external inputs (setpoints)

	Eigen::VectorXd wrenchComputed(6), wrenchExpected(6),wrenchErrorExpectedComputed(6) ;

	wrenchExpected<< 0.1 ,   0  ,  0  ,  0, 0.03   , 0;
	ASSERT_EQ(solver->Compute(inputPosValue,inputRotValue,wrenchComputed),1);
	wrenchErrorExpectedComputed=wrenchComputed-wrenchExpected;
	ASSERT_NEAR(wrenchErrorExpectedComputed.norm(), 0,0.000001);
	/*
	cout<<"Prepare returned: "<<solver->Prepare()<<endl;
	int res=solver->Compute(inputPosValue,inputRotValue,wrench_out);
	cout<<"trocar\n" <<w_P_trocar->value()<<endl;
	cout<<"ee pos\n" <<Vector(inputPosValue[0],inputPosValue[1],inputPosValue[2])<<endl;
	cout<<"ee rot\n" <<inputRotValue[0]<<endl;
	cout<<"computed wrench\n" <<wrench_out.transpose()<<endl;
	cout<<"Compute returned: "<<res<<endl;*/



	inputPosValue[0]=0.2;
	wrenchExpected<< -0.1 ,   0  ,  0  ,  0, -0.03   , 0;
	ASSERT_EQ(solver->Compute(inputPosValue,inputRotValue,wrenchComputed),1);
	wrenchErrorExpectedComputed=wrenchComputed-wrenchExpected;
	ASSERT_NEAR(wrenchErrorExpectedComputed.norm(), 0,0.000001);

/*
	res=solver->Compute(inputPosValue,inputRotValue,wrenchComputed);
	cout<<"trocar\n" <<w_P_trocar->value()<<endl;
	cout<<"ee pos\n" <<Vector(inputPosValue[0],inputPosValue[1],inputPosValue[2])<<endl;
	cout<<"ee rot\n" <<inputRotValue[0]<<endl;
	cout<<"computed wrench\n" <<wrenchComputed.transpose()<<endl;
	cout<<"Compute returned: "<<res<<endl;*/

	inputRotValue[0]=KDL::Rotation::RotX(0.3);

	wrenchExpected<< -0.1  , 0.0846964  , 0.0261997 , -0.0254089  ,  -0.02738,-0.00846964;
	ASSERT_EQ(solver->Compute(inputPosValue,inputRotValue,wrenchComputed),1);
	wrenchErrorExpectedComputed=wrenchComputed-wrenchExpected;
	ASSERT_NEAR(wrenchErrorExpectedComputed.norm(), 0,0.00001);

	/*
	res=solver->Compute(inputPosValue,inputRotValue,wrenchComputed);
	cout<<"trocar\n" <<w_P_trocar->value()<<endl;
	cout<<"ee pos\n" <<Vector(inputPosValue[0],inputPosValue[1],inputPosValue[2])<<endl;
	cout<<"ee rot\n" <<inputRotValue[0]<<endl;
	cout<<"computed wrench\n" <<wrenchComputed.transpose()<<endl;
	cout<<"Compute returned: "<<res<<endl;
*/

}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

