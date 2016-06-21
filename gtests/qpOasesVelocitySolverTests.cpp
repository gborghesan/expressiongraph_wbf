#include <gtest/gtest.h>

#include <expressiongraph_wbf/controllers/scalar_controllers.hpp>
#include <expressiongraph_wbf/controllers/rotation_controllers.hpp>
#include "expressiongraph_wbf/solver/velocity_solver.hpp"

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


TEST(qpOasesVelocitySolverTests, basicTest)
{
	FOUR_DOF_ROBOT;

	VariableType<double>::Ptr scalar_des;
	std::vector<int> ndx;
	int time_index=0;
	ndx.push_back(time_index);//index of time
	scalar_des = Variable<double>(ndx);
	scalar_des->setValue(0.2);//value of set-point
	scalar_des->setJacobian(time_index,0.0);//used for feed-forward

	//build  constraints
	const double K=1;
	space_description::Ptr space_x(new scalar_space(w_x_ee));
	space_description::Ptr space_y(new scalar_space(w_y_ee));
	space_description::Ptr space_z(new scalar_space(w_z_ee));

	Expression<double>::Ptr gain=Constant(K);
	controller::Ptr ctrl_x(new proportional_scalar_controller(w_x_ee,scalar_des,gain));
	controller::Ptr ctrl_y(new proportional_scalar_controller(w_y_ee,scalar_des,gain));
	controller::Ptr ctrl_z(new proportional_scalar_controller(w_z_ee,scalar_des,gain));

	constraint::Ptr c_x(new constraint (space_x,ctrl_x,ctrl_x));
	constraint::Ptr c_y(new constraint (space_y,ctrl_y,ctrl_y));
	constraint::Ptr c_z(new constraint (space_z,ctrl_z,ctrl_z));

	c_x->weight=input(time_index);

	Eigen::VectorXd qdot_out(4);
	//build solver, without time derivative support
	velocity_solver::Ptr s(new velocity_solver(joint_indexes,time_index));
	Eigen::VectorXd qdot_w(4);
	qdot_w<<1,2,3,4;
	s->setQweights(qdot_w);
	ASSERT_TRUE(s->addConstraint("pos_x",c_x));
	ASSERT_TRUE(s->addConstraint("pos_y",c_y));
	ASSERT_TRUE(s->addConstraint("pos_z",c_z));
	ASSERT_EQ(s->Prepare(),1);

	/*
	cout <<"ADD CONST:"<<s->addConstraint("pos_x",c_x)<<endl;
	cout <<"ADD CONST:"<<s->addConstraint("pos_y",c_y)<<endl;
	cout <<"ADD CONST:"<<s->addConstraint("pos_z",c_z)<<endl;
	cout <<"PREPARE:"<<s->Prepare()<<endl;
	*/

	KDL::Vector robotPositionReal, RobotPositionDesired(0.2,0.2,0.2), delta;
	int count=1;
	for (int i=0;i<10001;i++)
	{
		//cout <<"COMPUTE:"<<s->Compute(q,qdot_out)<<endl;
		ASSERT_EQ(	s->Compute(inp,i,qdot_out),1);
		for (unsigned int j=0;j<inp.size();j++)
			inp[j]=inp[j]+qdot_out(j)*0.01;
		count--;
	/*	if (count==0)
		{
			count=1000;
			cout <<"POSE:"<<origin(w_T_ee)->value()<<endl;
		}*/
	}

	robotPositionReal=origin(w_T_ee)->value();

	delta=RobotPositionDesired-robotPositionReal;


	ASSERT_NEAR(delta.Norm(), 0,0.000001);
	//std::cout<<"\n expected value [0.2 0.2 0.2]"<<endl;

	ASSERT_TRUE(s->RemoveConstraint("pos_z"));
	scalar_des->setValue(0.3);//value of set-point
	ASSERT_EQ(s->Prepare(),1);
	count=1;
	for (int i=0;i<10001;i++)
	{
		//cout <<"COMPUTE:"<<s->Compute(q,qdot_out)<<endl;
		ASSERT_EQ(	s->Compute(inp,i,qdot_out),1);
		for (unsigned int j=0;j<inp.size();j++)
			inp[j]=inp[j]+qdot_out(j)*0.01;
		count--;
	/*		if (count==0)
		{
			count=1000;
			cout <<"POSE:"<<origin(w_T_ee)->value()<<endl;
		}*/
	}
	robotPositionReal=origin(w_T_ee)->value();
	RobotPositionDesired=KDL::Vector(0.3,0.3,0);
	delta=RobotPositionDesired-robotPositionReal;
	delta.z(0);
	ASSERT_NEAR(delta.Norm(), 0,0.000001);

}


TEST(qpOasesVelocitySolverTests, freeFloatingTest)
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
	const double K=0.5;
	space_description::Ptr space_x(new scalar_space(coord_x(ee_P_trocar)));
	space_description::Ptr space_y(new scalar_space(coord_y(ee_P_trocar)));

	Expression<double>::Ptr gain=Constant(K);

	controller::Ptr	ctrl_x =controller::Ptr(new proportional_scalar_controller(
				coord_x(ee_P_trocar),KDL::Constant(0.0),gain));
	controller::Ptr	ctrl_y =controller::Ptr(new proportional_scalar_controller(
				coord_y(ee_P_trocar),KDL::Constant(0.0),gain));

	constraint::Ptr c_x(new constraint (space_x,ctrl_x));
	constraint::Ptr c_y(new constraint (space_y,ctrl_y));


	c_y->priority_level=1;

	velocity_solver::Ptr solver(new velocity_solver(
			joint_indexes_out,
			joint_indexes_scalar,
			joint_indexes_rot
			));





	Eigen::VectorXd qdot_out(6);


	Eigen::VectorXd qdot_w(6);
	qdot_w<<1,1,1,0.2,0.2,0.2;
	solver->setQweights(qdot_w);
	ASSERT_TRUE(solver->addConstraint("pos_x",c_x));
	ASSERT_TRUE(solver->addConstraint("pos_y",c_y));

	ASSERT_EQ(solver->Prepare(),1);

	std::vector<KDL::Rotation> R(1);
	std::vector<double> q(3);
	q[0]=0;q[1]=0;q[2]=0;
	R[0]=KDL::Rotation::RotX(0.3);


	for (int i=0;i<100;i++)
	{

		ASSERT_EQ(solver->Compute(q,R,qdot_out),1);



		for (unsigned int j=0;j<q.size();j++)
			q[j]=q[j]+qdot_out(j);
		addDelta(R[0],Vector(qdot_out(3),qdot_out(4),qdot_out(5)));


	}
	ASSERT_NEAR(coord_x(ee_P_trocar)->value(), 0,0.000001);
	ASSERT_NEAR(coord_y(ee_P_trocar)->value(), 0,0.000001);

}




int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

