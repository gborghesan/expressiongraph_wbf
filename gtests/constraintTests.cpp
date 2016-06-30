#include <gtest/gtest.h>

#include "testHelpFunctions.hpp"
#include <expressiongraph_wbf/controllers/scalar_controllers.hpp>
#include <expressiongraph_wbf/controllers/rotation_controllers.hpp>

#include "expressiongraph_wbf/solver/constraints.hpp"
#include "expressiongraph_wbf/solver/space_description.hpp"
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>

using namespace wbf;
using namespace std;
using namespace KDL;

//run with ```catkin_make run_tests_expressiongraph_wbf```

// Declare a test

TEST(constraintTests, allCostraintsInLevelTwo)
{
	FOUR_DOF_ROBOT;
	FOUR_DOF_KDL_ROBOT;


	VariableType<double>::Ptr scalarDesLower,scalarDesUpper;
	std::vector<int> ndx;
	int time_index=0;
	ndx.push_back(time_index);//index of time
	double lVal=-0.2, uVal=+0.2;
	scalarDesLower = Variable<double>(ndx);
	scalarDesLower->setValue(lVal);//value of set-point
	scalarDesLower->setJacobian(time_index,0.0);//used for feed-forward
	scalarDesUpper = Variable<double>(ndx);
	scalarDesUpper->setValue(uVal);//value of set-point
	scalarDesUpper->setJacobian(time_index,0.0);//used for feed-forward

	//build  constraints
	const double K=10;
	space_description::Ptr space_x(new scalar_space(w_x_ee));
	space_description::Ptr space_y(new scalar_space(w_y_ee));
	space_description::Ptr space_z(new scalar_space(w_z_ee));

	Expression<double>::Ptr gain=Constant(K);

	controller::Ptr ctrl_x_u(new proportional_scalar_controller(w_x_ee,scalarDesUpper,gain));
	controller::Ptr ctrl_y_u(new proportional_scalar_controller(w_y_ee,scalarDesUpper,gain));
	controller::Ptr ctrl_z_u(new proportional_scalar_controller(w_z_ee,scalarDesUpper,gain));
	controller::Ptr ctrl_x_l(new proportional_scalar_controller(w_x_ee,scalarDesLower,gain));
	controller::Ptr ctrl_y_l(new proportional_scalar_controller(w_y_ee,scalarDesLower,gain));
	controller::Ptr ctrl_z_l(new proportional_scalar_controller(w_z_ee,scalarDesLower,gain));
	unsigned int priority_level=2;
	constraint::Ptr c_x(new constraint (space_x,ctrl_x_l,ctrl_x_u,priority_level,Constant(1.0)));
	constraint::Ptr c_y(new constraint (space_y,ctrl_y_l,ctrl_y_u,priority_level,Constant(2.0)));
	constraint::Ptr c_z(new constraint (space_z,ctrl_z_l,ctrl_z_u,priority_level,Constant(3.0)));


	constraints::Ptr cnstr(new constraints(joint_indexes));
	ASSERT_TRUE(cnstr->addConstraint("xDir",c_x));
	ASSERT_TRUE(cnstr->addConstraint("yDir",c_y));
	ASSERT_TRUE(cnstr->addConstraint("zDir",c_z));
	bool noExceptionRaised=true;

	Eigen::VectorXd Wqdiag;
	std::vector<unsigned int> priorities;

	ASSERT_FALSE(cnstr->getQweights(Wqdiag));
	ASSERT_FALSE(cnstr->getPriorityCardinality(priorities));
	cnstr->setTimeIndex(time_index);

	try{cnstr->Prepare();}
	catch (constraintException& e) {
		noExceptionRaised=false;
	}
	ASSERT_TRUE(noExceptionRaised);


	ASSERT_TRUE(cnstr->getPriorityCardinality(priorities));
	ASSERT_EQ(priorities.size(),3);
	ASSERT_EQ(priorities[0],0);
	ASSERT_EQ(priorities[1],0);
	ASSERT_EQ(priorities[2],3);


	ASSERT_TRUE(cnstr->getQweights(Wqdiag));
	ASSERT_EQ(Wqdiag.size(),joint_indexes.size());

	double currentTime=0;
	bool timePresent=true;
	ASSERT_EQ(cnstr->computeJacobianAndBounds(
			inp,currentTime,timePresent),0);

	std::vector<Eigen::MatrixXd> JacobianPerPriority;
	ASSERT_TRUE(cnstr->getJacobian(JacobianPerPriority));
	ASSERT_EQ(JacobianPerPriority.size(),3);
	ASSERT_EQ(JacobianPerPriority[0].rows(),0);
	ASSERT_EQ(JacobianPerPriority[0].cols(),joint_indexes.size());
	ASSERT_EQ(JacobianPerPriority[1].rows(),0);
	ASSERT_EQ(JacobianPerPriority[1].cols(),joint_indexes.size());
	ASSERT_EQ(JacobianPerPriority[2].rows(),3);
	ASSERT_EQ(JacobianPerPriority[2].cols(),joint_indexes.size());



	Eigen::MatrixXd J3Err(3,4);
	J3Err=JKDL.data.block(0,0,3,4)-JacobianPerPriority[2];
	ASSERT_NEAR(J3Err.norm(), 0,0.000001);

	std::vector<Eigen::VectorXd> lowerBoundPerPriority,upperBoundPerPriority;
	ASSERT_TRUE(cnstr->getLowerBounds(lowerBoundPerPriority));
	ASSERT_TRUE(cnstr->getUpperBounds(upperBoundPerPriority));
	ASSERT_EQ(lowerBoundPerPriority.size(),3);
	ASSERT_EQ(lowerBoundPerPriority[0].size(),0);
	ASSERT_EQ(lowerBoundPerPriority[1].size(),0);
	ASSERT_EQ(lowerBoundPerPriority[2].size(),3);


	double velExpectedLower_x=K*(lVal-cartpos.p.x());
	double velExpectedLower_y=K*(lVal-cartpos.p.y());
	double velExpectedLower_z=K*(lVal-cartpos.p.z());
	double velExpectedUpper_x=K*(uVal-cartpos.p.x());
	double velExpectedUpper_y=K*(uVal-cartpos.p.y());
	double velExpectedUpper_z=K*(uVal-cartpos.p.z());

	ASSERT_NEAR(velExpectedLower_x, lowerBoundPerPriority[2](0),0.000001);
	ASSERT_NEAR(velExpectedLower_y, lowerBoundPerPriority[2](1),0.000001);
	ASSERT_NEAR(velExpectedLower_z, lowerBoundPerPriority[2](2),0.000001);
	ASSERT_NEAR(velExpectedUpper_x, upperBoundPerPriority[2](0),0.000001);
	ASSERT_NEAR(velExpectedUpper_y, upperBoundPerPriority[2](1),0.000001);
	ASSERT_NEAR(velExpectedUpper_z, upperBoundPerPriority[2](2),0.000001);

	std::vector<Eigen::VectorXd> WyPerPriority ;
	ASSERT_TRUE(cnstr->getYweights(WyPerPriority));
	ASSERT_EQ(WyPerPriority.size(),3);
	ASSERT_EQ(WyPerPriority[0].size(),0);
	ASSERT_EQ(WyPerPriority[1].size(),0);
	ASSERT_EQ(WyPerPriority[2].size(),3);

	ASSERT_NEAR( WyPerPriority[2](0),1,0.000001);
	ASSERT_NEAR( WyPerPriority[2](1),2,0.000001);
	ASSERT_NEAR( WyPerPriority[2](2),3,0.000001);
}



TEST(constraintTests, allCostraintsInLevelTwoTimeUsed)
{
	FOUR_DOF_ROBOT;
	FOUR_DOF_KDL_ROBOT;


	VariableType<double>::Ptr scalarDesLower,scalarDesUpper;
	std::vector<int> ndx;
	int time_index=0;
	ndx.push_back(time_index);//index of time
	double lVal=-0.2, uVal=+0.2;
	scalarDesLower = Variable<double>(ndx);
	scalarDesLower->setValue(lVal);//value of set-point
	scalarDesLower->setJacobian(time_index,0.0);//used for feed-forward
	scalarDesUpper = Variable<double>(ndx);
	scalarDesUpper->setValue(uVal);//value of set-point
	scalarDesUpper->setJacobian(time_index,0.0);//used for feed-forward

	//build  constraints
	const double K=10;
	space_description::Ptr space_x(new scalar_space(w_x_ee));
	space_description::Ptr space_y(new scalar_space(w_y_ee));
	space_description::Ptr space_z(new scalar_space(w_z_ee));

	Expression<double>::Ptr gain=Constant(K);

	controller::Ptr ctrl_x_u(new proportional_scalar_controller(w_x_ee,scalarDesUpper,gain));
	controller::Ptr ctrl_y_u(new proportional_scalar_controller(w_y_ee,scalarDesUpper,gain));
	controller::Ptr ctrl_z_u(new proportional_scalar_controller(w_z_ee,scalarDesUpper,gain));
	controller::Ptr ctrl_x_l(new proportional_scalar_controller(w_x_ee,scalarDesLower,gain));
	controller::Ptr ctrl_y_l(new proportional_scalar_controller(w_y_ee,scalarDesLower,gain));
	controller::Ptr ctrl_z_l(new proportional_scalar_controller(w_z_ee,scalarDesLower,gain));
	unsigned int priority_level=2;
	constraint::Ptr c_x(new constraint (space_x,ctrl_x_l,ctrl_x_u,priority_level,input(time_index)*Constant(1.0)));
	constraint::Ptr c_y(new constraint (space_y,ctrl_y_l,ctrl_y_u,priority_level,input(time_index)*Constant(2.0)));
	constraint::Ptr c_z(new constraint (space_z,ctrl_z_l,ctrl_z_u,priority_level,input(time_index)*Constant(3.0)));


	constraints::Ptr cnstr(new constraints(joint_indexes));
	ASSERT_TRUE(cnstr->addConstraint("xDir",c_x));
	ASSERT_TRUE(cnstr->addConstraint("yDir",c_y));
	ASSERT_TRUE(cnstr->addConstraint("zDir",c_z));
	bool noExceptionRaised=true;

	Eigen::VectorXd Wqdiag;
	std::vector<unsigned int> priorities;

	ASSERT_FALSE(cnstr->getQweights(Wqdiag));
	ASSERT_FALSE(cnstr->getPriorityCardinality(priorities));
	cnstr->setTimeIndex(time_index);

	try{cnstr->Prepare();}
	catch (constraintException& e) {
		noExceptionRaised=false;
	}
	ASSERT_TRUE(noExceptionRaised);


	ASSERT_TRUE(cnstr->getPriorityCardinality(priorities));
	ASSERT_EQ(priorities.size(),3);
	ASSERT_EQ(priorities[0],0);
	ASSERT_EQ(priorities[1],0);
	ASSERT_EQ(priorities[2],3);


	ASSERT_TRUE(cnstr->getQweights(Wqdiag));
	ASSERT_EQ(Wqdiag.size(),joint_indexes.size());

	double currentTime=1;
	bool timePresent=true;
	ASSERT_EQ(cnstr->computeJacobianAndBounds(
			inp,currentTime,timePresent),0);

	std::vector<Eigen::MatrixXd> JacobianPerPriority;
	ASSERT_TRUE(cnstr->getJacobian(JacobianPerPriority));
	ASSERT_EQ(JacobianPerPriority.size(),3);
	ASSERT_EQ(JacobianPerPriority[0].rows(),0);
	ASSERT_EQ(JacobianPerPriority[0].cols(),joint_indexes.size());
	ASSERT_EQ(JacobianPerPriority[1].rows(),0);
	ASSERT_EQ(JacobianPerPriority[1].cols(),joint_indexes.size());
	ASSERT_EQ(JacobianPerPriority[2].rows(),3);
	ASSERT_EQ(JacobianPerPriority[2].cols(),joint_indexes.size());



	Eigen::MatrixXd J3Err(3,4);
	J3Err=JKDL.data.block(0,0,3,4)-JacobianPerPriority[2];
	ASSERT_NEAR(J3Err.norm(), 0,0.000001);

	std::vector<Eigen::VectorXd> lowerBoundPerPriority,upperBoundPerPriority;
	ASSERT_TRUE(cnstr->getLowerBounds(lowerBoundPerPriority));
	ASSERT_TRUE(cnstr->getUpperBounds(upperBoundPerPriority));
	ASSERT_EQ(lowerBoundPerPriority.size(),3);
	ASSERT_EQ(lowerBoundPerPriority[0].size(),0);
	ASSERT_EQ(lowerBoundPerPriority[1].size(),0);
	ASSERT_EQ(lowerBoundPerPriority[2].size(),3);


	double velExpectedLower_x=K*(lVal-cartpos.p.x());
	double velExpectedLower_y=K*(lVal-cartpos.p.y());
	double velExpectedLower_z=K*(lVal-cartpos.p.z());
	double velExpectedUpper_x=K*(uVal-cartpos.p.x());
	double velExpectedUpper_y=K*(uVal-cartpos.p.y());
	double velExpectedUpper_z=K*(uVal-cartpos.p.z());

	ASSERT_NEAR(velExpectedLower_x, lowerBoundPerPriority[2](0),0.000001);
	ASSERT_NEAR(velExpectedLower_y, lowerBoundPerPriority[2](1),0.000001);
	ASSERT_NEAR(velExpectedLower_z, lowerBoundPerPriority[2](2),0.000001);
	ASSERT_NEAR(velExpectedUpper_x, upperBoundPerPriority[2](0),0.000001);
	ASSERT_NEAR(velExpectedUpper_y, upperBoundPerPriority[2](1),0.000001);
	ASSERT_NEAR(velExpectedUpper_z, upperBoundPerPriority[2](2),0.000001);

	std::vector<Eigen::VectorXd> WyPerPriority ;
	ASSERT_TRUE(cnstr->getYweights(WyPerPriority));
	ASSERT_EQ(WyPerPriority.size(),3);
	ASSERT_EQ(WyPerPriority[0].size(),0);
	ASSERT_EQ(WyPerPriority[1].size(),0);
	ASSERT_EQ(WyPerPriority[2].size(),3);
	for (currentTime=2;currentTime<10.0;currentTime+=1.0)
	{
		ASSERT_EQ(cnstr->computeJacobianAndBounds(
					  inp,currentTime,timePresent),0);
		ASSERT_TRUE(cnstr->getYweights(WyPerPriority));
		ASSERT_NEAR( WyPerPriority[2](0),1*currentTime,0.000001);
		ASSERT_NEAR( WyPerPriority[2](1),2*currentTime,0.000001);
		ASSERT_NEAR( WyPerPriority[2](2),3*currentTime,0.000001);
	}
}



int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

