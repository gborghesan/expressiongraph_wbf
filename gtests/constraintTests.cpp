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
TEST(constraintTests, checkConstraintValidity)
{

	FOUR_DOF_ROBOT;
	Expression<double>::Ptr scalar_des=Constant(0.1);
	Expression<double>::Ptr gain=Constant(3.1);
	controller::Ptr ctrl(new proportional_scalar_controller(w_x_ee,scalar_des,gain));
	space_description::Ptr space(new scalar_space(w_x_ee));
	space_description::Ptr spaceR(new rot_space(w_R_ee));

	constraint c(space,ctrl);
	constraint::Ptr cp(new constraint (space,ctrl));
	constraint::Ptr cRp(new constraint (spaceR,ctrl));
	ASSERT_TRUE(check_constraint_validity(c));
	ASSERT_TRUE(check_constraint_validity(cp));
	ASSERT_FALSE(check_constraint_validity(cRp));

}
TEST(constraintTests, scalarProportionalController)
{
	FOUR_DOF_ROBOT;
	controller::Ptr ctrl(
				new proportional_scalar_controller(
					w_x_ee,Constant(0.0),Constant(0.3)));
	ctrl->update_expressions(inp,joint_indexes);
	Eigen::VectorXd res(1);
	ctrl->compute_action(res);

	ASSERT_TRUE(ctrl->compute_action(res));
	ASSERT_NEAR(res(0), -0.068483213562657588,0.00001);
}
#include "kdl/frames_io.hpp"
TEST(constraintTests, RotationalProportionalController)
{
	FOUR_DOF_ROBOT;

	controller::Ptr ctrlR(
				new proportional_rotation_controller
				(w_R_ee,
				 Constant(KDL::Rotation::Identity()),
				 Constant(1.0)));

	ctrlR->update_expressions(inp,joint_indexes);
	Eigen::VectorXd resR(3);
	ASSERT_TRUE(ctrlR->compute_action(resR));

	Eigen::VectorXd resRexp(3), error(3);
	resRexp<<-2.65902,
			 -1.23848,
			 0.900128;

	error=resR-resRexp;
	ASSERT_NEAR(error.norm(), 0,0.00001);
}

TEST(constraintTests, spaceDescriptionScalar)
{
	FOUR_DOF_ROBOT;
	FOUR_DOF_KDL_ROBOT;

	Eigen::MatrixXd J(1,4), JErr(1,4);

	space_description::Ptr space_des_scalar_x(new scalar_space(w_x_ee));
	space_description::Ptr space_des_scalar_y(new scalar_space(w_y_ee));
	space_description::Ptr space_des_scalar_z(new scalar_space(w_z_ee));
	space_des_scalar_x->update_expressions(inp,joint_indexes);
	space_des_scalar_y->update_expressions(inp,joint_indexes);
	space_des_scalar_z->update_expressions(inp,joint_indexes);


	ASSERT_TRUE(space_des_scalar_x->compute_jacobian(J,joint_indexes));
	JErr=JKDL.data.row(0)-J;
	ASSERT_NEAR(JErr.norm(), 0,0.000001);

	ASSERT_TRUE(space_des_scalar_y->compute_jacobian(J,joint_indexes));
	JErr=JKDL.data.row(1)-J;
	ASSERT_NEAR(JErr.norm(), 0,0.000001);

	ASSERT_TRUE(space_des_scalar_z->compute_jacobian(J,joint_indexes));
	JErr=JKDL.data.row(2)-J;
	ASSERT_NEAR(JErr.norm(), 0,0.000001);

}

TEST(constraintTests, spaceDescriptionRotation)
{
	FOUR_DOF_ROBOT;
	FOUR_DOF_KDL_ROBOT;


	Eigen::MatrixXd J1(1,4), J1Err(1,4);
	Eigen::MatrixXd J3(3,4), J3Err(3,4);

	space_description::Ptr space_des_rot3, space_des_rotx,space_des_roty,space_des_rotz;

	space_des_rot3=space_description::Ptr(new rot_space(w_R_ee));
	space_des_rotx=space_description::Ptr(new rot_space(w_R_ee,ROT_X));
	space_des_roty=space_description::Ptr(new rot_space(w_R_ee,ROT_Y));
	space_des_rotz=space_description::Ptr(new rot_space(w_R_ee,ROT_Z));

	space_des_rot3->update_expressions(inp,joint_indexes);
	space_des_rotx->update_expressions(inp,joint_indexes);
	space_des_roty->update_expressions(inp,joint_indexes);
	space_des_rotz->update_expressions(inp,joint_indexes);

	ASSERT_TRUE(space_des_rot3->compute_jacobian(J3,joint_indexes));
	J3Err=JKDL.data.block(3,0,3,4)-J3;
	ASSERT_NEAR(J3Err.norm(), 0,0.000001);

	ASSERT_TRUE(space_des_rotx->compute_jacobian(J1,joint_indexes));
	J1Err=JKDL.data.row(3)-J1;
	ASSERT_NEAR(J1Err.norm(), 0,0.000001);

	ASSERT_TRUE(space_des_roty->compute_jacobian(J1,joint_indexes));
	J1Err=JKDL.data.row(4)-J1;
	ASSERT_NEAR(J1Err.norm(), 0,0.000001);

	ASSERT_TRUE(space_des_rotz->compute_jacobian(J1,joint_indexes));
	J1Err=JKDL.data.row(5)-J1;
	ASSERT_NEAR(J1Err.norm(), 0,0.000001);
}


TEST(constraintTests, spaceDescriptionRotationOwnFrame)
{
	FOUR_DOF_ROBOT;
	FOUR_DOF_KDL_ROBOT;

	JKDL.changeBase(cartpos.M);


	Expression<Rotation>::Ptr ee_R_ee=(make_constant<Rotation>(w_R_ee))*w_R_ee;


	Eigen::MatrixXd J1(1,4), J1Err(1,4);
	Eigen::MatrixXd J3(3,4), J3Err(3,4);

	space_description::Ptr space_des_rot3, space_des_rotx,space_des_roty,space_des_rotz;

	space_des_rot3=space_description::Ptr(new rot_space(ee_R_ee));
	space_des_rotx=space_description::Ptr(new rot_space(ee_R_ee,ROT_X));
	space_des_roty=space_description::Ptr(new rot_space(ee_R_ee,ROT_Y));
	space_des_rotz=space_description::Ptr(new rot_space(ee_R_ee,ROT_Z));

	space_des_rot3->update_expressions(inp,joint_indexes);
	space_des_rotx->update_expressions(inp,joint_indexes);
	space_des_roty->update_expressions(inp,joint_indexes);
	space_des_rotz->update_expressions(inp,joint_indexes);

	ASSERT_TRUE(space_des_rot3->compute_jacobian(J3,joint_indexes));
	J3Err=JKDL.data.block(3,0,3,4)-J3;
	ASSERT_NEAR(J3Err.norm(), 0,0.000001);

	ASSERT_TRUE(space_des_rotx->compute_jacobian(J1,joint_indexes));
	J1Err=JKDL.data.row(3)-J1;
	ASSERT_NEAR(J1Err.norm(), 0,0.000001);

	ASSERT_TRUE(space_des_roty->compute_jacobian(J1,joint_indexes));
	J1Err=JKDL.data.row(4)-J1;
	ASSERT_NEAR(J1Err.norm(), 0,0.000001);

	ASSERT_TRUE(space_des_rotz->compute_jacobian(J1,joint_indexes));
	J1Err=JKDL.data.row(5)-J1;
	ASSERT_NEAR(J1Err.norm(), 0,0.000001);
}


int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

