#include "expressiongraph_wbf/solver/space_description.hpp"

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

	std::vector<double> inp(4);
	inp[0] = 1;
	inp[1] = 1.5;
	inp[2] = -0.5;
	inp[3] = 2;


	KDL::Chain chain;
	KDL::Vector L2(0,0,0.2);
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(L2)));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(L2)));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(L2)));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(L2)));
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
 	ChainJntToJacSolver Jsolver = ChainJntToJacSolver(chain);
	unsigned int nj = chain.getNrOfJoints();
	KDL::JntArray jointpositions = JntArray(nj);
	for(unsigned int i=0;i<nj;i++)
		jointpositions(i)=inp[i];

	KDL::Frame cartpos;
	KDL::Jacobian JKDL(nj);

	// Calculate forward position kinematics

	fksolver.JntToCart(jointpositions,cartpos);
	std::cout<<"======KDL VALUES========:\n"<<endl;
	Jsolver.JntToJac(jointpositions,JKDL);

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

	w_T_ee->setInputValues(joint_indexes,inp);
	KDL::Frame Te=w_T_ee->value();
	cout<<"Expected w_T_ee\n" <<cartpos<<endl;
	cout<<"Expression w_T_ee\n" <<Te<<endl;
	cout<<"Expected Full Jacobian\n" <<JKDL.data<<endl;

	std::cout<<"======TEST ON SCALARS========:\n"<<endl;




	space_description::Ptr space_des_scalar_x(new scalar_space(w_x_ee));
	space_description::Ptr space_des_scalar_y(new scalar_space(w_y_ee));
	space_description::Ptr space_des_scalar_z(new scalar_space(w_z_ee));
	space_des_scalar_x->update_expressions(inp,joint_indexes);
	space_des_scalar_y->update_expressions(inp,joint_indexes);
	space_des_scalar_z->update_expressions(inp,joint_indexes);
	Eigen::MatrixXd J(1,4);
	space_des_scalar_x->compute_jacobian(J,joint_indexes);
	std::cout<<"JacobianX:\n"<<J<<std::endl;
	space_des_scalar_y->compute_jacobian(J,joint_indexes);
	std::cout<<"JacobianY:\n"<<J<<std::endl;
	space_des_scalar_z->compute_jacobian(J,joint_indexes);
	std::cout<<"JacobianZ:\n"<<J<<std::endl;


	std::cout<<"======TEST ON ROTATION========:\n"<<std::endl;

	w_R_ee->setInputValues(joint_indexes,inp);

	space_description::Ptr space_des_rot;

	space_des_rot=space_description::Ptr(new rot_space(w_R_ee));

	std::cout<<"======here========:\n"<<std::endl;


	space_des_rot->update_expressions(inp,joint_indexes);

	std::cout<<"======here 2======:\n"<<std::endl;

	Eigen::MatrixXd J2(3,4);
	space_des_rot->compute_jacobian(J2,joint_indexes);
	std::cout<<"Jacobian Rotation:\n"<<J2<<std::endl;
	std::cout<<"======TEST ON ROTATION SIGLE DIRECTIONS========:\n"<<std::endl;
	space_description::Ptr space_des_rot_X(new rot_space(w_R_ee,ROT_X));
	space_description::Ptr space_des_rot_Y(new rot_space(w_R_ee,ROT_Y));
	space_description::Ptr space_des_rot_Z(new rot_space(w_R_ee,ROT_Z));
	space_des_rot_X->update_expressions(inp,joint_indexes);
	space_des_rot_X->compute_jacobian(J,joint_indexes);
	std::cout<<"Jacobian Rotation X:\n"<<J<<std::endl;
	space_des_rot_Y->update_expressions(inp,joint_indexes);
	space_des_rot_Y->compute_jacobian(J,joint_indexes);
	std::cout<<"Jacobian Rotation Y:\n"<<J<<std::endl;
	space_des_rot_Z->update_expressions(inp,joint_indexes);
	space_des_rot_Z->compute_jacobian(J,joint_indexes);
	std::cout<<"Jacobian Rotation Z:\n"<<J<<std::endl;

	std::cout<<"======TEST ON POSITION (in base frame)========:\n"<<std::endl;
	space_description::Ptr space_pos(new pos_space(origin( (w_T_ee))));
	space_pos->update_expressions(inp,joint_indexes);
	space_pos->compute_jacobian(J2,joint_indexes);
	std::cout<<"Jacobian POSITION:\n"<<J2<<std::endl;


	std::cout<<"======TEST ON EE_FRAME========:\n"<<std::endl;




/* note, using the base frame=false can be done using the make constant */
	//space_description::Ptr space_des_rot_own(new rot_space(w_R_ee,FULL_ROTATION,false));
	// this is equivalent to the following two lines
	Expression<Rotation>::Ptr ee_R_ee=(make_constant<Rotation>(w_R_ee))*w_R_ee;
	space_description::Ptr space_des_rot_own(new rot_space(ee_R_ee));
	space_des_rot_own->update_expressions(inp,joint_indexes);
	space_des_rot_own->compute_jacobian(J2,joint_indexes);
	std::cout<<"Jacobian Rotation_in_own:\n"<<J2<<std::endl;

	Expression<Vector>::Ptr ee_P_ee=(make_constant<Rotation>(w_R_ee))*origin( (w_T_ee));
	space_description::Ptr space_pos_own(new pos_space(ee_P_ee));
	space_pos_own->update_expressions(inp,joint_indexes);
	space_pos_own->compute_jacobian(J2,joint_indexes);

	std::cout<<"Jacobian POs_in_own:\n"<<J2<<std::endl;
	JKDL.changeBase(cartpos.M);
	cout<<"Expected Full Jacobian\n" <<JKDL.data<<endl;
}
