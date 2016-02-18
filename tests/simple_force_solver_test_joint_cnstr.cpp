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

	std::vector<double> q(4);
	q[0] = 1;
	q[1] = 1.5;
	q[2] = -0.5;
	q[3] = 2;

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
		jointpositions(i)=q[i];

	KDL::Frame cartpos;
	KDL::Jacobian JKDL(nj);

	// Calculate forward position kinematics

	fksolver.JntToCart(jointpositions,cartpos);
	Jsolver.JntToJac(jointpositions,JKDL);



	// Calculate forward position kinematics



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

	w_T_ee->setInputValues(joint_indexes,q);
	KDL::Frame Te=w_T_ee->value();

	//cout<<"Expression w_T_ee\n" <<Te<<endl;


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

	space_description::Ptr space_x=space_description::Ptr(new scalar_space(w_x_ee));
	space_description::Ptr space_y(new scalar_space(w_y_ee));
	space_description::Ptr space_z(new scalar_space(w_z_ee));

	space_description::Ptr space_q4(new scalar_space(input(4)));


	Expression<double>::Ptr gain=Constant(K);
	//controller::Ptr ctrl_x(new proportional_scalar_controller(w_x_ee,scalar_des,gain));
	controller::Ptr ctrl_x=controller::Ptr(new proportional_scalar_controller(w_x_ee,scalar_des,gain));

	controller::Ptr ctrl_y(new proportional_scalar_controller(w_y_ee,scalar_des,gain));
	controller::Ptr ctrl_z(new proportional_scalar_controller(w_z_ee,scalar_des,gain));
	controller::Ptr ctrl_q(new proportional_scalar_controller(input(4),Constant(0.0),gain));

	constraint::Ptr c_x(new constraint (space_x,ctrl_x));
	constraint::Ptr c_y(new constraint (space_y,ctrl_y));
	constraint::Ptr c_z(new constraint (space_z,ctrl_z));
	constraint::Ptr c_q(new constraint (space_q4,ctrl_q));




	//external inputs (setpoints)






	std::cout<<"\n======Expected values========:"<<endl;
	cout<<"w_p_ee\n" <<cartpos.p<<endl;

	Eigen::VectorXd wrench(6);
	wrench(0)=-cartpos.p.x()*K;
	wrench(1)=-cartpos.p.y()*K;
	wrench(2)=-cartpos.p.z()*K;
	wrench(3)=wrench(4)=wrench(5)=0;
	cout<<"desired wrench\n" <<wrench.transpose()<<endl;
	Eigen::VectorXd tau(4);
	tau=JKDL.data.transpose()*wrench;
	tau(3)=+(0.0-q[3])*K;
	cout<<"expected tau\n" <<tau.transpose()<<endl;

	std::cout<<"\n======TEST ON CONTROLLER 3 DOF (Without time-variant trajectories) ========:"<<endl;

	Eigen::VectorXd tau_out(4);
	//build solver, without time derivative support
	simple_force_solver::Ptr s(new simple_force_solver(joint_indexes));
	s->addConstraint("pos_x",c_x);
	s->addConstraint("pos_y",c_y);
	s->addConstraint("pos_z",c_z);
	s->addConstraint("pos_q",c_q);
	s->Prepare();
	int res=s->Compute(q,tau_out);

	cout<<"Expression w_p_ee\n" <<Te.p<<endl;
	cout<<"Compute returned: "<<res<<endl;
	cout<<"Joint Torque is:\n "<<tau_out.transpose()<<endl;

	std::cout<<"\n======REMOVE CONSTRAINT ========:"<<endl;
	std::cout<< "returned: "<<s->RemoveConstraint("pos_x")<<endl;
	std::cout<<"\n======REMOVE NON EXISTING CONSTRAINT ========:"<<endl;
	std::cout<< "returned: "<<s->RemoveConstraint("pos_x")<<endl;
	std::cout<<"\n======CALL COMPUTE WITHOUT PREPARE ========:"<<endl;
	std::cout<< "compute returned: "<<s->Compute(q,tau_out)<<endl;
	std::cout<<"\n======CALL COMPUTE AFTER PREPARE ========:"<<endl;
	std::cout<< "prepare returned: "<<s->Prepare()<<endl;
	std::cout<< "compute returned: "<<s->Compute(q,tau_out)<<endl;


}
