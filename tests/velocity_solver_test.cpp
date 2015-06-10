#include "expressiongraph_wbf/solver/velocity_solver.hpp"

#include "expressiongraph_wbf/controllers/position_controller.hpp"


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





	// Calculate forward position kinematics



	//robot in expression
	Expression<Vector>::Ptr L      = Constant(Vector(0,0,0.2));
	Expression<Vector>::Ptr L0=  Constant(Vector(0,0,0.0));
	Expression<Rotation>::Ptr R0=  Constant(Rotation(KDL::Rotation::Identity()));
	Expression<Frame>::Ptr w_T_l1  =       frame( rot_z(input(1)), L0)*frame( R0, L);
	Expression<Frame>::Ptr w_T_l2 = w_T_l1*frame( rot_x(input(2)), L0)*frame( R0, L);
	Expression<Frame>::Ptr w_T_l3 = w_T_l2*frame( rot_x(input(3)), L0)*frame( R0, L);
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


	//cout<<"Expression w_T_ee\n" <<Te<<endl;


	//allows for changing set-points from outside.
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
	controller::Ptr ctrl_x(new position_controller(w_x_ee,scalar_des,gain));
	controller::Ptr ctrl_y(new position_controller(w_y_ee,scalar_des,gain));
	controller::Ptr ctrl_z(new position_controller(w_z_ee,scalar_des,gain));

	constraint::Ptr c_x(new constraint (space_x,ctrl_x,ctrl_x));
	constraint::Ptr c_y(new constraint (space_y,ctrl_y,ctrl_y));
	constraint::Ptr c_z(new constraint (space_z,ctrl_z,ctrl_z));

	c_x->weight=input(time_index);
	//c_x->weight=Constant(0.12);

	//external inputs (setpoints)






	std::cout<<"\n======TEST ON CONTROLLER 3 DOF (Without time-variant trajectories) ========:"<<endl;

	Eigen::VectorXd qdot_out(4);
	//build solver, without time derivative support
	velocity_solver::Ptr s(new velocity_solver(joint_indexes,time_index));
	Eigen::VectorXd qdot_w(4);
	qdot_w<<1,2,3,4;
	s->setQweights(qdot_w);
	cout <<"ADD CONST:"<<s->addConstraint("pos_x",c_x)<<endl;
	cout <<"ADD CONST:"<<s->addConstraint("pos_y",c_y)<<endl;
	cout <<"ADD CONST:"<<s->addConstraint("pos_z",c_z)<<endl;
	cout <<"PREPARE:"<<s->Prepare()<<endl;

	int count=1;
	for (int i=0;i<1001;i++)
	{
		//cout <<"COMPUTE:"<<s->Compute(q,qdot_out)<<endl;
		s->Compute(q,i,qdot_out);
		for (unsigned int j=0;j<q.size();j++)
			q[j]=q[j]+qdot_out(j)*0.01;
		count--;
		if (count==0)
		{
			count=100;
		cout <<"POSE:"<<origin(w_T_ee)->value()<<endl;
		}
	}
	std::cout<<"\n expected value [0.2 0.2 0.2]"<<endl;

	cout <<"REM CONST:"<<s->RemoveConstraint("pos_z")<<endl;
	scalar_des->setValue(0.3);//value of set-point
	s->Prepare();
	for (int i=0;i<1001;i++)
	{
		//cout <<"COMPUTE:"<<s->Compute(q,qdot_out)<<endl;
		s->Compute(q,i,qdot_out);
		for (unsigned int j=0;j<q.size();j++)
			q[j]=q[j]+qdot_out(j)*0.01;
		count--;
		if (count==0)
		{
			count=100;
			cout <<"POSE:"<<origin(w_T_ee)->value()<<endl;
		}
	}
	std::cout<<"\n expected value [0.3 0.3 ?]"<<endl;
	/*s->addConstraint("pos_y",c_y);
	s->addConstraint("pos_z",c_z);
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
	 */

}
