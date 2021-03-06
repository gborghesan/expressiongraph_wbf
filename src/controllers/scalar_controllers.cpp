#include "expressiongraph_wbf/controllers/scalar_controllers.hpp"
using namespace KDL;
namespace wbf {

direct_scalar_controller::direct_scalar_controller(
		Expression<double>::Ptr _val_des
)
{
	val_des=_val_des;
	type="direct";
	size_of_output=1;
}
void direct_scalar_controller::update_expressions(
		const std::vector<double> & q_in,
		const std::vector<int> & q_index)
{
	val_des->setInputValues(q_index,q_in);
}

void direct_scalar_controller::update_expressions_rot(const std::vector<KDL::Rotation>&R,
		const std::vector<int> & q_index)
{
	val_des->setInputValues(q_index,R);
}

void direct_scalar_controller::update_time(double time, int time_index)
{
	val_des->setInputValue(time_index,time);
}

bool direct_scalar_controller::compute_action(Eigen::VectorXd &res)
{
	if (res.size()!=size_of_output) return false;
	res[0]=val_des->value();
	return true;
}


proportional_scalar_controller::proportional_scalar_controller(Expression<double>::Ptr _p_meas,
		Expression<double>::Ptr _p_des,
		Expression<double>::Ptr _K
		)
{
	p_des=_p_des;
	p_meas=_p_meas;
	K=_K;
	type="prop";
	size_of_output=1;
}
void proportional_scalar_controller::update_expressions(
		const std::vector<double> & q_in,
		const std::vector<int> & q_index)
{
	p_meas->setInputValues(q_index,q_in);
	p_des->setInputValues(q_index,q_in);
	K->setInputValues(q_index,q_in);
}
void proportional_scalar_controller::update_expressions_rot(const std::vector<KDL::Rotation>&R,
		const std::vector<int> & q_index)
{
	p_meas->setInputValues(q_index,R);
	p_des->setInputValues(q_index,R);
	K->setInputValues(q_index,R);
};

void proportional_scalar_controller::update_time(double time, int time_index)
{
	p_meas->setInputValue(time_index,time);
	p_des->setInputValue(time_index,time);
	K->setInputValue(time_index,time);
}

 bool proportional_scalar_controller::compute_action(Eigen::VectorXd &res)
 {
	 if (res.size()!=size_of_output) return false;
	 double x=p_meas->value();
	 double xd=p_des->value();
	 double k=K->value();
	 res[0]=(xd-x)*k;

	 return true;
 }




proportional_deadzone_scalar_controller::proportional_deadzone_scalar_controller(
		Expression<double>::Ptr _p_meas,
		Expression<double>::Ptr _lower_bound,
		Expression<double>::Ptr _upper_bound,
		Expression<double>::Ptr _K
		)
{
	p_meas=_p_meas;
	lower_bound=_lower_bound;
	upper_bound=_upper_bound;
	K=_K;
	type="proportional_deadzone_scalar";
	size_of_output=1;
}
void proportional_deadzone_scalar_controller::update_expressions(
		const std::vector<double> & q_in,
		const std::vector<int> & q_index)
{
	p_meas->setInputValues(q_index,q_in);
	lower_bound->setInputValues(q_index,q_in);
	upper_bound->setInputValues(q_index,q_in);
	K->setInputValues(q_index,q_in);
}
void proportional_deadzone_scalar_controller::update_expressions_rot(const std::vector<KDL::Rotation>&R,
		const std::vector<int> & q_index)
{
	p_meas->setInputValues(q_index,R);
	lower_bound->setInputValues(q_index,R);
	upper_bound->setInputValues(q_index,R);
	K->setInputValues(q_index,R);
};

void proportional_deadzone_scalar_controller::update_time(double time, int time_index)
{
	p_meas->setInputValue(time_index,time);
	lower_bound->setInputValue(time_index,time);
	upper_bound->setInputValue(time_index,time);
	K->setInputValue(time_index,time);
}


 bool proportional_deadzone_scalar_controller::compute_action(Eigen::VectorXd &res)
 {
	 if (res.size()!=size_of_output) return false;
	 double x=p_meas->value();
	 double l_bound=lower_bound->value();
	 double u_bound=upper_bound->value();
	 if(l_bound>u_bound) return false;
	 double k=K->value();

	 if (x<l_bound)
		 res[0]=(l_bound-x)*k;
	 else if(x>u_bound)
		 res[0]=(u_bound-x)*k;
	 else
		 res[0]=0;
	 return true;
 }



	proportional_ff_scalar_controller::proportional_ff_scalar_controller(
			Expression<double>::Ptr _p_meas,
			Expression<double>::Ptr _p_des,
			Expression<double>::Ptr _K,
			Expression<double>::Ptr _ff,
			int _time_index
			)
	{
		p_des=_p_des;
		p_meas=_p_meas;
		K=_K;
		ff=_ff;
		type="prop_ff";
		time_index_ff=_time_index;
		size_of_output=1;
	}

	void proportional_ff_scalar_controller::update_expressions(
			const std::vector<double> & q_in,
			const std::vector<int> & q_index)
	{
		p_meas->setInputValues(q_index,q_in);
		ff->setInputValues(q_index,q_in);
	}
	void proportional_ff_scalar_controller::update_expressions_rot(const std::vector<KDL::Rotation>&R,
			const std::vector<int> & q_index)
	{
		p_meas->setInputValues(q_index,R);
		ff->setInputValues(q_index,R);
	};
	void proportional_ff_scalar_controller::update_time(double time, int time_index)
	{
		p_des->setInputValue(time_index,time);
		K->setInputValue(time_index,time);
		ff->setInputValue(time_index,time);

	}

	 bool proportional_ff_scalar_controller::compute_action(Eigen::VectorXd &res)
	 {
		 if (res.size()!=size_of_output) return false;

		 double x=p_meas->value();
		 double xd=p_des->value();

		 double k=K->value();
		 double _ff=ff->value();
		 double xdd=p_des->derivative(time_index_ff);
		 res[0]=(xd-x)*k+xdd*_ff;
		//std::cout<<"x:\t"<<x<<"\txd:\t"<<xd
		//		 <<"\tk:\t"<<k
		//		 <<"\txdd:\t"<<xdd
		//		 <<"\nres:\t"<<res[0]<<std::endl;
		 return true;
	 }

};
