#include "expressiongraph_wbf/controllers/position_deadzone_controller.hpp"
using namespace KDL;
namespace wbf {

	position_deadzone_controller::position_deadzone_controller(Expression<double>::Ptr _p_meas,
			Expression<double>::Ptr _lower_bound,
			Expression<double>::Ptr _upper_bound,
			Expression<double>::Ptr _K
			)
	{
		p_meas=_p_meas;
		lower_bound=_lower_bound;
		upper_bound=_upper_bound;
		K=_K;
		type="prop_dead_zone";
		size_of_output=1;
	}
	void position_deadzone_controller::update_expressions(
			const std::vector<double> & q_in,
			const std::vector<int> & q_index)
	{
		p_meas->setInputValues(q_index,q_in);
		//std::cout<<"OK"<<std::endl;
	}

	void position_deadzone_controller::update_time(double time, int time_index)
	{
		K->setInputValue(time_index,time);
		lower_bound->setInputValue(time_index,time);
		upper_bound->setInputValue(time_index,time);
	}


	 bool position_deadzone_controller::compute_action(Eigen::VectorXd &res)
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
		 //std::cout<<"x:\t"<<x<<"\txd:\t"<<xd<<"\tk:\t"<<k<<"\nres:\t"<<res[0]<<std::endl;
		 return true;
	 }
};
