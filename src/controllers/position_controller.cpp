#include "expressiongraph_wbf/controllers/position_controller.hpp"
using namespace KDL;
namespace wbf {

	position_controller::position_controller(Expression<double>::Ptr _p_meas,
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
	void position_controller::update_expressions(
			const std::vector<double> & q_in,
			const std::vector<int> & q_index)
	{
		p_meas->setInputValues(q_index,q_in);
		//std::cout<<"OK"<<std::endl;
	}

	void position_controller::update_time(double time, int time_index)
	{
		p_des->setInputValue(time_index,time);
		K->setInputValue(time_index,time);
	}

	 bool position_controller::compute_action(Eigen::VectorXd &res)
	 {
		 if (res.size()!=size_of_output) return false;
		 double x=p_meas->value();
		 double xd=p_des->value();
		 double k=K->value();
		 res[0]=(xd-x)*k;
		 //std::cout<<"x:\t"<<x<<"\txd:\t"<<xd<<"\tk:\t"<<k<<"\nres:\t"<<res[0]<<std::endl;
		 return true;
	 }
};
