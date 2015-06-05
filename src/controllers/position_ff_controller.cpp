#include "expressiongraph_wbf/controllers/position_ff_controller.hpp"
using namespace KDL;
namespace wbf {

	position_ff_controller::position_ff_controller(
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
	void position_ff_controller::update_expressions(
			const std::vector<double> & q_in,
			const std::vector<int> & q_index)
	{
		p_meas->setInputValues(q_index,q_in);
		//std::cout<<"OK"<<std::endl;
	}

	void position_ff_controller::update_time(double time, int time_index)
	{
		p_des->setInputValue(time_index,time);
		K->setInputValue(time_index,time);
		ff->setInputValue(time_index,time);

	}

	 bool position_ff_controller::compute_action(Eigen::VectorXd &res)
	 {
		 if (res.size()!=size_of_output) return false;

		 double x=p_meas->value();
		 double xd=p_des->value();

		 double k=K->value();
		 double _ff=ff->value();
		 double xdd=p_des->derivative(time_index_ff);
		 res[0]=(xd-x)*k+xdd*_ff;
		/*std::cout<<"x:\t"<<x<<"\txd:\t"<<xd
				 <<"\tk:\t"<<k
				 <<"\txdd:\t"<<xdd
				 <<"\nres:\t"<<res[0]<<std::endl;*/
		 return true;
	 }
};
