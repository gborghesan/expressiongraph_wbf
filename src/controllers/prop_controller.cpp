#include "expressiongraph_wbf/controllers/prop_controller.hpp"
using namespace KDL;
namespace wbf {

	prop_controller::prop_controller(Expression<double>::Ptr _p_meas,
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
	void prop_controller::update_expressions(std::vector<double> q_in)
	{
		p_meas->setInputValues(q_in);
		//std::cout<<"OK"<<std::endl;
	}
	 bool prop_controller::compute_action(Eigen::VectorXd &res)
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
