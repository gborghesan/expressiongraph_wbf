#include "expressiongraph_wbf/solver/controller_base.hpp"
#ifndef EXPRESSIONGRAPH_CONTROLLER_PROP_HPP
#define EXPRESSIONGRAPH_CONTROLLER_PROP_HPP
using namespace KDL;
namespace wbf {
class prop_controller:public controller
{
private:
	Expression<double>::Ptr p_des;
	Expression<double>::Ptr p_meas;
	Expression<double>::Ptr K;

public:
	prop_controller(Expression<double>::Ptr _p_meas ,
			Expression<double>::Ptr _p_des,
			Expression<double>::Ptr _K
	);

	void update_expressions(std::vector<double> q_in);
	bool compute_action(Eigen::VectorXd&res);


};
};
#endif
