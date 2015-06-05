#include "expressiongraph_wbf/solver/controller_base.hpp"
#ifndef EXPRESSIONGRAPH_CONTROLLER_PROP_DEADZONE_HPP
#define EXPRESSIONGRAPH_CONTROLLER_PROP_DEADZONE_HPP
using namespace KDL;
namespace wbf {
class position_deadzone_controller:public controller
{
private:
	Expression<double>::Ptr p_meas;
	Expression<double>::Ptr K;
	Expression<double>::Ptr lower_bound;
	Expression<double>::Ptr upper_bound;

public:
	position_deadzone_controller(Expression<double>::Ptr _p_meas ,
			Expression<double>::Ptr _lower_bound,
			Expression<double>::Ptr _upper_bound,
			Expression<double>::Ptr _K
	);

	void update_expressions(const std::vector<double> & q_in,
			const std::vector<int> & q_index);
	bool compute_action(Eigen::VectorXd&res);
	virtual void update_time(double time, int time_index);

};
};
#endif
