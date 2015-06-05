#include "expressiongraph_wbf/solver/controller_base.hpp"
#ifndef EXPRESSIONGRAPH_CONTROLLER_DIRECT_HPP
#define EXPRESSIONGRAPH_CONTROLLER_DIRECT_HPP
using namespace KDL;
namespace wbf {
/*this controller is the most simple, as it
 * simple impose the desired value
 * (generalised force in force resolved schemes)
 * to the given direction*/
class direct_controller:public controller
{
private:
	Expression<double>::Ptr val_des;

public:
	direct_controller(
			Expression<double>::Ptr _val_des);

	void update_expressions(const std::vector<double> & q_in,
			const std::vector<int> & q_index);
	bool compute_action(Eigen::VectorXd&res);
	virtual void update_time(double time, int time_index);

};
};
#endif
