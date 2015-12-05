#include "expressiongraph_wbf/solver/controller_base.hpp"
#include <expressiongraph_wbf/utils/common_def.hpp>
#ifndef EXPRESSIONGRAPH_CONTROLLER_ROT_HPP
#define EXPRESSIONGRAPH_CONTROLLER_ROT_HPP
using namespace KDL;
namespace wbf {
class proportional_rotation_controller:public controller
{
private:
	Expression<Rotation>::Ptr p_des;
	Expression<Rotation>::Ptr p_meas;
	Expression<double>::Ptr K;
	which_direction_type which_direction;

public:
	proportional_rotation_controller(Expression<Rotation>::Ptr _p_meas ,
			Expression<Rotation>::Ptr _p_des,
			Expression<double>::Ptr _K,
			which_direction_type _type=FULL_ROTATION
	);

	void update_expressions(const std::vector<double> & q_in,
			const std::vector<int> & q_index);
	void update_expressions_rot(const std::vector<KDL::Rotation>&R,
			const std::vector<int> & q_index);
	bool compute_action(Eigen::VectorXd&res);
	virtual void update_time(double time, int time_index);

};
};
#endif
