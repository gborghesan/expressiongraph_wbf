#include "expressiongraph_wbf/solver/controller_base.hpp"
#ifndef EXPRESSIONGRAPH_CONTROLLER_SCALAR_HPP
#define EXPRESSIONGRAPH_CONTROLLER_SCALAR_HPP
using namespace KDL;
namespace wbf {
/*this controller is the most simple, as it
 * simple impose the desired value
 * (generalised force in force resolved schemes)
 * to the given direction
 * */
class direct_scalar_controller:public controller
{
private:
	Expression<double>::Ptr val_des;

public:
	direct_scalar_controller(
			Expression<double>::Ptr _val_des);

	void update_expressions(const std::vector<double> & q_in,
			const std::vector<int> & q_index);
	virtual void update_expressions_rot(const std::vector<KDL::Rotation>&R,
			const std::vector<int> & q_index);
	bool compute_action(Eigen::VectorXd&res);
	void update_time(double time, int time_index);

};


class proportional_scalar_controller:public controller
{
private:
	Expression<double>::Ptr p_des;
	Expression<double>::Ptr p_meas;
	Expression<double>::Ptr K;

public:
	proportional_scalar_controller(Expression<double>::Ptr _p_meas ,
			Expression<double>::Ptr _p_des,
			Expression<double>::Ptr _K
	);

	void update_expressions(const std::vector<double> & q_in,
			const std::vector<int> & q_index);
	virtual void update_expressions_rot(const std::vector<KDL::Rotation>&R,
			const std::vector<int> & q_index);
	bool compute_action(Eigen::VectorXd&res);
	void update_time(double time, int time_index);

};

class proportional_deadzone_scalar_controller:public controller
{
private:
	Expression<double>::Ptr lower_bound;
	Expression<double>::Ptr upper_bound;
	Expression<double>::Ptr p_meas;
	Expression<double>::Ptr K;

public:
	/** Proportional controller with dead-zone.
	 * This controller is proportional outside the bounds,
	 * and zero inside the bounds.
	 *
	 * This controller has the following law:
	 * \f[
	 *  u=\begin{cases}
	 *  (l_b-x)K&\text{if} x<l_b\\
	 *  (u_b-x)K&\text{if} x>u_b\\
	 *  0&\text{otherwise}
	 *  \\\end{cases}
	 *   \f]
	 */
	proportional_deadzone_scalar_controller(
			Expression<double>::Ptr p_meas,
			Expression<double>::Ptr lower_bound,
			Expression<double>::Ptr upper_bound,
			Expression<double>::Ptr K
	);

	void update_expressions(const std::vector<double> & q_in,
			const std::vector<int> & q_index);
	virtual void update_expressions_rot(const std::vector<KDL::Rotation>&R,
			const std::vector<int> & q_index);
	bool compute_action(Eigen::VectorXd&res);
	void update_time(double time, int time_index);

};




class proportional_ff_scalar_controller:public controller
{
private:
	Expression<double>::Ptr p_des;
	Expression<double>::Ptr p_meas;
	Expression<double>::Ptr K;
	Expression<double>::Ptr ff;
	int time_index_ff;

public:
	proportional_ff_scalar_controller(Expression<double>::Ptr _p_meas ,
			Expression<double>::Ptr _p_des,
			Expression<double>::Ptr _K,
			Expression<double>::Ptr _ff,
			int _time_index_ff
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
