#include <string>
#include <vector>
#include <algorithm>
#include <kdl/expressiontree.hpp>
#ifndef EXPRESSIONGRAPH_CONTROLLER_BASE_HPP
#define EXPRESSIONGRAPH_CONTROLLER_BASE_HPP
namespace wbf {
class controller{
protected:
	//controller();
	std::string type;
	int size_of_output;
public:

	typedef boost::shared_ptr<controller> Ptr;
/*
 * this function is meant for updating the value of measuring equation
 * in function of joint angles
 */
	virtual void update_expressions(const std::vector<double>&,
			const std::vector<int>& q_index)=0;
	virtual void update_expressions_rot(const std::vector<KDL::Rotation>&R,
			const std::vector<int> & q_index)=0;
	/*
	 * this function is meant for updating the value of desired values
	 * that are expressed as a function of time
	 */
	virtual void update_time(double time, int time_index)=0;
	virtual bool compute_action(Eigen::VectorXd&)=0;
	virtual ~controller(){};
	int output_size(){return size_of_output;}
	std::string which_control(){return type;}
} ;
}
#endif

