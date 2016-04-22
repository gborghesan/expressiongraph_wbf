#include <string>
#include <vector>
#include <algorithm>
#include <kdl/expressiontree.hpp>
#ifndef EXPRESSIONGRAPH_CONTROLLER_BASE_HPP
#define EXPRESSIONGRAPH_CONTROLLER_BASE_HPP
namespace wbf {
class controller{
protected:
	std::string type;
	int size_of_output;
public:

	typedef boost::shared_ptr<controller> Ptr;
  /**@name Update expression functions.
   *  */
    //@{
	/// update scalar inputs.
	virtual void update_expressions(const std::vector<double>&,
			const std::vector<int>& q_index)=0;
	/// update rotational inputs.
	virtual void update_expressions_rot(const std::vector<KDL::Rotation>&R,
			const std::vector<int> & q_index)=0;
	   //@}
	/**
	 * this function is meant for updating the value of desired values
	 * that are expressed as a function of time.
	 */
	virtual void update_time(double time, int time_index)=0;
	/**
	 * Computes and returns the control action.
	 * \param action is the computed return action is returned by reference.
	 * \return true if all is ok.
	 *
	 *
	 */
	virtual bool compute_action(Eigen::VectorXd& action)=0;
	virtual ~controller(){}
	int output_size()const{return size_of_output;}
	std::string which_control()const{return type;}
} ;
}
#endif

