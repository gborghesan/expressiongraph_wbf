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

	virtual void update_expressions(std::vector<double>)=0;
	virtual bool compute_action(Eigen::VectorXd&)=0;
	virtual ~controller(){};
	int output_size(){return size_of_output;}
	std::string which_control(){return type;}
} ;
}
#endif

