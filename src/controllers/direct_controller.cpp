#include "expressiongraph_wbf/controllers/direct_controller.hpp"
using namespace KDL;
namespace wbf {

direct_controller::direct_controller(
		Expression<double>::Ptr _val_des
)
{
	val_des=_val_des;
	type="direct";
	size_of_output=1;
}
void direct_controller::update_expressions(
		const std::vector<double> & q_in,
		const std::vector<int> & q_index)
{}

void direct_controller::update_time(double time, int time_index)
{
	val_des->setInputValue(time_index,time);
}

bool direct_controller::compute_action(Eigen::VectorXd &res)
{
	if (res.size()!=size_of_output) return false;
	res[0]=val_des->value();
	//std::cout<<"x:\t"<<x<<"\txd:\t"<<xd<<"\tk:\t"<<k<<"\nres:\t"<<res[0]<<std::endl;
	return true;
}
};
