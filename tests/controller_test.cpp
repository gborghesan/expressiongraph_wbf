#include "expressiongraph_wbf/controllers/prop_controller.hpp"


using namespace wbf;

int main()
{
	Expression<double>::Ptr pos = Constant(2.0)*cos(input(0)) +
			Constant(3.0)*sin(input(1));
	std::vector<double> inp(2);
	inp[0] = 1;
	inp[1] = 2;



	controller::Ptr ctrl(new prop_controller(pos,Constant(0.0),Constant(0.3)));
	ctrl->update_expressions(inp);
	Eigen::VectorXd res(1);
	ctrl->compute_action(res);

	std::cout<<"res:\t"<<res[0]<<std::endl;
	inp[0] = 3;
	ctrl->update_expressions(inp);

	ctrl->compute_action(res);
}
