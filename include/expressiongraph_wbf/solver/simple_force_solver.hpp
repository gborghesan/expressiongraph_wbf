

#include "expressiongraph_wbf/solver/constraints.hpp"
#include "expressiongraph_wbf/solver/controller_base.hpp"
//#include <map>

using namespace wbf;
using namespace std;
using namespace KDL;
namespace wbf {
typedef map <std::string, constraint::Ptr> c_map_type;
class simple_force_solver{
private:
	std::vector<int> joint_indexes_for_output;
	std::vector<int> joint_indexes_input_scalar;
	std::vector<int> joint_indexes_input_rotation;
	int time_index;
	unsigned int n_of_joints;
	unsigned int n_of_output;
	c_map_type c_map;
	Eigen::MatrixXd J;
	Eigen::MatrixXd Jt;
	Eigen::MatrixXd J1,J3,J6;
	Eigen::VectorXd lambda_des;
	Eigen::VectorXd lambda1,lambda3,lambda6;
	bool prepared;
	int Compute(const std::vector<double> &q_in,
			const std::vector<KDL::Rotation> &R_in,
			double time,
			Eigen::VectorXd &tau_out,
			bool time_present);
public:
	//constructor for input rotational
	simple_force_solver(
			std::vector<int> _joint_indexes_for_output,
			std::vector<int> _joint_indexes_input_scalar,
			std::vector<int> _joint_indexes_input_rotation,
			const int time_index=-1 );
	//constructor for only scalar
	simple_force_solver(
			const std::vector<int>& joint_indexes,
			const int time_index=-1 );
	simple_force_solver();

	void setJointIndex(const std::vector<int>&indx);
	void setJointIndex(const std::vector<int>&indx_out,
			const std::vector<int>&indx_scalar,
			const std::vector<int>&indx_rot);
	void setTimeIndex(const int);
	bool addConstraint(const std::string& name,
			const constraint &c);
	bool addConstraint(const std::string& name,
			const constraint::Ptr &c);
	bool RemoveConstraint(const std::string &s);
	int Prepare();
	int Compute(const std::vector<double> &q_in, double time,
			Eigen::VectorXd &tau_out);
	int Compute(const std::vector<double> &q_in,Eigen::VectorXd &tau_out);

	int Compute(const std::vector<double> &q_in,
			const std::vector<KDL::Rotation> &R_in,
			double time,
			Eigen::VectorXd &tau_out);
	int Compute(const std::vector<double> &q_in,
			const std::vector<KDL::Rotation> &R_in,
			Eigen::VectorXd &tau_out);
	//memory assignment copy...
	Eigen::VectorXd getLastDesiredForce(){return lambda_des;};

	typedef boost::shared_ptr<simple_force_solver> Ptr;
};
}
