

#include "expressiongraph_wbf/solver/constraints.hpp"
#include "expressiongraph_wbf/solver/controller_base.hpp"

#include <qpOASES/SQProblem.hpp>
//#include <map>

using namespace wbf;
using namespace std;
using namespace KDL;
namespace wbf {

class velocity_solver{
private:
	std::vector<int> joint_indexes;
	int time_index;
	unsigned int n_of_joints;
	unsigned int n_of_output;
	c_map_type c_map;
	Eigen::MatrixXd J;
	Eigen::MatrixXd Jt;
	Eigen::MatrixXd J1,J3,J6;
	Eigen::VectorXd lambda_des;
	Eigen::VectorXd lambda1,lambda3,lambda6;


	qpOASES::SQProblem  QP;                 ///< QP Solver

	bool prepared;
/*	int Compute(const std::vector<double> &q_in, double time,
			Eigen::VectorXd &tau_out,bool time_present);*/
public:
	velocity_solver(
			const std::vector<int>& joint_indexes,
			const int time_index=-1 );
	velocity_solver();
/*
	void setJointIndex(const std::vector<int>&indx);
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

	//memory assignment copy...
	Eigen::VectorXd getLastDesiredForce(){return lambda_des;};
*/
	typedef boost::shared_ptr<velocity_solver> Ptr;
};
}
