

#include "expressiongraph_wbf/solver/constraints.hpp"
#include "expressiongraph_wbf/solver/controller_base.hpp"

#include <qpOASES/SQProblem.hpp>
//#include <map>

using namespace wbf;
using namespace std;
using namespace KDL;
namespace wbf {
#define HUGE_VALUE 1e20
class velocity_solver{
private:
	std::vector<int> joint_indexes_for_output;
	std::vector<int> joint_indexes_input_scalar;
	std::vector<int> joint_indexes_input_rotation;
	int time_index;
	unsigned int n_of_joints;
	unsigned int n_of_slack;
	unsigned int n_of_variables;
	unsigned int n_of_output;
	c_map_type c_map;

	int    nWSR    ;
	double cputime ;
	double regularization_factor;

	Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> A;    ///< constraint matrix  A
	Eigen::VectorXd     lb;                 ///< lb <= x
	Eigen::VectorXd     ub;                 ///< x <= ub
	Eigen::VectorXd     lbA;                ///< lba <= A*x
	Eigen::VectorXd     ubA;                ///< A*x <= ubA
	Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> H;    ///< optimize x'*H*x + g'*x
	Eigen::VectorXd     g;                  ///< optimize x'*H*x + g'*x
	Eigen::VectorXd     solution;           ///< results from the optimalisation: joint and feature velocities, slack variables


	Eigen::MatrixXd J1,J3,J6,J,J_slack;

	Eigen::VectorXd ydotlb1,ydotlb3,ydotlb6;
	Eigen::VectorXd ydotub1,ydotub3,ydotub6;
	Eigen::VectorXd Wqdiag;

	qpOASES::SQProblem  QP;                 ///< QP Solver

	bool prepared;
	bool firsttime;
	int Compute(const std::vector<double> &q_in, const std::vector<Rotation> &R_in, double time,
			Eigen::VectorXd &qdot_out,bool time_present);
public:
	velocity_solver(
			std::vector<int> _joint_indexes_for_output,
			std::vector<int> _joint_indexes_input_scalar,
			std::vector<int> _joint_indexes_input_rotation,
			const int time_index=-1,
			double max_cpu_time=0.01,
			double regularization_factor=0.001,
			int nWSR=10);
	velocity_solver(
			const std::vector<int>& joint_indexes,
			const int time_index=-1,
			double max_cpu_time=0.01,
			double regularization_factor=0.001,
			int nWSR=10);
	velocity_solver();

	void setJointIndex(const std::vector<int>&indx);
	void setJointIndex(const std::vector<int>&indx_out,
			const std::vector<int>&indx_scalar,
			const std::vector<int>&indx_rot);
	void setTimeIndex(const int);

	void setQweights(const Eigen::VectorXd& Wqdiag);
	bool addConstraint(const std::string& name,
			const constraint &c);
	bool addConstraint(const std::string& name,
			const constraint::Ptr &c);
	bool RemoveConstraint(const std::string &s);
	int Prepare();
	int Compute(const std::vector<double> &q_in, double time,
			Eigen::VectorXd &qdot_out);
	int Compute(const std::vector<double> &q_in,Eigen::VectorXd &qdot_out);
	int Compute(const std::vector<double> &q_in,
			const std::vector<Rotation> &R_in,
			double time,
			Eigen::VectorXd &qdot_out);
	int Compute(const std::vector<double> &q_in,
			const std::vector<Rotation> &R_in,
			Eigen::VectorXd &qdot_out);

	//memory assignment copy...
	Eigen::VectorXd getLastDesiredLBvel(){return lbA;};
	Eigen::VectorXd getLastDesiredUBvel(){return ubA;};
	typedef boost::shared_ptr<velocity_solver> Ptr;
};
}
