

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


	Eigen::MatrixXd J,J_slack;

	constraints::Ptr cnstr;
	std::vector<unsigned int> constraintsPerPriority;
	std::vector<unsigned int> constraintsPerPriorityCheck;
	std::vector<Eigen::MatrixXd> JPerPriority;
	std::vector<Eigen::VectorXd> LBPerPriority;
	std::vector<Eigen::VectorXd> UBPerPriority;
	std::vector<Eigen::VectorXd> WyPerPriority;

	Eigen::VectorXd Wq;
	Eigen::VectorXd Wy;

	int n_of_output;
	int n_of_slack;
	int n_of_joints;
int n_of_variables;
	qpOASES::SQProblem  QP;                 ///< QP Solver

	bool prepared;
	bool firsttime;
	int Compute(const std::vector<double> &q_in, const std::vector<Rotation> &R_in, double time,
			Eigen::VectorXd &qdot_out,bool time_present);
	velocity_solver();

public:
	velocity_solver(
			constraints::Ptr _cnstr,
			double max_cpu_time=0.01,
			double _regularization_factor=0.001,
			int _nWSR=100){
		cnstr=_cnstr;
		nWSR =_nWSR   ;
		cputime=max_cpu_time ;
		regularization_factor=_regularization_factor;
	}


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
	Eigen::VectorXd getLastDesiredLBvel(){return lbA;}
	Eigen::VectorXd getLastDesiredUBvel(){return ubA;}
	typedef boost::shared_ptr<velocity_solver> Ptr;
};
}
