

#include "expressiongraph_wbf/solver/constraints.hpp"
#include "expressiongraph_wbf/solver/controller_base.hpp"

#include <qpOASES/SQProblem.hpp>
//#include <map>

using namespace wbf;
using namespace std;
using namespace KDL;

struct velocitySolverException : public exception{};
struct constraintsNotPreparedException : public velocitySolverException{
	const char * what () const throw (){
		return "The constraint 'preapare' has been called without having been prepared";
	}};
struct wrongNumberOfPriorityException : public velocitySolverException{
	const char * what () const throw (){
		return "The constraints must be order in level 1 and 2";
	}};
struct wrongQsizeException : public velocitySolverException{
	const char * what () const throw (){
		return "Qout passed to compute has the wrong size";
	}};


namespace wbf {
#define HUGE_VALUE 1e20


/**
 * @brief The velocity_solver class
 * This class implements a velocity solver using qpOases.
 * At the moment the solver accepts constraints at level 1 (hard) and 2 (soft), (not 0!)
 * At level 2, also weights are used.
 */
class velocity_solver{
private:

	/** @name  Variable used in qpOases
	 *@{
	 * */
	/// max number of interation
	int    nWSR    ;
	/// max time of execution
	double cputime ;
	/// Weight scaling of joint velocity w.r.t. slack variables in Hessian matrix
	double regularization_factor;
	/// constraint matrix  A
	Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> A;
	/// lower bound of the state (not used)
	Eigen::VectorXd     lb;
	/// upper bound of the state (not used)
	Eigen::VectorXd     ub;
	/// lower bound (lba <= A*x)
	Eigen::VectorXd     lbA;
	/// upper bound (A*x <= ubA)
	Eigen::VectorXd     ubA;
	/// optimize x'*H*x + g'*x
	Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> H;
	/// optimize x'*H*x + g'*x (not used)
	Eigen::VectorXd     g;
	///results from the optimalisation: joint and slack variables
	Eigen::VectorXd     solution;
	///@}

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
	bool Compute(const std::vector<double> &q_in, const std::vector<Rotation> &R_in, double time,
			Eigen::VectorXd &qdot_out,bool time_present);
	velocity_solver();

public:
	/**
	 * @brief velocity_solver constructior
	 * @param _cnstr The constraints to be used. only level one and 2 costraints are accepted
	 * @param max_cpu_time Max time for which QP will be run
	 * @param _regularization_factor scale between velocity of q and slack variables
	 * @param _nWSR Max number of iteration for which QP will be run
	 */
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

	/**
	 * @brief Prepare
	 * This function must be called each time the costraints are changed, before a compute call
	 * In case of failiture it will raise an exeception
	 */

	void Prepare();
	///@}
	/** @name  Compute functions
	 * These functions return true if the solver is not prepared
	 * \param q_in input values scalar joint
	 * \param R_in input values rotational joint
	 * \param time current time value
	 * \param qdot_out Return value
	 * \return false if the solver is not prepared. other errors returned by exception.
	 *@{
	 * */
	bool Compute(const std::vector<double> &q_in, double time,
			Eigen::VectorXd &qdot_out);
	bool Compute(const std::vector<double> &q_in,Eigen::VectorXd &qdot_out);
	bool Compute(const std::vector<double> &q_in,
			const std::vector<Rotation> &R_in,
			double time,
			Eigen::VectorXd &qdot_out);
	bool Compute(const std::vector<double> &q_in,
			const std::vector<Rotation> &R_in,
			Eigen::VectorXd &qdot_out);
	///@}
	//memory assignment copy...
	Eigen::VectorXd getLastDesiredLBvel(){return lbA;}
	Eigen::VectorXd getLastDesiredUBvel(){return ubA;}
	typedef boost::shared_ptr<velocity_solver> Ptr;
};
}
