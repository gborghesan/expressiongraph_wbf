

#include "expressiongraph_wbf/solver/constraints.hpp"
#include "expressiongraph_wbf/solver/controller_base.hpp"

#include <qpOASES/SQProblem.hpp>
#include <boost/lexical_cast.hpp>
//#include <map>

using namespace wbf;
using namespace std;
using namespace KDL;

struct velocitySolverException : public exception{};
struct constraintsNotPreparedException : public velocitySolverException{
	const char * what () const throw (){
		return "The constraint 'prepare' has been called without having been prepared";
	}};
struct constraintsSizeLevelsDifferentException : public velocitySolverException{
	const char * what () const throw (){
		return "there is a mismatch in size between constraints and internal size structure."
				"proabably, the constraints has been modified and Prepared externally, ";
	}};
struct wrongNumberOfPriorityException : public velocitySolverException{
	const char * what () const throw (){
		return "The constraints must be order in level 1 and 2";
	}};
struct wrongQsizeException : public velocitySolverException{
	const char * what () const throw (){
		return "Qout passed to compute has the wrong size";
	}};

struct QPoasesRaisedAnErrorException : public velocitySolverException
{
  std::string m_msg;
  ~QPoasesRaisedAnErrorException() throw () {} // Updated
  QPoasesRaisedAnErrorException(const int& error):
	  m_msg(std::string("QPoases Returned = ")  +  boost::lexical_cast<std::string>(error)){}
 virtual const char* what() const throw(){
	return m_msg.c_str();
  }
};




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
///Pointer to te constraints
	constraints::Ptr cnstr;
	/** @name  Vectors divided per priority, used to get values from the constraints.
	 *@{
	 * */
	std::vector<unsigned int> constraintsPerPriority;
	std::vector<unsigned int> constraintsPerPriorityCheck;
	std::vector<Eigen::MatrixXd> JPerPriority;
	std::vector<Eigen::VectorXd> LBPerPriority;
	std::vector<Eigen::VectorXd> UBPerPriority;
	std::vector<Eigen::VectorXd> WyPerPriority;
	///@}
	/// Joint weights (diagonal of the matrix)
	Eigen::VectorXd Wq;
	/// Constraint weights (diagonal of the matrix, for the 2nd LvL only)
	Eigen::VectorXd Wy;

	int n_of_output;
	int n_of_slack;
	int n_of_joints;
	int n_of_variables;
	///QP Solver
	qpOASES::SQProblem  QP;

	bool prepared;
	bool firsttime;
	/**
	 * @brief Compute Private function that is used by the public interface  \ref ComputeGroup "compute functions"
	 * Errors are comunicated by exception
	 * \param q_in input values scalar joint
	 * \param R_in input values rotational joint
	 * \param time current time value
	 * \param qdot_out Return value
	 * @param time_present boolean that enalbe the time derivation
	 * @return true if is prepared
	 *  \sa See also the public \ref ComputeGroupVelocity "Compute functions"
	 */
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

	/**  @name  Compute functions
	 * @anchor ComputeGroupVelocity
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
