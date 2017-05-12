

#include "expressiongraph_wbf/solver/constraints.hpp"
#include "expressiongraph_wbf/solver/controller_base.hpp"
//#include <map>

//#include <boost/lexical_cast.hpp>
struct forceSolverException : public exception{};
struct constraintsNotPreparedException : public forceSolverException{
	const char * what () const throw (){
		return "The constraint 'prepare' has been called without having been prepared";
	}};
struct constraintsSizeLevelsDifferentException : public forceSolverException{
	const char * what () const throw (){
		return "there is a mismatch in size between constraints and internal size structure."
				"proabably, the constraints has been modified and Prepared externally, ";
	}};
struct wrongNumberOfPriorityException : public forceSolverException{
	const char * what () const throw (){
		return "The constraints must be order in level 1";
	}};
struct wrongQsizeException : public forceSolverException{
	const char * what () const throw (){
		return "Qout passed to compute has the wrong size";
	}};



using namespace wbf;
using namespace std;
using namespace KDL;
namespace wbf {
typedef map <std::string, constraint::Ptr> c_map_type;
class simple_force_solver{
private:

	constraints::Ptr cnstr;

	Eigen::MatrixXd J;
	Eigen::MatrixXd Jt;

	Eigen::VectorXd lambda_des;

	std::vector<unsigned int> constraintsPerPriority;
	std::vector<unsigned int> constraintsPerPriorityCheck;
	std::vector<Eigen::MatrixXd> JPerPriority;
	std::vector<Eigen::VectorXd> LBPerPriority;
	std::vector<Eigen::VectorXd> WyPerPriority;

	Eigen::VectorXd Wq;
	Eigen::VectorXd Wy;
	int n_of_output;
	int n_of_joints;


	bool prepared;
	/** Computes the joint torques.
	 *
	 * Private function that is called after
	 * some configuration from the public interface;
	 * it computes the desired torque to be applied,
	 *  given the desired force computed by constraints.
	 * \param q_in vector of input (scalars).
	 * \param R_in vector of input (Rotation).
	 * \param time time input (used if time_present=true).
	 * \param tau_out computed force to be actuated.
	 * \param time_present is set to true if the time
	 * 			is used (_e.g._ for feed-forward terms of trajectories)
	 * \return true if it ok, otherwise TODO exceptions (not yet documented).
	 *  \sa See also the public \ref ComputeGroupForce "Compute functions"

 * */
	bool Compute(const std::vector<double> &q_in,
			const std::vector<KDL::Rotation> &R_in,
			double time,
			Eigen::VectorXd &tau_out,
			bool time_present);
	simple_force_solver();


public:
	simple_force_solver(constraints::Ptr _cnstr){
		cnstr=_cnstr;
	}
	/**
	 * @brief Prepare
	 * This function must be called each time the costraints are changed, before a compute call
	 * In case of failiture it will raise an exeception
	 */

	void Prepare();

	/**  @name  Compute functions
	 * @anchor ComputeGroupForce
	 * These functions return true if the solver is not prepared
	 * \param q_in input values scalar joint
	 * \param R_in input values rotational joint
	 * \param time current time value
	 * \param qdot_out Return value
	 * \return false if the solver is not prepared. other errors returned by exception.
	 *@{
	 * */

	/** Compute for robot whose joints are __scalar__,
	and some expressions are time dependent.*/
	bool Compute(const std::vector<double> &q_in, double time,
			Eigen::VectorXd &tau_out);
	/** Compute for robot whose joints are __scalar__,
	and no expression is time dependent.*/
	bool Compute(const std::vector<double> &q_in,Eigen::VectorXd &tau_out);
	/** Compute for robot whose joints are __scalar AND rotational__
	 * (_e.g._ free-floating base)
	and some expressions are time dependent.*/
	bool Compute(const std::vector<double> &q_in,
			const std::vector<KDL::Rotation> &R_in,
			double time,
			Eigen::VectorXd &tau_out);
	/** Compute for robot whose joints are __scalar AND rotational__
	 * (_e.g._ free-floating base)
	and no expression is time dependent.*/
	bool Compute(const std::vector<double> &q_in,
			const std::vector<KDL::Rotation> &R_in,
			Eigen::VectorXd &tau_out);
	///@}
	//memory assignment copy...
	Eigen::VectorXd getLastDesiredForce(){return lambda_des;}

	typedef boost::shared_ptr<simple_force_solver> Ptr;//<shared pointer.
};
}
