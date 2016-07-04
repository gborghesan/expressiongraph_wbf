

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
	 * \return 1 if it ok, otherwise error code (not yet documented).
	 * */
	int Compute(const std::vector<double> &q_in,
			const std::vector<KDL::Rotation> &R_in,
			double time,
			Eigen::VectorXd &tau_out,
			bool time_present);
	simple_force_solver();


public:
	simple_force_solver(constraints::Ptr _cnstr){
		cnstr=_cnstr;
	}
	///
	/// \brief Prepare
	/// \return 1 if ok TODO put execeptions
	///
	int Prepare();

/** @name Public interfaces for computing the force
 *
 * All these functions calls #Compute(const std::vector<double> &q_in, 	const std::vector<KDL::Rotation> &R_in,double time, Eigen::VectorXd &tau_out,bool time_present) .
 * the indexes used for the ordering
 * of the vectors (both input and output) must be the same given to the constructor.
 * @{
 */

	/** Compute for robot whose joints are __scalar__,
	and some expressions are time dependent.*/
	int Compute(const std::vector<double> &q_in, double time,
			Eigen::VectorXd &tau_out);
	/** Compute for robot whose joints are __scalar__,
	and no expression is time dependent.*/
	int Compute(const std::vector<double> &q_in,Eigen::VectorXd &tau_out);
	/** Compute for robot whose joints are __scalar AND rotational__
	 * (_e.g._ free-floating base)
	and some expressions are time dependent.*/
	int Compute(const std::vector<double> &q_in,
			const std::vector<KDL::Rotation> &R_in,
			double time,
			Eigen::VectorXd &tau_out);
	/** Compute for robot whose joints are __scalar AND rotational__
	 * (_e.g._ free-floating base)
	and no expression is time dependent.*/
	int Compute(const std::vector<double> &q_in,
			const std::vector<KDL::Rotation> &R_in,
			Eigen::VectorXd &tau_out);
	///@}
	//memory assignment copy...
	Eigen::VectorXd getLastDesiredForce(){return lambda_des;}

	typedef boost::shared_ptr<simple_force_solver> Ptr;//<shared pointer.
};
}
