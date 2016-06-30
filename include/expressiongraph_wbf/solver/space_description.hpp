#include <string>
#include <vector>
#include <algorithm>
#include <kdl/expressiontree.hpp>

#include <expressiongraph_wbf/utils/common_def.hpp>

#ifndef EXPRESSIONGRAPH_SPACE_DESCRIPTION_HPP
#define EXPRESSIONGRAPH_SPACE_DESCRIPTION_HPP
using namespace KDL;
namespace wbf {


class space_description{
protected:
	std::string type;
	int size_of_output;

public:

	typedef boost::shared_ptr<space_description> Ptr;

	virtual void update_expressions(const std::vector<double>&q,
			const std::vector<int> & q_index)=0;
	/*the index vector should contains only the initial index of the rotation, not all 3 numbers
	these should be at least 3 indices apart
	*/
	virtual void update_expressions_rot(const std::vector<KDL::Rotation>&R,
			const std::vector<int> & q_index)=0;
	virtual bool compute_jacobian(Eigen::MatrixXd& J_partial,
			const std::vector<int> index_of_variables)=0;
	virtual ~space_description(){}



	int  output_size() const{return size_of_output;}
	std::string  which_control() const{return type;}
} ;

/*
 * derived classes
 * */

class scalar_space:public space_description
{
private:
	Expression<double>::Ptr space_output;


public:
	scalar_space(Expression<double>::Ptr space_output);
	//scalar_space& operator=(scalar_space const &s);
	void update_expressions(const std::vector<double>&q,
			const std::vector<int> & q_index);
	void update_expressions_rot(const std::vector<KDL::Rotation>&R,
				const std::vector<int> & q_index);
	 bool compute_jacobian(Eigen::MatrixXd& J_partial,
				const std::vector<int> index_of_variables);
};
class pos_space:public space_description
{
private:
	Expression<Vector>::Ptr space_output;

	/*
	 * if base frame is set to false, the space is rotated
	 * 	in the current frame value of space_output
	 *  set to false to provide direct force-velocity in ee frame
	 *  */


public:
	pos_space(Expression<Vector>::Ptr _space_output);
	//pos_space& operator=(pos_space const &s);

	void update_expressions(const std::vector<double>&q,
			const std::vector<int> & q_index);
	void update_expressions_rot(const std::vector<KDL::Rotation>&R,
				const std::vector<int> & q_index);
	 bool compute_jacobian(Eigen::MatrixXd& J_partial,
				const std::vector<int> index_of_variables);
};



class rot_space:public space_description
{
private:
	Expression<Rotation>::Ptr space_output;
	which_direction_type which_direction;
	/*
	 * if base frame is set to false, the space is rotated
	 * 	in the current frame value of space_output
	 *  set to false to provide direct force-velocity in ee frame
	 *  */


public:
	rot_space(Expression<Rotation>::Ptr _space_output,
			which_direction_type _which_direction);
	rot_space(Expression<Rotation>::Ptr _space_output);
//	rot_space& operator=(rot_space const &s);

	void update_expressions(const std::vector<double>&q,
			const std::vector<int> & q_index);
	void update_expressions_rot(const std::vector<KDL::Rotation>&R,
				const std::vector<int> & q_index);
	 bool compute_jacobian(Eigen::MatrixXd& J_partial,
				const std::vector<int> index_of_variables);
};
}
#endif

