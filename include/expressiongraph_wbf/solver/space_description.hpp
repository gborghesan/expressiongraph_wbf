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
	virtual bool compute_jacobian(Eigen::MatrixXd& J_partial,
			const std::vector<int> index_of_variables)=0;
	virtual ~space_description(){};
	int output_size(){return size_of_output;}
	std::string which_control(){return type;}
} ;

//derived classes
class scalar_space:public space_description
{
private:
	Expression<double>::Ptr space_output;


public:
	scalar_space(Expression<double>::Ptr space_output);
	void update_expressions(const std::vector<double>&q,
			const std::vector<int> & q_index);
	 bool compute_jacobian(Eigen::MatrixXd& J_partial,
				const std::vector<int> index_of_variables);
};
class pos_space:public space_description
{
private:
	Expression<Vector>::Ptr space_output;
	Expression<Rotation>::Ptr new_base;
	which_position_type which_direction;
	/*
	 * if base frame is set to false, the space is rotated
	 * 	in the current frame value of space_output
	 *  set to false to provide direct force-velocity in ee frame
	 *  */


public:
	pos_space(Expression<Vector>::Ptr _space_output,
			which_position_type _which_direction=FULL_POSITION,
			Expression<Rotation>::Ptr _new_base=Expression<Rotation>::Ptr());
	void update_expressions(const std::vector<double>&q,
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
	bool base_frame;

public:
	rot_space(Expression<Rotation>::Ptr _space_output,
			which_direction_type _which_direction=FULL_ROTATION,
			bool _base_frame=true);
	void update_expressions(const std::vector<double>&q,
			const std::vector<int> & q_index);
	 bool compute_jacobian(Eigen::MatrixXd& J_partial,
				const std::vector<int> index_of_variables);
};
}
#endif

