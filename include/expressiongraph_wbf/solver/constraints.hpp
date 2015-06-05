#ifndef _CONSTRAINTS_HPP_
#define _CONSTRAINTS_HPP_

#include <kdl/expressiontree.hpp>
#include <expressiongraph_wbf/solver/controller_base.hpp>
#include <expressiongraph_wbf/solver/space_description.hpp>
namespace wbf{
typedef struct constraint
{
	space_description::Ptr space;
	controller::Ptr ctrl;
	unsigned int priority_level;


	typedef boost::shared_ptr<constraint> Ptr;
	constraint(
			space_description::Ptr _space,
			controller::Ptr _ctrl,
			unsigned int _priority_level=1){
		space=_space;
		ctrl=_ctrl;
		priority_level=_priority_level;
	};
	constraint(const constraint&c){
		space=c.space;
		ctrl=c.ctrl;
		priority_level=c.priority_level;
	};

}constraint;

bool check_constraint_validity(const constraint & cs)
{
	if (cs.ctrl->output_size()!=cs.space->output_size()) return false;
	return true;
};
bool check_constraint_validity(const constraint::Ptr & cs)
{
	return check_constraint_validity(*cs);

};

};//namespace
#endif
