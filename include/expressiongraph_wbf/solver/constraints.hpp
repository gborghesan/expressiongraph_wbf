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
	controller::Ptr ctrl_sec;
	unsigned int priority_level;
	Expression<double>::Ptr weight;


	typedef boost::shared_ptr<constraint> Ptr;
	constraint(
			space_description::Ptr _space,
			controller::Ptr _ctrl,
			unsigned int _priority_level=2,
			Expression<double>::Ptr _weight=Constant(1.0)
	){
		space=_space;
		ctrl=_ctrl;
		priority_level=_priority_level;
		weight=_weight;
		ctrl_sec=_ctrl;
	};
	constraint(
			space_description::Ptr _space,
			controller::Ptr _ctrl,
			controller::Ptr _ctrl_sec,
			unsigned int _priority_level=2,
			Expression<double>::Ptr _weight=Constant(1.0)
	){
		space=_space;
		ctrl=_ctrl;
		priority_level=_priority_level;
		weight=_weight;
		ctrl_sec=_ctrl_sec;
	};
	constraint(const constraint&c){
		space=c.space;
		ctrl=c.ctrl;
		priority_level=c.priority_level;
		weight=c.weight;
		ctrl_sec=c.ctrl_sec;
	};

}constraint;
typedef std::map <std::string, constraint::Ptr> c_map_type;
inline bool check_constraint_validity(const constraint & cs)
{
	if (cs.ctrl->output_size()!=cs.space->output_size()) return false;
	if (cs.ctrl_sec)
		if(cs.ctrl->output_size()!=cs.ctrl_sec->output_size())
			return false;
	return true;
};
inline bool check_constraint_validity(const constraint::Ptr & cs)
{
	return check_constraint_validity(*cs);

};



};//namespace
#endif
