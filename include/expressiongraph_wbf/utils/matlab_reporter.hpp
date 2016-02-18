#ifndef EXPRESSIONGRAPH_MATLAB_REPORTER_HPP
#define EXPRESSIONGRAPH_MATLAB_REPORTER_HPP
#include <kdl/expressiontree.hpp>
#include <fstream>
#include <sstream>
using namespace KDL;
class matlab_reporter{
public:
	matlab_reporter(const std::string & file_name);


	bool add_var(const std::string & name,Expression<double>::Ptr e);
	bool add_var(const std::string & name,Expression<Vector>::Ptr e);
	bool add_var(const std::string & name,Expression<Rotation>::Ptr e);
	bool add_var(const std::string & name,Expression<Frame>::Ptr e);
	bool add_var(const std::string & name,Expression<Twist>::Ptr e);
	bool add_var(const std::string & name,Expression<Wrench>::Ptr e);

	bool add_var(const std::string & name,double * e);

	void write_headers();

	void write_values();
	void write_footer();
private:
	std::ofstream myfile;
	std::map<std::string,  Expression<double>::Ptr> 	map_expr_double;
	std::map<std::string,  Expression<Vector>::Ptr> 	map_expr_vector;
	std::map<std::string,  Expression<Rotation>::Ptr> 	map_expr_rotation;
	std::map<std::string,  Expression<Frame>::Ptr> 		map_expr_frame;
	std::map<std::string,  Expression<Twist>::Ptr> 		map_expr_twist;
	std::map<std::string,  Expression<Wrench>::Ptr> 	map_expr_wrench;

	std::map<std::string,  double * > 	map_double;

	std::vector<std::string> names;
};


#endif

