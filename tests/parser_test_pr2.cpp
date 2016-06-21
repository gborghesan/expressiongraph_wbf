#include "expressiongraph_wbf/solver/constraints.hpp"
#include "expressiongraph_wbf/utils/Urdf2Expr.hpp"


#include <ros/package.h>

using namespace wbf;
using namespace std;
using namespace KDL;
int main()
{
	//set input values

	std::string path = ros::package::getPath("expressiongraph_wbf");


	Urdf2Expressions u;

	if(!u.readFromFile(path+"/urdf/pr2.urdf"))
	{
		cout<<"cannot find/parse urdf/pr2.urdf"<<endl;
		return -1;
	}

	std::vector<string> names;
	cout<<"+++++ get names ++++++"<<endl;
	u.getAllJointNames(names);
	for (std::vector<string>::iterator it=names.begin();it!=names.end();it++)
		cout<<"\t"<<*it<<endl;
	cout<<"+++++ auto generate map ++++++"<<endl;
	u.generateJointMap(1);//start to generate from index 1, index zero is left empty
	for (std::map<string,unsigned int>::iterator it=u.joint_map.begin();it!=u.joint_map.end();it++)
		cout<<"\t"<<it->first<<"\t"<<it->second<<endl;

	cout<<"+++++ join properties ++++++"<<endl;
	for (int i=0;i<u.j_props.size();i++)
	{
		cout<<"name\t"<<u.j_props[i].name<<endl;
		cout<<"\ttype\t"<<u.j_props[i].j_type<<endl;
		cout<<"\tmin pos\t"<<u.j_props[i].min_pos<<endl;
		cout<<"\tmax pos\t"<<u.j_props[i].max_pos<<endl;
		cout<<"\tmax vel\t"<<u.j_props[i].max_vel<<endl;
		cout<<"\tmax eff\t"<<u.j_props[i].max_effort<<endl;
		cout<<endl;
	}

	for (int i=0;i<u.j_props.size();i++)
		{
			cout<<"name\t"<<u.j_props[i].name<<endl;
			cout<<"\ttype\t"<<u.j_props[i].j_type<<endl;
			cout<<"\tmin pos\t"<<u.j_props[i].min_pos<<endl;
			cout<<"\tmax pos\t"<<u.j_props[i].max_pos<<endl;
			cout<<"\tmax vel\t"<<u.j_props[i].max_vel<<endl;
			cout<<"\tmax eff\t"<<u.j_props[i].max_effort<<endl;
			cout<<endl;
		}
	/*std::ofstream of("joint_limits.lua");
	of<<"joint_table={}"<<endl;
	for (int i=0;i<u.j_props.size();i++)
	{
		if(u.j_props[i].min_pos<u.j_props[i].max_pos)
		{
			of<<"joint_table[\""<<u.j_props[i].name<<"\"]={"<<u.j_props[i].min_pos<<","<<u.j_props[i].max_pos<<"}"<<endl;
		}
	}
	of.close();*/
}
