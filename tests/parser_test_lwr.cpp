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

	if(!u.readFromFile(path+"/urdf/lwr.urdf"))
	{
		cout<<"readFromFile failed"<<endl;
		exit(-1);
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

	cout<<"+++++ add transform ++++++"<<endl;
	u.addTransform("lwr_arm_hand_link","calib_lwr_arm_base_link");
	u.addTransform("lwr_arm_link_5","lwr_arm_link_3");

	cout<<"+++++ get expression ++++++"<<endl;
	Expression<Frame>::Ptr T=u.getExpression(0);
	Expression<Frame>::Ptr T2=u.getExpression(1);

	//Expression<Frame>::Ptr T=u.compose_tree(u.root_link,u.rootlist[1]);
	//cout<<T<<endl;
	std::ofstream of("urdf_test_total.dot");
	std::ofstream of2("urdf_test_3_to_5.dot");
	T->write_dotfile(of);
	T2->write_dotfile(of2);
	of.close();of2.close();
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
	std::vector<link_property> l_props=u.l_props;

	cout<<"+++++ link properties ++++++"<<endl;
	for (int i=0;i<l_props.size();i++)
		{
			cout<<"name\t"<<l_props[i].name<<endl;
			cout<<"\tmass\t"<<l_props[i].mass<<endl;

					cout<<endl;
		}
	system ("dot -Tpdf urdf_test_total.dot -o urdf_test_total.pdf");
	system ("evince urdf_test_total.pdf");

}
