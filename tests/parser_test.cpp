#include "expressiongraph_wbf/solver/constraints.hpp"
#include "expressiongraph_wbf/utils/UrdfExpr2.hpp"



using namespace wbf;
using namespace std;
using namespace KDL;
int main()
{
	//set input values

	std::vector<double> inp(4);
	inp[0] = 1;
	inp[1] = 1.5;
	inp[2] = -0.5;
	inp[3] = 2;
	std::vector<int> joint_indexes(4);
	joint_indexes[0]=0;
	joint_indexes[1]=1;
	joint_indexes[2]=2;
	joint_indexes[3]=3;

	UrdfExpressions2 u;

	u.readFromFile("urdf/lwr.urdf");

	std::vector<string> names;
	cout<<"+++++ get names ++++++"<<endl;
	u.getAllJointNames(names);
	for (std::vector<string>::iterator it=names.begin();it!=names.end();it++)
		cout<<"\t"<<*it<<endl;
	cout<<"+++++ auto generate map ++++++"<<endl;
	u.generateJointMap();
	for (std::map<string,unsigned int>::iterator it=u.joint_map.begin();it!=u.joint_map.end();it++)
		cout<<"\t"<<it->first<<"\t"<<it->second<<endl;

	cout<<"+++++ add transform ++++++"<<endl;
	u.addTransform("lwr_arm_hand_link","calib_lwr_arm_base_link");
	u.addTransform("lwr_arm_link_5","lwr_arm_link_3");

	cout<<"+++++ get expression ++++++"<<endl;
	Expression<Frame>::Ptr T=u.getExpression(0);
	Expression<Frame>::Ptr T2=u.getExpression(1);
	cout<<T<<endl;
	//Expression<Frame>::Ptr T=u.compose_tree(u.root_link,u.rootlist[1]);
	//cout<<T<<endl;
	std::ofstream of("urdf_test_total.dot");
	std::ofstream of2("urdf_test_3_to_5.dot");
	T->write_dotfile(of);
	T2->write_dotfile(of2);
	of.close();of2.close();
}
