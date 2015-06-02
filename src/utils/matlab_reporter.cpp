#include <expressiongraph_wbf/utils/matlab_reporter.hpp>
#include <sstream>
using namespace KDL;

matlab_reporter::matlab_reporter(const std::string & file_name)
{	   myfile.open (file_name.c_str());	}

bool matlab_reporter::add_var(const std::string & name,Expression<double>::Ptr e)
{
	if(std::find(names.begin(), names.end(), name)!=names.end())
		return false;
	map_expr_double[name]=e;
	names.push_back(name);
	return true;
}
bool matlab_reporter::add_var(const std::string & name,Expression<Vector>::Ptr e)
{
	if(std::find(names.begin(), names.end(), name)!=names.end())
		return false;
	map_expr_vector[name]=e;
	names.push_back(name);
	return true;
}
bool matlab_reporter::add_var(const std::string & name,Expression<Rotation>::Ptr e)
{
	if(std::find(names.begin(), names.end(), name)!=names.end())
		return false;
	map_expr_rotation[name]=e;
	names.push_back(name);
	return true;
}
bool matlab_reporter::add_var(const std::string & name,Expression<Frame>::Ptr e)
{
	if(std::find(names.begin(), names.end(), name)!=names.end())
		return false;
	map_expr_frame[name]=e;
	names.push_back(name);
	return true;
}
bool matlab_reporter::add_var(const std::string & name,Expression<Twist>::Ptr e)
{
	if(std::find(names.begin(), names.end(), name)!=names.end())
		return false;
	map_expr_twist[name]=e;
	names.push_back(name);
	return true;
}
bool matlab_reporter::add_var(const std::string & name,Expression<Wrench>::Ptr e)
{
	if(std::find(names.begin(), names.end(), name)!=names.end())
		return false;
	map_expr_wrench[name]=e;
	names.push_back(name);
	return true;
}

void matlab_reporter::write_headers()
{
	std::map<std::string,  Expression<double>::Ptr>::iterator itd;
	std::map<std::string,  Expression<Vector>::Ptr>::iterator itv;
	std::map<std::string,  Expression<Rotation>::Ptr>::iterator itr;
	std::map<std::string,  Expression<Frame>::Ptr>::iterator itf;
	std::map<std::string,  Expression<Twist>::Ptr>::iterator itt;
	std::map<std::string,  Expression<Wrench>::Ptr>::iterator itw;

	std::vector<std::string>::iterator it;
	std::stringstream D;
	std::stringstream N;
	int i=1;
/*	std::cout<<"map_expr_double: "<<map_expr_double.size()<<std::endl;
	std::cout<<"map_expr_vector: "<<map_expr_vector.size()<<std::endl;
	std::cout<<"map_expr_rotation: "<<map_expr_rotation.size()<<std::endl;
	std::cout<<"map_expr_frame: "<<map_expr_frame.size()<<std::endl;
	std::cout<<"map_expr_twist: "<<map_expr_twist.size()<<std::endl;
	std::cout<<"map_expr_wrench: "<<map_expr_wrench.size()<<std::endl;

	std::cout<<"names: "<<names.size()<<std::endl;*/
	D <<"D=[];"<<std::endl;
	N <<"N={";
	for (it=names.begin(); it!=names.end(); ++it)
	{
		//scalar
		itd=map_expr_double.find(*it);
		if(itd!= map_expr_double.end()){
			D <<"D."<<*it<<"="<<i<<";"<<std::endl;i++;
			N<<"'"<<*it<<"'";if ((it == names.end()-1))	N<< "};"<<std::endl;else N<< ",";
			continue;
		}
		//vector
		itv=map_expr_vector.find(*it);
		if(itv!= map_expr_vector.end()){
			D <<"D."<<*it<<"_x="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_y="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_z="<<i<<";"<<std::endl;i++;
			N<<"'"<<*it<<"_x','"<<*it<<"_y','"<<*it<<"_z'";
			if ((it == names.end()-1))	N<< "};"<<std::endl;else N<< ",";
			continue;
		}
		//rotation
		itr=map_expr_rotation.find(*it);
		if(itr!= map_expr_rotation.end()){
			D <<"D."<<*it<<"_Xx="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_Xy="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_Xz="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_Yx="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_Yy="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_Yz="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_Zx="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_Zy="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_Zz="<<i<<";"<<std::endl;i++;
			N<<"'"<<*it<<"_Xx','"<<*it<<"_Xy','"<<*it<<"_Xz',"
					"'"<<*it<<"_Yx','"<<*it<<"_Yy','"<<*it<<"_Yz',"
					"'"<<*it<<"_Zx','"<<*it<<"_Zy','"<<*it<<"_Zy'";
			if ((it == names.end()-1))	N<< "};"<<std::endl;else N<< ",";
			continue;
		}
		//frame
		itf=map_expr_frame.find(*it);
		if(itf!= map_expr_frame.end()){
			D <<"D."<<*it<<"_x="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_y="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_z="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_Xx="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_Xy="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_Xz="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_Yx="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_Yy="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_Yz="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_Zx="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_Zy="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_Zz="<<i<<";"<<std::endl;i++;
			N<<"'"<<*it<<"_x','"<<*it<<"_y','"<<*it<<"_z',"
					"'"<<*it<<"_Xx','"<<*it<<"_Xy','"<<*it<<"_Xz',"
					"'"<<*it<<"_Yx','"<<*it<<"_Yy','"<<*it<<"_Yz',"
					"'"<<*it<<"_Zx','"<<*it<<"_Zy','"<<*it<<"_Zy'";
			if ((it == names.end()-1))	N<< "};"<<std::endl;else N<< ",";
			continue;
		}
		//twist
		itt=map_expr_twist.find(*it);
		if(itt!= map_expr_twist.end()){
			D <<"D."<<*it<<"_vel_x="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_vel_y="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_vel_z="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_rotvel_x="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_rotvel_y="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_rotvel_z="<<i<<";"<<std::endl;i++;
			N<<"'"<<*it<<"_vel_x','"<<*it<<"_vel_y','"<<*it<<"_vel_z',"
					"'"<<*it<<"_rotvel_x','"<<*it<<"_rotvel_y','"<<*it<<"_rotvel_z'";
			if ((it == names.end()-1))	N<< "};"<<std::endl;else N<< ",";
			continue;
		}
		//wrench
		itw=map_expr_wrench.find(*it);
		if(itw!= map_expr_wrench.end()){
			D <<"D."<<*it<<"_for_x="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_for_y="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_for_z="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_trq_x="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_trq_y="<<i<<";"<<std::endl;i++;
			D <<"D."<<*it<<"_trq_z="<<i<<";"<<std::endl;i++;
			N<<"'"<<*it<<"_for_x','"<<*it<<"_for_y','"<<*it<<"_for_z',"
					"'"<<*it<<"_trq_x','"<<*it<<"_trq_y','"<<*it<<"_trq_z'";
			if ((it == names.end()-1))	N<< "};"<<std::endl;else N<< ",";
			continue;
		}

	}


	myfile <<D.rdbuf()<<std::endl;
	myfile <<N.rdbuf()<<std::endl;

	myfile<<"L=["<<std::endl;


}
void matlab_reporter::write_values(){

	std::vector<std::string>::iterator it;
	std::map<std::string,  Expression<double>::Ptr>::iterator itd;
	std::map<std::string,  Expression<Vector>::Ptr>::iterator itv;
	std::map<std::string,  Expression<Rotation>::Ptr>::iterator itr;
	std::map<std::string,  Expression<Frame>::Ptr>::iterator itf;
	std::map<std::string,  Expression<Twist>::Ptr>::iterator itt;
	std::map<std::string,  Expression<Wrench>::Ptr>::iterator itw;

	for (it=names.begin(); it!=names.end(); ++it)
	{
		//scalar
		itd=map_expr_double.find(*it);
		if(itd!= map_expr_double.end())
		{
			myfile <<itd->second->value();
			if (it == names.end()-1) myfile<<std::endl;else myfile<< "\t";
		}
		//vector
		itv=map_expr_vector.find(*it);
		if(itv!= map_expr_vector.end()){
			Vector v = itv->second->value();
			myfile <<v(0)<< "\t"<<v(1)<< "\t"<<v(2);
			if (it == names.end()-1) myfile<<std::endl;else myfile<< "\t";
		}
		//rotation
		itr=map_expr_rotation.find(*it);
		if(itr!= map_expr_rotation.end()){
			Rotation r = itr->second->value();
			myfile <<r(0,0)<< "\t"<<r(1,0)<< "\t"<<r(2,0)<< "\t"
					<<r(0,1)<< "\t"<<r(1,1)<< "\t"<<r(2,1)<< "\t"
					<<r(0,2)<< "\t"<<r(1,2)<< "\t"<<r(2,2);
			if (it == names.end()-1) myfile<<std::endl;else myfile<< "\t";
		}
		//frame
		itf=map_expr_frame.find(*it);
		if(itf!= map_expr_frame.end()){
			Frame T = itf->second->value();
			Rotation r=T.M;
			Vector p=T.p;
			myfile <<p(0)<< "\t"<<p(1)<< "\t"<<p(2)<< "\t"
					<<r(0,0)<< "\t"<<r(1,0)<< "\t"<<r(2,0)<< "\t"
					<<r(0,1)<< "\t"<<r(1,1)<< "\t"<<r(2,1)<< "\t"
					<<r(0,2)<< "\t"<<r(1,2)<< "\t"<<r(2,2);
			if (it == names.end()-1) myfile<<std::endl;else myfile<< "\t";
		}
		//Twist
		itt=map_expr_twist.find(*it);
		if(itt!= map_expr_twist.end()){
			Twist t = itt->second->value();
			myfile <<t(0)<< "\t"<<t(1)<< "\t"<<t(2)<< "\t"<<t(3)<< "\t"<<t(4)<< "\t"<<t(5);
			if (it == names.end()-1) myfile<<std::endl;else myfile<< "\t";
		}
		//Wrench
		itw=map_expr_wrench.find(*it);
		if(itw!= map_expr_wrench.end()){
			Wrench t = itw->second->value();
			myfile <<t(0)<< "\t"<<t(1)<< "\t"<<t(2)<< "\t"<<t(3)<< "\t"<<t(4)<< "\t"<<t(5);
			if (it == names.end()-1) myfile<<std::endl;else myfile<< "\t";
		}
	}

}
void matlab_reporter::write_footer(){
	myfile<<"];"<<std::endl;
	myfile.close();

}



