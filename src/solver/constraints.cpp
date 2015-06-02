#include <solver/context.hpp>
#include <iostream>
#include <iomanip>
#include <boost/format.hpp>

namespace KDL {

bool LightContext::existsConstraint(const std::string& name, LightContext::cstr_type type=LightContext::UNKNOW) 
{
  CnstrNameMap::iterator it;
  switch(type)
  {
    case BOX :
    {
      it = cnstr_box_names.find(name);
      if(it!=cnstr_box_names.end())
        return true;
      break;
    }
    case SCALAR :
    {
        it = cnstr_scalar_names.find(name);
        if(it!=cnstr_scalar_names.end())
          return true;
        break;
    }
    case ROTATION :
    {
      it = cnstr_rot_names.find(name);
        if(it!=cnstr_rot_names.end())
          return true;
        break;
    }
    case FORCE :
        {
            it = cnstr_force_names.find(name);
            if(it!=cnstr_force_names.end())
              return true;
            break;
        }
    case ADMITTANCE :
        {
            it = cnstr_admittance_names.find(name);
            if(it!=cnstr_admittance_names.end())
              return true;
            break;
        }
    case UNKNOW :
    {
      if (existsConstraint(name,BOX))
          return true;
      if (existsConstraint(name,SCALAR))
          return true;
      if (existsConstraint(name,ROTATION))
          return true;
      if (existsConstraint(name,FORCE))
          return true;
      if (existsConstraint(name,ADMITTANCE))
          return true;
        break;
    }
  }

    return false;
}

Expression<double>::Ptr LightContext::getConstraintExpr(const std::string& name, LightContext::cstr_type type=LightContext::UNKNOW) {
    for (size_t i=0;i<cnstr_scalar.size();++i) {
        if (cnstr_scalar[i].name==name) return cnstr_scalar[i].expr;
    }
    return Expression<double>::Ptr();
}

int LightContext::addInequalityConstraint(
        const std::string&          name,
        Expression<double>::Ptr     expr,
        Expression<double>::Ptr     target_lower,
        double                      K_lower, 
        Expression<double>::Ptr     target_upper,
        double                      K_upper,
        Expression<double>::Ptr     weight,
        int                         priority
    ) {
    if (K_lower      < 0) return -1;
    if (K_upper      < 0) return -1;
    if (priority < 0) return -2;
    if (existsConstraint(name)) return -3; 
//     if (target_lower > target_upper) return -4;
    ConstraintScalar c;
    c.name                        = name;
    c.expr                        = expr;
    c.target_lower                = target_lower;
    c.K_lower                     = K_lower;
    c.target_upper                = target_upper;
    c.K_upper                     = K_upper;
    c.weight                      = weight;
    c.priority                    = priority;  
    cnstr_scalar.push_back(c);
    cnstr_scalar_names.insert(make_pair(name,cnstr_scalar.size()-1));
    activity_changed();
    return 0;
}

int LightContext::addInequalityConstraint(
        const std::string&          name,
        Expression<double>::Ptr     expr,
        double                      target_lower,
        double                      K_lower, 
        double                      target_upper,
        double                      K_upper,
        Expression<double>::Ptr     weight,
        int                         priority
    ) {
    if (K_lower      < 0) return -1;
    if (K_upper      < 0) return -1;
    if (priority < 0) return -2;
    if (existsConstraint(name)) return -3; 
    if (target_lower > target_upper) return -4;
    ConstraintScalar c;
    c.name                        = name;
    c.expr                        = expr;
    c.target_lower                = Constant(target_lower);
    c.K_lower                     = K_lower;
    c.target_upper                = Constant(target_upper);
    c.K_upper                     = K_upper;
    c.weight                      = weight;
    c.priority                    = priority;  
    cnstr_scalar.push_back(c);
    cnstr_scalar_names.insert(make_pair(name,cnstr_scalar.size()-1));
    activity_changed();
    return 0;
}

int LightContext::addInequalityConstraint(
        const std::string&          name,
        Expression<double>::Ptr     expr,
        double                      target_lower,
        double                      K_lower, 
        double                      target_upper,
        double                      K_upper,
        double                      weight,
        int                         priority
    ) {
    return addInequalityConstraint(name,expr,target_lower,K_lower, target_upper, K_upper, Constant(weight), priority);
}

int LightContext::addVelocityConstraint(
            const std::string& name,
            Expression<double>::Ptr e,
            double target_velocity,
            Expression<double>::Ptr weight,
            int priority
        ) {
        return addConstraint(
                 name,
                 e - input(0)*Constant(target_velocity),
                 0.0, weight,priority
          );

}

int LightContext::addConstraint(ConstraintScalar& c) {
    cnstr_scalar.push_back(c);
    cnstr_scalar_names.insert(make_pair(c.name,cnstr_scalar.size()-1));
    activity_changed();
    return 0;
}

int LightContext::addConstraint(
        const std::string&          name,
        Expression<double>::Ptr     expr,
        double                      K, 
        Expression<double>::Ptr     weight,
        int                         priority
    ) {
    if (K      < 0) return -1;
    if (priority < 0) return -2;
    if (existsConstraint(name)) return -3; 

    ConstraintScalar c;

    c.name                        = name;
    c.expr                        = expr;
    c.target_lower                = Constant(0.0);
    c.K_lower                     = K;
    c.target_upper                = Constant(0.0);
    c.K_upper                     = K;
    c.weight                      = weight;
    c.priority                    = priority;  
    cnstr_scalar.push_back(c);
    cnstr_scalar_names.insert(make_pair(name,cnstr_scalar.size()-1));
    activity_changed();
    return 0;
}

int LightContext::addConstraint(
        const std::string&          name,
        Expression<double>::Ptr     expr,
        double                      K, 
        double                      weight,
        int                         priority
    ) {
    return addConstraint(name,expr,K,Constant(weight), priority);
}

int LightContext::addConstraint(
        const std::string&          name,
        Expression<Rotation>::Ptr   expr,
        double                      K, 
        Expression<double>::Ptr     weight,
        int                         priority
    ) {
    if (K      < 0) return -1;
    if (priority < 0) return -2;
    if (existsConstraint(name)) return -3; 


    ConstraintRotation c;
    c.name                        = name;
    c.expr                        = expr;
    c.K                           = K;
    c.weight                      = weight;
    c.priority                    = priority;  
    cnstr_rot.push_back(c);
    cnstr_rot_names.insert(make_pair(name,cnstr_rot.size()-1));
    activity_changed();
    return 0;
}
int LightContext::addConstraint(
        const std::string&          name,
        Expression<Rotation>::Ptr   expr,
        double                      K, 
        double                      weight,
        int                         priority
    ) {
    return addConstraint(name,expr,K,Constant(weight), priority);
} 

int LightContext::addBoxConstraint(
        const std::string&          name,
        int                         variablenr,
        double                      target_lower,
        double                      target_upper
    ) {
    if (variablenr < 0) return -5;
    if (target_lower > target_upper) return -4;
    //if (exists(name)) return -3; 


    ConstraintBox c;
 
    c.name                   = name;
    c.variablenr             = variablenr;
    c.target_lower           = target_lower;
    c.target_upper           = target_upper;
    cnstr_box_names.insert(make_pair(name,cnstr_box_names.size()-1));
    cnstr_box.push_back(c); 
    activity_changed();
    return 0;
}


int  LightContext::addForceConstraint(
		 const std::string&          name,
		 const std::string&          name_of_direction,
		 Expression<double>::Ptr     expr,
		 double                      K,
		 Expression<double>::Ptr     weight,
		 int                         priority,
		 ToS::TypeOfSpace				  which_dir
) {

    if (K      < 0) return -1;
    if (priority < 1) return -2;
    if (existsConstraint(name)) return -3;


    ConstraintForce c;

    c.name                        = name;
    c.expr                        = expr;
    c.name_of_output_direction   = name_of_direction;
    c.type_of_space				  = which_dir;
    c.target_lower                = Constant(0.0);
    c.K_lower                     = K;
    c.target_upper                = Constant(0.0);
    c.K_upper                     = K;
    c.weight                      = weight;
    c.priority                    = priority;

    cnstr_force.push_back(c);

    cnstr_force_names.insert(make_pair(name,cnstr_force.size()-1));
    activity_changed();
    return 0;
}

int LightContext::addForceConstraint(
		 const std::string&          name,
		 const std::string&          name_of_direction,
		 Expression<double>::Ptr     expr,
		 double                      K,
		 double                      weight,
		 int                         priority,
		 ToS::TypeOfSpace			 which_dir
) {
    return addForceConstraint(name,name_of_direction,
    		expr,K,Constant(weight), priority,which_dir);
}


int LightContext::addAdmittanceConstraint(
		const std::string&      	name,
		const std::string&      	name_of_direction,
		Expression<double>::Ptr 	expr,
		double                  	Binv,
		Expression<double>::Ptr 	weight,
		int                    	priority,
		ToS::TypeOfSpace		 	which_dir
){
	if (Binv      <= 0) return -1;
	if (priority < 1) return -2;
	if (existsConstraint(name)) return -3;


	ConstraintAdmittance c;

	c.name                        = name;
	c.expr                        = expr;
	c.name_of_output_direction   = name_of_direction;
	c.type_of_space				  = which_dir;
	c.target_lower                = Constant(0.0);
	c.Binv                     = Binv;
	c.target_upper                = Constant(0.0);
	c.weight                      = weight;
	c.priority                    = priority;

	cnstr_admittance.push_back(c);

	cnstr_admittance_names.insert(make_pair(name,cnstr_admittance_names.size()-1));
	activity_changed();
	return 0;
}
/** see overloaded method */
int LightContext::addAdmittanceConstraint(
		  const std::string&       	name,
		  const std::string&       	name_of_direction,
		  Expression<double>::Ptr  	expr,
		  double                   	Binv,
		  double                   	weight,
		  int                		priority,
		  ToS::TypeOfSpace			which_dir
){
	 return addAdmittanceConstraint(name,name_of_direction,
	    		expr,Binv,Constant(weight), priority,which_dir);
	}





int LightContext::addAdmittanceInequalityConstraint(
		const std::string&      	name,
		const std::string&      	name_of_direction,
		Expression<double>::Ptr 	expr,
		double                  	Binv,
		Expression<double>::Ptr     target_lower,
		Expression<double>::Ptr     target_upper,
		Expression<double>::Ptr 	weight,
		int                   		priority,
		ToS::TypeOfSpace		 	which_dir
){
	//if (Binv      <= 0) return -1;
	if (priority < 1) return -2;
	if (existsConstraint(name)) return -3;


	ConstraintAdmittance c;

	c.name                        = name;
	c.expr                        = expr;
	c.name_of_output_direction   = name_of_direction;
	c.type_of_space				  = which_dir;
	c.target_lower                =  target_lower;
	c.Binv                     = Binv;
	c.target_upper                =   target_upper;
	c.weight                      = weight;
	c.priority                    = priority;

	cnstr_admittance.push_back(c);

	cnstr_admittance_names.insert(make_pair(name,cnstr_admittance_names.size()-1));
	activity_changed();
	return 0;
}

/** see overloaded method */
int LightContext::addAdmittanceInequalityConstraint(
		const std::string&       	name,
		const std::string&       	name_of_direction,
		Expression<double>::Ptr  	expr,
		double                   	Binv,
		double                      target_lower,
		double                      target_upper,
		double                   	weight,
		int                		priority,
		ToS::TypeOfSpace			which_dir
){
	return addAdmittanceInequalityConstraint(
			name,name_of_direction,
			expr,Binv,Constant(target_lower),Constant(target_upper),
			Constant(weight), priority,which_dir);
}


/** see overloaded method */
int LightContext::addAdmittanceInequalityConstraint(
		const std::string&       	name,
		const std::string&       	name_of_direction,
		Expression<double>::Ptr  	expr,
		double                   	Binv,
		double                      target_lower,
		double                      target_upper,
		Expression<double>::Ptr     weight,
		int                		priority,
		ToS::TypeOfSpace			which_dir
){
	return addAdmittanceInequalityConstraint(
			name,name_of_direction,
			expr,Binv,Constant(target_lower),Constant(target_upper),
			weight, priority,which_dir);
}














/*
 * Functions for space description
 * */

void LightContext::clearOutputDirection()

{

	output_space_description.expressions_scalar.clear();
	output_space_description.names_scalar.clear();
	output_space_description.expressions_rot.clear();
	output_space_description.names_rot.clear();
}

bool LightContext::addOutputDirection(Expression<double>::Ptr exp,
		const std::string& name)
{
	int i;
	ToS::TypeOfSpace t=whichForceSpace(name,i);
	if(t==ToS::NOT_PRESENT) //the name is not present-> here is ok
	{
		output_space_description.expressions_scalar.push_back(exp);
		output_space_description.names_scalar.push_back(name);

		return true;
	}
	else return false;
}
bool LightContext::addOutputDirection(
		Expression<Rotation>::Ptr expr,
		const std::string&          name)
{
	int i;
	if(whichForceSpace(name,i)==ToS::NOT_PRESENT) //the name is not present-> here is ok
	{
		output_space_description.expressions_rot.push_back(expr);
		output_space_description.names_rot.push_back(name);
		return true;
	}
	else return false;
}


ToS::TypeOfSpace LightContext::whichForceSpace(const std::string& name,int & i) {
	size_t sn= output_space_description.expressions_scalar.size();
	size_t sr= output_space_description.expressions_rot.size();
	for (size_t indx=0;indx<sn;++indx)
		if(output_space_description.names_scalar[indx]==name)
		{
			i=indx;
			return ToS::SCALAR;
		}
	//for each rotation, i have to look in each of 3 components
	for (size_t indx=0;indx<sr;++indx)
		if(output_space_description.names_rot[indx]==name)
		{
			i=indx;
			return ToS::ROTATION;
		}
	i=0;
	return ToS::NOT_PRESENT;
}






//BROKEN!!!
bool LightContext::removeConstraint(const std::string& name, LightContext::cstr_type type)
{
    switch(type)
    {
      case BOX :
      {
        CnstrNameMap::iterator it = cnstr_box_names.find(name);
        if(it!=cnstr_box_names.end()) {
          cnstr_box.erase(cnstr_box.begin()+it->second);
          cnstr_box_names.erase(it);
          activity_changed();
          return true;
        }
        break;
      }
      case SCALAR : //This is broken, fix for now TODO FIXME
      {
        int rmidx = -1;
        for(unsigned int i=0;i<cnstr_scalar.size();i++) {
          if(cnstr_scalar[i].name == name) {
            rmidx = i;
            break;
          }
        }
        if (rmidx == -1) break;  //nothing found, return

        cnstr_scalar.erase(cnstr_scalar.begin()+rmidx);
        //rebuild scalar_names - retrocompatibility mode
        cnstr_scalar_names.clear();
        for(unsigned int i=0;i<cnstr_scalar.size();i++) {
          cnstr_scalar_names.insert(make_pair(cnstr_scalar[i].name,i));
        }
        activity_changed();
        return true;
      }   
      case ROTATION :
      {
        CnstrNameMap::iterator it = cnstr_rot_names.find(name);
        if(it!=cnstr_rot_names.end()) {
          cnstr_rot.erase(cnstr_rot.begin()+it->second);
          cnstr_rot_names.erase(it);
          activity_changed();
          return true;
        }
        break;
      }

      case FORCE :
      {
    	  int rmidx = -1;
    	  for(unsigned int i=0;i<cnstr_force.size();i++) {
    		  if(cnstr_force[i].name == name) {
    			  rmidx = i;
    			  break;
    		  }
    	  }
    	  if (rmidx == -1) break;  //nothing found, return

    	  cnstr_force.erase(cnstr_force.begin()+rmidx);
    	  //rebuild scalar_names - retrocompatibility mode
    	  cnstr_force_names.clear();
    	  for(unsigned int i=0;i<cnstr_force.size();i++) {
    		  cnstr_force_names.insert(make_pair(cnstr_force[i].name,i));
    	  }
    	  activity_changed();
    	  return true;
      }
      case ADMITTANCE :
      {
    	  int rmidx = -1;
    	  for(unsigned int i=0;i<cnstr_admittance.size();i++) {
    		  if(cnstr_admittance[i].name == name) {
    			  rmidx = i;
    			  break;
    		  }
    	  }
    	  if (rmidx == -1) break;  //nothing found, return

    			  cnstr_admittance.erase(cnstr_admittance.begin()+rmidx);
    			  //rebuild scalar_names - retrocompatibility mode
    			  cnstr_admittance_names.clear();
    			  for(unsigned int i=0;i<cnstr_admittance.size();i++) {
    				  cnstr_admittance_names.insert(make_pair(cnstr_admittance[i].name,i));
    			  }
    			  activity_changed();
    			  return true;
      }


      case UNKNOW :
      {
        if (removeConstraint(name,BOX))
          return true;
        if (removeConstraint(name,SCALAR))
          return true;
        if (removeConstraint(name,ROTATION))
          return true;
        if (removeConstraint(name,FORCE))
          return true;
        if (removeConstraint(name,ADMITTANCE))
           return true;
        break;
    }
    }
     //If it arrives here, constraints not found!: Not inserted in the context
    return false;
}
    
void LightContext::setInputValues_constraints(const std::vector<int>& ndx,const Eigen::VectorXd& values) {
//     update_active();
    for (unsigned int i=0;i<cnstr_scalar.size();++i) {
        cnstr_scalar[i].expr->setInputValues(ndx,values);
        cnstr_scalar[i].weight->setInputValues(ndx,values);
    }
    for (unsigned int i=0;i<cnstr_rot.size();++i) {
        cnstr_rot[i].expr->setInputValues(ndx,values);
        cnstr_rot[i].weight->setInputValues(ndx,values);
    }
    for (unsigned int i=0;i<cnstr_force.size();++i) {
    	cnstr_force[i].expr->setInputValues(ndx,values);
    	cnstr_force[i].weight->setInputValues(ndx,values);
    }
    for (unsigned int i=0;i<cnstr_admittance.size();++i) {
    	cnstr_admittance[i].expr->setInputValues(ndx,values);
    	cnstr_admittance[i].weight->setInputValues(ndx,values);
    }
    //space of force directions
    unsigned int sn= output_space_description.expressions_scalar.size();
    unsigned int sr= output_space_description.expressions_rot.size();
    for (unsigned int i=0;i<sn;++i)
    	output_space_description.expressions_scalar[i]->setInputValues(ndx,values);
    for (unsigned int i=0;i<sr;++i)
    	output_space_description.expressions_rot[i]->setInputValues(ndx,values);

}


void LightContext::setInputValues_constraints(const std::vector<int>& ndx,const std::vector<double>& values) {
	//     update_active();
	for (unsigned int i=0;i<cnstr_scalar.size();++i) {
		cnstr_scalar[i].expr->setInputValues(ndx,values);
		cnstr_scalar[i].weight->setInputValues(ndx,values);
	}
	for (unsigned int i=0;i<cnstr_rot.size();++i) {
		cnstr_rot[i].expr->setInputValues(ndx,values);
		cnstr_rot[i].weight->setInputValues(ndx,values);
	}
	for (unsigned int i=0;i<cnstr_force.size();++i) {
		cnstr_force[i].expr->setInputValues(ndx,values);
		cnstr_force[i].weight->setInputValues(ndx,values);
	}
	for (unsigned int i=0;i<cnstr_admittance.size();++i) {
		cnstr_admittance[i].expr->setInputValues(ndx,values);
		cnstr_admittance[i].weight->setInputValues(ndx,values);
	}
	//space of force directions
	unsigned int sn= output_space_description.expressions_scalar.size();
	unsigned int sr= output_space_description.expressions_rot.size();
	for (unsigned int i=0;i<sn;++i)
		output_space_description.expressions_scalar[i]->setInputValues(ndx,values);
	for (unsigned int i=0;i<sr;++i)
		output_space_description.expressions_rot[i]->setInputValues(ndx,values);
}

void LightContext::addToOptimizer_constraints(ExpressionOptimizer& opt) {
//     update_active();
    for (unsigned int i=0;i<cnstr_scalar.size();++i) {
        cnstr_scalar[i].expr->addToOptimizer(opt);
        cnstr_scalar[i].weight->addToOptimizer(opt);
    }
    for (unsigned int i=0;i<cnstr_rot.size();++i) {
        cnstr_rot[i].expr->addToOptimizer(opt);
        cnstr_rot[i].weight->addToOptimizer(opt);
    }
    for (unsigned int i=0;i<cnstr_force.size();++i) {
        cnstr_force[i].expr->addToOptimizer(opt);
        cnstr_force[i].weight->addToOptimizer(opt);
    }
    for (unsigned int i=0;i<cnstr_admittance.size();++i) {
    	cnstr_admittance[i].expr->addToOptimizer(opt);
    	cnstr_admittance[i].weight->addToOptimizer(opt);
    }
    //space of force
    unsigned int sn= output_space_description.expressions_scalar.size();
    unsigned int sr= output_space_description.expressions_rot.size();
    for (unsigned int indx=0;indx<sn;++indx)
    	output_space_description.expressions_scalar[indx]->addToOptimizer(opt);
    for (unsigned int indx=0;indx<sr;++indx)
    	output_space_description.expressions_rot[indx]->addToOptimizer(opt);
}


//NOTE:: TODO this can be semplified a LOT!!!!! (enum)
int LightContext::countConstraints(std::vector<int>& nrofprior) {
//     update_active();
    for (size_t i=0;i<nrofprior.size();++i) {
        nrofprior[i]=0;
    }
    for (unsigned int i=0;i<cnstr_scalar.size();++i) {
        if (cnstr_scalar[i].priority >= (int)nrofprior.size()) {
            return -1;
        }
        nrofprior[ cnstr_scalar[i].priority ] += 1;
    }
    for (unsigned int i=0;i<cnstr_rot.size();++i) {
    	if (cnstr_rot[i].priority >= (int)nrofprior.size()) {
    		return -1;
    	}
    	nrofprior[ cnstr_rot[i].priority ] += 3;
    }
    for (unsigned int i=0;i<cnstr_force.size();++i) {
    	if (cnstr_force[i].priority >= (int)nrofprior.size()) {
    		return -1;
    	}
    	nrofprior[cnstr_force[i].priority ] += 1;
    }
    for (unsigned int i=0;i<cnstr_admittance.size();++i) {
     	if (cnstr_admittance[i].priority >= (int)nrofprior.size()) {
     		return -1;
     	}
     	nrofprior[cnstr_admittance[i].priority ] += 1;
     }
    return 0;  
}
 
int LightContext::countEqConstraints(std::vector<int>& nrofprior) {
//     update_active();
    for (size_t i=0;i<nrofprior.size();++i) {
        nrofprior[i]=0;
    }
    for (unsigned int i=0;i<cnstr_scalar.size();++i) {
        if (cnstr_scalar[i].priority >= (int)nrofprior.size()) {
            return -1;
        }
        if (cnstr_scalar[i].target_lower == cnstr_scalar[i].target_upper) {
            nrofprior[ cnstr_scalar[i].priority ] += 1;
        }
    }
    for (unsigned int i=0;i<cnstr_rot.size();++i) {
        if (cnstr_rot[i].priority >= (int)nrofprior.size()) {
            return -1;
        }
        nrofprior[ cnstr_rot[i].priority ] += 3;
 
    }
    for (unsigned int i=0;i<cnstr_force.size();++i) {
           if (cnstr_force[i].priority >= (int)nrofprior.size()) {
               return -1;
           }
           if (cnstr_force[i].target_lower == cnstr_force[i].target_upper) {
               nrofprior[ cnstr_force[i].priority ] += 1;
           }
       }
    for (unsigned int i=0;i<cnstr_admittance.size();++i) {
           if (cnstr_admittance[i].priority >= (int)nrofprior.size()) {
               return -1;
           }
           if (cnstr_admittance[i].target_lower == cnstr_admittance[i].target_upper) {
               nrofprior[ cnstr_admittance[i].priority ] += 1;
           }
       }
    return 0;  
}

void LightContext::printConstraints( std::ostream& os ) {
	//     update_active();
	using namespace std;
	using boost::format;
	using boost::io::group;
	os << "Constraints("<< (cnstr_scalar.size() + cnstr_rot.size() + cnstr_box.size()+ cnstr_force.size()+ cnstr_admittance.size()) << ") :"<< endl;
	os << format("%|166T-|")<<endl;
	os << format("%1$=10s|%2$=60s|%3$=16s|%4$=16s|%5$=12s|%6$=10s|%7$=12s|%8$=17s\n") % "Type" % "Name" % "lower bound" % "upper bound" % "weight" % "Priority" % "Current Val." % "tags";
	os << format("%|166T-|")<<endl;
	std::string red = "\033[1;31m";
	std::string off = "\033[0m";
	//os << "\033[1;31mbold red text\033[0m\n";
	for (unsigned int i=0;i<cnstr_scalar.size();++i) {
		double value = cnstr_scalar[i].expr->value();
		if (!((cnstr_scalar[i].target_lower->value() <= value) && (value <= cnstr_scalar[i].target_upper->value())) && (cnstr_scalar[i].priority!=2)) {
			os << red;
		}
		os << format("%1$=10s|%2$=60s|%3$=16d|%4$=16d|%5$=12d|%6$=10d|%7$=12d|%8$=17s\n")
                		 % "Scalar"
                		 % cnstr_scalar[i].name
                		 % cnstr_scalar[i].target_lower->value()
                		 % cnstr_scalar[i].target_upper->value()
                		 % cnstr_scalar[i].weight->value()
                		 % cnstr_scalar[i].priority
                		 % value
                		 % "empty";
		if (!((cnstr_scalar[i].target_lower->value() <= value) && (value <= cnstr_scalar[i].target_upper->value())) && (cnstr_scalar[i].priority!=2)) {
			os << off;
		}
	}
	for (unsigned int i=0;i<cnstr_rot.size();++i) {
		os << format("%1$=10s|%2$=60s|%3$=16s|%4$=16s|%5$=12d|%6$=10d|%7$=12d|%8$=17s\n")
        		//os << format("\t%1$=20s|%2$=40s|%3$=16s\n")
                		 % "Rotational"
                		 % cnstr_rot[i].name
                		 % "-"
                		 % "-"
                		 % cnstr_rot[i].weight->value()
                		 % cnstr_rot[i].priority
                		 % cnstr_rot[i].expr->value()
                		 %"empty";
	}
	for (unsigned int i=0;i<cnstr_box.size();++i) {
		//os << format("\t%1$=20s|%2$=40s|  Var%3$-11s\n")
		os << format("%1$=10s|%2$=60s|%3$=16s|%4$=16s|%5$=12s|  Var%6$-5s|%7$=12s|%8$=17s\n")
                		 % "Box"
                		 % cnstr_box[i].name
                		 % cnstr_box[i].target_lower
                		 % cnstr_box[i].target_upper
                		 % "-"
                		 % cnstr_box[i].variablenr
                		 % "-"
                		 % "empty";
	}
	for (unsigned int i=0;i<cnstr_force.size();++i) {

		double value = cnstr_force[i].expr->value();
		double up = cnstr_force[i].target_upper->value();
		double lo = cnstr_force[i].target_lower->value();
		if (!((lo <= value) && (value <= up)) && (cnstr_force[i].priority!=2)) {
			os << red;
		}
		os << format("%1$=10s|%2$=60s|%3$=16d|%4$=16d|%5$=12d|%6$=10d|%7$=12d|%8$=17s\n")
                    		 % "Force"
                    		 % cnstr_force[i].name
                    		 % lo
                    		 % up
                    		 % cnstr_force[i].weight->value()
                    		 % cnstr_force[i].priority
                    		 % value
                    		 % "empty";
		if (!((lo <= value) && (value <= up)) && (cnstr_force[i].priority!=2)) {
			os << off;
		}
	}
	for (unsigned int i=0;i<cnstr_admittance.size();++i) {

		double value = cnstr_admittance[i].expr->value();
		double up = cnstr_admittance[i].target_upper->value();
		double lo = cnstr_admittance[i].target_lower->value();
		if (!((lo <= value) && (value <= up)) && (cnstr_admittance[i].priority!=2)) {
			os << red;
		}
		os << format("%1$=10s|%2$=60s|%3$=16d|%4$=16d|%5$=12d|%6$=10d|%7$=12d|%8$=17s\n")
                        		 % "Admittance"
                        		 % cnstr_admittance[i].name
                        		 % lo
                        		 % up
                        		 % cnstr_admittance[i].weight->value()
                        		 % cnstr_admittance[i].priority
                        		 % value
                        		 % "empty";
		if (!((lo <= value) && (value <= up)) && (cnstr_admittance[i].priority!=2)) {
			os << off;
		}
	}
	os << format("%|166T-|\n");
}

} // namespace KDL

