#include <string>
#include <vector>
#include <algorithm>
#include <solver/context.hpp>
#include <boost/algorithm/string/case_conv.hpp>

namespace KDL {

template <typename T>
void call_setInputValues(
    boost::any& obj, 
    const std::vector<int>& ndx,
    const Eigen::VectorXd& values ) {
        try {
            boost::any_cast< typename Expression<T>::Ptr >( obj ) -> setInputValues(ndx,values);
        } catch(boost::bad_any_cast& ) {
        }
}

template <typename T>
void call_setInputValues(
    boost::any& obj, 
    const std::vector<int>& ndx,
    const std::vector<double>& values) {
        try {
            boost::any_cast< typename Expression<T>::Ptr >( obj ) -> setInputValues(ndx,values);
        } catch(boost::bad_any_cast& ) {
        }
}



void LightContext::setInputValues_outputvars(
    const std::vector<int>& ndx,
    const Eigen::VectorXd& values
) {
    for (OutputVarMap::iterator it = output_vars.begin(); it!=output_vars.end();++it) {
        it->second->setInputValues(ndx,values);
    }
}


void LightContext::setInputValues_outputvars(
    const std::vector<int>& ndx,
    const std::vector<double>& values)
{
    for (OutputVarMap::iterator it = output_vars.begin(); it!=output_vars.end();++it) {
        it->second->setInputValues(ndx,values);
    }
}

void LightContext::addToOptimizer_outputvars(ExpressionOptimizer& opt) {
    for (OutputVarMap::iterator it = output_vars.begin(); it!=output_vars.end();++it) {
        it->second->addToOptimizer(opt); 
    }
}


 
/***********************************************************************************
 * GENERAL
***********************************************************************************/
double LightContext::getSolverProperty(const std::string& name, double defaultval) {
    StringMap::iterator it = solver_property.find(name);
    if (it !=solver_property.end()) {
        return it->second;
    } else {
        return defaultval;
    } 
}
std::string LightContext::getSolverStringProperty(const std::string& name, const std::string& defaultvalue) {
    std::map<std::string,std::string>::iterator it = solver_string_property.find(name);
    if (it !=solver_string_property.end()) {
        return it->second;
    } 
    return defaultvalue;
}


bool LightContext::setVariableWeight(const std::string& varname, const double value) {
  VariableScalar* var = getScalarStruct(varname);
  if(var)
  {
    var->weight = Constant(value);
    return true;
  }
  
  return false;
}

void LightContext::setSolverProperty(const std::string& name, double value) {
    solver_property[name] = value;
}

void LightContext::setSolverStringProperty(const std::string& name, const std::string& value) {
    solver_string_property[name] = value;
}

LightContext::LightContext() {
    next_varnr = 0;
    addType("time");
    time    = addScalarVariable("time","time",0.0,Constant(1.0)); 
    // just for the ones that forget to call this to initialize:
    clearFinishStatus();
}

void LightContext::setInputValues(const std::vector<int>& ndx,const Eigen::VectorXd& values) {
    setInputValues_constraints(ndx,values);
//     setInputValues_monitors(ndx,values);
//     setInputValues_outputs(ndx,values);
    setInputValues_outputvars(ndx,values);
    setInputValues_variables(ndx,values);
}

void LightContext::setInputValues(const std::vector<int>& ndx,const std::vector<double>& values) {
    setInputValues_constraints(ndx,values);
//     setInputValues_monitors(ndx,values);
//     setInputValues_outputs(ndx,values);
    setInputValues_outputvars(ndx,values);
    setInputValues_variables(ndx,values);
}

void LightContext::addToOptimizer(ExpressionOptimizer& opt) {
    addToOptimizer_constraints(opt);
//     addToOptimizer_monitors(opt);
//     addToOptimizer_outputs(opt);
    addToOptimizer_outputvars(opt);
    addToOptimizer_variables(opt);
}

std::ostream& operator << ( std::ostream& os, const LightContext::Ptr v ) {
    v->printVariables(os);
    v->printConstraints(os);
    return os;
}

} // namespace KDL

