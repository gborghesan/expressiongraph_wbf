/* Author: Erwin Aertbelien, Enea Scioni*/
#include <expressiongraph_wbf/utils/UrdfExpr2.hpp>


using namespace std;


namespace KDL
{

//void UrdfExpressions2::hash_names( const boost::shared_ptr<urdf::Link>&  link, int level=0);

KDL::Vector UrdfExpressions2::toKdl(urdf::Vector3 v) {
	return KDL::Vector(v.x, v.y, v.z);
}

KDL::Rotation UrdfExpressions2::toKdl(urdf::Rotation r) {
	return KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

KDL::Frame UrdfExpressions2::toKdl(urdf::Pose p) {
	return KDL::Frame(toKdl(p.rotation), toKdl(p.position));

}

bool UrdfExpressions2::readFromFile(const std::string& filename) {
	std::string xml_string;
	std::fstream xml_file(filename.c_str(), std::fstream::in);
	while ( xml_file.good() ) {
		std::string line;
		std::getline( xml_file, line);
		xml_string += (line + "\n");
	}
	xml_file.close();
	if (!readFromString(xml_string)) {
		return false;
	};
	return true;
}

bool UrdfExpressions2::readFromString(const std::string& xml_string) {
	using namespace urdf;
	boost::shared_ptr<ModelInterface> robot = parseURDF(xml_string);
	if (!robot){
		std::cout << "ERROR: Model Parsing the xml failed: " << std::endl;
		return false;
	}
	std::cout << "robot name is: " << robot->getName() << std::endl;
	// get info from parser
	std::cout << "---------- Successfully Parsed XML ---------------" << std::endl;
	// get root link
	root_link=robot->root_link_;
	if (!root_link) return false;
	std::cout << "root Link: " << root_link->name << " has " << root_link->child_links.size() << " child(ren)" << std::endl;
	hash_names(root_link);
	return true;
}


void UrdfExpressions2::hash_names( const boost::shared_ptr<urdf::Link>& link, int level) {
	using namespace urdf;
	linkmap[ link->name ] = link;
	for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++) {
		if (*child) {
			// indent:
			 for(int j=0;j<level;j++) std::cout << "\t";
			// debug output:
			 std::cout << "child(" << (*child)->name <<  ")" << std::endl;
			hash_names(*child,level+1);
		} else {
			std::cerr << "root link: " << link->name << " has a null child!" << *child << std::endl;
		}
	}
}

void UrdfExpressions2::addJoint(const boost::shared_ptr<urdf::Link>& link, std::vector<std::string>& names){
    using namespace std;
    // add joint:
    if ((link->parent_joint)  &&
        ( (link->parent_joint->type == urdf::Joint::PRISMATIC) ||
          (link->parent_joint->type == urdf::Joint::CONTINUOUS) ||
          (link->parent_joint->type == urdf::Joint::REVOLUTE)
        )
       )
    {
        names.push_back(link->parent_joint->name);
        //std::cout << "joint name : " << link->parent_joint->name << endl;
    }
    // add children:
    for (std::vector<boost::shared_ptr<urdf::Link> >::const_iterator child = link->child_links.begin();
         child!= link->child_links.end();
        ++child) {
            if (*child) {
                addJoint(*child,names);
            }
     }
}

void UrdfExpressions2::getAllJointNames(std::vector<std::string>& names) {
    addJoint(root_link,names);
}
void UrdfExpressions2::generateJointMap(unsigned int initial_index)
{
	std::vector<string> names;
	getAllJointNames(names);
	joint_map.clear();
	for (std::vector<string>::iterator it=names.begin();it!=names.end();it++)
	{
		joint_map.insert ( std::pair<string,unsigned int>(*it,initial_index) );
		initial_index++;

	}

}

bool UrdfExpressions2::addTransform(const std::string& ee, const std::string& base) {
    std::cout << "addTransform(" << ee << "," << base << ")"<<endl;
    LinkMap::iterator it_ee, it_base;
    it_ee = linkmap.find(ee);
    if (it_ee==linkmap.end()) {
        return false;
    }
    it_base = linkmap.find(base);
    if (it_base==linkmap.end()) {
        return false;
    }
    transformlist.push_back(Transform( it_ee->second, it_base->second) );
    LinkSet linkset;
    LinkPtr p = it_ee->second;
    while (p) {
        linkset.insert(p);
        p = p->getParent();
    }
    p = it_base->second;
    while (p) {
        LinkSet::iterator it = linkset.find(p);
        if (it!=linkset.end()) {
            rootlist.push_back(*it);
            std::cout << "\troot: " << (*it)->name << std::endl;
            break;
        }
        p = p->getParent();
    }
    if (!p) {
        transformlist.pop_back();
        return false;
    }
    return true;
}




Expression<Frame>::Ptr  UrdfExpressions2::toKdl(const UrdfExpressions2::JointPtr& jnt) {
    Expression<Frame>::Ptr expr;
    std::map<string, unsigned int>::iterator it;
    Frame F_parent_jnt = toKdl(jnt->parent_to_joint_origin_transform);

    if(jnt->type!=urdf::Joint::FIXED)
    {
    	it=joint_map.find(jnt->name);
    	if(it==joint_map.end())
    		//error!
    		return Expression<Frame>::Ptr();
    }

    switch (jnt->type){
        case urdf::Joint::FIXED : {
            return  Constant(F_parent_jnt);
        }
        case urdf::Joint::REVOLUTE :
        case urdf::Joint::CONTINUOUS :
        {
            Vector axis = toKdl(jnt->axis);
            return Constant(F_parent_jnt)*frame(rot(axis,input(it->second)));
        }

        case urdf::Joint::PRISMATIC : {

            Vector axis = toKdl(jnt->axis);
            return Constant(F_parent_jnt)*frame(Constant(axis)*input(it->second));
        }
        default : {
            cout<<"Converting unknown joint type of joint "<<jnt->name.c_str()<<" into a fixed joint!"<<endl;
            return  Constant(F_parent_jnt);
        }
    }
    return Constant(F_parent_jnt);
}
Expression<Frame>::Ptr UrdfExpressions2::compose_tree(

        const UrdfExpressions2::LinkPtr& p,
        const UrdfExpressions2::LinkPtr& p_root
) {
   if (p==p_root) {
            return Constant(Frame::Identity());
    } else {
            std::string name;
            Expression<Frame>::Ptr e;
            if (p->parent_joint) {
                name = p->parent_joint->name;
                e    = toKdl(p->parent_joint);
            } else {
                e    = Constant(Frame::Identity());
            }
            return cached<Frame>(
                        name,
                        compose_tree(p->getParent(),p_root)*e
                    );
    }
}
Expression<Frame>::Ptr UrdfExpressions2::getExpression(int i) {
    //std::cout << "getExpression(ctx,"<< i << ")"<< endl;
    assert( (0<=i) && ( i < (int)transformlist.size() ) );
    LinkPtr p_ee   = transformlist[i].first;
    LinkPtr p_base = transformlist[i].second;
    LinkPtr p_root = rootlist[i];
    Expression<Frame>::Ptr e_root_ee;
    Expression<Frame>::Ptr e_root_base;
    e_root_ee   = compose_tree(p_ee,p_root);
    e_root_base = compose_tree(p_base,p_root);

    return cached<Frame>(inv(e_root_base)*e_root_ee);
}
/*
    KDL::Expression<KDL::Frame>::Ptr  toKdl(LightContext::Ptr& ctx, const JointPtr& jnt);

    KDL::Expression<KDL::Frame>::Ptr compose_tree(
        LightContext::Ptr& ctx,
        const LinkPtr& p,
        const LinkPtr& p_root
    );

    void addJoint(const boost::shared_ptr<urdf::Link>& link, std::vector<std::string>& names);

    UrdfExpressions2(double K_limits,double velocity_scale=1.0, bool poslimits=true, bool vellimits=true);

    bool readFromFile( const std::string& filename );
    bool readFromParam(const std::string& parameter);

    //bool read_fromFile(const std::string& filename);
    bool read_fromString(const std::string& xml_string);


    bool addTransform(const std::string& frame, const std::string& base);

    void getAllJointNames(std::vector<std::string>& names);
    KDL::Expression<KDL::Frame>::Ptr getExpression(LightContext::Ptr& ctx, int i);

    void getAllLinkProperties( std::vector<link_property>& props, UrdfExpressions2::LinkPtr p_root=UrdfExpressions2::LinkPtr());
};


UrdfExpressions2::UrdfExpressions2(double K_limits,double velocity_scale, bool poslimits, bool vellimits) {
    this->K_limits = K_limits;
    this->velocity_scale = velocity_scale;
    this->poslimits = poslimits;
    this->vellimits = vellimits;
    jointset.clear();
}



void UrdfExpressions2::getAllLinkProperties( std::vector<link_property>& props, UrdfExpressions2::LinkPtr p_root){
    if (p_root) {
        link_property L;
        L.name = p_root->name;
        if (p_root->inertial) {
            L.mass = p_root->inertial->mass;
            L.Ixx = p_root->inertial->ixx;
            L.Ixy = p_root->inertial->ixy;
            L.Ixz = p_root->inertial->ixz;
            L.Iyy = p_root->inertial->iyy;
            L.Iyz = p_root->inertial->iyz;
            L.Izz = p_root->inertial->izz;
            L.origin = toKdl(p_root->inertial->origin);
            props.push_back(L);
            //std::cout << L.name << "\t" << L.mass << std::endl;
        }
        for (size_t i=0; i< p_root->child_links.size();++i) {
            getAllLinkProperties(props,p_root->child_links[i]);
        }
    }
}





Expression<Frame>::Ptr UrdfExpressions2::getExpression(LightContext::Ptr& ctx,int i) {
    //std::cout << "getExpression(ctx,"<< i << ")"<< endl;
    assert( (0<=i) && ( i < (int)transformlist.size() ) );
    LinkPtr p_ee   = transformlist[i].first;
    LinkPtr p_base = transformlist[i].second;
    LinkPtr p_root = rootlist[i];
    Expression<Frame>::Ptr e_root_ee;
    Expression<Frame>::Ptr e_root_base;
    e_root_ee   = compose_tree(ctx,p_ee,p_root);
    e_root_base = compose_tree(ctx,p_base,p_root);

    return cached<Frame>(inv(e_root_base)*e_root_ee);
}

 */

/*

Expression<Frame>::Ptr  UrdfExpressions2::toKdl(LightContext::Ptr& ctx, const UrdfExpressions2::JointPtr& jnt) {
    Expression<Frame>::Ptr expr;
    Frame F_parent_jnt = toKdl(jnt->parent_to_joint_origin_transform);
    switch (jnt->type){
        case urdf::Joint::FIXED : {
            return  Constant(F_parent_jnt);
        }
        case urdf::Joint::REVOLUTE : {
            Expression<double>::Ptr inp  = ctx->addScalarVariable(jnt->name,"robot");
            Vector axis = toKdl(jnt->axis);
            JointSet::iterator it=jointset.find(jnt);
            if ((it==jointset.end())&&(jnt->limits)) {
                jointset.insert(jnt);
                if (vellimits) {
                    int robotvar = ctx->getScalarNdx(jnt->name);
                    ctx->addBoxConstraint( jnt->name + ":velocitylimit", robotvar,
                                           -velocity_scale*jnt->limits->velocity,
                                            velocity_scale*jnt->limits->velocity);
                }
                if (poslimits) {
                    ctx->addInequalityConstraint(jnt->name+":positionlimit",
                                                 inp,
                                                 jnt->limits->lower, K_limits,
                                                 jnt->limits->upper, K_limits, 1.0, 1);
                }
            }
            return Constant(F_parent_jnt)*frame(rot(axis,inp));
        }
        case urdf::Joint::CONTINUOUS : {
            // std::cout << "joint : " << jnt->name << " is continuous" << std::endl;
            Expression<double>::Ptr inp  = ctx->addScalarVariable(jnt->name,"robot");
            Vector axis = toKdl(jnt->axis);
            JointSet::iterator it=jointset.find(jnt);
            if ((it==jointset.end())&&(jnt->limits)) {
                jointset.insert(jnt);
                //cerr << "joint : " << jnt->name << " added " << endl;
                if (vellimits) {
                    int robotvar = ctx->getScalarNdx(jnt->name);
                    ctx->addBoxConstraint( jnt->name + ":velocitylimit", robotvar,
                                           -velocity_scale*jnt->limits->velocity,
                                            velocity_scale*jnt->limits->velocity);
                }
            }
            return Constant(F_parent_jnt)*frame(rot(axis,inp));
        }
        case urdf::Joint::PRISMATIC : {
            Expression<double>::Ptr inp  = ctx->addScalarVariable(jnt->name,"robot");
            Vector axis = toKdl(jnt->axis);
            JointSet::iterator it=jointset.find(jnt);
            if ((it==jointset.end())&&(jnt->limits)) {
                jointset.insert(jnt);
                if (vellimits) {
                    int robotvar = ctx->getScalarNdx(jnt->name);
                    ctx->addBoxConstraint( jnt->name + ":velocitylimit", robotvar,
                                           -velocity_scale*jnt->limits->velocity,
                                            velocity_scale*jnt->limits->velocity);
                }
                if (poslimits) {
                    ctx->addInequalityConstraint(jnt->name+":positionlimit",
                                                 inp,
                                                 jnt->limits->lower, K_limits,
                                                 jnt->limits->upper, K_limits, 1.0, 1);
                }
            }
            return Constant(F_parent_jnt)*frame(Constant(axis)*inp);
        }
        default : {
            ROS_WARN("Converting unknown joint type of joint '%s' into a fixed joint", jnt->name.c_str());
            return  Constant(F_parent_jnt);
        }
    }
    return Constant(F_parent_jnt);
}



 */

}// namespace KDL
