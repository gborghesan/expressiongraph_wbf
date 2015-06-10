#ifndef EXPRESSIONGRAPH_URDFEXPR2_HPP
#define EXPRESSIONGRAPH_URDFEXPR2_HPP


#include <string>
#include <set>
#include <map>
#include <vector>

#include <kdl/frames_io.hpp>
#include <kdl/expressiontree.hpp>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>


#include <kdl/frames_io.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <ros/ros.h>
#include <stdlib.h>

namespace KDL {

struct link_property {
    link_property() {
        name="";
        mass=0;
        Ixx=0;
        Ixy=0;
        Ixz=0;
        Iyy=0;
        Iyz=0;
        Izz=0;
        origin = Frame::Identity();
    }
    std::string name;
    double mass;
    double Ixx;
    double Ixy;
    double Ixz;
    double Iyy;
    double Iyz;
    double Izz;
    Frame  origin;
};

class UrdfExpressions2 {
public:
    typedef boost::shared_ptr<const urdf::Link>                               LinkPtr;
    typedef boost::shared_ptr<const urdf::Joint>                              JointPtr;
    typedef std::map<std::string, LinkPtr>                                    LinkMap;
    typedef std::map<std::string, typename KDL::Expression<KDL::Frame>::Ptr > ExprMap;
    typedef std::pair<LinkPtr,LinkPtr>                                        Transform;
    typedef std::vector<Transform >                                           TransformList;
    typedef std::vector<LinkPtr>                                              LinkList;
    typedef std::set<JointPtr>                                                JointSet;
    typedef std::set< LinkPtr >                                               LinkSet;
    LinkMap  linkmap;
    boost::shared_ptr<urdf::Link>    root_link;
    TransformList                    transformlist;
    LinkList                         rootlist;



    std::map<std::string, unsigned int> joint_map;


    bool readFromFile( const std::string& filename );
    bool readFromString(const std::string& xml_string);

    void hash_names( const boost::shared_ptr<urdf::Link>&  link, int level=0);
    KDL::Vector toKdl(urdf::Vector3 v) ;
    KDL::Rotation toKdl(urdf::Rotation r);
    KDL::Frame toKdl(urdf::Pose p) ;

     void addJoint(const boost::shared_ptr<urdf::Link>& link, std::vector<std::string>& names);

    void getAllJointNames(std::vector<std::string>& names);

    void generateJointMap(unsigned int initial_index=0);

    KDL::Expression<KDL::Frame>::Ptr  toKdl(const JointPtr& jnt);

    bool addTransform(const std::string& frame, const std::string& base);

    KDL::Expression<KDL::Frame>::Ptr compose_tree(
        const LinkPtr& p,
        const LinkPtr& p_root
    );
    KDL::Expression<KDL::Frame>::Ptr getExpression( int i);

    void getAllLinkProperties( std::vector<link_property>& props, UrdfExpressions2::LinkPtr p_root=UrdfExpressions2::LinkPtr());

};


} // namespace
#endif
