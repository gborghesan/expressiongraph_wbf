#define FOUR_DOF_ROBOT \
	std::vector<double> inp(4);\
	inp[0] = 1;\
	inp[1] = 1.5;\
	inp[2] = -0.5;\
	inp[3] = 2;\
	std::vector<int> joint_indexes(4);\
	joint_indexes[0]=1;\
	joint_indexes[1]=2;\
	joint_indexes[2]=3;\
	joint_indexes[3]=4;\
	Expression<Vector>::Ptr L      = Constant(Vector(0,0,0.2));\
	Expression<Vector>::Ptr L0=  Constant(Vector(0,0,0.0));\
	\
	Expression<Rotation>::Ptr R0=  Constant(Rotation(KDL::Rotation::Identity()));\
	\
	Expression<Frame>::Ptr w_T_l1  =       frame( rot_z(input(1)), L0)*frame( R0, L);\
	Expression<Frame>::Ptr w_T_l2 = w_T_l1*frame( rot_x(input(2)), L0)*frame( R0, L);\
	Expression<Frame>::Ptr w_T_l3 = w_T_l2*frame( rot_z(input(3)), L0)*frame( R0, L);\
	Expression<Frame>::Ptr w_T_ee = w_T_l3*frame( rot_x(input(4)), L0)*frame( R0, L);\
	\
	Expression<Rotation>::Ptr w_R_ee=rotation(w_T_ee);\
	Expression<double>::Ptr w_x_ee=coord_x(origin( (w_T_ee)));\
	Expression<double>::Ptr w_y_ee=coord_y(origin( (w_T_ee)));\
	Expression<double>::Ptr w_z_ee=coord_z(origin( (w_T_ee)));


#define FOUR_DOF_KDL_ROBOT \
	\
	KDL::Chain chain;\
	KDL::Vector L2(0,0,0.2);\
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(L2)));\
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(L2)));\
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(L2)));\
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(L2)));\
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);\
	ChainJntToJacSolver Jsolver = ChainJntToJacSolver(chain);\
	unsigned int nj = chain.getNrOfJoints();\
	KDL::JntArray jointpositions = JntArray(nj);\
	for(unsigned int i=0;i<nj;i++)\
	jointpositions(i)=inp[i];\
	\
	KDL::Frame cartpos;\
	KDL::Jacobian JKDL(nj);\
	fksolver.JntToCart(jointpositions,cartpos);\
	Jsolver.JntToJac(jointpositions,JKDL);


#define FLOATING_BASE_ROBOT \
	KDL::Expression<KDL::Frame>::Ptr w_T_ee= \
	frame( inputRot(4),KDL::vector(input(1),input(2),input(3)));\
	\
	std::vector<int> joint_indexes_out(6);\
	std::vector<int> joint_indexes_scalar(3);\
	std::vector<int> joint_indexes_rot(1);\
	joint_indexes_out[0]=1;\
	joint_indexes_out[1]=2;\
	joint_indexes_out[2]=3; \
	joint_indexes_out[3]=4;\
	joint_indexes_out[4]=5;\
	joint_indexes_out[5]=6; \
	joint_indexes_scalar[0]=1;\
	joint_indexes_scalar[1]=2; \
	joint_indexes_scalar[2]=3;\
	joint_indexes_rot[0]=4;\
	std::vector<double> inputPosValue(3,0.0);\
	std::vector<KDL::Rotation> inputRotValue(1,Rotation::Identity());


