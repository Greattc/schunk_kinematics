#include "schunk_kinematics/arm_kinematics.h"

using namespace std;
using namespace Eigen;

#define WARN_ANGLE_1 2.147
#define WARN_ANGLE_2 2.147
#define WARN_ANGLE_3 2.200
#define RANDNM(N,M) N + ((M-N) * (rand() / ((double)RAND_MAX + 1))) // random # between N&M

#define L_ACC 0.4	   //笛卡尔直线运动加速度极限
#define C_ACC 0.4	   //笛卡尔圆周运动加速度极限
#define S_ACC 10.0	   //笛卡尔自运动加速度极限
#define VEL 1.0       //笛卡尔运动速度极限

////正向运动学////
MatrixXd forward_kinematics(const VectorXd& seta, const VectorXd& Linklen)
{
    MatrixXd A1(4,4), A2(4,4), A3(4,4), A4(4,4), A5(4,4), A6(4,4), A7(4,4), A(4,4);
    A1 << cos(seta(0)),0,-sin(seta(0)),0,sin(seta(0)),0,cos(seta(0)),0,0,-1,0,Linklen(0),0,0,0,1;
    A2 << cos(seta(1)),0,sin(seta(1)),0,sin(seta(1)),0,-cos(seta(1)),0,0,1,0,0,0,0,0,1;
    A3 << cos(seta(2)),0,-sin(seta(2)),0,sin(seta(2)),0,cos(seta(2)),0,0,-1,0,Linklen(2),0,0,0,1;
    A4 << cos(seta(3)),0,sin(seta(3)),0,sin(seta(3)),0,-cos(seta(3)),0,0,1,0,0,0,0,0,1;
    A5 << cos(seta(4)),0,-sin(seta(4)),0,sin(seta(4)),0,cos(seta(4)),0,0,-1,0,Linklen(4),0,0,0,1;
    A6 << cos(seta(5)),0,sin(seta(5)),0,sin(seta(5)),0,-cos(seta(5)),0,0,1,0,0,0,0,0,1;
    A7 << cos(seta(6)),-sin(seta(6)),0,0,sin(seta(6)),cos(seta(6)),0,0,0,0,1,Linklen(6),0,0,0,1;
    A = A1*A2*A3*A4*A5*A6*A7;
    return A;
}

////逆向运动学////
VectorXd& inverse_kinematics(VectorXd& seta, const VectorXd& Linklen, const MatrixXd& Goal, bool random, const double user_theta)
{
    MatrixXd A1(4,4), A2(4,4), A3(4,4), A4(4,4), A5(4,4), A6(4,4), A7(4,4), A(4,4);
    //三个关节位置
    VectorXd s_pos(3), w_pos(3), e_pos(3), pos(3), orient_z(3);
    Vector3d z_axis_middle, x_axis, y_axis, z_axis, s_e, e_w, temp_a, temp_b, temp_c;

    /*############################------准备过程-------###################################*/

    double d[4], angle;
    for(size_t i = 0; i<4; ++i) d[i] = Linklen(2*i);
    for(size_t i = 0; i<3; ++i) pos(i) = Goal(i,3);
    for(size_t i = 0; i<3; ++i) orient_z(i) = Goal(i,2);
    for(size_t i = 0; i<3; ++i) w_pos(i) = pos(i)-d[3] * orient_z(i);

    s_pos(0) = s_pos(1) = 0; s_pos(2) = d[0];

    temp_b = w_pos - s_pos;
    double distance = temp_b.norm();
//    cout<<"distance ="<<distance<<endl;
    if(distance > (d[1]+d[2])){
        cout<<"############ The Designated Goal is Too Far! ##############"<<endl;
        return seta;
    }
    else if(distance < fabs(d[1]-d[2])){
        cout<<"############ The Designated Goal is Too Near! #############"<<endl;
        return seta;
    }
    else if(distance == (d[1] + d[2])){
        //直立奇异位置，设最后一个关节转动
        cout << "###### Singular Position! #######" << endl;
		for(size_t i=0; i<6; ++i)
			seta(i) = 0;
        seta(6) = atan2(Goal(1,0), Goal(0,0));
        return seta;
    }

    //检查三角形的角度是否超出Joint4的限制
    double warn_angle_4 = M_PI - triangle(d[1], d[2], distance);
    if(warn_angle_4 >= WARN_ANGLE_2){
        cout<<"########### Joint 4 is Beyond the Limit ############"<<endl;
		return seta;
    }

    /*############################------求解算法过程------################################*/

    //确定x-y-z轴位置,转换为单位向量
	temp_a = s_pos.normalized();
    z_axis = temp_b.normalized();
    //末端位置在世界坐标系Z轴上
    if(temp_a == z_axis){
        x_axis << 1.0, 0.0, 0.0;
    }
    else{
        x_axis = temp_a.cross(z_axis);
    }
	y_axis = z_axis.cross(x_axis);
	x_axis.normalize();
	y_axis.normalize();

    /*#####################----随机产生角度并求得Elbow处的坐标----#########################*/

	double theta;
    if(random){
        theta = RANDNM(0, 2*M_PI);
    }
    else{
        theta = user_theta * M_PI / 180.0 - M_PI;
    }
    angle = triangle(d[1], distance, d[2]);
	double radius = d[1]*sin(angle);
    temp_c = s_pos + d[1]*cos(angle)*z_axis;
    e_pos = radius*(sin(theta)*x_axis - cos(theta)*y_axis) + temp_c;

    s_e = e_pos - s_pos;  //shoulder-to-elbow
    e_w = w_pos - e_pos;  //elbow-to-wrist
	s_e.normalize();
	e_w.normalize();

	bool sign;
    if(RANDNM(0,1) < 0.5){
		z_axis_middle = s_e.cross(e_w);
		sign = true;
	}
    else{
		z_axis_middle = -s_e.cross(e_w);//Joint4的Z-axis方向。
		sign = false;
	}
	z_axis_middle.normalize();


    //计算seta[0,1,2,3]
    Vector3d temp_h = - s_e;  //y_axis
    temp_h.normalize();
    if(temp_h(0) == 0 && temp_h(1) == 0){   //sin(seta(5)) == 0
		seta(1) = atan2(0,-temp_h(2));
		seta(0) = 0;		//Random value. seta(0)+seta(2)=fixed value
		seta(2) = atan2(-z_axis_middle(0), z_axis_middle(1));
	}
    else{
        if(temp_h(0) == 0){    ///cos(seta(1)) == 0
            if(RANDNM(0,1) < 0.5){
                seta(0) = atan2(1,0);
            }
            else{
                seta(0) = atan2(-1,0);

            }
            seta(1) = atan2(-temp_h(1)/sin(seta(0)), -temp_h(2));
        }
        else if(temp_h(1) == 0){    //sin(seta(0)) == 0
            if(RANDNM(0,1) < 0.5){
                seta(0) = atan2(0,1);
            }
            else{
                seta(0) = atan2(0,-1);
            }
            seta(1) = atan2(-temp_h(0)/cos(seta(0)), -temp_h(2));
        }
        else{
            if(RANDNM(0,1) < 0.5)	seta(1) = acos(-temp_h(2));
            else	    seta(1) = -acos(-temp_h(2));
            seta(0) = atan2(-temp_h(1)/sin(seta(1)), -temp_h(0)/sin(seta(1)));
        }
        Vector3d temp_i = temp_h.cross(z_axis_middle);  //x_axis
        temp_i.normalize();
        seta(2) = atan2(z_axis_middle(2)/sin(seta(1)), -temp_i(2)/sin(seta(1)));
    }

    if(sign){
        seta(3) = acos(s_e.dot(e_w));
    }
    else{
        seta(3) = -acos(s_e.dot(e_w));
    }

    A1<<cos(seta(0)),0,-sin(seta(0)),0,sin(seta(0)),0,cos(seta(0)),0,0,-1,0,Linklen(0),0,0,0,1;
    A2<<cos(seta(1)),0,sin(seta(1)),0,sin(seta(1)),0,-cos(seta(1)),0,0,1,0,0,0,0,0,1;
    A3<<cos(seta(2)),0,-sin(seta(2)),0,sin(seta(2)),0,cos(seta(2)),0,0,-1,0,Linklen(2),0,0,0,1;
    A4<<cos(seta(3)),0,sin(seta(3)),0,sin(seta(3)),0,-cos(seta(3)),0,0,1,0,0,0,0,0,1;
    A = (A1*A2*A3*A4).inverse()*Goal;

    //cout<<A1*A2*A3<<endl;

    //计算seta[4,5]
    if(A(0,2) == 0 && A(1,2) == 0){   //sin(seta(5)) == 0
        seta(5) = atan2(0,A(2,2));
        seta(4) = 0;		//Random value. seta(4)+seta(6)=fixed value
    }
    else{
        if(A(0,2)==0){	   //cos(seta(4)) == 0
            if(RANDNM(0,1) < 0.5){
                seta(4) = atan2(1,0);
            }
            else{
                seta(4) = atan2(-1,0);
            }
            seta(5) = atan2(A(1,2)/sin(seta(4)), A(2,2));
        }
        else if(A(1,2)==0){	  //sin(seta(4)) == 0
            if(RANDNM(0,1) < 0.5){
                seta(4) = atan2(0,1);
            }
            else{
                seta(4) = atan2(0,-1);
            }
            seta(5) = atan2(A(0,2)/cos(seta(4)), A(2,2));
        }
        else{
            if(RANDNM(0,1) < 0.5){
                seta(5) = acos(A(2,2));
            }
            else{
                seta(5) = -acos(A(2,2));
            }
			seta(4) = atan2(A(1,2)/sin(seta(5)), A(0,2)/sin(seta(5)));
		}
	}

    //计算seta[6]
	A5<<cos(seta(4)),0,-sin(seta(4)),0,sin(seta(4)),0,cos(seta(4)),0,0,-1,0,Linklen(4),0,0,0,1;
	A6<<cos(seta(5)),0,sin(seta(5)),0,sin(seta(5)),0,-cos(seta(5)),0,0,1,0,0,0,0,0,1;
	A = (A5*A6).inverse()*A;
	seta(6) = atan2(A(1,0),A(0,0));

    if(fabs(seta(1)) > WARN_ANGLE_1){
        cout<<"########### Joint 2 is Beyond the Limit ############"<<endl;
        return seta;
    }
    if(fabs(seta(5)) > WARN_ANGLE_3){
        cout<<"########### Joint 6 is Beyond the Limit ############"<<endl;
        return seta;
    }

	return seta;
}

////当前的三角平面角度////
double angleRotation(const VectorXd& seta, const VectorXd& Linklen)
{
    MatrixXd A1(4,4), A2(4,4), A3(4,4), A4(4,4), A5(4,4), A(4,4);
    A1 << cos(seta(0)),0,-sin(seta(0)),0,sin(seta(0)),0,cos(seta(0)),0,0,-1,0,Linklen(0),0,0,0,1;
    A2 << cos(seta(1)),0,sin(seta(1)),0,sin(seta(1)),0,-cos(seta(1)),0,0,1,0,0,0,0,0,1;
    A3 << cos(seta(2)),0,-sin(seta(2)),0,sin(seta(2)),0,cos(seta(2)),0,0,-1,0,Linklen(2),0,0,0,1;
    A4 << cos(seta(3)),0,sin(seta(3)),0,sin(seta(3)),0,-cos(seta(3)),0,0,1,0,0,0,0,0,1;
    A5 << cos(seta(4)),0,-sin(seta(4)),0,sin(seta(4)),0,cos(seta(4)),0,0,-1,0,Linklen(4),0,0,0,1;

    VectorXd s_pos(3), w_pos(3), e_pos(3);

    A = A1;
    for(size_t i = 0; i<3; ++i) s_pos(i) = A(i,3);
    A *= A2*A3;
    for(size_t i = 0; i<3; ++i) e_pos(i) = A(i,3);
    A *= A4*A5;
    for(size_t i = 0; i<3; ++i) w_pos(i) = A(i,3);

    Vector3d s_e, s_w, normal_init, normal;
    for(size_t i = 0; i<3; ++i) s_e(i) = e_pos[i] - s_pos[i];
    for(size_t i = 0; i<3; ++i) s_w(i) = w_pos[i] - s_pos[i];

    s_e.normalize();
    s_w.normalize();

    Vector3d z_axis;
    z_axis << 0,0,1;
    normal_init = s_w.cross(z_axis);
    normal_init.normalize();

    normal = s_w.cross(s_e);
    normal.normalize();
//    cout << s_e << endl;
//    cout << s_w << endl;

    VectorXd temp = normal_init.cross(normal);
    double angle = acos(normal.dot(normal_init));

    if(temp.dot(s_w) < 0)
        angle = -angle;
    angle = angle*180/M_PI;
    return angle;
}

////选择离先前角度最接近的逆解（８组中选择一个）////
void minimum_energy(VectorXd& seta, const VectorXd& beta)
{
	vector<VectorXd> vec;
	
	for(int i=0; i<8; ++i)
		vec.push_back(seta);
	
	for(int i=0; i<4; ++i)
		vec[i][3] = - vec[i][3];
	for(int i=0; i<2; ++i)
	{
		vec[i][0] = vec[i][0]>0? (vec[i][0]-M_PI):(vec[i][0]+M_PI);
		vec[i][1] = -vec[i][1];
	}
	for(int i=2; i<4; ++i)
	{
		vec[i][2] = vec[i][2]>0? (vec[i][2]-M_PI):(vec[i][2]+M_PI);
	}	
		
	for(int i=4; i<6; ++i)
	{
		vec[i][0] = vec[i][0]>0? (vec[i][0]-M_PI):(vec[i][0]+M_PI);
		vec[i][1] = -vec[i][1];
		vec[i][2] = vec[i][2]>0? (vec[i][2]-M_PI):(vec[i][2]+M_PI);
	}
	for(int i=0; i<2; ++i)
	{
		vec[2*i][5] = -vec[2*i][5];
		vec[2*i][6] = vec[2*i][6]>0? (vec[2*i][6]-M_PI):(vec[2*i][6]+M_PI);
	}
	for(int i=0; i<2; ++i)
	{
		vec[2*i+1][4] = vec[2*i+1][4]>0? (vec[2*i+1][4]-M_PI):(vec[2*i+1][4]+M_PI);
	}
	for(int i=0; i<2; ++i)
	{
		vec[2*i+5][4] = vec[2*i+5][4]>0? (vec[2*i+5][4]-M_PI):(vec[2*i+5][4]+M_PI);
		vec[2*i+5][5] = -vec[2*i+5][5];
		vec[2*i+5][6] = vec[2*i+5][6]>0? (vec[2*i+5][6]-M_PI):(vec[2*i+5][6]+M_PI);
	}
	
	int count;
	double max_distance = 10000;
	for(int i=0; i<8; ++i)
	{
		double actual_distance = (vec[i]-beta).norm();
		if( actual_distance < max_distance)
		{
			max_distance = actual_distance;
			count = i;
		}
	}
	seta = vec[count];
}

////求解单个关节齐次变换矩阵////
MatrixXd homoMatrix(const double theta, const double d, const double alpha, const double a)
{
	MatrixXd transform(4,4);
	transform(0,0) = cos(theta);
    transform(0,1) = -sin(theta)*cos(alpha);
    transform(0,2) = sin(theta)*sin(alpha);
    transform(0,3) = a*cos(theta);

	transform(1,0) = sin(theta);
    transform(1,1) = cos(theta)*cos(alpha);
    transform(1,2) = -cos(theta)*sin(alpha);
    transform(1,3) = a*sin(theta);

	transform(2,0) = 0;
    transform(2,1) = sin(alpha);
    transform(2,2) = cos(alpha);
    transform(2,3) = d;

	transform(3,0) = transform(3,1) = transform(3,2) = 0;
	transform(3,3) = 1;

	return transform;
}
////雅克比矩阵求解////
void getJacobi(MatrixXd& J, const VectorXd& seta)
{
    unsigned int ndim = seta.size();

    VectorXd Linklen(ndim), alpha(ndim), a(ndim);

    Linklen << 0.3, 0.0, 0.328, 0.0, 0.276, 0.0, 0.3977;  //Grasp Link
    alpha << -M_PI_2, M_PI_2, -M_PI_2, M_PI_2, -M_PI_2, M_PI_2, 0.0;
    a << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    J.resize(6, ndim);
    MatrixXd transform = MatrixXd::Identity(4,4);

    std::vector<Vector3d> vec;
    for(unsigned int i=0; i<ndim; ++i)
    {
        MatrixXd transform_t = homoMatrix(seta[i], Linklen[i], alpha[i], a[i]);
        transform = transform*transform_t;
        if( i < ndim -1)
            J.block(3,i+1,3,1) = transform.block(0,2,3,1);

        Vector3d vector;
        vector = transform.block(0,3,3,1);
        vec.push_back(vector);
    }

    VectorXd z_axis(3);
    z_axis << 0,0,1;
    J.block(3,0,3,1) = z_axis ;

    for(unsigned int i=0; i<ndim; ++i)
    {
        //r1--r7
        Vector3d vector = i ? (vec[ndim-1]-vec[i-1]) : (vec[ndim-1]);

        Vector3d vecBlock = J.block(3,i,3,1);
        Vector3d crossVec = vecBlock.cross(vector);
        J.block(0,i,3,1) = crossVec;
    }
}

VectorXd zyzAngle(const MatrixXd& init_matrix, const MatrixXd& goal_matrix)
{
    VectorXd rotation_angle(3);
    MatrixXd init_rotation = init_matrix.topLeftCorner(3,3);
    MatrixXd goal_rotation = goal_matrix.topLeftCorner(3,3);
    MatrixXd rotation = init_rotation.transpose()*goal_rotation;

    /*
    double sin_beta = sqrt(rotation(0,2) * rotation(0,2) + rotation(1,2) *  rotation(1,2));
    rotation_angle[1] = atan2(sin_beta, rotation(2,2));
    if(sin_beta!= 0){
        rotation_angle[0] = atan2(rotation(1,2), rotation(0,2));
        rotation_angle[2]= atan2(rotation(2,1), -rotation(2,0));
    }
    else if(rotation(2,2) = 1){
        rotation_angle[0] = 0;
        rotation_angle[2]= atan2(-rotation(2,1), rotation(2,0));

    }
    else{
        rotation_angle[0] = 0;
        rotation_angle[2]= atan2(rotation(2,1), -rotation(2,0));
    }

    return rotation_angle;
    */
/*
    double sin_beta = sqrt(rotation(2,0) * rotation(2,0) + rotation(2,1) *  rotation(2,1));
    rotation_angle[1] = atan2(sin_beta, rotation(2,2));
    if(sin(rotation_angle[1]) != 0)
    {
        rotation_angle[0] = atan2(rotation(1,2)/sin(rotation_angle[1]),  rotation(0,2)/sin(rotation_angle[1]));
        rotation_angle[2] = atan2(rotation(2,1)/sin(rotation_angle[1]), -rotation(2,0)/sin(rotation_angle[1]));
    }
    else if(sin_beta == 0)
    {
        rotation_angle[0] = 0;
        rotation_angle[1] = 0;
        rotation_angle[2] = atan2(-rotation(0,1), rotation(0,0));

    }
    return rotation_angle;
*/
    double epsilon = 1E-12;
    if (fabs(rotation(2,2)) > 1-epsilon  ) {
        rotation_angle[2]=0.0;
        if (rotation(2,2)>0) {
            rotation_angle[1] = 0.0;
            rotation_angle[0]= atan2(rotation(1,0),rotation(0,0));
        } else {
            rotation_angle[1] = M_PI;
            rotation_angle[0]= atan2(-rotation(1,0),-rotation(0,0));
        }
    } else {
        rotation_angle[0]=atan2(rotation(1,2), rotation(0,2));
        rotation_angle[1]=atan2(sqrt( pow(rotation(2,0), 2) +pow(rotation(2,1), 2) ),rotation(2,2));
        rotation_angle[2]=atan2(rotation(2,1), -rotation(2,0));
    }

    return rotation_angle;

}

MatrixXd zyzRotation(const VectorXd& angle){
    assert(angle.size()==3);
    MatrixXd zyzRationMatrix_(4,4);
    zyzRationMatrix_(0,0) = cos(angle[0])*cos(angle[1])*cos(angle[2])-sin(angle[0])*sin(angle[2]);
    zyzRationMatrix_(0,1) = -cos(angle[0])*cos(angle[1])*sin(angle[2])-sin(angle[0])*cos(angle[2]);
    zyzRationMatrix_(0,2) = cos(angle[0])*sin(angle[1]);
    zyzRationMatrix_(0,3) = 0;

    zyzRationMatrix_(1,0) = sin(angle[0])*cos(angle[1])*cos(angle[2])+cos(angle[0])*sin(angle[2]);
    zyzRationMatrix_(1,1) = -sin(angle[0])*cos(angle[1])*sin(angle[2])+cos(angle[0])*cos(angle[2]);
    zyzRationMatrix_(1,2) = sin(angle[0])*sin(angle[1]);
    zyzRationMatrix_(1,3) = 0;

    zyzRationMatrix_(2,0) = -sin(angle[1])*cos(angle[2]);
    zyzRationMatrix_(2,1) = sin(angle[1])*sin(angle[2]);
    zyzRationMatrix_(2,2) = cos(angle[1]);
    zyzRationMatrix_(2,3) = 0;

    zyzRationMatrix_(3,0) = 0;
    zyzRationMatrix_(3,1) = 0;
    zyzRationMatrix_(3,2) = 0;
    zyzRationMatrix_(3,3) = 1;

    return zyzRationMatrix_;
}

////笛卡尔空间直线插值运动////
MatrixXd lineInterplot(const MatrixXd& init_matrix, const MatrixXd& goal_matrix, double run_time, double& duration_time)
{
    assert(run_time >= 0);

    MatrixXd transform_matrix(4,4);
    VectorXd init_position(3),goal_position(3),delta_position(3);
    for(int i=0; i<3; ++i){
        init_position[i] = init_matrix(i,3);
        goal_position[i] = goal_matrix(i,3);
    }
    delta_position = goal_position - init_position;

    double linelength = (goal_position-init_position).norm();

    double length;

    double time_1 = VEL/L_ACC, time_2;
    double length_1 = 0.5*VEL*VEL/L_ACC;
    double length_2 = linelength - 2*length_1;

    if(length_2 >= 0){
        time_2 = length_2/VEL;
        duration_time = 2*time_1+time_2;
        if(run_time < time_1 )
            length = 0.5*L_ACC*run_time*run_time;
        else if(run_time < (time_1+time_2))
            length = length_1 + VEL*(run_time-time_1);
        else
            length = linelength - 0.5*L_ACC*(duration_time-run_time)*(duration_time-run_time);
    }
    else{
        length_1 = length_2 = linelength/2;
        time_1 = time_2 = sqrt(2*length_1/L_ACC);
        duration_time = time_1+time_2;
        if(run_time < time_1)
            length = 0.5*L_ACC*run_time*run_time;
        else
            length = linelength - 0.5*L_ACC*(duration_time-run_time)*(duration_time-run_time);
    }

    //cout<<linelength<<endl;
    //cout<<length_2<<endl;
    //cout<<time_1<<"\t"<<time_2<<"\t"<<duration_time<<endl;
    double ratio = length/linelength;
    VectorXd zyz_angle(3);
    zyz_angle = zyzAngle(init_matrix, goal_matrix);
    zyz_angle = ratio * zyz_angle;

    transform_matrix = zyzRotation(zyz_angle);
    transform_matrix = init_matrix*transform_matrix;

    for(int i=0; i<3; ++i){
        transform_matrix(i,3) = init_position[i]+delta_position[i]*ratio;
    }

    return transform_matrix;
}

////笛卡尔空间圆周插值运动////
MatrixXd circleInterplot(const MatrixXd& init_matrix, const MatrixXd& goal_matrix, const VectorXd& center, \
                         double run_time, double& duration_time, bool clockwise){

    assert(run_time >= 0);

    MatrixXd transform_matrix(4,4);
    VectorXd init_position(3),goal_position(3),delta_position(3);
    for(int i=0; i<3; ++i){
        init_position[i] = init_matrix(i,3);
        goal_position[i] = goal_matrix(i,3);
    }

    double radius = (init_position-center).norm();
    double radius_copy = (goal_position-center).norm();
    //cout<<radius<<"\t"<<radius_copy<<endl;
    assert((radius-radius_copy)<1e-5);

    delta_position = goal_position-init_position;
    double linelength = delta_position.norm();
    double angle;
    if((linelength - 2*radius)<1e-5){
        angle = M_PI;
    }
    else if(linelength<1e-5){
        angle = 2*M_PI;
    }
    else{
        double height = sqrt(radius*radius-(linelength/2)*(linelength/2));
        angle = 2*atan2(linelength/2, height);
    }
    double anglength = angle*radius;
//    cout<<anglength<<endl;

    double time_1 = VEL/C_ACC, time_2;
    double length_1 = 0.5*VEL*VEL/C_ACC;
    double length_2 = anglength - 2*length_1;
    double length;
    //cout<<anglength<<endl;
    if(length_2 >= 0){

        time_2 = length_2/VEL;
        duration_time = 2*time_1+time_2;

        if(run_time < time_1 )
            length = 0.5*C_ACC*run_time*run_time;
        else if(run_time < (time_1+time_2))
            length = length_1 + VEL*(run_time-time_1);
        else
            length = anglength - 0.5*C_ACC*(duration_time-run_time)*(duration_time-run_time);
    }
    else{
        length_1 = length_2 = anglength/2;
        time_1 = time_2 = sqrt(2*length_1/C_ACC);
        duration_time = time_1+time_2;
        if(run_time < time_1)
            length = 0.5*C_ACC*run_time*run_time;
        else
            length = anglength - 0.5*C_ACC*(duration_time-run_time)*(duration_time-run_time);
    }

    //cout<<linelength<<endl;
    //cout<<length_2<<endl;
//    cout<<time_1<<"\t"<<time_2<<"\t"<<duration_time<<endl;

    double ratio = length/anglength;
    double run_angle = ratio*angle;
//    cout<<run_angle<<endl;

    Vector3d x_axis,y_axis,z_axis,temp_pos, goal_axis;
    x_axis = (init_position-center).normalized();
    goal_axis = (goal_position-center).normalized();
    z_axis = x_axis.cross(goal_axis);
    if(!clockwise)    z_axis = -z_axis;
    y_axis = z_axis.cross(x_axis);
    temp_pos = radius*(cos(run_angle)*x_axis+sin(run_angle)*y_axis);
    temp_pos = temp_pos+center;

    VectorXd zyz_angle(3);
    zyz_angle = zyzAngle(init_matrix, goal_matrix);
    zyz_angle = ratio * zyz_angle;

    transform_matrix = zyzRotation(zyz_angle);
    transform_matrix = init_matrix*transform_matrix;

    for(int i=0; i<3; ++i){
        transform_matrix(i,3) = temp_pos[i];
//        cout << temp_pos[i] << "\t";
    }
//    cout << endl;

    return transform_matrix;
}

////笛卡尔空间自运动////
double selfMotionTime(const double run_time, double& duration_time, const double init_angle, const double angle, bool clockwise)
{
    double max_vel = VEL * 5;
    double time_1 = max_vel/S_ACC, time_2;
    double angle_1= 0.5*max_vel*max_vel/S_ACC;
    double angle_2 = angle - 2*angle_1;
    double length;

    if(angle_2 >= 0){
        time_2 = angle_2/max_vel;
        duration_time = 2*time_1+time_2;
        if(run_time < time_1 )
            length = 0.5*S_ACC*run_time*run_time;
        else if(run_time < (time_1+time_2))
            length = angle_1+ max_vel*(run_time-time_1);
        else
            length = angle - 0.5*S_ACC*(duration_time-run_time)*(duration_time-run_time);
    }
    else{
        angle_1 = angle_2 = angle/2;
        time_1 = time_2 = sqrt(2*angle_1/S_ACC);
        duration_time = time_1+time_2;
        if(run_time < time_1)
            length = 0.5*S_ACC*run_time*run_time;
        else
            length = angle - 0.5*S_ACC*(duration_time-run_time)*(duration_time-run_time);
    }

    double current_angle;
    if(clockwise)
        current_angle = init_angle + length;
    else
        current_angle = init_angle - length;

    return current_angle;
}

////笛卡尔空间自运动插值////
VectorXd selfMotionInterplot(const MatrixXd& init_matrix, const VectorXd& last, const VectorXd& Linklen, const double current_angle)
{
    VectorXd seta(7);

    inverse_kinematics(seta, Linklen, init_matrix, false, current_angle);
    minimum_energy(seta, last);

    return seta;
}
