#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <cstring>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

// srv폴더에 만든 서비스 파일에서 생성되는 헤더파일로 catkin_make 실행 중 생성된다.
#include "turtlebot3_practice_move/square.h"


// 기본 속도; 너무 빠르면 전진 거리/회전 각도가 부정확해진다.
const double default_forward_speed = 0.5;
const double default_angular_speed = 1;
const int polygon = 4;

// Turtlebot3 Limitations
const float BURGER_MAX_LIN_VEL = 0.22;
const float BURGER_MAX_ANG_VEL = 2.84;
const float WAFFLE_MAX_LIN_VEL = 0.26;
const float WAFFLE_MAX_ANG_VEL = 1.82;
const float LIN_VEL_STEP_SIZE = 0.01;
const float ANG_VEL_STEP_SIZE = 0.1;


// Turtlebot3 모델 이름
std::string turtlebot3_model;

// main함수만이 아닌 Service Callback 함수에서도 publisher를 참조해야 하므로 전역 변수로 설정
// NodeHandler를 전역변수로 설정할 경우 ros::init이 실행되기 전 초기화를 시도하면서 오류가 발생한다.
ros::Publisher pub;

// Odometry Subscription
ros::Subscriber sub_tb3_odom;
nav_msgs::Odometry::ConstPtr tb3_odom;

// 그리기 옵션들
float side_length = 0;
int32_t rotations = 0;
int currentDrawingLine = -1;// 현재 그리고 있는 사각형 몇 개 선분 완성됐는지, -1이면 쉬고 있는 상태
bool isTurning = false; // 사각형 그리는 중, 현재 회전중인지
double initPositionX = 0;
double initPositionY = 0;
double initRotationZ = 0;


const float makeSimpleProfile(float output, float input, float slop){
	if(input > output){
        output = (input < output + slop) ? input : (output+slop);
	}else if (input < output){
		output = (input > output - slop) ? input : (output-slop);
	}else{
		output = input;
	}
	return output;
}

const float constrain(float input, float low, float high){
	if (input < low){
		input = low;
	}else if (input > high){
		input = high;
	}else{
		input = input;
	}
	return input;
}

const float checkLinearLimitVelocity(float vel){
	if (turtlebot3_model == "burger"){
		vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL);
	}else if (turtlebot3_model == "waffle" || turtlebot3_model == "waffle_pi"){
		vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL);
	}else{
		vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL);
	}
	return vel;
}

const float checkAngularLimitVelocity(float vel){
	if (turtlebot3_model == "burger"){
		vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL);
	}else if (turtlebot3_model == "waffle" || turtlebot3_model == "waffle_pi"){
		vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL);
	}else{
		vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL);
	}
	return vel;
}

const double pointDistance(double x0, double y0, double x1, double y1)
{
	return sqrt(pow(x0-x1,2)+pow(y0-y1,2));
}

const double getAngleFromOdom(nav_msgs::Odometry::ConstPtr &odom, double &roll, double &pitch, double &yaw){
	tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
	return yaw;
}

const double angleDistance(double y_final, double y_init){
	double result = y_final - y_init;
	return ((result < 0) ? result + 2*M_PI : result) ;
}

const float smoothAngularSpeed(double y_target, double y_current, double threshold, double angular_speed)
{
	if (angleDistance(y_target, y_current) >= threshold){
		return angular_speed;
	}else{
		return angular_speed/5 * (4 + angleDistance(y_target, y_current) / threshold);
	}
}

// 직선 그리기 (&vel_msg=속도 메시지(토픽); &pub=vel_msg 퍼블리셔; forward_speed=병진 속도 )
void forward(geometry_msgs::Twist &vel_msg, ros::Publisher &pub, float forward_speed)
{
	// vel_msg에 포함된 3차원 속도 값 초기화
	vel_msg.linear.x = forward_speed; //turtle은 전진만 하므로 x를 제외한 나머지 속도를 0으로 맞춘다.
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	pub.publish(vel_msg);
	// ROS_INFO("x:%f y:%f z:%f", tb3_odom->pose.pose.position.x, tb3_odom->pose.pose.position.y, tb3_odom->pose.pose.position.z);
}

// 직각 회전하기 (&vel_msg=속도 메시지(토픽); &pub=vel_msg 퍼블리셔; angular_speed=각속도)
void rotate(geometry_msgs::Twist &vel_msg, ros::Publisher &pub, float angular_speed)
{
	// vel_msg에 포함된 3차원 속도 값 중 각속도만 초기화
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = angular_speed;

	pub.publish(vel_msg);
}

// STOP
void stop(geometry_msgs::Twist &vel_msg, ros::Publisher &pub)
{
	// vel_msg에 포함된 3차원 속도 값 중 각속도만 초기화
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	pub.publish(vel_msg);
}

// 사각형 그리기 서비스 Request에 응답하는 Callback 함수
bool handlesquare(turtlebot3_practice_move::square::Request &req, turtlebot3_practice_move::square::Response &res)
{
	//TODO: 이미 그리고 있는데 그리기 명령이 들어오는 것 처리하기

	ROS_INFO("[draw_square_server] I just got incoming request!");

	// service client로부터 요청된 변 길이 받아오기
	side_length = req.sidelength;
	ROS_INFO_STREAM("[draw_square_server] req.sidelength=" << req.sidelength);

	// service client로부터 요청된 반복 회수 받아오기
	rotations = req.rotations;
	ROS_INFO_STREAM("[draw_square_server] req.rotations=" << req.rotations);

	// 그리기 시작
	currentDrawingLine = 0;
	initPositionX = tb3_odom->pose.pose.position.x;
	initPositionY = tb3_odom->pose.pose.position.y;

	// 여기서 return값에 의미는 없으나 콜백함수 형식을 바꿀 수 없으므로 return 값을 임의로 true로 설정
	return true;
}

void tb3_odom_callback(const nav_msgs::Odometry::ConstPtr &msg){
	tb3_odom = msg;
}

int main(int argc, char **argv)
{
	// 노드/노드핸들 초기화
	ros::init(argc, argv, "draw_square_server");
	ros::NodeHandle nh;
	ROS_INFO("[draw_square_server] Startup");

	// 노드 반복 실행(spin) 주기 설정(ms)
	ros::Rate loop_rate(10);

	// 퍼블리셔 실행 (turtlesim 조종용 토픽 퍼블리시)
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ROS_INFO("[draw_square_server] Advertising publisher /cmd_vel");

	sub_tb3_odom = nh.subscribe("odom", 1000, tb3_odom_callback);

	// Turtlebot3 모델 정보 받아오기
	//turtlebot3_model = rospy.get_param("model", "burger");
	// turtlelbot3_model = nh.getParam();
	nh.param<std::string>("model", turtlebot3_model, "burger");

	// 서비스 서버 실행 (그릴 사각형 크기와 반복회수 받는 서비스 서버)
	ros::ServiceServer s = nh.advertiseService<turtlebot3_practice_move::square::Request, turtlebot3_practice_move::square::Response>("draw_square", handlesquare);
	ROS_INFO("[draw_square_server] Advertising service server draw_square");

	// vel_msg 메시지 객체 생성
	geometry_msgs::Twist vel_msg;

	// CTRL+C 등 종료 명령 받기 위해 ros::spin()대신 ros::ok(), 루프, ros::spinOnce() 조합
	int logCount = 0;
	while (ros::ok()){
		// 콜백함수 대기 등
		ros::spinOnce();

		if(0 <= currentDrawingLine && currentDrawingLine < polygon){
			if(isTurning){
				// 회전중
				double roll, pitch, yaw;
				getAngleFromOdom(tb3_odom, roll, pitch, yaw);
				if (angleDistance(yaw, initRotationZ) >= (M_PI * 2 / polygon))
				{
					//회전 종료
					stop(vel_msg, pub);
					isTurning = false;
					currentDrawingLine++;
					if(currentDrawingLine >= polygon){
						//완성
						currentDrawingLine = -1;
						stop(vel_msg, pub);
					}else{
						//직진 시작점 저장
						initPositionX = tb3_odom->pose.pose.position.x;
						initPositionY = tb3_odom->pose.pose.position.y;
					}
				}
				else
				{
					rotate(vel_msg, pub, smoothAngularSpeed(initRotationZ+(M_PI * 2 / polygon), yaw, 30*M_PI/180, default_angular_speed));
					logCount++;
					if (logCount >= 30){
						ROS_INFO("yaw:%f", yaw);
						logCount = 0;
					}
				}
			}else{
				// 직진중
				if (pointDistance(initPositionX, initPositionY, tb3_odom->pose.pose.position.x, tb3_odom->pose.pose.position.y) >= side_length)
				{
					//직진 종료
					isTurning = true;
					//회전 시작점 저장
					double roll, pitch, yaw;
					getAngleFromOdom(tb3_odom, roll, pitch, yaw);
					initRotationZ = yaw;
				}
				else
				{
					forward(vel_msg, pub, default_forward_speed);
				}
			}
		}
	}

	return 0;
}