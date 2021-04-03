#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>

// srv폴더에 만든 서비스 파일에서 생성되는 헤더파일로 catkin_make 실행 중 생성된다.
#include "turtlebot3_practice_move/square.h"

int main(int argc, char **argv)
{
	// 노드/노드핸들 초기화
	ros::init(argc, argv, "draw_square_client");
	ros::NodeHandle nh;
	ROS_INFO("[draw_square_client] Startup");

	// 서비스 서버가 켜질 때까지 대기한다.
	// rospy.wait_for_service('draw_square')

	// 변 길이와 반복회수
	double side_length, rotations;

	try
	{
		// 입력받은 문자열 버퍼
		std::string inputString;

		// 한 변 길이 받아오기
		std::cout << "Enter Side length (0-5):";
		std::cin.clear(); // 기존에 혹시모를 처리되지 않은 입력값을 삭제, 입력값 오류 방지
		std::getline(std::cin, inputString); // 엔터키 칠 때까지 받음
		double side_length = std::stod(inputString); // 문자열로 받은 입력값을 double 형으로 변경
		if (side_length <= 0)  // 잘못된 길이가 입력되었을 경우 실행 중단, 에러 출력 후 강제종료
			throw "side_length <= 0";
		ROS_DEBUG_STREAM("side_length=" << side_length);

		// 문자열 버퍼 비우기(재활용)
		inputString = "";

		// 반복 횟수 받아오기
		std::cout << "Enter number of rotations : ";
		std::cin.clear(); // 기존에 혹시모를 처리되지 않은 입력값을 삭제, 입력값 오류 방지
		std::getline(std::cin, inputString); // 엔터키 칠 때까지 받음
		int rotations = std::stoi(inputString); // 문자열로 받은 입력값을 int 형으로 변경
		if (rotations <= 0)						// 잘못된 횟수가 입력되었을 경우 실행 중단, 에러 출력 후 강제종료
			throw "rotations <= 0";
		ROS_DEBUG_STREAM("rotations=" << rotations);

		// 서비스 클라이언트 연결
		turtlebot3_practice_move::square svcsquare;
		ros::ServiceClient svcsquareClient = nh.serviceClient<turtlebot3_practice_move::square>("draw_square");
		// 서비스 서버로 요청값(side_length와 rotations) 저장
		svcsquare.request.rotations = rotations;
		svcsquare.request.sidelength = side_length;
		// 저장된 요청값 전송(실행)
		svcsquareClient.call(svcsquare);
	}
	catch (const ros::Exception e)
	{
		ROS_ERROR_STREAM("Service call failed: " << e.what());
	}
	catch (const char* s)
	{
		ROS_ERROR_STREAM(s);
	}
}