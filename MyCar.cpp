#include <iostream>
#include <algorithm>
#include <vector>
#include <ctime>
#include "RuleBasedDriving.h"

using namespace std;
using namespace Car;

bool is_debug = false;
bool is_accident = false;
int accident_count = 0;
int recovery_count = 0;

ControlValues control_driving(CarStateValues sensing_info)
{
	if (is_debug)
	{
		cout << "=========================================================" << endl;
		cout << "[MyCar] to middle: " << sensing_info.to_middle << endl;

		cout << "[MyCar] collided: " << sensing_info.collided << endl;
		cout << "[MyCar] car speed: " << sensing_info.speed << "km/h" << endl;

		cout << "[MyCar] is moving forward: " << sensing_info.moving_forward << endl;
		cout << "[MyCar] moving angle: " << sensing_info.moving_angle << endl;
		cout << "[MyCar] lap_progress: " << sensing_info.lap_progress << endl;

		cout << "[MyCar] track_forward_angles: ";
		for (int i = 0; i < sensing_info.track_forward_angles.size(); i++)
		{
			cout << sensing_info.track_forward_angles[i] << ", ";
		}
		cout << endl;

		cout << "[MyCar] track_forward_obstacles: ";
		for (int i = 0; i < sensing_info.track_forward_obstacles.size(); i++)
		{
			cout << "{dist: " << sensing_info.track_forward_obstacles[i].dist
				<< ", to_middle: " << sensing_info.track_forward_obstacles[i].to_middle << "}, ";
		}
		cout << endl;

		cout << "[MyCar] opponent_cars_info: ";
		for (int i = 0; i < sensing_info.opponent_cars_info.size(); i++)
		{
			cout << "{dist: " << sensing_info.opponent_cars_info[i].dist
				<< ", to_middle: " << sensing_info.opponent_cars_info[i].to_middle
				<< ", speed: " << sensing_info.opponent_cars_info[i].speed << "km/h}, ";
		}
		cout << endl;

		cout << "[MyCar] distance_to_way_points: ";
		for (int i = 0; i < sensing_info.distance_to_way_points.size(); i++)
		{
			cout << sensing_info.distance_to_way_points[i] << ", ";
		}
		cout << endl;

		cout << "=========================================================" << endl;
	}

	ControlValues car_controls;
	// ===========================================================
	// Area for writing code about driving rule ==================
	// ===========================================================
	// Editing area starts from here
	//	

	// Moving straight forward

	////핸들, 브레이크, 액셀
	//float set_throttle = 0.83;
	//float set_brake = 0;

	////전방의 커브를 앞서 인식하기 위해 속도에 따라 2~30m 앞서서 가져온다.
	//int angle_num = (int)(sensing_info.speed / 45);
	//float ref_angle = sensing_info.track_forward_angles[angle_num];
	////음수 -> 왼쪽에 있음 -> 오른쪽으로 꺾어야함
	////양수 -> 오른쪽에 있음 -> 왼쪽으로 꺾어야함
	////float middle_add = (sensing_info.to_middle / 70) * (-1);
	////sensing_info.to_middle : 중심으로 부터의 거리
	//float middle_add = (sensing_info.to_middle / 60) * (-1);
	////printf("sensing_info.to_middle : %.4f\n", sensing_info.to_middle);
	//if (ref_angle <= 20)
	//{
	//	middle_add *= 0.5;
	//}
	////전방의 커브 값에 내 차의 방향을 빼면 꺾어야 할 방향값이 나온다. -1.0 ~ 1.0
	//float set_steering = (ref_angle - sensing_info.moving_angle) / (180 - sensing_info.speed);
	//set_steering += middle_add;

	////sj
	////전방 4개의 각도를 가져온다.
	//float avr_angle = 0;
	//for (int i = 0; i < 4; i++)
	//{
	//	avr_angle += sensing_info.track_forward_angles[i];
	//}
	//avr_angle /= 4;
	//if (ref_angle >= 20)
	//{
	//	middle_add *= 0.8;
	//}
	//else if (ref_angle >= 10)
	//{
	//	middle_add *= 0.5;
	//}


	//set_steering = (avr_angle - sensing_info.moving_angle) / (180 - sensing_info.speed);
	//middle_add = (sensing_info.to_middle / 60) * (-1);
	//set_steering += middle_add;





	//if (sensing_info.track_forward_obstacles.size() > 0) {
	//	ObstaclesInfo fwd_obstacle = sensing_info.track_forward_obstacles[0];

	//	if (fwd_obstacle.dist < 60 && fwd_obstacle.dist > 0 && abs(fwd_obstacle.to_middle) < 8.0) {
	//		double avoid_width = 2.7;
	//		float diff = fwd_obstacle.to_middle - sensing_info.to_middle;
	//		if (abs(diff) < avoid_width) {
	//			ref_angle = (float)(abs(atan((diff - avoid_width) / fwd_obstacle.dist) * 57.29579));
	//			middle_add = 0;
	//			if (diff > 0) ref_angle += -1;
	//			
	//		}
	//	}
	//}


	//if (sensing_info.opponent_cars_info.size() > 0) {
	//	//상대 차량이 있는 경우
	//	CarsInfo opp_car = sensing_info.opponent_cars_info[0];
	//	if (opp_car.dist < 20 && opp_car.dist > -5) {
	//		float offset = (float)(8 - abs(opp_car.to_middle)) / (float)2;
	//		if (opp_car.to_middle > sensing_info.to_middle) {
	//			middle_add = ((sensing_info.to_middle + offset) / 70) * (-1);
	//		}
	//		else {
	//			middle_add = ((sensing_info.to_middle - offset) / 70) * (-1);
	//		}
	//		set_steering += middle_add;
	//	}
	//}





	//////////////////////////////////////////////////sw
	 float half_load_width = sensing_info.half_road_limit - 1.25f;

	 // 차량 핸들 조정을 위해 참고할 전방의 커브 값 가져오기
	 int angle_num = (int)(sensing_info.speed / 45);
	 float ref_angle = sensing_info.track_forward_angles[angle_num];

	 // 차량의 차선 중앙 정렬을 위한 미세 조정 값 계산
	 float middle_add = (sensing_info.to_middle / 80) * -1;

	 // 전방의 커브 각도에 따라 throttle 값을 조절하여 속도를 제어함
	 float throttle_factor = 0.6f / (abs(ref_angle) + 0.1f);
	 if (throttle_factor > 0.11f) throttle_factor = 0.11f;   // throttle 값을 최대 0.81 로 설정
	 float set_throttle = 0.7f + throttle_factor;
	 if (sensing_info.speed < 60) set_throttle = 0.9f;  // 속도가 60Km/h 이하인 경우 0.9 로 설정

	 // 차량의 Speed 에 따라서 핸들을 돌리는 값을 조정함
	 float steer_factor = sensing_info.speed * 1.5f;
	 float set_steering = ref_angle - sensing_info.moving_angle;
	 if(abs(sensing_info.to_middle) > half_load_width){    //도로 제한을 벗어나는 경우 반대방향으로 살짝 틀어 움직인다. --> 페널티가 적용된 상태
	     float steer_factor = sensing_info.speed * 3.0f; //속도에 0.9의 페널티가 매겨지므로 감안하고 곱하는 숫자를 살짝 증가
	     set_steering /= (steer_factor + 0.001f);
	     if(sensing_info.to_middle>0){   //차가 중앙선 오른쪽에 위치한 경우
	        if(set_steering>0){  //현재 상태가 오른쪽에서 차선 밖인데 또 오른쪽으로 트는 것으로 결정되었으므로 방향을 바꿔야한다.
	             set_steering = -set_steering;
	        }
	     }
	     else{   //왼쪽에 위치한 경우
	        if(set_steering<0){  //현재 상태가 왼쪽에서 차선 밖인데 또 왼쪽으로 트는 것으로 결정되었으므로 방향을 바꿔야한다.
	             set_steering = -set_steering;
	        }
	     }
	 }
	 else{
	     if (sensing_info.speed > 100) steer_factor = sensing_info.speed * 0.7f;
	     else if (sensing_info.speed > 70) steer_factor = sensing_info.speed * 0.85f;
	     set_steering /= (steer_factor + 0.001f);
	 }


	 // (참고할 전방의 커브 - 내 차량의 주행 각도) / (계산된 steer factor) 값으로 steering 값을 계산


	 // 차선 중앙정렬 값을 추가로 고려함
	 set_steering += middle_add;

	 //// 긴급 및 예외 상황 처리 ////////////////////////////////////////////////////////////////////////////////////////
	 bool full_throttle = true;
	 bool emergency_brake = false;

	 // 전방 커브의 각도가 큰 경우 속도를 제어함
	 // 차량 핸들 조정을 위해 참고하는 커브 보다 조금 더 멀리 참고하여 미리 속도를 줄임
	 int road_range = (int)(sensing_info.speed / 30);
	 for (int i = 0; i < road_range; i++) {
	     float fwd_angle = abs(sensing_info.track_forward_angles[i]);
	     if (fwd_angle > 80) {   
	         full_throttle = false;
	         emergency_brake = true;
	         break;
	     }
	     else if (fwd_angle > 45) {   
	         full_throttle = false;
	     }
	 }
	 // brake, throttle 제어
	 float set_brake = 0.0f;
	 if (!full_throttle) {
	     if (sensing_info.speed > 100) {
	         set_brake = 0.3f;
	     }
	     if (sensing_info.speed > 120) {
	         set_throttle = 0.7f;
	         set_brake = 0.7f;
	     }
	     if (sensing_info.speed > 130) {
	         set_throttle = 0.5f;
	         set_brake = 1.0f;
	     }
	 }

	 // steering 까지 추가로 제어
	 if (emergency_brake) {
	     if (set_steering > 0) {
	         set_steering += 0.3f;
	     }
	     else {
	         set_steering -= 0.3f;
	     }
	 }


	 if (sensing_info.track_forward_obstacles.size() > 0) {
 		ObstaclesInfo fwd_obstacle = sensing_info.track_forward_obstacles[0];

 		if (fwd_obstacle.dist < 60 && fwd_obstacle.dist > 0 && abs(fwd_obstacle.to_middle) < 8.0) {
 			double avoid_width = 2.7;
 			float diff = fwd_obstacle.to_middle - sensing_info.to_middle;
 			if (abs(diff) < avoid_width) {
 				ref_angle = (float)(abs(atan((diff - avoid_width) / fwd_obstacle.dist) * 57.29579));
 				middle_add = 0;
 				if (diff > 0) ref_angle += -1;
 			
 			}
 		}
	 }


	 if (sensing_info.opponent_cars_info.size() > 0) {
 		//상대 차량이 있는 경우
 		CarsInfo opp_car = sensing_info.opponent_cars_info[0];
 		if (opp_car.dist < 20 && opp_car.dist > -5) {
 			float offset = (float)(8 - abs(opp_car.to_middle)) / (float)2;
 			if (opp_car.to_middle > sensing_info.to_middle) {
 				middle_add = ((sensing_info.to_middle + offset) / 70) * (-1);
 			}
 			else {
 				middle_add = ((sensing_info.to_middle - offset) / 70) * (-1);
 			}
 			set_steering += middle_add;
 		}
	 }


	 // 충돌 상황 감지 후 회피 하기 (1~5 단계)
	 // 1. 30Km/h 이상의 속도로 달리는 경우 정상 적인 상황으로 간주
	 if (sensing_info.speed > 30.0f) {
	     is_accident = false;
	     recovery_count = 0;
	     accident_count = 0;
	 }

	 // 2. 레이싱 시작 후 Speed 1km/h 이하가 된 경우 상황 체크
	 if (sensing_info.lap_progress > 0.5 && !is_accident &&
	     (sensing_info.speed < 1.0 && sensing_info.speed > -1.0)) {
	     accident_count += 1;
	 }

	 // 3. Speed 1km/h 이하인 상태가 지속된 경우 충돌로 인해 멈준 것으로 간주
	 if (accident_count > 6) {
	     is_accident = true;
	 }

	 // 4. 충돌로 멈춘 경우 후진 시작
	 if (is_accident) {
	     set_steering = 0.02f;
	     set_brake = 0;
	     set_throttle = -1;
	     recovery_count += 1;
	 }

	 // 5. 어느 정도 후진이 되었을 때 충돌을 회피 했다고 간주 후 정상 주행 상태로 돌림
	 if (recovery_count > 20) {
	     is_accident = false;
	     recovery_count = 0;
	     accident_count = 0;
	     set_steering = 0;
	     set_brake = 0;
	     set_throttle = 0;
	 }







	//distance_to_way_points : way points 라는 가상의 점, 현재 달리고 있는 차량으로부터 전방 10개의 점
	car_controls.steering = set_steering;	//방향
	car_controls.throttle = set_throttle;	//악셀
	car_controls.brake = set_brake;			//브레이크

	if (is_debug)
	{
		cout << "[MyCar] steering:" << car_controls.steering << ", throttle:" << car_controls.throttle
			<< ", brake:" << car_controls.brake << endl;
	}
	//
	// Editing area ends
	// ===========================================================

	return car_controls;
}

int main()
{
	// ===========================================================
	// Don't remove below area. ==================================
	// ===========================================================
	cout << "[MyCar] Start Bot!" << endl;
	StartDriving(control_driving);
	cout << "[MyCar] End Bot!" << endl;
	// ==========================================================

	return 0;
}
