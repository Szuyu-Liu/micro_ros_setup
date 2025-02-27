#include <iostream>
using namespace std;

enum ParkingActions {
	parallel_in = 0,
	parallel_out = 1,
	cross_in = 2,
	cross_out = 3,
	parking_end = 4,
};

struct ParkingMotorValues {
	float speed = 1.0;
	float steering = 1.0;
};

struct LidarInfo {
	float lidar_scale = 1;
	float scan_angle_min = 0;
	float scan_angle_increment = 1;
	float scan_size = 0;
	};
