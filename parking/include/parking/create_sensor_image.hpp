#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include <vector>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <math.h>
//#include "parking_datatypes.hpp"

using namespace std;
using std::placeholders::_1;

#define RAD2DEG(x) ((x) * 180. / M_PI)


	
class CreateSensorImage {
public:
  CreateSensorImage() { }
  
  /*_________Create Image with the sensor measurements_____*/
  void create_image(cv::Mat* image, int img_size, LidarInfo* lidar_info, 
  	float* scan_points, float yaw, float yaw_init,  vector<double> parking_params,
  	float* us_dist, double right_side_angle, int ray_start, int ray_end) 
  	{ 	
  	
  //Global Variables
  	int x, y;
  	float img_scale = 1;
  	cv::Point center(img_size / 2, img_size / 2);
   	
    //Iterate through laser scan ranges
    if ((lidar_info != nullptr) && (scan_points != nullptr)) {
    	img_scale = lidar_info->lidar_scale;
    	for (size_t i = 0; i < lidar_info->scan_size; ++i) {
    		float range = scan_points[i];
    		if(std::isfinite(range)){
    			float angle_lidar = lidar_info->scan_angle_min + i * lidar_info->scan_angle_increment;
    			//float angle = angle_lidar + parking_params[13];
    			float angle = angle_lidar + right_side_angle;
    			//Cos and Sin expect an angle in radians
    			x = static_cast<int>(center.x - range * 
    				lidar_info->lidar_scale * std::cos(angle));
    			y = static_cast<int>(center.y + range * 
    				lidar_info->lidar_scale * std::sin(angle));
    			//float degree = RAD2DEG(angle);
    			//Draw the point
    			if (angle_lidar < 0)
	    			cv::circle(*image, cv::Point(x,y), 2, cv::Scalar(0,0,255), -1);
	    		else if (angle_lidar > 0) {
	    		//Choose a 10-20 degree angle region to look for holes in the parking
	    			//if ( (angle_lidar > (parking_params[14] + 1.57 - 0.0872) ) &&
	    			//(angle_lidar < (parking_params[14] + 1.57 + 0.0872) ) ) {
	    			if ( (i > ray_start ) && ( i < ray_end) ) 
	    			{
	    				cv::circle(*image, cv::Point(x,y), 2, cv::Scalar(0, 255, 255), -1);
	    				//cout << " i: " << i;
	    			}
	    			else 
	    				cv::circle(*image, cv::Point(x,y), 2, cv::Scalar(255,0,0), -1);
	    		}
    		}
    	}
    	//cout << endl;
    }
    
    /*-------------Set the points that represent the Ultrasounds-----*/   
    // Front Ultrasound Sensors
    x = static_cast<int>(center.x + img_scale * std::cos(0));
    y = static_cast<int>(center.y + img_scale * std::sin(0));
    cv::circle(*image, cv::Point((x + parking_params[1]), 
    	(y + parking_params[2] - (int)(us_dist[0]/parking_params[0]))), 5, cv::Scalar(255,0,255), -1);
    	
    cv::circle(*image, cv::Point((x + parking_params[3]), 
    	(y + parking_params[4] - (int)(us_dist[1]/parking_params[0]))), 5, cv::Scalar(200,0,255), -1);
    	
    //Rear Ultrasound Sensors
    cv::circle(*image, cv::Point((x + parking_params[5]), 
    	(y + parking_params[6] + (us_dist[2]/parking_params[0]))), 5, cv::Scalar(255,0, 0), -1);
    	
    cv::circle(*image, cv::Point((x + parking_params[7]), 
    	(y + parking_params[8] + (us_dist[3]/parking_params[0]))), 5, cv::Scalar(200,0, 0), -1);
    	
    //Right Ultrasound Sensor
     cv::circle(*image, cv::Point((x + parking_params[11]), 
    	(y + parking_params[12] + (int)(us_dist[5]/parking_params[0]))), 5, cv::Scalar(100, 50, 255), -1);
    	
    
    //cv::circle(image, cv::Point((x - 10), y), 5, cv::Scalar(255,0,255), -1);
    //cv::circle(image, cv::Point((x - 10), y), 5, cv::Scalar(255,0,255), -1);
    
    //cv::circle(image, cv::Point(x,(int)us_4), 5, cv::Scalar(255,0,255), -1);
    //cv::circle(image, cv::Point(x,(int)us_5), 5, cv::Scalar(255,0,255), -1);
    
    //Process the Text that displays the IMU Message
    std::string imu_text = to_string(yaw);
    cv::Point position(10,30);
    int fontType = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 1.0;
    cv::Scalar color(255, 255, 255);
    int thickness = 2;
    int lineType = cv::LINE_AA;
    cv::putText(*image, imu_text, position, fontType, fontScale, color, thickness,lineType);
    
    //Draw an Arrow for the IMU
    double length = 50;
    //We define the initial yaw of the IMU as up in OpenCV window = 270 degrees
    double angleInRadians;
    /*if (yaw_init < -360.0)
    	angleInRadians = 270.0 * CV_PI / 180.0;
    else*/	
    //angleInRadians = (yaw * 270.0 / yaw_init) * CV_PI / 180.0;
    angleInRadians = (yaw * CV_PI / 180.0);
    angleInRadians -= parking_params[13];
    //angleInRadians -= (1.5 * CV_PI);
    //angleInRadians *= -1;
    cv::Point startPoint(img_size / 2, img_size / 2);
    cv::Point endPoint(
    	startPoint.x + static_cast<int>(length * std::cos(angleInRadians)),
    	startPoint.y + static_cast<int>(length * std::sin(angleInRadians))
    	);
    //cv::Point endPoint(200, 240);
    cv::arrowedLine(*image, startPoint, endPoint, cv::Scalar(0,255, 0), thickness, lineType);
    
    //cv::Point test1(100, 100);
    //cv::Point test2(100, 150);
    //cv::arrowedLine(*image, test1, test2, cv::Scalar(0,255, 0), thickness, lineType);
    
    //cv::imshow("Sensor Image", image);
    //cv::waitKey(1);
  }

};
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------