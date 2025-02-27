#include "cross_in/sensor_parking.h"
#include <iostream>
#include <math.h>

#define RAD2DEG(x) ((x) * 180. / M_PI)

using namespace CrossParkingIn;

// #######################################################################################
//                                                                                       #
//                SENSOR CLASS IMPLEMENTTION                                             #
//                                                                                       #
// #######################################################################################

Sensors::Sensors()
{

    this->imu.heading = .0f;
    this->ultrasound.left = -.0f;
    this->ultrasound.rear_left = .0f;
    this->ultrasound.rear_right = .0f;
    this->ultrasound.right = .0f;

    first_reset_imu = 0;
    first_reset_odometry = 0;
}
//-------------------------------------------------------------------------------------

Sensors &Sensors::getInstance()
{
    static Sensors instance; // Statische Variable, um sicherzustellen, dass nur eine Instanz erstellt wird um zu verhindern, dass verschiedene
    return instance;         // states mit unterschiedlichen Werten für Distance und Heading arbeiten können
}
//-------------------------------------------------------------------------------------

/// @brief fill the updated ultrasound measurement in the struct and pass it to update
/// @param ultrasound
void Sensors::Update(Ultrasound &ultrasound)
{

    if ((ultrasound.left == -1.0))
    {
        this->ultrasound.left = 10.0;
    }
    else
    {
        this->ultrasound.left = ultrasound.left / 1000;
    }
    //-------------
    if ((ultrasound.right == -1.0))
    {
        this->ultrasound.right = 10.0;
    }
    else
    {
        this->ultrasound.right = ultrasound.right / 1000;
    }
    //-------------
    if ((ultrasound.rear_left == -1.0))
    {
        this->ultrasound.rear_left = 10.0;
    }
    else
    {
        this->ultrasound.rear_left = ultrasound.rear_left / 1000;
    }
    //-------------
    if ((ultrasound.rear_right == -1.0))
    {
        this->ultrasound.rear_right = 10.0;
    }
    else
    {
        this->ultrasound.rear_right = ultrasound.rear_right / 1000;
    }
    //-------------
    if (ultrasound.front_left == -1.0)
    {
        this->ultrasound.front_left = 10.0;
    }
    else
    {
        this->ultrasound.front_left = ultrasound.front_left / 1000;
    }
    //-------------
    if ((ultrasound.front_right == -1.0))
    {
        this->ultrasound.front_right = 10.0;
    }
    else
    {

        this->ultrasound.front_right = ultrasound.front_right / 1000;
    }
}
//-------------------------------------------------------------------------------------

/// @brief update current heading with imu euler angle
/// @param imu
void Sensors::Update(Angle &imu)
{
    this->imu.heading = imu;
}
//-------------------------------------------------------------------------------------

/// @brief update odometry with  current motorencoder counts
/// @param counts
void Sensors::Update(MotorCounts &counts)
{
    this->odometry.update(counts);
}
//-------------------------------------------------------------------------------------

/// @brief  Todo
/// @param lidar
void Sensors::Update(Lidar &lidar)
{
    this->lidar = lidar;
}
//-------------------------------------------------------------------------------------

/// @brief reset the delta distance counter
void Sensors::ResetDistance()
{
    this->odometry.Reset();
}
//-------------------------------------------------------------------------------------

/// @brief reset imu delta heading angle
void Sensors::ResetHeading()
{

    if (first_reset_imu == 0)
    {
        first_reset_imu = 1;
        this->imu_start.heading = this->imu.heading;
    }
    this->imu_old.heading = this->imu.heading;
}
//-------------------------------------------------------------------------------------

/// @brief set imu delta heading to an angle
void Sensors::ResetHeading(float angle)
{

    if (first_reset_imu == 0)
    {
        first_reset_imu = 1;
        this->imu_start.heading = this->imu.heading;
    }
    this->imu_old.heading = this->imu.heading + angle;
}
//-------------------------------------------------------------------------------------



/// @brief  get current delta heading angle in ar range from -180° to 180°
/// @return
float Sensors::Heading()
{
    // imu overflow after 360° ->0
    float retvalue = this->imu.heading - this->imu_old.heading;
    if (retvalue < 0)
        retvalue = 360 + retvalue;

    // center around 180° since parking should never cause a 180° turn, but will have a 0° -> 360° crossover
    if (retvalue > 180)
        retvalue = retvalue - 360;

    return retvalue;
}
//-------------------------------------------------------------------------------------

/// @brief  get current delta Distance
/// @return
double Sensors::Distance()
{
    return this->odometry.CountsToDistance();
}
//-------------------------------------------------------------------------------------

/// @brief  get current delta motorencodercounts
/// @return
float Sensors::Counts()
{
    return this->odometry.Counts();
}
//-------------------------------------------------------------------------------------

float Sensors::ULeft()
{
    return this->ultrasound.left;
}
//-------------------------------------------------------------------------------------

float Sensors::URight()
{
    return this->ultrasound.right;
}
//-------------------------------------------------------------------------------------

float Sensors::URearLeft()
{
    return this->ultrasound.rear_left;
}
//-------------------------------------------------------------------------------------

float Sensors::URearRight()
{
    return this->ultrasound.rear_right;
}
//-------------------------------------------------------------------------------------
float Sensors::UFrontLeft()
{
    return this->ultrasound.front_left;
}
//-------------------------------------------------------------------------------------
float Sensors::UFrontRight()
{
    return this->ultrasound.front_right;
}
//-------------------------------------------------------------------------------------

/// @brief get the full distance the car moved forward  since start
/// @return
float Sensors::getAbsoluteForward()
{
    return this->odometry.DistanceTravelledForward();
}
//-------------------------------------------------------------------------------------

/// @brief get the full distance the car moved in reverse  since start
/// @return
float Sensors::getAbsoluteReverse()
{
    return this->odometry.DistanceTravelledReverse();
}
//-------------------------------------------------------------------------------------

/// @brief update the parameter that is used to calculate distance from motorcounts
/// @param countstometer
void Sensors::setCountsToMeter(float countstometer)
{
    this->odometry.Setup(countstometer);
}
//-------------------------------------------------------------------------------------

/// @brief uses  lidardata to emulate a prescision TOF sensor looking right
/// @return distance in meter
float Sensors::LidarRightMax()
{

    constexpr float angle_min = -1 * M_PI;
    constexpr float min_range = 0.15;
    constexpr float max_range = 2.0;
    constexpr float min_angle = -85;
    constexpr float max_angle = -95;
    int count = lidar.scan_time / lidar.time_increment;

    float tempangle = 0, tempdist = 0;
    for (int i = 0; i < count; i++)
    {
        float degree = RAD2DEG(angle_min + lidar.angleincrement * i);
        if (degree > 180)
            break;
        if (!isinf(lidar.distances[i]))
        {
            if ((lidar.distances[i] > tempdist) && (lidar.distances[i] > min_range) &&
                (lidar.distances[i] < max_range) &&
                (lidar.distances[i] > min_range) && (degree < min_angle) && (degree > max_angle)) // right side  has negative angles  0  to -180
            {
                tempdist = lidar.distances[i];
                tempangle = degree;
            }
        }
    }
    (void)tempangle;
    return tempdist;
}
//-------------------------------------------------------------------------------------

/// @brief uses  lidardata to emulate a prescision TOF sensor looking right
/// @return distance in meter
float Sensors::LidarLeftMax()
{

    constexpr float angle_min = -1 * M_PI;
    constexpr float min_range = 0.15;
    constexpr float max_range = 2.0;
    constexpr float min_angle = 85;
    constexpr float max_angle = 95;
    int count = lidar.scan_time / lidar.time_increment;

    float tempangle = 0, tempdist = 0;
    for (int i = 0; i < count; i++)
    {
        float degree = RAD2DEG(angle_min + lidar.angleincrement * i);
        if (degree > 180)
            break;
        if (!isinf(lidar.distances[i]))
        {
            if ((lidar.distances[i] > tempdist) && (lidar.distances[i] > min_range) &&
                (lidar.distances[i] < max_range) &&
                (degree > min_angle) && (degree < max_angle)) // right side  has negative angles  0  to -180
            {
                tempdist = lidar.distances[i];
                tempangle = degree;
            }
        }
    }
    (void)tempangle;
    return tempdist;
}
//-------------------------------------------------------------------------------------

/// @brief uses  lidardata to emulate a prescision TOF sensor looking right
/// @return distance in meter
float Sensors::LidarRightMin()
{

    constexpr float angle_min = -1 * M_PI;
    constexpr float min_range = 0.15;
    constexpr float max_range = 2.0;
    constexpr float min_angle = -85;
    constexpr float max_angle = -95;
    int count = lidar.scan_time / lidar.time_increment;

    float tempangle = 0, tempdist = max_range;
    for (int i = 0; i < count; i++)
    {
        float degree = RAD2DEG(angle_min + lidar.angleincrement * i);
        if (degree > 180)
            break;
        if (!isinf(lidar.distances[i]))
        {
            if ((lidar.distances[i] < tempdist) && (lidar.distances[i] > min_range) &&
                (degree < min_angle) && (degree > max_angle)) // right side  has negative angles  0  to -180
            {
                tempdist = lidar.distances[i];
                tempangle = degree;
            }
        }
    }
    (void)tempangle;
    return tempdist;
}
//-------------------------------------------------------------------------------------

/// @brief uses  lidardata to emulate a prescision TOF sensor looking right
/// @return distance in meter
float Sensors::LidarLeftMin()
{

    constexpr float angle_min = -1 * M_PI;
    constexpr float min_range = 0.15;
    constexpr float max_range = 2.0;
    constexpr float min_angle = 85;
    constexpr float max_angle = 95;
    int count = lidar.scan_time / lidar.time_increment;

    float tempangle = 0, tempdist = max_range;
    for (int i = 0; i < count; i++)
    {
        float degree = RAD2DEG(angle_min + lidar.angleincrement * i);
        if (degree > 180)
            break;
        if (!isinf(lidar.distances[i]))
        {

            if ((lidar.distances[i] < tempdist) && (lidar.distances[i] > min_range) &&
                 (degree > min_angle) && (degree < max_angle)) // right side  has negative angles  0  to -180
            {
                tempdist = lidar.distances[i];
                tempangle = degree;
            }
        }
    }
    (void)tempangle;
    return tempdist;
}
//-------------------------------------------------------------------------------------

float Sensors::LidarRightMinDistanceAngle()
{
    constexpr float angle_min = -1 * M_PI;
    constexpr float min_range = 0.15;
    constexpr float max_range = 2.0;
    constexpr float min_angle = -80;
    constexpr float max_angle = -100;
    int count = lidar.scan_time / lidar.time_increment;

    float tempangle = 0, tempdist = max_range;
    for (int i = 0; i < count; i++)
    {
        float degree = RAD2DEG(angle_min + lidar.angleincrement * i);
        if (degree > 180)
            break;
        if (!isinf(lidar.distances[i]))
        {
            if ((lidar.distances[i] < tempdist) && (lidar.distances[i] > min_range) &&
                (degree < min_angle) && (degree > max_angle)) // right side  has negative angles  0  to -180
            {
                tempdist = lidar.distances[i];
                tempangle = degree;
            }
        }
    }
    (void)tempdist;
    return tempangle;
}
//-------------------------------------------------------------------------------------


float Sensors::LidarLeftMinDistanceAngle()
{
    constexpr float angle_min = -1 * M_PI;
    constexpr float min_range = 0.15;
    constexpr float max_range = 2.0;
    constexpr float min_angle = 80;
    constexpr float max_angle = 100;
    int count = lidar.scan_time / lidar.time_increment;

    float tempangle = 0, tempdist = max_range;
    for (int i = 0; i < count; i++)
    {
        float degree = RAD2DEG(angle_min + lidar.angleincrement * i);
        if (degree > 180)
            break;
        if (!isinf(lidar.distances[i]))
        {
            if ((lidar.distances[i] < tempdist) && (lidar.distances[i] > min_range) &&
                (lidar.distances[i] > min_range) && (degree > min_angle) && (degree < max_angle)) // right side  has negative angles  0  to -180
            {
                tempdist = lidar.distances[i];
                tempangle = degree;
            }
        }
    }
    (void)tempdist;
    return tempangle;
}
//-------------------------------------------------------------------------------------

float Sensors::LidarFrontMin(){
        constexpr float angle_min = -1 * M_PI;
    constexpr float min_range = 0.15;
    constexpr float max_range = 2.0;
    constexpr float min_angle = -5;
    constexpr float max_angle = 5;
    int count = lidar.scan_time / lidar.time_increment;

    float tempangle = 0, tempdist = max_range;
    for (int i = 0; i < count; i++)
    {
        float degree = RAD2DEG(angle_min + lidar.angleincrement * i);
        if (degree > 180)
            break;
        if (!isinf(lidar.distances[i]))
        {
            if ((lidar.distances[i] < tempdist) && (lidar.distances[i] > min_range) &&
                (lidar.distances[i] > min_range) && ((degree > min_angle) && (degree < max_angle))) // right side  has negative angles  0  to -180
            {
                tempdist = lidar.distances[i];
                tempangle = degree;
            }
        }
    }
    (void)tempangle;
    return tempdist;
}

// #######################################################################################
//                                                                                       #
//                ODOMETRY CLASS IMPLEMENTTION                                           #
//                                                                                       #
// #######################################################################################

Odometry::Odometry()
{

    this->first = 0;
    this->countstometers = 0.00833333333;
}

//-------------------------------------------------------------------------------------

void Odometry::update(int64_t counts_new)
{

    if (first == 0)
    {
        first = 1;
        this->counts = counts_new;
        this->counts_old = counts_new;
        this->counts_start = counts_new;
    }
    else
    {
        if (this->counts - counts_new > 0)
            this->counts_forward += this->counts - counts_new;
        else
            this->counts_reverse += counts_new - this->counts;
        this->counts = counts_new;
    }
}
//-------------------------------------------------------------------------------------

int64_t Odometry::Counts(void)
{
    return this->counts - this->counts_old;
}
//-------------------------------------------------------------------------------------

double Odometry::CountsToDistance()
{

    double returnval = (this->counts - this->counts_old) * this->countstometers;

    return returnval;
}
//-------------------------------------------------------------------------------------

void Odometry::Reset(void)
{
    this->counts_old = this->counts;
}
//-------------------------------------------------------------------------------------

float Odometry::DistanceTravelledForward()
{
    return (this->counts_forward * this->countstometers);
}
//-------------------------------------------------------------------------------------

float Odometry::DistanceTravelledReverse()
{
    return (this->counts_reverse * this->countstometers);
}
//-------------------------------------------------------------------------------------

void Odometry::Setup(float countstometer)
{
    this->countstometers = countstometer;
}