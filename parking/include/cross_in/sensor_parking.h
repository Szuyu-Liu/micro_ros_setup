#ifndef SENSORPARKING_H
#define SENSORPARKING_H

#include <stdint.h>
#include <vector>

namespace CrossParkingIn
{
    struct Ultrasound
    {
        double front_left = -1;
        double front_right = -1;
        double left = -1;
        double right = -1;
        double rear_left = -1;
        double rear_right = -1;
    };

    struct Lidar
    {
        float angleincrement = 0.0;
        float scan_time = 0.0;
        float time_increment = 0.0;
        std::vector<float> distances;
    };

    struct Imu
    {
        float heading = 0;
    };

    typedef int MotorCounts;
    typedef float Angle;

    class Odometry
    {
    public:
        Odometry(void);
        void Setup(float);
        void update(int64_t);
        int64_t Counts(void);
        double CountsToDistance();
        void Reset(void);
        float DistanceTravelledForward(void);
        float DistanceTravelledReverse(void);

        // todo copyconstructor

    private:
        double countstometers;
        int64_t counts;
        int64_t counts_old;
        int64_t counts_start;
        int64_t counts_forward;
        int64_t counts_reverse;
        uint8_t first;
    };

    class Sensors
    {
    public:
        static Sensors &getInstance();

        void Update(Ultrasound &); // todo alle Sensoren bekommen eine update Methode
        void Update(Angle &);
        void Update(MotorCounts &);
        void Update(Lidar &);
        void ResetDistance(void);
        void ResetHeading(void); 
        void ResetHeading(float);
        float Heading(void);
        double Distance(void);
        float Counts(void);
        float ULeft(void);
        float URight(void);
        float URearLeft(void);
        float URearRight(void);
        float UFrontLeft(void);
        float UFrontRight(void);
        float getAbsoluteForward(void);
        float getAbsoluteReverse(void);
        Sensors(const Sensors &) = delete;
        void operator=(const Sensors &) = delete;
        void setCountsToMeter(float);
        float LidarRightMax(void);
        float LidarLeftMax(void);
        float LidarRightMin(void);
        float LidarLeftMin(void);
         float LidarFrontMin(void);
        float LidarRightMinDistanceAngle(void);
        float LidarLeftMinDistanceAngle(void);

    private:
        Sensors();
        ~Sensors() {};
        Ultrasound ultrasound;
        Lidar lidar;
        Imu imu;
        Imu imu_old;
        Imu imu_start;
        int64_t distance_start;
        Odometry odometry;
        uint8_t first_reset_imu;
        uint8_t first_reset_odometry;
    };
}
#endif // SENSORPARKING_H