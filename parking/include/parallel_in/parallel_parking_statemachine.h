#ifndef ParallelPARKINGSTATEMACHINE_H
#define ParallelPARKINGSTATEMACHINE_H

#include <vector>
#include <memory>

#include "cross_in/sensor_parking.h"

namespace ParallelParkingIn
{

    struct PositonCorrection
    {
        double lidar_angle_offset = 0.0;
        double wall_distance = 0.7;
        double wall_offset = 0.0;
    };

    struct MaxValuesParallelParking
    {
        float steering_straight = 1.0;
        float steering_left = .0;
        float steering_right = 2.0;
        float speed_stop = 1.0;
        float speed_forward = 1.03;
        float speed_reverse = 0.98;
        double distance_reverse = 5.000;
        double distance_forward = 5.000;
        double break_distance_forward = 0.0;
        double break_distance_reverse = 0.0;
        PositonCorrection lidar_correction;
    };

    struct TransitionConditions
    {
        float imu_min = 0;
        float imu_max = 0;
        float imu = 0;
        float lidar = 0;
        float us_left = 0;
        float us_right = 0;
        float us_rear_left = 0;
        float us_rear_right = 0;
        float us_front_left = 0;
        float us_front_right = 0;
        double distance = 0;
        int previous_state = 0;
        bool previous_state_active = false;
        CrossParkingIn::Sensors *sensors; 
    };
    /**
     * Wie funktioniere ich?
     * Die Bewegungsfolge ist Position_found -> Forward -> Reverse_Right -> Reverse_Straight -> Reverse_Left -> Forward_Right  -> Finish
     * jede Bewegung kann jederzeit durch Stop unterbrochen werden, wenn eine Kollision droht.
     * Aus Stop heraus wird dann entschieden welche Korrektur ausgeführt werden muss, um das Auto wieder in eine Position
     * zu bewegen, die ein weiterführen des Parkmanövers erlaubt.
     * Normale Bewegungen werden berechnet:
     * Positon_Found ist die Startposition und nullt winkel und Distaz
     * Forward  ist distance = 0,8m - (Abstand Hinterachse zu Rand Parklücke)
     * Reverse_right ist imu ~ 45° und distance < ca .5m
     * reverse_straight ist distance < 0,5m  und us_rear >= 0,025m
     * Reverse_Left ist zeitbasiert 0,5 Sekunden um den Bremsweg auszunutzen und die Reifen zu drehen 
     * Forward_right ist imu ca ~0° us_rear >= 0,01m und us_front >= 0,01m
     * finish ist us_rear < 0,05m  und imu ~0°
     * sollange ein state noch aktiv ist , kann kein nachfolgender aktiviert werden.
     * das wird in beforeUpdate() getestet, allerdings nicht auf die Startbedingung sondern nur auf Zielbedingungen
     *
     *
     * Was fehlt:
     * die Möglichkeit online abhängig vom seitlichen Abstand in position found beliebige zusätzliche Bewegungen einfügen
     * um das Auto in eine  geeignete Position zu manövrieren hierzu sollen die correct states nicht als Singleton
     * ausgeführt sein, sondern dynamisch über stop angelegt werden
     *
     *
     *
     */

    enum ParallelParkingStates
    {
        FINISH,
        START,
        POSITION_FOUND,
        STOP,
        FORWARD_STRAIGHT,
        FORWARD_RIGHT,
        REVERSE_RIGHT,
        REVERSE_STRAIGHT,
        REVERSE_LEFT,
        ENUM_LAST
    };

    class States;

    const int statehistory = 10;

    class Statemachine
    {
    private:
        float speed;
        float steering;
        MaxValuesParallelParking maxvalues;
        // TransitionConditions conditions;
        int laststate[statehistory];
        std::vector<TransitionConditions> allstateconditions;

    protected:
        void setParkingSpeed(float);
        void setParkingSteering(float);
        void setLastState(int);

    public:
        CrossParkingIn::Sensors *sensors;
        Statemachine(void);
        Statemachine(MaxValuesParallelParking, std::vector<TransitionConditions>);
        int Setup(MaxValuesParallelParking, std::vector<TransitionConditions>);
        ~Statemachine(void);
        States *currentstate;
        void UpdateState(States &);
        int Update(void);
        float getParkingSpeed(void);
        float getParkingSteering(void);
        int getLastState(void);

        friend class States;
    };
    //-------------------------------------------------------------------------------------
    class States
    {

    private:
    protected:
        std::vector<TransitionConditions> *allstateconditions = nullptr;
        TransitionConditions stateconditions;
        MaxValuesParallelParking maxvalues;
        std::vector<States *> nextstates;
        int first = 0;
        int isstate;
        int laststate;
        bool active = false;
        virtual void setActive(bool) final;

    public:
        virtual int testConditions(TransitionConditions *) = 0;
        virtual int testActive(TransitionConditions *) = 0;
        virtual void afterChange(Statemachine *) = 0;
        virtual void beforeChange(Statemachine *) final;
        virtual void Setup(void) = 0;
        virtual int Handle(Statemachine *) final;
        virtual void setParkingSpeed(float, Statemachine *) final;
        virtual void setParkingSteering(float, Statemachine *) final;
        virtual void setLastState(int, Statemachine *) final;
        virtual bool isActive(void) final;
        virtual int WhoAmI(void) final;
        virtual void Update(Statemachine *) = 0;
        virtual ~States() {};
        PositonCorrection lidar_correction;
    };
    //-------------------------------------------------------------------------------------

    //-------------------------------------------------------------------------------------
    /* class Template : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesParallelParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        Template();
        void operator=(const Template &) = delete;
        Template(const Template &) = delete;
    };
    //-------------------------------------------------------------------------------------
*/
    class Start : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesParallelParking *, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        Start();
        void operator=(const Start &) = delete;
        Start(const Start &) = delete;
    };
    //-------------------------------------------------------------------------------------

    class Stop : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesParallelParking *, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        Stop();
        void operator=(const Stop &) = delete;
        Stop(const Stop &) = delete;
    };
    //-------------------------------------------------------------------------------------

    class Finish : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesParallelParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        Finish();
        void operator=(const Finish &) = delete;
        Finish(const Finish &) = delete;
    };
    //-------------------------------------------------------------------------------------


    class PositionFound : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesParallelParking *, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        float distancelast;
        PositionFound();
        void operator=(const PositionFound &) = delete;
        PositionFound(const PositionFound &) = delete;
    };
    //-------------------------------------------------------------------------------------

    class ForwardStraight : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesParallelParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        ForwardStraight();
        void operator=(const ForwardStraight &) = delete;
        ForwardStraight(const ForwardStraight &) = delete;
    };
    //-------------------------------------------------------------------------------------
    class ReverseRight : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesParallelParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        ReverseRight();
        void operator=(const ReverseRight &) = delete;
        ReverseRight(const ReverseRight &) = delete;
    };
    //-------------------------------------------------------------------------------------
    class ReverseStraight : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesParallelParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        ReverseStraight();
        void operator=(const ReverseStraight &) = delete;
        ReverseStraight(const ReverseStraight &) = delete;
    };
    //-------------------------------------------------------------------------------------

class ReverseLeft : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesParallelParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        ReverseLeft();
        void operator=(const ReverseLeft &) = delete;
        ReverseLeft(const ReverseLeft &) = delete;
    };
    //-------------------------------------------------------------------------------------

    class ForwardRight : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesParallelParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        ForwardRight();
        void operator=(const ForwardRight &) = delete;
        ForwardRight(const ForwardRight &) = delete;
    };
    //-------------------------------------------------------------------------------------


}

#endif //  ParallelPARKINGSTATEMACHINE_H