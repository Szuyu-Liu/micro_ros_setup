#ifndef CROSSPARKINGSTATEMACHINE_H
#define CROSSPARKINGSTATEMACHINE_H

#include <vector>
#include <memory>

#include "cross_in/sensor_parking.h"

namespace CrossParkingIn
{

    struct PositonCorrection{
        double lidar_angle_offset = 0.0;
        double wall_distance = 0.7;
        double wall_offset = 0.0;

    };

    struct MaxValuesCrossParking
    {
        float steering_straight = 1.0;
        float steering_left = .0;
        float steering_right = 2.0;
        float speed_stop = 1.0;
        float speed_forward = 1.03;
        float speed_reverse = 0.98;
        double distance_forward_single_step = .100;
        double distance_reverse_single_step = .100;
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
    };
/**
 * Wie funktioniere ich?
 * Die Bewegungsfolge ist Position_found -> Forward_Left -> Reverse_Right -> Reverse_Straight -> Finish
 * jede Bewegung kann jederzeit durch Stop unterbrochen werden, wenn eine Kollision droht.
 * Aus Stop heraus wird dann entschieden welche Korrektur ausgeführt werden muss, um das Auto wieder in eine Position 
 * zu bewegen, die ein weiterführen des Parkmanövers erlaubt.
 * Normale Bewegungen werden berechnet:
 * Positon_Found ist die Startposition und nullt winkel und Distaz
 * Forward _Left ist distance = 0,4m - (Abstand Hinterachse zu Rand Parklücke)
 * Reverse_right ist imu ~ 85° und distance < ca 1,2m
 * reverse_straight ist distance < 0,5m  und us_rear >= 0,05m
 * finish ist us_rear < 0,05m  und imu ~85°  
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



    enum CrossParkingStates
    {
        FINISH,
        START,
        POSITION_FOUND,
        STOP,
        FORWARD_CORRECT_LEFT,   // correct moves  can be called all the time while  standard moves are fixed to a movemend pattern 
        FORWARD_CORRECT_RIGHT,
        REVERSE_CORRECT_LEFT,
        REVERSE_CORRECT_RIGHT,
        FORWARD_CORRECT_STRAIGHT,
        REVERSE_CORRECT_STRAIGHT,
        FORWARD_STRAIGHT,
        FORWARD_RIGHT,
        FORWARD_LEFT,
        REVERSE_RIGHT,
        REVERSE_LEFT,
        REVERSE_STRAIGHT,
        ENUM_LAST
    };

    class States;
 
    const int statehistory = 10;

    class Statemachine
    {
    private:
        float speed;
        float steering;
        MaxValuesCrossParking maxvalues;
       // TransitionConditions conditions;
        int laststate[statehistory];
        std::vector<TransitionConditions> allstateconditions;

    protected:
        void setParkingSpeed(float);
        void setParkingSteering(float);
        void setLastState(int);

    public:
        
        Sensors *sensors;
        Statemachine(void);
        Statemachine(MaxValuesCrossParking, std::vector<TransitionConditions>);
        int Setup(MaxValuesCrossParking, std::vector<TransitionConditions>);
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
        MaxValuesCrossParking maxvalues;
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
    class Stop : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesCrossParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        Stop();
        void operator=(const Stop &) = delete;
        Stop(const Stop &) = delete;
    };
    //-------------------------------------------------------------------------------------
    class Start : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesCrossParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        Start();
        void operator=(const Start &) = delete;
        Start(const Start &) = delete;
    };
    //-------------------------------------------------------------------------------------

    class PositionFound : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesCrossParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        PositionFound();
        void operator=(const PositionFound &) = delete;
        PositionFound(const PositionFound &) = delete;
    };
    //-------------------------------------------------------------------------------------
    /* class ForwardCorrectLeft : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesCrossParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        ForwardCorrectLeft();
        void operator=(const ForwardCorrectLeft &) = delete;
        ForwardCorrectLeft(const ForwardCorrectLeft &) = delete;
    };
    //-------------------------------------------------------------------------------------
    class ForwardCorrectRight : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesCrossParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        ForwardCorrectRight();
        void operator=(const ForwardCorrectRight &) = delete;
        ForwardCorrectRight(const ForwardCorrectRight &) = delete;
    };
    //-------------------------------------------------------------------------------------
    class ForwardCorrectStraight : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesCrossParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        ForwardCorrectStraight();
        void operator=(const ForwardCorrectStraight &) = delete;
        ForwardCorrectStraight(const ForwardCorrectStraight &) = delete;
    };
    //-------------------------------------------------------------------------------------
        class ReverseCorrectStraight : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesCrossParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        ReverseCorrectStraight();
        void operator=(const ReverseCorrectStraight &) = delete;
        ReverseCorrectStraight(const ReverseCorrectStraight &) = delete;
    };
    //-------------------------------------------------------------------------------------
        class ReverseCorrectLeft : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesCrossParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        ReverseCorrectLeft();
        void operator=(const ReverseCorrectLeft &) = delete;
        ReverseCorrectLeft(const ReverseCorrectLeft &) = delete;
    };
    //-------------------------------------------------------------------------------------
        class ReverseCorrectRight : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesCrossParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        ReverseCorrectRight();
        void operator=(const ReverseCorrectRight &) = delete;
        ReverseCorrectRight(const ReverseCorrectRight &) = delete;
    }; */
    //-------------------------------------------------------------------------------------
   /*  class ReverseLeft : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesCrossParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        ReverseLeft();
        void operator=(const ReverseLeft &) = delete;
        ReverseLeft(const ReverseLeft &) = delete;
    }; */
    //-------------------------------------------------------------------------------------
    class ReverseRight : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesCrossParking*, std::vector<TransitionConditions> *);
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
        static States &getInstance(MaxValuesCrossParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        ReverseStraight();
        void operator=(const ReverseStraight &) = delete;
        ReverseStraight(const ReverseStraight &) = delete;
    };
    //-------------------------------------------------------------------------------------
        //-------------------------------------------------------------------------------------
    class ForwardLeft : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesCrossParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        ForwardLeft();
        void operator=(const ForwardLeft &) = delete;
        ForwardLeft(const ForwardLeft &) = delete;
    };
    //-------------------------------------------------------------------------------------
   /*  class ForwardRight : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesCrossParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        ForwardRight();
        void operator=(const ForwardRight &) = delete;
        ForwardRight(const ForwardRight &) = delete;
    }; */
    //-------------------------------------------------------------------------------------
    class ForwardStraight : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesCrossParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        ForwardStraight();
        void operator=(const ForwardStraight &) = delete;
        ForwardStraight(const ForwardStraight &) = delete;
    };
    //-------------------------------------------------------------------------------------
    class Finish : public States
    {
    public:
        void Update(Statemachine *);
        void afterChange(Statemachine *statemachine);
        static States &getInstance(MaxValuesCrossParking*, std::vector<TransitionConditions> *);
        int testConditions(TransitionConditions *currentconditions);
        int testActive(TransitionConditions *);
        void Setup(void);

    private:
        Finish();
        void operator=(const Finish &) = delete;
        Finish(const Finish &) = delete;
    };
    //-------------------------------------------------------------------------------------

}

#endif // CROSSPARKINGSTATEMACHINE_H