#include "cross_in/cross_parking_statemachine.h"
#include <iostream>

// #######################################################################################
//                                                                                       #
//                STATEMACHINE  CLASS IMPLEMENTTION                                      #
//                                                                                       #
// #######################################################################################
using namespace CrossParkingIn;

Statemachine::Statemachine()
{

    this->maxvalues.distance_forward = 0;
    this->maxvalues.distance_forward_single_step = 0;
    this->maxvalues.distance_reverse = 0;
    this->maxvalues.distance_reverse_single_step = 0;
    this->maxvalues.speed_forward = 1.05;
    this->maxvalues.speed_reverse = .95;
    this->maxvalues.speed_stop = 1.0;
    this->maxvalues.steering_left = .0;
    this->maxvalues.steering_right = 2.0;
    this->maxvalues.steering_straight = 1.0;

    sensors = &Sensors::getInstance();
    this->speed = 1.0;
    this->steering = 1.0;

    //    initial state
    currentstate = nullptr;
}
//---------------------------------------------------------------------------------

Statemachine::Statemachine(MaxValuesCrossParking maxvalues, std::vector<TransitionConditions> conditions)
{
    //  create  initial state

    std::cout << "CROSS_IN Statemachine Constructor with parameters" << std::endl;

    this->maxvalues = maxvalues;
    this->allstateconditions = conditions;
    currentstate = &Start::getInstance(&this->maxvalues, &this->allstateconditions);
    sensors = &Sensors::getInstance();
    this->speed = 1.0;
    this->steering = 1.0;
}
//---------------------------------------------------------------------------------

int Statemachine::Setup(MaxValuesCrossParking maxvalues, std::vector<TransitionConditions> states)
{
    //  create  initial state
    int returnvalue = 0;

    this->maxvalues = maxvalues;
    std::cout << "Statemachine Setup copy state conditions" << std::endl;

    if (states.size() == (CrossParkingStates::ENUM_LAST))
    {
        this->allstateconditions = states;
        std::cout << "Statemachine Setup create initial state" << std::endl;
        currentstate = &Start::getInstance(&maxvalues, &allstateconditions);
        returnvalue = 1;
    }
    else
    {
        std::cerr << "Setup vector state transition conditions incorrect size " << states.size() << " instead of " << (CrossParkingStates::ENUM_LAST) << std::endl;
    }
    std::cout << "Statemachine Setup end" << std::endl;
    return returnvalue;
}
//---------------------------------------------------------------------------------

Statemachine::~Statemachine()
{
 
}
//---------------------------------------------------------------------------------

/// @brief returns currently active speed scaled around 1.0
/// @return float
float Statemachine::getParkingSpeed()
{
    float speed = this->speed;

    if (speed > 2 || speed < 0)
        speed = 1.0;
    return speed;
}
//---------------------------------------------------------------------------------

/// @brief returns currently active steering value scaled around 1.0
/// @return float
float Statemachine::getParkingSteering()
{
    float steering = this->steering;

    if (steering > 2 || steering < 0)
        steering = 1.0;
    return steering;
}
//---------------------------------------------------------------------------------

/// @brief
/// @param
void Statemachine::setParkingSpeed(float speed)
{
    this->speed = speed;
}
//---------------------------------------------------------------------------------

/// @brief
/// @param
void Statemachine::setParkingSteering(float steering)
{
    this->steering = steering;
}
//---------------------------------------------------------------------------------

/// @brief
/// @return
int Statemachine::getLastState()
{
    return laststate[statehistory - 1];
}
//---------------------------------------------------------------------------------

/// @brief
/// @param state
void Statemachine::setLastState(int state)
{

    for (uint8_t i = 0; i < statehistory - 1; i++)
    {
        laststate[i] = laststate[i + 1];
    }
    laststate[statehistory - 1] = state;
}
//---------------------------------------------------------------------------------

// #######################################################################################
//                                                                                       #
//                ABSTRACT STATES  CLASS IMPLEMENTTION                                   #
//                                                                                       #
// #######################################################################################

/// @brief
/// @return
int States::WhoAmI()
{
    return this->isstate;
}
//--------------------------------------------------------------------------------------

/// @brief every state declares if its busy or needs to be changed for a running state only stop can be reached
///        auswertung in beforeChange()
/// @return
bool States::isActive()
{
    return this->active;
}
//--------------------------------------------------------------------------------------

/// @brief 
void States::setActive(bool active)
{
    this->active = active;
}
//--------------------------------------------------------------------------------------

/// @brief
/// @param speed
/// @param statemachine
void States::setParkingSpeed(float speed, Statemachine *statemachine)
{
    statemachine->setParkingSpeed(speed);
}
//--------------------------------------------------------------------------------------

/// @brief
/// @param steering
/// @param statemachine
void States::setParkingSteering(float steering, Statemachine *statemachine)
{
    statemachine->setParkingSteering(steering);
}
//--------------------------------------------------------------------------------------

/// @brief
/// @param state
/// @param statemachine
void States::setLastState(int state, Statemachine *statemachine)
{
    statemachine->setLastState(state);
}
//--------------------------------------------------------------------------------------



// #######################################################################################
//                                                                                       #
//                CONCRETE STATES CLASSES IMPLEMENTTION                                  #
//                                                                                       #
// #######################################################################################

// #######################################################################################
//                                                                                       #
//                POSITIONFOUND CLASS IMPLEMENTTION                                      #
//                                                                                       #
// #######################################################################################

/// @brief  Singleton: creates a new Object or returns a reference if there already exist one
/// @return
States &PositionFound::getInstance(MaxValuesCrossParking *values, std::vector<TransitionConditions> *allconditions)
{

    static PositionFound instance;
    if (instance.first == 0 ){
        instance.first = 1;
        instance.maxvalues = *values;
    }
    if (instance.allstateconditions == nullptr)
    {
        instance.allstateconditions = allconditions;
        instance.stateconditions = allconditions->at(instance.WhoAmI());
        instance.Setup();
    }
    return instance;
}
//--------------------------------------------------------------------------------------

/// @brief  Constructor
PositionFound::PositionFound()
{

    this->isstate = POSITION_FOUND;

}
//--------------------------------------------------------------------------------------

/// @brief adds all possible states, that can follow this one to a vector
void PositionFound::Setup()
{
    std::cout << " PositionFound Setup " << std::endl;
    setActive(false);
    stateconditions.distance = 0.03;

    this->nextstates.push_back(&ForwardLeft::getInstance(&this->maxvalues, this->allstateconditions));
    //this->nextstates.push_back(&ReverseRight::getInstance(this->maxvalues, this->allstateconditions));
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                       #
//                START CLASS IMPLEMENTTION                                               #
//                                                                                       #
// #######################################################################################



/// @brief  Singleton: creates a new Object or returns a reference if there already exist one
/// @return
States &Start::getInstance(MaxValuesCrossParking *values, std::vector<TransitionConditions> *conditions)
{
    static Start instance;
    if (instance.first == 0 ){
        instance.first = 1;
        instance.maxvalues = *values;
    }
    if (instance.allstateconditions == nullptr)
    {
        instance.allstateconditions = conditions;
        instance.stateconditions = conditions->at(instance.WhoAmI());
        instance.Setup();
    }
    return instance;
}
//--------------------------------------------------------------------------------------

/// @brief  Constructor
Start::Start()
{
    this->isstate = START;
}
//--------------------------------------------------------------------------------------

/// @brief adds all possible states, that can follow this one, to the nextstates vector
void Start::Setup()
{
    std::cout << " Start Setup " << std::endl;
    setActive(true); 
    this->nextstates.push_back(&PositionFound::getInstance(&this->maxvalues, this->allstateconditions));

}
//--------------------------------------------------------------------------------------


// #######################################################################################
//                                                                                       #
//                STOP CLASS IMPLEMENTTION                                               #
//                                                                                       #
// #######################################################################################

/**
 *    The Stop class is a savety class to prevent collison it checks all sensors
 *    if they exceed minimum or maximum limits and trys to recover by moving in the opposite
 *    direction
 */

/// @brief  Singleton: creates a new Object or returns a reference if there already exist one
/// @return
States &Stop::getInstance(MaxValuesCrossParking *values, std::vector<TransitionConditions> *conditions)
{
    static Stop instance;
    if (instance.first == 0 ){
        instance.first = 1;
        instance.maxvalues = *values;
    }
    if (instance.allstateconditions == nullptr)
    {
        instance.allstateconditions = conditions;
        instance.stateconditions = conditions->at(instance.WhoAmI());
        instance.Setup();
    }
    return instance;
}
//--------------------------------------------------------------------------------------

/// @brief  Constructor
Stop::Stop()
{
    this->isstate = STOP;
}
//--------------------------------------------------------------------------------------

/// @brief adds all possible states, that can follow this one, to the nextstates vector
void Stop::Setup()
{
    setActive(false);
    this->nextstates.push_back(&Stop::getInstance(&this->maxvalues, this->allstateconditions));
/*     this->nextstates.push_back(&ReverseCorrectLeft::getInstance(this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ReverseCorrectRight::getInstance(this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ReverseCorrectStraight::getInstance(this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ForwardCorrectLeft::getInstance(this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ForwardCorrectRight::getInstance(this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ForwardCorrectStraight::getInstance(this->maxvalues, this->allstateconditions));*/
} 
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                      #
//               FORWARDCORRECTLEFT CLASS IMPLEMENTTION                                 #
//                                                                                      #
// #######################################################################################

/* 
/// @brief  Singleton: creates a new Object or returns a reference if there already exist one
/// @return
States &ForwardCorrectLeft::getInstance(MaxValuesCrossParking *values, std::vector<TransitionConditions> *conditions)
{
    static ForwardCorrectLeft instance;
    if (instance.first == 0 ){
        instance.first = 1;
        instance.maxvalues = *values;
    }
    if (instance.allstateconditions == nullptr)
    {
        instance.allstateconditions = conditions;
        instance.stateconditions = conditions->at(instance.WhoAmI());
        instance.Setup();
    }
    return instance;
}
//--------------------------------------------------------------------------------------

/// @brief Constructor
ForwardCorrectLeft::ForwardCorrectLeft()
{
    this->isstate = FORWARD_CORRECT_LEFT;
}
//--------------------------------------------------------------------------------------

/// @brief adds all possible states, that can follow this one
void ForwardCorrectLeft::Setup()
{
    std::cout << " ForwardCorrectLeft Setup " << std::endl;
    setActive(false);
    this->nextstates.push_back(&Stop::getInstance(&this->maxvalues, this->allstateconditions));
    //this->nextstates.push_back(&ReverseLeft::getInstance(&this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ReverseRight::getInstance(&this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ReverseStraight::getInstance(&this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ForwardCorrectRight::getInstance(&this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ForwardCorrectStraight::getInstance(&this->maxvalues, this->allstateconditions));
}
//--------------------------------------------------------------------------------------

/// @brief // this code  gets executed after a statechange
/// @param statemachine

// #######################################################################################
//                                                                                      #
//               FORWARDCORRECTRIGHT CLASS IMPLEMENTTION                                #
//                                                                                      #
// #######################################################################################

/// @brief  Singleton: creates a new Object or returns a reference if there already exist one
/// @return
States &ForwardCorrectRight::getInstance(MaxValuesCrossParking *values, std::vector<TransitionConditions> *conditions)
{
    static ForwardCorrectRight instance;
    if (instance.first == 0 ){
        instance.first = 1;
        instance.maxvalues = *values;
    }
    if (instance.allstateconditions == nullptr)
    {
        instance.allstateconditions = conditions;
        instance.stateconditions = conditions->at(instance.WhoAmI());
        instance.Setup();
    }
    return instance;
}
//--------------------------------------------------------------------------------------

/// @brief Constructor
ForwardCorrectRight::ForwardCorrectRight()
{
    this->isstate = FORWARD_CORRECT_RIGHT;
}
//--------------------------------------------------------------------------------------

/// @brief adds all possible states, that can follow this one

void ForwardCorrectRight::Setup()
{
    std::cout << " ForwardCorrectRight Setup " << std::endl;
    setActive(false);
    // this->nextstates.push_back(&ForwardCorrectRight::getInstance(this->maxvalues, this->allstateconditions));
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                      #
//               FORWARDCORRECTSTRAIGHT CLASS IMPLEMENTTION                             #
//                                                                                      #
// #######################################################################################

/// @brief  Singleton: creates a new Object or returns a reference if there already exist one
/// @return
States &ForwardCorrectStraight::getInstance(MaxValuesCrossParking *values, std::vector<TransitionConditions> *conditions)
{
    static ForwardCorrectStraight instance;
    if (instance.first == 0 ){
        instance.first = 1;
        instance.maxvalues = *values;
    }
    if (instance.allstateconditions == nullptr)
    {
        instance.allstateconditions = conditions;
        instance.stateconditions = conditions->at(instance.WhoAmI());
        instance.Setup();
    }
    return instance;
}
//--------------------------------------------------------------------------------------

/// @brief Constructor
ForwardCorrectStraight::ForwardCorrectStraight()
{
    this->isstate = FORWARD_STRAIGHT;
}
//--------------------------------------------------------------------------------------

/// @brief adds all possible states, that can follow this one

void ForwardCorrectStraight::Setup()
{
    std::cout << " ForwardStraight Setup " << std::endl;
    setActive(false);
    this->nextstates.push_back(&Stop::getInstance(&this->maxvalues, this->allstateconditions));
    //this->nextstates.push_back(&ReverseLeft::getInstance(&this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ReverseRight::getInstance(&this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ReverseStraight::getInstance(&this->maxvalues, this->allstateconditions));
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                      #
//               REVERSECORRECTLEFT CLASS IMPLEMENTTION                                 #
//                                                                                      #
// #######################################################################################

/// @brief  Singleton: creates a new Object or returns a reference if there already exist one
/// @return
States &ReverseCorrectLeft::getInstance(MaxValuesCrossParking *values, std::vector<TransitionConditions> *conditions)
{
    static ReverseCorrectLeft instance;
    if (instance.first == 0 ){
        instance.first = 1;
        instance.maxvalues = *values;
    }
    if (instance.allstateconditions == nullptr)
    {
        instance.allstateconditions = conditions;
        instance.stateconditions = conditions->at(instance.WhoAmI());
        instance.Setup();
    }
    return instance;
}
//--------------------------------------------------------------------------------------

/// @brief Constructor
ReverseCorrectLeft::ReverseCorrectLeft()
{
    this->isstate = REVERSE_CORRECT_LEFT;
}
//--------------------------------------------------------------------------------------

/// @brief adds all possible states, that can follow this one

void ReverseCorrectLeft::Setup()
{
    std::cout << " ReverseCorrectLeft Setup " << std::endl;
    setActive(false);
    this->nextstates.push_back(&Stop::getInstance(&this->maxvalues, this->allstateconditions));
    //this->nextstates.push_back(&ReverseLeft::getInstance(&this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ReverseRight::getInstance(&this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ReverseStraight::getInstance(&this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ForwardCorrectRight::getInstance(&this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ForwardCorrectStraight::getInstance(&this->maxvalues, this->allstateconditions));
}
//--------------------------------------------------------------------------------------

/// @brief // this code  gets executed after a statechange
/// @param statemachine

// #######################################################################################
//                                                                                      #
//               REVERSECORRECTRIGHT CLASS IMPLEMENTTION                                #
//                                                                                      #
// #######################################################################################

/// @brief  Singleton: creates a new Object or returns a reference if there already exist one
/// @return
States &ReverseCorrectRight::getInstance(MaxValuesCrossParking *values, std::vector<TransitionConditions> *conditions)
{
    static ReverseCorrectRight instance;
    if (instance.first == 0 ){
        instance.first = 1;
        instance.maxvalues = *values;
    }
    if (instance.allstateconditions == nullptr)
    {
        instance.allstateconditions = conditions;
        instance.stateconditions = conditions->at(instance.WhoAmI());
        instance.Setup();
    }
    return instance;
}
//--------------------------------------------------------------------------------------

/// @brief Constructor
ReverseCorrectRight::ReverseCorrectRight()
{
    this->isstate = FORWARD_CORRECT_RIGHT;
}
//--------------------------------------------------------------------------------------

/// @brief adds all possible states, that can follow this one
void ReverseCorrectRight::Setup()
{
    std::cout << " ReverseCorrectRight Setup " << std::endl;
    setActive(false);
    // this->nextstates.push_back(&ReverseCorrectRight::getInstance(this->maxvalues, this->allstateconditions));
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                      #
//               REVERSECORRECTSTRAIGHT CLASS IMPLEMENTTION                             #
//                                                                                      #
// #######################################################################################

/// @brief  Singleton: creates a new Object or returns a reference if there already exist one
/// @return
States &ReverseCorrectStraight::getInstance(MaxValuesCrossParking *values, std::vector<TransitionConditions> *conditions)
{
    static ReverseCorrectStraight instance;
    if (instance.first == 0 ){
        instance.first = 1;
        instance.maxvalues = *values;
    }
    if (instance.allstateconditions == nullptr)
    {
        instance.allstateconditions = conditions;
        instance.stateconditions = conditions->at(instance.WhoAmI());
        instance.Setup();
    }
    return instance;
}
//--------------------------------------------------------------------------------------

/// @brief Constructor
ReverseCorrectStraight::ReverseCorrectStraight()
{
    this->isstate = REVERSE_CORRECT_STRAIGHT;
}
//--------------------------------------------------------------------------------------

/// @brief adds all possible states, that can follow this one

void ReverseCorrectStraight::Setup()
{
    std::cout << " ReverseCorrectStraight Setup " << std::endl;
    setActive(false);
    this->nextstates.push_back(&Stop::getInstance(&this->maxvalues, this->allstateconditions));
    //this->nextstates.push_back(&ReverseLeft::getInstance(&this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ReverseRight::getInstance(&this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ReverseStraight::getInstance(&this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ReverseCorrectLeft::getInstance(&this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ForwardCorrectRight::getInstance(&this->maxvalues, this->allstateconditions));
}
//--------------------------------------------------------------------------------------
 */
// #######################################################################################
//                                                                                      #
//               REVERSELEFT CLASS IMPLEMENTTION                                        #
//                                                                                      #
// #######################################################################################
/* 
/// @brief  Singleton: creates a new Object or returns a reference if there already exist one
/// @return
States &ReverseLeft::getInstance(MaxValuesCrossParking *values, std::vector<TransitionConditions> *conditions)
{
    static ReverseLeft instance;
    if (instance.first == 0 ){
        instance.first = 1;
        instance.maxvalues = *values;
    }
    if (instance.allstateconditions == nullptr)
    {
        instance.allstateconditions = conditions;
        instance.stateconditions = conditions->at(instance.WhoAmI());
        instance.Setup();
    }
    return instance;
}
//--------------------------------------------------------------------------------------

/// @brief Constructor
ReverseLeft::ReverseLeft()
{
    this->isstate = REVERSE_LEFT;
}
//--------------------------------------------------------------------------------------

/// @brief adds all possible states, that can follow this one

void ReverseLeft::Setup()
{
    std::cout << " ReverseLeft Setup " << std::endl;
    setActive(false);
    this->nextstates.push_back(&Stop::getInstance(&this->maxvalues, this->allstateconditions));
}
//--------------------------------------------------------------------------------------
 */
// #######################################################################################
//                                                                                      #
//               FORWARDRIGHT CLASS IMPLEMENTTION                                        #
//                                                                                      #
// #######################################################################################
/* 
/// @brief  Singleton: creates a new Object or returns a reference if there already exist one
/// @return
States &ForwardRight::getInstance(MaxValuesCrossParking *values, std::vector<TransitionConditions> *conditions)
{
    static ForwardRight instance;
    if (instance.first == 0 ){
        instance.first = 1;
        instance.maxvalues = *values;
    }
    if (instance.allstateconditions == nullptr)
    {
        instance.allstateconditions = conditions;
        instance.stateconditions = conditions->at(instance.WhoAmI());
        instance.Setup();
    }
    return instance;
}
//--------------------------------------------------------------------------------------

/// @brief Constructor
ForwardRight::ForwardRight()
{
    this->isstate = FORWARD_RIGHT;
}
//--------------------------------------------------------------------------------------

/// @brief adds all possible states, that can follow this one

void ForwardRight::Setup()
{
    std::cout << " ForwardRight Setup " << std::endl;
    setActive(false);
    this->nextstates.push_back(&Stop::getInstance(&this->maxvalues, this->allstateconditions));
}
//--------------------------------------------------------------------------------------
 */
// #######################################################################################
//                                                                                      #
//               FORWARDLEFT CLASS IMPLEMENTTION                                        #
//                                                                                      #
// #######################################################################################

/// @brief  Singleton: creates a new Object or returns a reference if there already exist one
/// @return
States &ForwardLeft::getInstance(MaxValuesCrossParking *values, std::vector<TransitionConditions> *conditions)
{
    static ForwardLeft instance;
    if (instance.first == 0 ){
        instance.first = 1;
        instance.maxvalues = *values;
    }
    if (instance.allstateconditions == nullptr)
    {
        instance.allstateconditions = conditions;
        instance.stateconditions = conditions->at(instance.WhoAmI());
        instance.Setup();
    }
    return instance;
}
//--------------------------------------------------------------------------------------

/// @brief Constructor
ForwardLeft::ForwardLeft()
{
    this->isstate = FORWARD_LEFT;
}
//--------------------------------------------------------------------------------------

/// @brief adds all possible states, that can follow this one
void ForwardLeft::Setup()
{


    std::cout << " ForwardLeft Setup " << std::endl;
setActive(false);
    this->nextstates.push_back(&Stop::getInstance(&this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ReverseRight::getInstance(&this->maxvalues, this->allstateconditions));
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                      #
//               FORWARDSTRAIGHT CLASS IMPLEMENTTION                                        #
//                                                                                      #
// #######################################################################################

/// @brief  Singleton: creates a new Object or returns a reference if there already exist one
/// @return
States &ForwardStraight::getInstance(MaxValuesCrossParking *values, std::vector<TransitionConditions> *conditions)
{
    static ForwardStraight instance;
    if (instance.first == 0 ){
        instance.first = 1;
        instance.maxvalues = *values;
    }
    if (instance.allstateconditions == nullptr)
    {
        instance.allstateconditions = conditions;
        instance.stateconditions = conditions->at(instance.WhoAmI());
        instance.Setup();
    }
    return instance;
}
//--------------------------------------------------------------------------------------

/// @brief Constructor
ForwardStraight::ForwardStraight()
{
    this->isstate = FORWARD_STRAIGHT;
}
//--------------------------------------------------------------------------------------

/// @brief adds all possible states, that can follow this one
void ForwardStraight::Setup()
{
    std::cout << " ForwardStraight Setup " << std::endl;
    setActive(false);
    this->nextstates.push_back(&Stop::getInstance(&this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ReverseRight::getInstance(&this->maxvalues, this->allstateconditions));
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                      #
//               REVERSERIGHT CLASS IMPLEMENTTION                                        #
//                                                                                      #
// #######################################################################################

/// @brief  Singleton: creates a new Object or returns a reference if there already exist one
/// @return
States &ReverseRight::getInstance(MaxValuesCrossParking *values, std::vector<TransitionConditions> *conditions)
{
    static ReverseRight instance;
    if (instance.first == 0 ){
        instance.first = 1;
        instance.maxvalues = *values;
    }
    if (instance.allstateconditions == nullptr)
    {
        instance.allstateconditions = conditions;
        instance.stateconditions = conditions->at(instance.WhoAmI());
        instance.Setup();
    }
    return instance;
}
//--------------------------------------------------------------------------------------

/// @brief Constructor
ReverseRight::ReverseRight()
{
    this->isstate = REVERSE_RIGHT;
}
//--------------------------------------------------------------------------------------

/// @brief adds all possible states, that can follow this one
void ReverseRight::Setup()
{
    std::cout << " ReverseRight Setup " << std::endl;
    setActive(false);
    this->nextstates.push_back(&Stop::getInstance(&this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&ReverseStraight::getInstance(&this->maxvalues, this->allstateconditions));
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                      #
//               REVERSESTRAIGHT CLASS IMPLEMENTTION                                        #
//                                                                                      #
// #######################################################################################

/// @brief  Singleton: creates a new Object or returns a reference if there already exist one
/// @return
States &ReverseStraight::getInstance(MaxValuesCrossParking *values, std::vector<TransitionConditions> *conditions)
{
    static ReverseStraight instance;
    if (instance.first == 0 ){
        instance.first = 1;
        instance.maxvalues = *values;
    }
    if (instance.allstateconditions == nullptr)
    {
        instance.allstateconditions = conditions;
        instance.stateconditions = conditions->at(instance.WhoAmI());
        instance.Setup();
    }
    return instance;
}
//--------------------------------------------------------------------------------------

/// @brief Constructor
ReverseStraight::ReverseStraight()
{
    this->isstate = REVERSE_STRAIGHT;
}
//--------------------------------------------------------------------------------------

/// @brief adds all possible states, that can follow this one
void ReverseStraight::Setup()
{
    std::cout << " ReverseStraight Setup " << std::endl;
    setActive(false);
    // this->nextstates.push_back(&ReverseStraight::getInstance(this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&Stop::getInstance(&this->maxvalues, this->allstateconditions));
    this->nextstates.push_back(&Finish::getInstance(&this->maxvalues, this->allstateconditions));
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                      #
//               FINISH CLASS IMPLEMENTTION                                            #
//                                                                                      #
// #######################################################################################

/// @brief  Singleton: creates a new Object or returns a reference if there already exist one
/// @return
States &Finish::getInstance(MaxValuesCrossParking *values, std::vector<TransitionConditions> *conditions)
{
    static Finish instance;
    if (instance.first == 0 ){
        instance.first = 1;
        instance.maxvalues = *values;
    }
    if (instance.allstateconditions == nullptr)
    {
        instance.allstateconditions = conditions;
        instance.stateconditions = conditions->at(instance.WhoAmI());
        instance.Setup();
    }
    return instance;
}
//--------------------------------------------------------------------------------------

/// @brief Constructor
Finish::Finish()
{
    this->isstate = FINISH;
}
//--------------------------------------------------------------------------------------

/// @brief adds all possible states, that can follow this one

void Finish::Setup()
{
    std::cout << " Finish Setup " << std::endl;
    setActive(false);
    // this state has no successors
}
//--------------------------------------------------------------------------------------
