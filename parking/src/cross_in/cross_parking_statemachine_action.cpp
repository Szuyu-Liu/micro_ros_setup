#include "cross_in/cross_parking_statemachine.h"
#include <math.h>
#include <iostream>

using namespace CrossParkingIn;

// #######################################################################################
//                                                                                       #
//                STATEMACHINE  CLASS IMPLEMENTTION                                      #
//                                                                                       #
// #######################################################################################

//---------------------------------------------------------------------------------

/// @brief  this Method has to be called to update the statemachine
/// @return true if successful
int Statemachine::Update()
{
    int success = -1;
    this->speed = 1.0; // reset old commands so a failed state update doesnt cause a crash
    this->steering = 1.0;
    success = currentstate->Handle(this);
    if (success != -1)
        return currentstate->WhoAmI();
    else
        return success;
}
//--------------------------------------------------------------------------------

/// @brief DO NOT TOUCH
/// @param newstate
void Statemachine::UpdateState(States &newstate)
{
    currentstate->beforeChange(this); // old state do something before we change state
    if (currentstate->WhoAmI() == newstate.WhoAmI())
    {
        currentstate->Update(this); // do something if  state has not changed
    }
    else
    {
        currentstate = &newstate;        // change the state
        currentstate->afterChange(this); // new state do something after we change state
    }
}
//---------------------------------------------------------------------------------

// #######################################################################################
//                                                                                       #
//                ABSTRACT STATES  CLASS IMPLEMENTTION                                   #
//                                                                                       #
// #######################################################################################

//--------------------------------------------------------------------------------------

/// @brief  Do something
/// @param esc
void States::beforeChange(Statemachine *statemachine)
{
    // Stop state needs information about previous state manouver to correct it
    // therefor it should never overwrite laststate
    if (this->isstate != STOP)
        statemachine->setLastState(this->isstate);
}
//--------------------------------------------------------------------------------------

/// @brief Test if State  needs to be changed to a new state
///        if the current state is to be kept, it has to be in the "nextstates" list
/// @param statemachine
/// @return
int States::Handle(Statemachine *statemachine)
{
    int success = 0;
    States *newstate = nullptr;
    TransitionConditions test;

    //   Update values for current state

    test.distance = statemachine->sensors->Distance();
    test.imu = statemachine->sensors->Heading();
    test.us_left = statemachine->sensors->ULeft();
    test.us_front_left = statemachine->sensors->UFrontLeft();
    test.us_front_right = statemachine->sensors->UFrontRight();
    test.us_rear_left = statemachine->sensors->URearLeft();
    test.us_rear_right = statemachine->sensors->URearRight();
    test.us_right = statemachine->sensors->LidarRightMin();
    // test.us_right = statemachine->sensors->URight();
    test.previous_state = this->WhoAmI();

    test.previous_state_active = statemachine->currentstate->testActive(&test);

    std::cout << "---------------------------------------------------" << std::endl;
    std::cout << "State         " << test.previous_state << std::endl;
    std::cout << "distance is     " << test.distance << std::endl;
    std::cout << "distance      " << statemachine->currentstate->stateconditions.distance << std::endl;
    std::cout << "still active " << test.previous_state_active << std::endl;
    std::cout << "imu           " << test.imu << std::endl;
    std::cout << "us_front_left " << test.us_front_left << std::endl;
    std::cout << "us_front_right" << test.us_front_right << std::endl;
    std::cout << "us_left       " << test.us_left << std::endl;
    std::cout << "us_rear_left  " << test.us_rear_left << std::endl;
    std::cout << "us_rear_right " << test.us_rear_right << std::endl;
    std::cout << "us_right      " << test.us_right << std::endl;
    std::cout << "---------------------------------------------------" << std::endl;

    for (uint8_t i = 0; i < nextstates.size(); i++)
    { // search for the correct new state
        if (nextstates[i]->testConditions(&test))
        {
            newstate = nextstates[i];
            success = 1;
            break;
        }
    }
    if (success == 1 && newstate != nullptr)
    {
        std::cout << "new State         " << newstate->WhoAmI() << std::endl;
        statemachine->UpdateState(*newstate); // replace the state
    }
    else if (statemachine->currentstate != nullptr)
    {
        std::cout << "old State         " << statemachine->currentstate->WhoAmI() << std::endl;
        statemachine->UpdateState(*statemachine->currentstate); // update the state
        success = 2;
    }

    return success;
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
//                  THIS IS THE STARTING CONDITION                                       #
//                                                                                       #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void PositionFound::afterChange(Statemachine *statemachine)
{
    setActive(true);
    statemachine->sensors->ResetDistance();
    if (statemachine->sensors->LidarRightMax() >= maxvalues.lidar_correction.wall_distance)
    {
        statemachine->sensors->ResetHeading(statemachine->sensors->LidarRightMinDistanceAngle() + 90 - maxvalues.lidar_correction.lidar_angle_offset);
        ReverseStraight::getInstance(&maxvalues, allstateconditions).lidar_correction.wall_offset =
            statemachine->sensors->LidarRightMin() - maxvalues.lidar_correction.wall_distance;

        ForwardLeft::getInstance(&maxvalues, allstateconditions).lidar_correction.wall_offset =
            statemachine->sensors->LidarRightMin() - maxvalues.lidar_correction.wall_distance;
    }
    else
    {
        statemachine->sensors->ResetHeading();
    }
}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int PositionFound::testConditions(TransitionConditions *currentconditions)
{
    return (currentconditions->previous_state_active == false);
}
//--------------------------------------------------------------------------------------

/// @brief
/// @param currentconditions
/// @return
int PositionFound::testActive(TransitionConditions *currentconditions)
{

    setActive((stateconditions.distance > currentconditions->distance));
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void PositionFound::Update(Statemachine *statemachine)
{
    setParkingSteering(this->maxvalues.steering_straight, statemachine);
    setParkingSpeed(this->maxvalues.speed_forward, statemachine);
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                       #
//                START CLASS IMPLEMENTTION                                              #
//                                                                                       #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void Start::afterChange(Statemachine *statemachine)
{
    statemachine->sensors->ResetDistance();
    statemachine->sensors->ResetHeading();
    setActive(true);
    setParkingSteering(this->maxvalues.steering_straight, statemachine);
    setParkingSpeed(this->maxvalues.speed_reverse, statemachine);
}
//--------------------------------------------------------------------------------------

/// @brief Test if the given conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int Start::testConditions(TransitionConditions *currentconditions)
{
    (void)currentconditions;
    return true;
}
//--------------------------------------------------------------------------------------

/// @brief
/// @param currentconditions
/// @return
int Start::testActive(TransitionConditions *currentconditions)
{

    setActive(
        //(stateconditions.us_rear_left < currentconditions->us_rear_left) &&
        //(stateconditions.us_rear_right < currentconditions->us_rear_right)&&
        (stateconditions.distance < currentconditions->distance));

    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void Start::Update(Statemachine *statemachine)
{
    static int first = 0;
    if (!first)
    {
        first = 1;
        statemachine->sensors->ResetDistance();
        statemachine->sensors->ResetHeading();
        setActive(true);
        setParkingSteering(this->maxvalues.steering_straight, statemachine);
        setParkingSpeed(this->maxvalues.speed_reverse, statemachine);
    }
    else
    {
        setParkingSteering(this->maxvalues.steering_straight, statemachine);
        setParkingSpeed(this->maxvalues.speed_reverse, statemachine);
    }
    std::cout << " Angle to wall " << statemachine->sensors->LidarRightMinDistanceAngle() - maxvalues.lidar_correction.lidar_angle_offset
              << std::endl;
    std::cout << " Distance to wall " << statemachine->sensors->LidarRightMin()
              << std::endl;
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

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void Stop::afterChange(Statemachine *statemachine)
{
    // first get car to stop moving
    //  any corrections are done in update()
    setParkingSteering(this->maxvalues.steering_straight, statemachine);
    setParkingSpeed(this->maxvalues.speed_stop, statemachine);
}
//--------------------------------------------------------------------------------------

/// @brief Test if the given conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int Stop::testConditions(TransitionConditions *currentconditions)
{
    // this state can follow any state except position_found or finish
    // it is activated if the us distances fall below 1/3 of defined minimum
    // or the travelled distance exceeds the defined value

    int returnvalue =
        ((stateconditions.us_left >= currentconditions->us_left) ||
         (stateconditions.us_right >= currentconditions->us_right) ||
         (stateconditions.us_front_left >= currentconditions->us_front_left) ||
         (stateconditions.us_front_right >= currentconditions->us_front_right) ||
         (stateconditions.us_rear_left >= currentconditions->us_rear_left) ||
         (stateconditions.us_rear_right >= currentconditions->us_rear_right)) &&
        //(stateconditions.lidar > currentconditions->lidar) ||
        ((stateconditions.previous_state != FINISH) ||
         (stateconditions.previous_state != START) ||
         (stateconditions.previous_state != POSITION_FOUND));

    return returnvalue;
}
//--------------------------------------------------------------------------------------

/// @brief
/// @param currentconditions
/// @return
int Stop::testActive(TransitionConditions *currentconditions)
{
    (void)currentconditions;

    setActive(false); // this state is the entrypoint never true
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void Stop::Update(Statemachine *statemachine)
{
    // as long as the obstacle isnt cleared stop will be called again so we need to meve the car carefully
    // in the opposite direction from the last regular state
    (void)statemachine;
    // todo
    //   if(car == crash)  don't;
}
//--------------------------------------------------------------------------------------
/*
// #######################################################################################
//                                                                                      #
//               FORWARDCORRECTLEFT CLASS IMPLEMENTTION                                 #
//                                                                                      #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void ForwardCorrectLeft::afterChange(Statemachine *statemachine)
{
    statemachine->sensors->ResetDistance();
    setParkingSteering(this->maxvalues.speed_forward, statemachine);
    setParkingSpeed(this->maxvalues.steering_left, statemachine);
}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int ForwardCorrectLeft::testConditions(TransitionConditions *currentconditions)
{
    // imu is dont care
    // right us is standard safety distancy
    // left us needs some distance but since the car is moving right safety margin is halfed
    // us front is dont care since there should never be a measureable object  in front also lidar
    //  will give a more reliable signal
    //
    int returnvalue = 0;
    returnvalue =
        (stateconditions.us_left >= currentconditions->us_left) ||
        (stateconditions.us_right >= currentconditions->us_right / 2) ||
        //(stateconditions.us_front_left >= currentconditions->us_front_left) ||
        //(stateconditions.us_front_right >= currentconditions->us_front_right) ||
        (stateconditions.distance < currentconditions->distance) ||
        (stateconditions.lidar > currentconditions->lidar) ||
        (stateconditions.previous_state != FINISH);

    return returnvalue;
}
//--------------------------------------------------------------------------------------

/// @brief
/// @param currentconditions
/// @return
int ForwardCorrectLeft::testActive(TransitionConditions *currentconditions)
{
    (void)currentconditions;

    setActive(false); // this state is the entrypoint never true
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void ForwardCorrectLeft::Update(Statemachine *statemachine)
{
    setParkingSteering(this->maxvalues.speed_forward, statemachine);
    setParkingSpeed(this->maxvalues.steering_left, statemachine);
}
//--------------------------------------------------------------------------------------
// #######################################################################################
//                                                                                      #
//               FORWARDCORRECTRIGHT CLASS IMPLEMENTTION                                #
//                                                                                      #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void ForwardCorrectRight::afterChange(Statemachine *statemachine)
{
    statemachine->sensors->ResetDistance();
    setParkingSteering(this->maxvalues.speed_forward, statemachine);
    setParkingSpeed(this->maxvalues.steering_right, statemachine);
}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int ForwardCorrectRight::testConditions(TransitionConditions *currentconditions)
{
    // imu is dont care
    // right us is standard safety distancy
    // left us needs some distance but since the car is moving right safety margin is halfed
    // us front is dont care since there should never be a measureable object  in front also lidar
    //  will give a more reliable signal
    //
    int returnvalue = 0;
    returnvalue =
        (stateconditions.us_left >= currentconditions->us_left / 2) ||
        (stateconditions.us_right >= currentconditions->us_right) ||
        //(stateconditions.us_front_left >= currentconditions->us_front_left) ||
        //(stateconditions.us_front_right >= currentconditions->us_front_right) ||
        (stateconditions.distance < currentconditions->distance) ||
        (stateconditions.lidar > currentconditions->lidar) ||
        (stateconditions.previous_state != FINISH);

    return returnvalue;
}
//--------------------------------------------------------------------------------------

/// @brief
/// @param currentconditions
/// @return
int ForwardCorrectRight::testActive(TransitionConditions *currentconditions)
{
    (void)currentconditions;

    setActive(false); // this state is the entrypoint never true
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void ForwardCorrectRight::Update(Statemachine *statemachine)
{
    setParkingSteering(this->maxvalues.speed_forward, statemachine);
    setParkingSpeed(this->maxvalues.steering_right, statemachine);
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                      #
//               FORWARDCORRECTSTRAIGHT CLASS IMPLEMENTTION                             #
//                                                                                      #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void ForwardCorrectStraight::afterChange(Statemachine *statemachine)
{
    statemachine->sensors->ResetDistance();
    setParkingSteering(this->maxvalues.speed_forward, statemachine);
    setParkingSpeed(this->maxvalues.steering_straight, statemachine);
}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int ForwardCorrectStraight::testConditions(TransitionConditions *currentconditions)
{
    // imu is dont care
    // right us needs some distance but since the car is moving straight safety margin is halfed
    // left us needs some distance but since the car is moving straight safety margin is halfed
    // us front is dont care since there should never be a measureable object  in front also lidar
    //  will give a more reliable signal
    //
    int returnvalue = 0;
    returnvalue =
        (stateconditions.us_left >= currentconditions->us_left / 2) ||
        (stateconditions.us_right >= currentconditions->us_right / 2) ||
        //(stateconditions.us_front_left >= currentconditions->us_front_left) ||
        //(stateconditions.us_front_right >= currentconditions->us_front_right) ||
        (stateconditions.distance < currentconditions->distance) ||
        (stateconditions.lidar > currentconditions->lidar) ||
        (stateconditions.previous_state != FINISH);

    return returnvalue;
}
//--------------------------------------------------------------------------------------

/// @brief
/// @param currentconditions
/// @return. ins    loc bash
int ForwardCorrectStraight::testActive(TransitionConditions *currentconditions)
{
    (void)currentconditions;

    setActive(false); // this state is the entrypoint never true
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void ForwardCorrectStraight::Update(Statemachine *statemachine)
{
    setParkingSteering(this->maxvalues.speed_forward, statemachine);
    setParkingSpeed(this->maxvalues.steering_straight, statemachine);
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                      #
//               REVERSECORRECTLEFT CLASS IMPLEMENTTION                                 #
//                                                                                      #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void ReverseCorrectLeft::afterChange(Statemachine *statemachine)
{
    statemachine->sensors->ResetDistance();
    setParkingSteering(this->maxvalues.speed_forward, statemachine);
    setParkingSpeed(this->maxvalues.steering_left, statemachine);
}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int ReverseCorrectLeft::testConditions(TransitionConditions *currentconditions)
{
    // imu is dont care
    // right us is standard safety distancy
    // left us needs some distance but since the car is moving right safety margin is halfed
    // us front is dont care since there should never be a measureable object  in front also lidar
    //  will give a more reliable signal
    //
    int returnvalue = 0;
    returnvalue =
        (stateconditions.us_left >= currentconditions->us_left) ||
        (stateconditions.us_right >= currentconditions->us_right / 2) ||
        //(stateconditions.us_front_left >= currentconditions->us_front_left) ||
        //(stateconditions.us_front_right >= currentconditions->us_front_right) ||
        (stateconditions.distance < currentconditions->distance) ||
        (stateconditions.lidar > currentconditions->lidar) ||
        (stateconditions.previous_state != FINISH);

    return returnvalue;
}
//--------------------------------------------------------------------------------------

/// @brief
/// @param currentconditions
/// @return
int ReverseCorrectLeft::testActive(TransitionConditions *currentconditions)
{
    (void)currentconditions;

    setActive(false); // this state is the entrypoint never true
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void ReverseCorrectLeft::Update(Statemachine *statemachine)
{
    setParkingSteering(this->maxvalues.speed_forward, statemachine);
    setParkingSpeed(this->maxvalues.steering_left, statemachine);
}
//--------------------------------------------------------------------------------------
// #######################################################################################
//                                                                                      #
//               FORWARDCORRECTRIGHT CLASS IMPLEMENTTION                                #
//                                                                                      #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void ReverseCorrectRight::afterChange(Statemachine *statemachine)
{
    statemachine->sensors->ResetDistance();
    setParkingSteering(this->maxvalues.speed_forward, statemachine);
    setParkingSpeed(this->maxvalues.steering_right, statemachine);
}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int ReverseCorrectRight::testConditions(TransitionConditions *currentconditions)
{
    // imu is dont care
    // right us is standard safety distancy
    // left us needs some distance but since the car is moving right safety margin is halfed
    // us front is dont care since there should never be a measureable object  in front also lidar
    //  will give a more reliable signal
    //
    int returnvalue = 0;
    returnvalue =
        (stateconditions.us_left >= currentconditions->us_left / 2) ||
        (stateconditions.us_right >= currentconditions->us_right) ||
        //(stateconditions.us_front_left >= currentconditions->us_front_left) ||
        //(stateconditions.us_front_right >= currentconditions->us_front_right) ||
        (stateconditions.distance < currentconditions->distance) ||
        (stateconditions.lidar > currentconditions->lidar) ||
        (stateconditions.previous_state != FINISH);

    return returnvalue;
}
//--------------------------------------------------------------------------------------

/// @brief
/// @param currentconditions
/// @return
int ReverseCorrectRight::testActive(TransitionConditions *currentconditions)
{
    (void)currentconditions;

    setActive(false); // this state is the entrypoint never true
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void ReverseCorrectRight::Update(Statemachine *statemachine)
{
    setParkingSteering(this->maxvalues.steering_right, statemachine);
    setParkingSpeed(this->maxvalues.speed_reverse, statemachine);
}
//--------------------------------------------------------------------------------------
// #######################################################################################
//                                                                                      #
//               ReverseCorrectStraight CLASS IMPLEMENTTION                             #
//                                                                                      #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void ReverseCorrectStraight::afterChange(Statemachine *statemachine)
{
    statemachine->sensors->ResetDistance();
    setParkingSteering(this->maxvalues.speed_forward, statemachine);
    setParkingSpeed(this->maxvalues.steering_straight, statemachine);
}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int ReverseCorrectStraight::testConditions(TransitionConditions *currentconditions)
{
    // imu is dont care
    // right us needs some distance but since the car is moving straight safety margin is halfed
    // left us needs some distance but since the car is moving straight safety margin is halfed
    // us front is dont care since there should never be a measureable object  in front also lidar
    //  will give a more reliable signal
    //
    int returnvalue = 0;
    returnvalue =
        (stateconditions.us_left >= currentconditions->us_left / 2) ||
        (stateconditions.us_right >= currentconditions->us_right / 2) ||
        //(stateconditions.us_front_left >= currentconditions->us_front_left) ||
        //(stateconditions.us_front_right >= currentconditions->us_front_right) ||
        (stateconditions.distance < currentconditions->distance) ||
        (stateconditions.lidar > currentconditions->lidar) ||
        (stateconditions.previous_state != FINISH);

    return returnvalue;
}
//--------------------------------------------------------------------------------------

/// @brief
/// @param currentconditions
/// @return
int ReverseCorrectStraight::testActive(TransitionConditions *currentconditions)
{
    (void)currentconditions;

    setActive(false); // this state is the entrypoint never true
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void ReverseCorrectStraight::Update(Statemachine *statemachine)
{
    setParkingSteering(this->maxvalues.steering_straight, statemachine);
    setParkingSpeed(this->maxvalues.speed_reverse, statemachine);
}
//--------------------------------------------------------------------------------------
 */
// #######################################################################################
//                                                                                      #
//               FORWARDLEFT CLASS IMPLEMENTTION                                        #
//                                                                                      #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void ForwardLeft::afterChange(Statemachine *statemachine)
{
    statemachine->sensors->ResetDistance();
    setActive(true);
    setParkingSteering(this->maxvalues.steering_left, statemachine);
    setParkingSpeed(this->maxvalues.speed_forward, statemachine);
    stateconditions.distance += lidar_correction.wall_offset * 0.25;
}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int ForwardLeft::testConditions(TransitionConditions *currentconditions)
{
    // imu is dont care
    // right us needs some distance but since the car is moving to the left safety margin is halfed
    // left us needs standard safety distance
    //  rear us needs some distance but less than the distance to be travelled since car moves in a curve
    //
    double correct = this->lidar_correction.wall_offset;
 
    if (correct >= 0.6)
        correct = 0; // Trackwidth is only 0.7m s

    int returnvalue =
        (stateconditions.us_right + correct > currentconditions->us_right) && // object (parking car) close proximity right otherwise  no left move is needed
        (currentconditions->previous_state_active == false) &&
        (currentconditions->previous_state != FINISH);

    return returnvalue;
}
//--------------------------------------------------------------------------------------

/// @brief test if task is done
/// @param currentconditions
/// @return
int ForwardLeft::testActive(TransitionConditions *currentconditions)
{
    setActive((stateconditions.distance > currentconditions->distance)); // only the driven distance is important
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void ForwardLeft::Update(Statemachine *statemachine)
{
    if (isActive())
    {
        setParkingSteering(this->maxvalues.steering_left, statemachine);
        setParkingSpeed(this->maxvalues.speed_forward, statemachine);
    }
    else
    {
        setParkingSteering(this->maxvalues.steering_straight, statemachine);
        setParkingSpeed(this->maxvalues.speed_stop, statemachine);
    }
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                       #
//               FORWARDRIGHT CLASS IMPLEMENTTION                                        #
//                                                                                       #
// #######################################################################################
/*
/// @brief // this code  gets executed after a statechange
/// @param statemachine
void ForwardRight::afterChange(Statemachine *statemachine)
{
    statemachine->sensors->ResetDistance();
    setActive(true);
    setParkingSteering(this->maxvalues.speed_reverse, statemachine);
    setParkingSpeed(this->maxvalues.steering_right, statemachine);
}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int ForwardRight::testConditions(TransitionConditions *currentconditions)
{
    (void)currentconditions;
    // imu is dont care
    // left us needs some distance but since the car is moving to the right safety margin is halfed
    // right us needs standard safety distance
    //  rear us needs some distance but less than the distance to be travelled since car moves in a curve
    // distance

    int returnvalue = 0;

    return returnvalue;
}
//--------------------------------------------------------------------------------------

/// @brief
/// @param currentconditions
/// @return
int ForwardRight::testActive(TransitionConditions *currentconditions)
{
    (void)currentconditions;

    setActive(false);
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void ForwardRight::Update(Statemachine *statemachine)
{
    setParkingSteering(this->maxvalues.speed_reverse, statemachine);
    setParkingSpeed(this->maxvalues.steering_right, statemachine);
}
//--------------------------------------------------------------------------------------
 */
// #######################################################################################
//                                                                                       #
//               ForwardStraight CLASS IMPLEMENTTION                                     #
//                                                                                       #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void ForwardStraight::afterChange(Statemachine *statemachine)
{
    statemachine->sensors->ResetDistance();
    setActive(true);
    setParkingSteering(this->maxvalues.speed_reverse, statemachine);
    setParkingSpeed(this->maxvalues.steering_right, statemachine);
}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  false if no match
int ForwardStraight::testConditions(TransitionConditions *currentconditions)
{
    // imu smaller 5° or larger 85°
    // us sides more than 150mm
    // us rear more than 100mm
    // distance min 100mm   else car wont be able to brake reliable
    // previous state: any except Finish

    int returnvalue = 0;
    returnvalue =
        (stateconditions.us_front_left >= currentconditions->us_front_left) &&
        (stateconditions.us_front_right >= currentconditions->us_front_right) &&
        (currentconditions->previous_state_active == false) &&
        (stateconditions.previous_state != FINISH);

    return returnvalue;
}
//--------------------------------------------------------------------------------------

/// @brief
/// @param currentconditions
/// @return
int ForwardStraight::testActive(TransitionConditions *currentconditions)
{

    setActive(this->stateconditions.distance > currentconditions->distance); //
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void ForwardStraight::Update(Statemachine *statemachine)
{
    setParkingSteering(this->maxvalues.speed_reverse, statemachine);
    setParkingSpeed(this->maxvalues.steering_right, statemachine);
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                      #
//               REVERSELEFT CLASS IMPLEMENTTION                                        #
//                                                                                      #
// #######################################################################################
/*
/// @brief // this code  gets executed after a statechange
/// @param statemachine
void ReverseLeft::afterChange(Statemachine *statemachine)
{
    statemachine->sensors->ResetDistance();
    setActive(true);
    setParkingSteering(this->maxvalues.speed_reverse, statemachine);
    setParkingSpeed(this->maxvalues.steering_right, statemachine);
}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int ReverseLeft::testConditions(TransitionConditions *currentconditions)
{
    // imu is dont care
    // right us needs some distance but since the car is moving to the left safety margin is halfed
    // left us needs standard safety distance
    //  rear us needs some distance but less than the distance to be travelled since car moves in a curve
    //
    int returnvalue = 0;
    returnvalue =
        (stateconditions.us_left >= currentconditions->us_left) ||
        (stateconditions.us_right >= currentconditions->us_right) ||
        (stateconditions.us_rear_left >= currentconditions->us_rear_left) ||
        (stateconditions.us_rear_right >= currentconditions->us_rear_right) ||
        (stateconditions.distance < currentconditions->distance) ||
        (stateconditions.previous_state != FINISH);

    return returnvalue;
}
//--------------------------------------------------------------------------------------

/// @brief
/// @param currentconditions
/// @return
int ReverseLeft::testActive(TransitionConditions *currentconditions)
{
    (void)currentconditions;

    setActive(false); // this state is the entrypoint never true
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void ReverseLeft::Update(Statemachine *statemachine)
{
    setParkingSteering(this->maxvalues.speed_reverse, statemachine);
    setParkingSpeed(this->maxvalues.steering_left, statemachine);
}
//-------------------------------------------------------------------------------------- */

// #######################################################################################
//                                                                                       #
//               REVERSERIGHT CLASS IMPLEMENTTION                                        #
//                                                                                       #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void ReverseRight::afterChange(Statemachine *statemachine)
{
    statemachine->sensors->ResetDistance();
    setActive(true);
    setParkingSpeed(this->maxvalues.speed_reverse, statemachine);

    setParkingSteering(this->maxvalues.steering_straight, statemachine);

    //     setParkingSteering(this->maxvalues.steering_right, statemachine);
}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int ReverseRight::testConditions(TransitionConditions *currentconditions)
{
    // imu is dont care
    // left us should never see an object
    // right us needs standard safety distance but is covered by Stop
    //  rear us needs some empty space but less than the distance to be travelled since car moves in a curve to the free spot
    //

    int returnvalue = 0;
    returnvalue =
        (stateconditions.us_rear_left < currentconditions->us_rear_left) &&
        (stateconditions.us_rear_right < currentconditions->us_rear_right) &&
        (currentconditions->previous_state_active == false) &&
        (stateconditions.previous_state != FINISH);

    return returnvalue;
}
//--------------------------------------------------------------------------------------

/// @brief
/// @param currentconditions
/// @return
int ReverseRight::testActive(TransitionConditions *currentconditions)
{

    setActive((this->stateconditions.imu_max > abs(currentconditions->imu)) &&
              (this->stateconditions.imu_min > abs(currentconditions->imu)) &&
              (this->stateconditions.distance > abs(currentconditions->distance)));

    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void ReverseRight::Update(Statemachine *statemachine)
{
    setParkingSpeed(this->maxvalues.speed_reverse, statemachine);

    if (statemachine->sensors->Distance() > 0.03)
    {
        std::cout << "MIST " << statemachine->sensors->Distance() << std::endl;
        setParkingSteering(this->maxvalues.steering_straight, statemachine);
    }
    else
        setParkingSteering(this->maxvalues.steering_right, statemachine);
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                       #
//               REVERSESTRAIGHT CLASS IMPLEMENTTION                                     #
//                                                                                       #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void ReverseStraight::afterChange(Statemachine *statemachine)
{
    statemachine->sensors->ResetDistance();
    setActive(true);
    setParkingSpeed(this->maxvalues.speed_reverse, statemachine);
    setParkingSteering(this->maxvalues.steering_straight, statemachine);
}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int ReverseStraight::testConditions(TransitionConditions *currentconditions)
{
    // imu smaller 95° and larger 85°
    // us sides more than 150mm
    // us rear more than 100mm
    // previous state: any except Finish

    int returnvalue = 0;

    returnvalue =

        (stateconditions.us_rear_left <= currentconditions->us_rear_left) &&
        (stateconditions.us_rear_right <= currentconditions->us_rear_right) &&
        (currentconditions->previous_state_active == false) &&
        (stateconditions.previous_state != FINISH);

    return returnvalue;
}
//--------------------------------------------------------------------------------------

/// @brief
/// @param currentconditions
/// @return
int ReverseStraight::testActive(TransitionConditions *currentconditions)
{
    setActive(
        ((this->stateconditions.distance - lidar_correction.wall_offset) < currentconditions->distance) &&
        (stateconditions.us_rear_left < currentconditions->us_rear_left) &&
        (stateconditions.us_rear_right < currentconditions->us_rear_right));
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void ReverseStraight::Update(Statemachine *statemachine)
{
    setParkingSpeed(maxvalues.speed_reverse, statemachine);
    setParkingSteering(maxvalues.steering_straight, statemachine);
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                       #
//               FINISH CLASS IMPLEMENTTION                                              #
//                                                                                       #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void Finish::afterChange(Statemachine *statemachine)
{
    setActive(true);
    setParkingSpeed(maxvalues.speed_stop, statemachine);
    setParkingSteering(maxvalues.steering_straight, statemachine);

}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int Finish::testConditions(TransitionConditions *currentconditions)
{
    // imu is ~ 90° or -90°   but not smaller than 85°
    // rear us are <= 50mm
    // side us are <= 200mm
    // last step was reverse straight
    // ERRORCASE  car travelled  more than x meters forward or reverse

    int returnvalue = 0;
    returnvalue =
        ((
             ((stateconditions.us_left <= currentconditions->us_left) &&
              (stateconditions.us_right <= currentconditions->us_right)) &&
             ((stateconditions.us_rear_left >= currentconditions->us_rear_left) &&
              (stateconditions.us_rear_right >= currentconditions->us_rear_right))) ||
         (currentconditions->previous_state_active == false)) &&
        (stateconditions.previous_state != FINISH);

    return returnvalue;
}
//--------------------------------------------------------------------------------------

int Finish::testActive(TransitionConditions *currentconditions)
{
    (void)currentconditions;

    setActive(true); // finish is always true
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void Finish::Update(Statemachine *statemachine)
{
    (void)statemachine;
    // nop
}
//--------------------------------------------------------------------------------------