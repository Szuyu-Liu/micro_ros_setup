#include "parallel_in/parallel_parking_statemachine.h"
#include <math.h>
#include <iostream>

using namespace ParallelParkingIn;

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
    /*
    test.distance = statemachine->sensors->Distance();
    test.imu = statemachine->sensors->Heading();
    test.us_left = statemachine->sensors->ULeft();
    test.us_front_left = statemachine->sensors->UFrontLeft();
    test.us_front_right = statemachine->sensors->UFrontRight();
    test.us_rear_left = statemachine->sensors->URearLeft();
    test.us_rear_right = statemachine->sensors->URearRight();
    test.us_right = statemachine->sensors->URight(); */
    test.sensors = statemachine->sensors;
    test.previous_state = this->WhoAmI();

    test.previous_state_active = statemachine->currentstate->testActive(&test);

    std::cout << "---------------------------------------------------" << std::endl;
    std::cout << "State         " << test.previous_state << std::endl;
    std::cout << "distance is   " << statemachine->sensors->Distance() << std::endl;
    std::cout << "distance      " << statemachine->currentstate->stateconditions.distance << std::endl;
    std::cout << "still active  " << test.previous_state_active << std::endl;
    std::cout << "imu is        " << statemachine->sensors->Heading() << std::endl;
    std::cout << "imu           " << statemachine->currentstate->stateconditions.imu_min << std::endl;
    std::cout << "us_front_left " << statemachine->sensors->UFrontLeft() << std::endl;
    std::cout << "us_front_right" << statemachine->sensors->UFrontRight() << std::endl;
    std::cout << "us_left       " << statemachine->sensors->ULeft() << std::endl;
    std::cout << "us_rear_left  " << statemachine->sensors->URearLeft() << std::endl;
    std::cout << "us_rear_right " << statemachine->sensors->URearRight() << std::endl;
    std::cout << "us_right      " << statemachine->sensors->URight() << std::endl;
    std::cout << "lidar right   " << statemachine->sensors->LidarRightMin() << std::endl;
    std::cout << "lidar left    " << statemachine->sensors->LidarLeftMin() << std::endl;
    std::cout << "lidar front   " << statemachine->sensors->LidarFrontMin() << std::endl;
    std::cout << "lidar r angle " << statemachine->sensors->LidarRightMinDistanceAngle() << std::endl;
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
        // std::cout << "new State         " << newstate->WhoAmI() << std::endl;
        statemachine->UpdateState(*newstate); // replace the state
    }
    else if (statemachine->currentstate != nullptr)
    {
        // std::cout << "old State         " << statemachine->currentstate->WhoAmI() << std::endl;
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
    setParkingSpeed(this->maxvalues.speed_stop, statemachine);
}
//--------------------------------------------------------------------------------------

/// @brief Test if the given conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int Start::testConditions(TransitionConditions *currentconditions)
{
    (void)currentconditions;
    return false;
}
//--------------------------------------------------------------------------------------

/// @brief
/// @param currentconditions
/// @return
int Start::testActive(TransitionConditions *currentconditions)
{
    (void)currentconditions;
    setActive(
        false);

    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void Start::Update(Statemachine *statemachine)
{
    (void)statemachine;
    /*
      this should be done in the first state after start
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
                  << std::endl; */
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
        ((stateconditions.us_left >= currentconditions->sensors->ULeft()) ||
         (stateconditions.us_right >= currentconditions->sensors->URight()) ||
         (stateconditions.us_front_left >= currentconditions->sensors->UFrontLeft()) ||
         (stateconditions.us_front_right >= currentconditions->sensors->UFrontRight()) ||
         (stateconditions.us_rear_left >= currentconditions->sensors->URearLeft()) ||
         (stateconditions.us_rear_right >= currentconditions->sensors->URearRight())) &&
        //(stateconditions.lidar > currentconditions->lidar) ||
        ((stateconditions.previous_state != FINISH) ||
         (stateconditions.previous_state != START));

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
    while(1);
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
        (stateconditions.previous_state != START);

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
        statemachine->sensors->ResetHeading(-1*(statemachine->sensors->LidarRightMinDistanceAngle() +
                                            90 - maxvalues.lidar_correction.lidar_angle_offset));

        this->lidar_correction.wall_offset = statemachine->sensors->LidarRightMin() -
                                             maxvalues.lidar_correction.wall_distance;
        ForwardStraight::getInstance(&maxvalues, allstateconditions).lidar_correction.wall_offset =
            this->lidar_correction.wall_offset;
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

    setActive((stateconditions.distance > currentconditions->sensors->Distance()) &&
              (stateconditions.us_right < currentconditions->sensors->LidarRightMin()));

    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void PositionFound::Update(Statemachine *statemachine)
{
    float keepitstraight = (0 - statemachine->sensors->Heading()) * 0.05;
    setParkingSteering(this->maxvalues.steering_straight + keepitstraight, statemachine);
    setParkingSpeed(this->maxvalues.speed_forward, statemachine);
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                       #
//               FORWARD STRAIGHT CLASS IMPLEMENTTION                                    #
//                                                                                       #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void ForwardStraight::afterChange(Statemachine *statemachine)
{
    setActive(true);
    statemachine->sensors->ResetDistance();
    if (lidar_correction.wall_offset > 0)
    {
        stateconditions.distance += lidar_correction.wall_offset;
    }
    ReverseStraight::getInstance(&maxvalues, allstateconditions).lidar_correction.wall_offset =
        sqrt(2 * (pow(this->stateconditions.distance, 2))); // c² = a² + b²  45° means a = b = 2a²

    setParkingSpeed(maxvalues.speed_forward, statemachine);
    //setParkingSteering(maxvalues.steering_straight, statemachine);
}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int ForwardStraight::testConditions(TransitionConditions *currentconditions)
{

    int returnvalue =
        (((stateconditions.us_front_left < currentconditions->sensors->UFrontLeft()) &&
          (stateconditions.us_front_right < currentconditions->sensors->UFrontRight()) &&
          (currentconditions->previous_state_active == false)) ||
         ((currentconditions->previous_state_active == false) &&
          (stateconditions.previous_state != START) &&
          (stateconditions.previous_state != FINISH)));

    return returnvalue;
}
//--------------------------------------------------------------------------------------

int ForwardStraight::testActive(TransitionConditions *currentconditions)
{
    setActive((stateconditions.distance > currentconditions->sensors->Distance()));
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void ForwardStraight::Update(Statemachine *statemachine)
{
    float keepitstraight = (0 - statemachine->sensors->Heading()) * 0.05; // <===   HHAARG
    setParkingSteering(this->maxvalues.steering_straight + keepitstraight, statemachine);
    setParkingSpeed(this->maxvalues.speed_forward, statemachine);
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                       #
//               REVERSE RIGHT CLASS IMPLEMENTTION                                              #
//                                                                                       #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void ReverseRight::afterChange(Statemachine *statemachine)
{
    setActive(true);
    first = 0;
    statemachine->sensors->ResetDistance();
    setParkingSpeed(maxvalues.speed_stop, statemachine);
    setParkingSteering(maxvalues.steering_straight, statemachine);
}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int ReverseRight::testConditions(TransitionConditions *currentconditions)
{
    int returnvalue = 0;
    returnvalue =
        ((
             ((stateconditions.us_rear_left >= currentconditions->sensors->URearLeft()) &&
              (stateconditions.us_rear_right >= currentconditions->sensors->URearRight()))) ||
         ((currentconditions->previous_state_active == false) &&
          ((stateconditions.previous_state != START) &&
           (stateconditions.previous_state != FINISH))));

    return returnvalue;
}
//--------------------------------------------------------------------------------------

int ReverseRight::testActive(TransitionConditions *currentconditions)
{

    setActive((stateconditions.distance < currentconditions->sensors->Distance()) ||
              (stateconditions.imu_min > abs(currentconditions->sensors->Heading())));
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void ReverseRight::Update(Statemachine *statemachine)
{
    if (first < 5)
    {
        setParkingSpeed(maxvalues.speed_stop, statemachine);
        setParkingSteering(maxvalues.steering_straight, statemachine);
    }
    else
    {
        setParkingSpeed(maxvalues.speed_reverse, statemachine);
        setParkingSteering(maxvalues.steering_right, statemachine);
    }

    first++;
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                       #
//               REVERSE STRAIGHT CLASS IMPLEMENTTION                                    #
//                                                                                       #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void ReverseStraight::afterChange(Statemachine *statemachine)
{
    setActive(true);
    statemachine->sensors->ResetDistance();
    setParkingSpeed(maxvalues.speed_reverse, statemachine);
    setParkingSteering(maxvalues.steering_straight, statemachine);
    stateconditions.distance -= lidar_correction.wall_offset;
}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int ReverseStraight::testConditions(TransitionConditions *currentconditions)
{
    int returnvalue =
        (((stateconditions.us_rear_left > currentconditions->sensors->URearLeft()) &&
          (stateconditions.us_rear_right > currentconditions->sensors->URearRight())) ||
         ((currentconditions->previous_state_active == false) &&
          (stateconditions.previous_state != FINISH) &&
          (stateconditions.previous_state != START)));

    return returnvalue;
}
//--------------------------------------------------------------------------------------

int ReverseStraight::testActive(TransitionConditions *currentconditions)
{

    setActive((stateconditions.distance < currentconditions->sensors->Distance()) &&
              (stateconditions.us_rear_left < currentconditions->sensors->URearLeft()) &&
              (stateconditions.us_rear_right < currentconditions->sensors->URearRight()));

    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void ReverseStraight::Update(Statemachine *statemachine)
{
    float keepitstraight = 45 - statemachine->sensors->Heading() * 0.01; // <===   HHAARG
    setParkingSteering(this->maxvalues.steering_straight + keepitstraight, statemachine);
    setParkingSpeed(this->maxvalues.speed_reverse, statemachine);
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                       #
//               REVERSE LEFT CLASS IMPLEMENTTION                                        #
//                                                                                       #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void ReverseLeft::afterChange(Statemachine *statemachine)
{
    setActive(true);
    first = 0;
    statemachine->sensors->ResetDistance();
    setParkingSpeed(maxvalues.speed_reverse, statemachine);
    setParkingSteering(maxvalues.steering_left, statemachine);
}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int ReverseLeft::testConditions(TransitionConditions *currentconditions)
{
    // imu is ~ 90° or -90°   but not smaller than 85°
    // rear us are <= 50mm
    // side us are <= 200mm
    // last step was reverse straight
    // ERRORCASE  car travelled  more than x meters forward or reverse

    int returnvalue = 0;
    returnvalue =
        (((stateconditions.us_right < currentconditions->sensors->LidarRightMin()) &&
          (stateconditions.us_rear_left > currentconditions->sensors->URearLeft()) &&
          (stateconditions.us_rear_right > currentconditions->sensors->URearRight())) ||
         ((currentconditions->previous_state_active == false) &&
          (stateconditions.previous_state != FINISH) &&
          (stateconditions.previous_state != START)));

    return returnvalue;
}
//--------------------------------------------------------------------------------------

int ReverseLeft::testActive(TransitionConditions *currentconditions)
{
    (void)currentconditions;

    setActive((stateconditions.us_right < currentconditions->sensors->LidarRightMin()) &&
              (stateconditions.us_rear_left < currentconditions->sensors->URearLeft()) &&
              (stateconditions.us_rear_right < currentconditions->sensors->URearRight()) &&
              (stateconditions.distance < currentconditions->sensors->Distance()) &&
              (first < 10));
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void ReverseLeft::Update(Statemachine *statemachine)
{
    if (stateconditions.distance > statemachine->sensors->Distance())
    {
        first++;
        setParkingSpeed(maxvalues.speed_stop, statemachine);
        setParkingSteering(maxvalues.steering_left, statemachine);
    }
    else
    {
        setParkingSpeed(maxvalues.speed_reverse, statemachine);
        setParkingSteering(maxvalues.steering_left, statemachine);
    }
}
//--------------------------------------------------------------------------------------

// #######################################################################################
//                                                                                       #
//               FORWARD RIGHT CLASS IMPLEMENTTION                                              #
//                                                                                       #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void ForwardRight::afterChange(Statemachine *statemachine)
{
    setActive(true);
    statemachine->sensors->ResetDistance();
    setParkingSpeed(maxvalues.speed_forward, statemachine);
    setParkingSteering(maxvalues.steering_right, statemachine);
}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int ForwardRight::testConditions(TransitionConditions *currentconditions)
{
    // imu is ~ 90° or -90°   but not smaller than 85°
    // rear us are <= 50mm
    // side us are <= 200mm
    // last step was reverse straight
    // ERRORCASE  car travelled  more than x meters forward or reverse

    int returnvalue = 0;
    returnvalue =
        (((stateconditions.us_right <= currentconditions->sensors->LidarRightMin()) &&
          (stateconditions.us_front_left >= currentconditions->sensors->LidarFrontMin())) ||
         ((currentconditions->previous_state_active == false) &&
          (stateconditions.previous_state != START)));

    return returnvalue;
}
//--------------------------------------------------------------------------------------

int ForwardRight::testActive(TransitionConditions *currentconditions)
{
    (void)currentconditions;

    setActive(
        (stateconditions.us_front_left <= currentconditions->sensors->LidarFrontMin()) &&
        (stateconditions.distance > currentconditions->sensors->Distance()) &&
        (stateconditions.imu_max < abs(currentconditions->sensors->Heading())));
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void ForwardRight::Update(Statemachine *statemachine)
{
    setParkingSpeed(maxvalues.speed_forward, statemachine);
    setParkingSteering(maxvalues.steering_right, statemachine);

}
//--------------------------------------------------------------------------------------

/*
// #######################################################################################
//                                                                                       #
//               TEMPLATE CLASS IMPLEMENTTION                                              #
//                                                                                       #
// #######################################################################################

/// @brief // this code  gets executed after a statechange
/// @param statemachine
void Template::afterChange(Statemachine *statemachine)
{
    setActive(true);
    setParkingSpeed(maxvalues.speed_stop, statemachine);
    setParkingSteering(maxvalues.steering_straight, statemachine);

}
//--------------------------------------------------------------------------------------

/// @brief Test if the gicen conditions match this states conditions
/// @param currentconditions List of measured values
/// @return  0 if no match
int Template::testConditions(TransitionConditions *currentconditions)
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
        (stateconditions.previous_state != START);

    return returnvalue;
}
//--------------------------------------------------------------------------------------

int Template::testActive(TransitionConditions *currentconditions)
{
    (void)currentconditions;

    setActive(true); // Template is always true
    return isActive();
}
//--------------------------------------------------------------------------------------

/// @brief this code gets executed if the there is no statechange
void Template::Update(Statemachine *statemachine)
{
    (void)statemachine;
    // nop
}
//--------------------------------------------------------------------------------------
*/