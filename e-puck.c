// By Pamir Ghimire (pamirghimire <> gmail)
// With Gopikrishna Erabati
// For Autonomous Robotics coursework
// Graduate Student, M1
// MSCV (Computer Vision and Robotics CVR)
// Universite De Bourgogne, France 

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 32
#define Dwth 135
#define Dwth_wall 150

// Robot constants for odometry
#define WHEEL_RADIUS 0.0205 // in m
#define AXLE_LENGTH 0.052 // in m
#define STEPS_ROT 1000 // 1000 steps per rotatation
#define PI 3.141592654
#define PI2 6.283185307

// Global variable: Robot's pose
double pose[3];

// Goal position : for bug0 algorithm
double goal[2] = {-0.35,-1};

//SOME BEHAVIORS: Declarations here, definitions after main()
//----------------------------------------------------------------------
// Return true if robot has arrived at goal
bool arrivedAtGoal();
// Get orientation of target from current position
double getGoalTheta();
// Turn the robot towards target
void turnTowardGoal();
// Return id of sensor facing goal
int getSensorFacingGoal();
// Return true if open space toward goal
bool openSpaceTowardGoal();
// Get displacements of left and right wheels given change in encoder positions
void getWheelDisplacements(double *dispLeftW, double *dispRightW, double del_enLeftW, double del_enRightW);
// Update pose vector (after a simulation step) and prints it to the console
void updatePose();
// Move forward with speed = sForward, over ntimeSteps
bool moveForward(double sForward);
// Move forward slowly for a fixed distance
bool clearObstacle();
// Stop robot!
bool stop();
// Turn robot 90 degrees ccw in place
bool turn90ccw();
// Follow the obstacle boundary on right side of robot
bool followWall(double sForward, double eprev);
// Return true if open space is available in front of robot
bool openSpaceAhead();
// Return true if obstacle present on right hand side of the robot
bool wallToRight();
// Read the indicated distance sensor and return value
double readDsensor(int nsensor);
//-----------------------------------------------------------------------
//---------------  MAIN : -----------------------------------------------
int main(int argc, char *argv[]) {

  // initialize robot
  wb_robot_init();
  
  //intialize encoder
  wb_differential_wheels_enable_encoders(TIME_STEP);
  
  // Define forward speed
  double sForward = 300;
 
  // Errors for guaging alignment along wall
  // For WALL FOLLOWING MODE:
  double eprev = 0;
  double ecurr = 0;
  
  // Robot's pose
  pose[0] = 0; pose[1] = 0; pose[2] = 0;
  
  //-------------------------------------------------------------
  // GO TO GOAL BEHAVIOR:
  //-------------------------------------------------------------
  turnTowardGoal();
  
  /* main loop */
  for (;;) {
    // Reset wheel encoders
    wb_differential_wheels_set_encoders(0, 0);
    
    // Error term for WALL FOLLOWING MODE
    eprev = ecurr;
  
    if (openSpaceAhead()){
    
      // MOVE FORWARD MODE
      printf("\n MOVE-FORWARD MODE:");
      printf("\n Sensor Facing Goal = %d", getSensorFacingGoal());
      moveForward(sForward);
      
      // stop if robot has arrived at goal
      if (arrivedAtGoal()){
        stop();
        break;
      }

      
    }
    else{
      
      // WALL FOLLOWING MODE:
      turn90ccw();
      while (openSpaceAhead()){
        
        // Reset wheel encoders
        wb_differential_wheels_set_encoders(0, 0);
    
        printf("\n\n Wall following mode:");
        eprev = ecurr;
        followWall(sForward, eprev);
        
        // Stop if arrived at goal while following wall!
        if (arrivedAtGoal()){
          stop();
          break;
        }
        
        // If open space towards goal, stop following wall
        if (!wallToRight()){
          followWall(0.5*sForward, eprev);
          if(!wallToRight()){
            // To clear the obstacle
            clearObstacle();
            turnTowardGoal();
            break;            
          }
        }
        
        // Error term for wall following mode, PID controlled
        ecurr = (0.7*readDsensor(2) + 0.3*readDsensor(1))- Dwth;
        
        wb_robot_step(TIME_STEP);
        updatePose();
        printf("\n Goal orientation wrt (y = 0) = %f \n", getGoalTheta());
      }
    }
    
    ecurr = (0.7*readDsensor(2) + 0.3*readDsensor(1))- Dwth;
    
    wb_robot_step(TIME_STEP);
    updatePose();
    printf("\n Goal orientation wrt (y = 0) = %f \n", getGoalTheta());

  }

  wb_robot_step(TIME_STEP);
  updatePose();
  wb_robot_cleanup();
  // main ends
  return 0;
}


//---------------------------------------------------------------------
// Move robot forward by setting left and right wheels to same speeds
bool moveForward(double sForward){
    
    wb_differential_wheels_set_speed(sForward, sForward);
    
    return true;
}
//----------------------------------------------------------------------
// Stop robot by setting both wheel speeds to zero
bool stop(){
    wb_differential_wheels_set_speed(0, 0);
    return true;
}

//-----------------------------------------------------------------------
// Turn robot in place
bool turn90ccw(){
  stop();
  printf("\n\n Turning pi/2 radians ccw: \n\n");
  double goalTheta = fmodf(pose[2] + PI/2.00, PI2);
  
  while(pow(pose[2] - goalTheta, 2) > 0.0001){
    wb_differential_wheels_set_encoders(0, 0);
    wb_differential_wheels_set_speed(-50, 50);
    wb_robot_step(TIME_STEP);
    updatePose();
  }
  return true;  
}

//-----------------------------------------------------------------------
// Stop the robot and orient it towards goal 
void turnTowardGoal(){
  stop();
  printf("\n\n Turning toward goal \n\n");
  double goalTheta = getGoalTheta();
  
  while(pow(pose[2] - goalTheta, 2) > 0.0001){
    wb_differential_wheels_set_encoders(0, 0);
    wb_differential_wheels_set_speed(-100, 100);
    wb_robot_step(TIME_STEP);
    updatePose();
  }
}
//------------------------------------------------------------------------
// Move forward slowly for some length
bool clearObstacle(){
  printf("\n\n Clearing Obstacle! \n\n");
  updatePose();
  int i;
  for (i = 0; i < 400; i++){
    printf("\n\n Clearing Obstacle! \n\n");
    wb_differential_wheels_set_encoders(0, 0);
    wb_differential_wheels_set_speed(80, 50);
    wb_robot_step(TIME_STEP);
    updatePose();
  }
  
  return true;
}
//------------------------------------------------------------------------
// Return true if robot has arrived at goal
bool arrivedAtGoal(){
  updatePose();
  double distFromGoal = pow(pow(goal[0] - pose[0], 2) + pow(goal[1] - pose[1], 2), 0.5);
  if (distFromGoal < AXLE_LENGTH){
    printf("\n\n Arrived at Goal! \n\n");
    return true;
  }
  else{
    return false;
  }
}

//------------------------------------------------------------------------
// Return true if open space in front of the robot
bool openSpaceAhead(){
  double D0;
  double D7;
  D0 = readDsensor(0);
  D7 = readDsensor(7);
  
  if (D0 >= Dwth_wall && D7 >= Dwth_wall){
  return 0;
  }
  else{
  return true;
  }
  
  return false;
}

//------------------------------------------------------------------------
int getSensorFacingGoal(){
  updatePose();
  int senFacingGoal = 0;
  double minDevFromGoalAngle = 1000;
  
  // get orientation of goal
  double goalOrientation = getGoalTheta();
  
  // Compute orientations of different range sensors in global frame
  // Fixed: orientations of sensors in local (robot) frame
  double sensorOrientations[8] = {6.021, 5.498,  4.712, 3.665, 2.618, 1.571, 0.785, 0.262};
  
  // Add robot's orientation to get sensor orientations in global frame
  int i;for(i = 0; i < 8; i++){
    // Get global orientations of sensors
    sensorOrientations[i] = fmodf(sensorOrientations[i] + pose[3] + PI2, PI2);
    // Measure deviations of global sensor orientations from goal heading
    double currDevFromGoalAngle = pow(sensorOrientations[i] - goalOrientation, 2);
    
    // pick the sensor with orientation most similar to the orientation of goal
    if (currDevFromGoalAngle < minDevFromGoalAngle){
      minDevFromGoalAngle = currDevFromGoalAngle;
      senFacingGoal = i;
    }
    
  }
  
  return senFacingGoal;
  
}
//------------------------------------------------------------------------
bool openSpaceTowardGoal(){
  int sensFacingGoal = getSensorFacingGoal();
  
  if (readDsensor(sensFacingGoal) < 0.2*Dwth_wall){
  return true;
  }
  else{
    return false;
  }
}

//------------------------------------------------------------------------
// Return true if obstacle is present on right hand side of the robot
bool wallToRight(){
  if (readDsensor(2) > 0.10*Dwth_wall){
  return true;
  }
  else{
  return false;
  }
}

//------------------------------------------------------------------------
// Make robot follow an obstacle
bool followWall(double sForward, double eprev){
  
  // compute error term (to guage alignment along 'wall')
  double w_d2 = 0.6;
  double ecurr = (w_d2*readDsensor(2) + (1-w_d2)*readDsensor(1))- Dwth;
  
  // compute proportional, differential and integral errors
  double ep = ecurr;
  double ed = ecurr - eprev;
  double ei = ecurr + eprev;
  // define pid gains
  double Kp = 0.05;
  double Kd = 0.192;
  double Ki = 0.11;
  
  double sLeft = sForward;
  double sRight = sForward + (Kp*ep + Kd*ed + Ki*ei);
  
  printf("\n Current error = %f", ecurr);
  printf("\n speed_LeftW = %f, speed_RightW = %f", sLeft, sRight);

  wb_differential_wheels_set_speed(sLeft, sRight);

  
  return true;
}

//------------------------------------------------------------------------------
//Function to return displacements of left and right wheel given previous encoder positions
void getWheelDisplacements(double *dispLeftW, double *dispRightW, double del_enLeftW, double del_enRightW) {

  // compute displacement of left wheel in meters
  *dispLeftW = del_enLeftW / STEPS_ROT * 2 * PI * WHEEL_RADIUS; 
  // compute displacement of right wheel in meters
  *dispRightW = del_enRightW / STEPS_ROT * 2 * PI * WHEEL_RADIUS; 
}

//------------------------------------------------------------------------------
// Function to update pose vector and print it to the console
void updatePose(){
  // compute current encoder positions
  double del_enLeftW = wb_differential_wheels_get_left_encoder();
  double del_enRightW = wb_differential_wheels_get_right_encoder();
  
  // compute wheel displacements
  double dispLeftW; double dispRightW;
  getWheelDisplacements(&dispLeftW, &dispRightW, del_enLeftW, del_enRightW);

  // displacement of robot
  double dispRobot = (dispLeftW + dispRightW)/2.0; 
  
  // Update position vector:
  // Update in position along X direction
  pose[0] +=  dispRobot * cos(pose[2]); 
  // Update in position along Y direction
  pose[1] +=  dispRobot * sin(pose[2]); // robot position w.r.to Y direction
  // Update in orientation
  pose[2] += (dispRightW - dispLeftW)/AXLE_LENGTH; // orientation
  pose[2] = fmodf(pose[2], PI2) ;
  
  printf("current position of robot:\n");
  printf("estimated robot_x : %f m\n",pose[0]);
  printf("estimated robot_y : %f m\n",pose[1]);
  printf("estimated robot_theta : %f radians\n", pose[2]);

}
//------------------------------------------------------------------------------
double getGoalTheta(){
  double targetTheta = PI2 + atan2((goal[1]-pose[1]),(goal[0] - pose[0]));
  return fmodf(targetTheta, PI2);
}

//------------------------------------------------------------------------------
// Function for reading distance sensors: to be called by other functions
double readDsensor(int nsensor){
  // Declare some device tags
  // Distance sensor tags: 
  WbDeviceTag distsensor0;
  WbDeviceTag distsensor1;
  WbDeviceTag distsensor2;
  WbDeviceTag distsensor3;
  WbDeviceTag distsensor4;
  WbDeviceTag distsensor5;
  WbDeviceTag distsensor6;
  WbDeviceTag distsensor7;
  
  /* initialize Webots before calling any other webots functions:*/
  wb_robot_init();

  // Initialize the declared device tags:
  // Initialize distance sensors
  distsensor0 = wb_robot_get_device("ps0");
  distsensor1 = wb_robot_get_device("ps1");
  distsensor2 = wb_robot_get_device("ps2");
  distsensor3 = wb_robot_get_device("ps3");
  distsensor4 = wb_robot_get_device("ps4");
  distsensor5 = wb_robot_get_device("ps5");
  distsensor6 = wb_robot_get_device("ps6");
  distsensor7 = wb_robot_get_device("ps7");
  
  
  // Enable the distance sensors
  wb_distance_sensor_enable(distsensor0, TIME_STEP);
  wb_distance_sensor_enable(distsensor1, TIME_STEP);
  wb_distance_sensor_enable(distsensor2, TIME_STEP);
  wb_distance_sensor_enable(distsensor3, TIME_STEP);
  wb_distance_sensor_enable(distsensor4, TIME_STEP);
  wb_distance_sensor_enable(distsensor5, TIME_STEP);
  wb_distance_sensor_enable(distsensor6, TIME_STEP);
  wb_distance_sensor_enable(distsensor7, TIME_STEP);
  //------------------------------------------------------
  switch (nsensor){
    case 0:
      return wb_distance_sensor_get_value(distsensor0);
      break;
    case 1:
      return wb_distance_sensor_get_value(distsensor1);
      break;
    case 2:
      return wb_distance_sensor_get_value(distsensor2);
      break;
    case 3:
      return wb_distance_sensor_get_value(distsensor3);
      break;
    case 4:
      return wb_distance_sensor_get_value(distsensor4);
      break;
    case 5:
      return wb_distance_sensor_get_value(distsensor5);
      break;
    case 6:
      return wb_distance_sensor_get_value(distsensor6);
      break;
    case 7:
      return wb_distance_sensor_get_value(distsensor7);
      break;
    default:
      return 0;
  }
 
}

