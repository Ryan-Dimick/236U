#include "UserCode.hpp"
#include "UtilityFunctions.hpp"
#include "Vec3f.hpp"

#include <stdio.h> //forprintf

//An example of a variable that persists beyond the function call.
float exampleVariable_float = 0.0f;  //Note the trailing 'f' in the number. This is to force single precision floating point.
Vec3f exampleVariable_Vec3f = Vec3f(0, 0, 0);
int exampleVariable_int = 0;

//We keep the last inputs and outputs around for debugging:
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

//Some constants that we may use:
const float mass = 32e-3f;  // mass of the quadcopter [kg]
const float gravity = 9.81f;  // acceleration of gravity [m/s^2]
const float inertia_xx = 16e-6f;  //MMOI about x axis [kg.m^2]
const float inertia_yy = inertia_xx;  //MMOI about y axis [kg.m^2]
const float inertia_zz = 29e-6f;  //MMOI about z axis [kg.m^2]

const float l = 33e-3f; //Propeller Distance [m]
const float k = 0.01f; //Force to torque coefficient[m]
const float dt = 1.0f / 500.0f;  //[s] period between successive calls to MainLoop


//Initializing Estimated Gyroscope Bias
Vec3f estGyroBias = Vec3f(0, 0, 0);

// Initializing Attitude Estimations
float estRoll = 0;
float estPitch = 0;
float estYaw = 0;

//Initializing Height and Translational Velocity estimations
float estHeight = 0;
float lastHeightMeas_meas = 0;
float lastHeightMeas_time = 0;
float estVelocity_1 = 0;
float estVelocity_2 = 0;
float estVelocity_3 = 0;
float estPosition_1 = 0;
float estPosition_2 = 0;

//Define Tuning Constant for estimation
const float rho = 0.001f; //Tuning constant for attitude estimation
float const mixHorizVel = 0.20f; //Tuning constant for velocity estimation
float const mixHeight = 0.30f; //Tuning constant for height estimation

//Initialize Desired Angles
Vec3f angle_cmd= Vec3f(0,0,0);

//Initialize Desired Position
float des_Position_1 = 0;
float des_Position_2 = 0;
float des_Height = 0;

//Initialize Desired Accelerations
float desAcc1 = 0; //Desired Acceleration in Earth-fixed 1 direction
float desAcc2 = 0; //Desired Acceleration in Earth-fixed 2 direction
float desAcc3 = 0;

//Defining Angular Velocity Time Constants
float const timeConstant_rollRate = 0.04f; //[s]
float const timeConstant_pitchRate = timeConstant_rollRate; //[s]
float const timeConstant_yawRate = 0.1f; //[s]

//Defining Angle Controller Time Constants
float const  timeConstant_rollAngle = 0.10f; //[s]
float const  timeConstant_pitchAngle = timeConstant_rollAngle; //[s]
float const  timeConstant_yawAngle = 0.2f; //[s]

//Defining Horizontal Velocity and Position Time Constants
const float timeConst_horizVel = 0.50f;
const float timeConst_horizPos = 2.50f;

//Defining Vertical Position Natural Frequency and damping ratio
const float natFreq_height = 2.0f;
const float dampingRatio_height = 0.7f;


MainLoopOutput MainLoop(MainLoopInput const &in) {
  //Your code goes here!
  // The function input (named "in") is a struct of type
  // "MainLoopInput". You can understand what values it
  // contains by going to its definition (click on "MainLoopInput",
  // and then hit <F3> -- this should take you to the definition).
  // For example, "in.userInput.buttonBlue" is true if the
  // blue button is pushed, false otherwise.

  //Define the output numbers (in the struct outVals):
  MainLoopOutput outVals;
//  motorCommand1 -> located at body +x +y
//  motorCommand2 -> located at body +x -y
//  motorCommand3 -> located at body -x -y
//  motorCommand4 -> located at body -x +y

//Reducing Bias in rate Gyroscope outputs
  if (in.currentTime < 1.0f) {
    estGyroBias = estGyroBias + (in.imuMeasurement.rateGyro/ 500.0f);
  }
  Vec3f rateGyro_corr = in.imuMeasurement.rateGyro - estGyroBias;

///////////////////////Attitude Estimation////////////////////////////
  //Defining the measured AttitudeAccelerometer Inputs
  float measRoll = in.imuMeasurement.accelerometer.y/gravity;
  float measPitch = -in.imuMeasurement.accelerometer.x/gravity;

  //Implementing attitude estimators
  estRoll = (1-rho)*(estRoll + dt*rateGyro_corr.x) + rho*measRoll;
  estPitch = (1-rho)*(estPitch + dt*rateGyro_corr.y) + rho*measPitch;
  estYaw = estYaw + dt*rateGyro_corr.z;

  ///////////////////////Height Estimation////////////////////////////
  //Prediction Step
  estHeight = estHeight + estVelocity_3*dt;
  estVelocity_3 = estVelocity_3 + 0*dt;

  //Correction step:
  if (in.heightSensor.updated) {
    //check that the measurement is reasonable
    if (in.heightSensor.value < 5.0f){
      float hMeas = in.heightSensor.value * cosf(estRoll) * cosf(estPitch);
      estHeight = (1-mixHeight) * estHeight + mixHeight*hMeas;

      float v3Meas = (hMeas - lastHeightMeas_meas) / (in.currentTime - lastHeightMeas_time);
      estVelocity_3 = (1 - mixHeight) * estVelocity_3 + mixHeight * v3Meas;
      //Store this variable for the next velocity update
      lastHeightMeas_meas = hMeas;
      lastHeightMeas_time = in.currentTime;
    }
  }

  ///////////////////////Horizontal State Estimation////////////////////////////
  //Predictor
  float estAcc1 = in.imuMeasurement.accelerometer.x*cosf(estYaw) - in.imuMeasurement.accelerometer.y*sinf(estYaw);
  float estAcc2 = in.imuMeasurement.accelerometer.x*sinf(estYaw) + in.imuMeasurement.accelerometer.y*cosf(estYaw);

  estVelocity_1 = estVelocity_1 + estAcc1*dt; //Estimate Horizontal Velocity based on previous estimates
  estVelocity_2 = estVelocity_2 + estAcc2*dt;

  estPosition_1 = estPosition_1 + estVelocity_1*dt; //Estimate Horizontal Positions based on previous estimates
  estPosition_2 = estPosition_2 + estVelocity_2*dt;



  //Correction Using Optical Flow Sensor to measure Horizontal Velocity
  if (in.opticalFlowSensor.updated){
    float sigma_1 = in.opticalFlowSensor.value_x;
    float sigma_2 = in.opticalFlowSensor.value_y;
    float div = (cosf(estRoll) * cosf(estPitch));
    if (div > 0.5f) {
      float deltaPredict = estHeight/div;

      float v1Meas = (-sigma_1 + rateGyro_corr.y) * deltaPredict; //Using unbiased rate gyro value to measure horizontal velocity
      float v2Meas = (-sigma_2 - rateGyro_corr.x) * deltaPredict;

      estVelocity_1 = (1 - mixHorizVel) * estVelocity_1 + mixHorizVel * v1Meas; //Define estimated horizontal velocity velocity
      estVelocity_2 = (1 - mixHorizVel) * estVelocity_2 + mixHorizVel * v2Meas;
    }
  }


  ///////////////////////Controller////////////////////////////
  if (in.userInput.buttonRed == true){ //When the GUI red button is pressed, this function will enable
    des_Height = 1.60f;
    des_Position_1 = 0.0f;
    des_Position_2 = 0.0f;
    desAcc1 = -(1/timeConst_horizVel)*estVelocity_1; //Set desired acceleration to cause drone to hover
    desAcc2 = -(1/timeConst_horizVel)*estVelocity_2;
    desAcc3 = -2*dampingRatio_height * natFreq_height * estVelocity_3 - natFreq_height * natFreq_height*(estHeight - des_Height);


    if (in.currentTime > 3.5f){
      des_Position_1 = 1.00f;
      des_Position_2 = 1.00f;//Set Horizontal Displacement once drone is at desired height

      float desVel1 = -(1/timeConst_horizPos)*(estPosition_1 - des_Position_1); //Define desired horizontal velocities
      float desVel2 = -(1/timeConst_horizPos)*(estPosition_2 - des_Position_2);
      desAcc1 = -(1/timeConst_horizVel)*(estVelocity_1 - desVel1); //Set Desired translational acceleration
      desAcc2 = -(1/timeConst_horizVel)*(estVelocity_2 - desVel2);
      desAcc3 = -2*dampingRatio_height * natFreq_height * estVelocity_3 - natFreq_height * natFreq_height*(estHeight - des_Height);

      if (in.currentTime > 7.0f){
        des_Height = -0.15f;
        desAcc1 = -(1/timeConst_horizVel)*estVelocity_1;
        desAcc2 = -(1/timeConst_horizVel)*estVelocity_2;
        const float natFreq_height = 1.00f;
        const float dampingRatio_height = 0.90f;
        desAcc3 = -2*dampingRatio_height * natFreq_height * estVelocity_3 - natFreq_height * natFreq_height*(estHeight - des_Height);
      }
    }
  }



  float desRoll = -desAcc2/gravity; //Use desired acceleration to compute desired attitude
  float desPitch = desAcc1/gravity;
  float desYaw = 0;
  angle_cmd = Vec3f(desRoll, desPitch, desYaw);


  //Use desired vertical acceleration and estimated attitude to determine required normalized acceleration
  float desNormalizedAcceleration = (gravity + desAcc3)/(cosf(estRoll)*cosf(estPitch));


  //Using attitude estimate to create angle commands
  float roll_cmd = -(1/timeConstant_rollAngle)*(estRoll - angle_cmd.x);
  float pitch_cmd = -(1/timeConstant_pitchAngle)*(estPitch - angle_cmd.y);
  float yaw_cmd = -(1/timeConstant_yawAngle)*(estYaw - angle_cmd.z);

  Vec3f AngVel_cmd = Vec3f(roll_cmd, pitch_cmd, yaw_cmd); //Set desired angular velocity

  //Use Rate gyro outputs to create angular velocity commands
  float rollRate_cmd = -(1/timeConstant_rollRate)*(rateGyro_corr.x - AngVel_cmd.x);
  float pitchRate_cmd = -(1/timeConstant_pitchRate)*(rateGyro_corr.y - AngVel_cmd.y);
  float yawRate_cmd = -(1/timeConstant_yawRate)*(rateGyro_corr.z - AngVel_cmd.z);

  Vec3f cmdAngAcc = Vec3f(rollRate_cmd,pitchRate_cmd,yawRate_cmd); //Set desired angular acceleration

  //Use desired acceleration to get desired torque and thrust
  Vec3f des_torque = Vec3f(cmdAngAcc.x*inertia_xx, cmdAngAcc.y*inertia_xx, cmdAngAcc.z*inertia_zz);
  float des_force = mass*desNormalizedAcceleration;

  float Cp1 = 0.25*(des_force + des_torque.x/l - des_torque.y/l + des_torque.z/k); //Use Mixer Matrix to convert
  float Cp2 = 0.25*(des_force - des_torque.x/l - des_torque.y/l - des_torque.z/k); //from desired thrust and torques
  float Cp3 = 0.25*(des_force - des_torque.x/l + des_torque.y/l + des_torque.z/k); //into individual propeller
  float Cp4 = 0.25*(des_force + des_torque.x/l + des_torque.y/l - des_torque.z/k); //forces.

  float speed1 = speedFromForce(Cp1); //Calling speed from force mapping
  float speed2 = speedFromForce(Cp2);
  float speed3 = speedFromForce(Cp3);
  float speed4 = speedFromForce(Cp4);

  int PWM1 = pwmCommandFromSpeed(speed1); //Calling PWM from speed mapping
  int PWM2 = pwmCommandFromSpeed(speed2);
  int PWM3 = pwmCommandFromSpeed(speed3);
  int PWM4 = pwmCommandFromSpeed(speed4);

  outVals.motorCommand1 = PWM1; // Set motor cmd to PWM value calculated from desired force
  outVals.motorCommand2 = PWM2;
  outVals.motorCommand3 = PWM3;
  outVals.motorCommand4 = PWM4;





  outVals.telemetryOutputs_plusMinus100[0] = estRoll;
  outVals.telemetryOutputs_plusMinus100[1] = estPitch;
  outVals.telemetryOutputs_plusMinus100[2] = estYaw;
  outVals.telemetryOutputs_plusMinus100[3] = estVelocity_1;
  outVals.telemetryOutputs_plusMinus100[4] = estVelocity_2;
  outVals.telemetryOutputs_plusMinus100[5] = estVelocity_3;
  outVals.telemetryOutputs_plusMinus100[6] = des_Position_1;
  outVals.telemetryOutputs_plusMinus100[7] = des_Position_2;
  outVals.telemetryOutputs_plusMinus100[8] = estPosition_1;
  outVals.telemetryOutputs_plusMinus100[9] = estPosition_2;
  outVals.telemetryOutputs_plusMinus100[10] = estHeight;



  //copy the inputs and outputs:
  lastMainLoopInputs = in;
  lastMainLoopOutputs = outVals;
  return outVals;
}

void PrintStatus() {
  //For a quick reference on the printf function, see: http://www.cplusplus.com/reference/cstdio/printf/
  // Note that \n is a "new line" character.
  // Also, note that to print a `float` variable, you have to explicitly cast it to
  //  `double` in the printf function, and explicitly specify precision using something
  //  like %6.3f (six significant digits, three after the period). Example:
  //   printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //Accelerometer measurement
  printf("Acc: ");
  printf("x=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.x));
  printf("y=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.y));
  printf("z=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.z));
  //Rate gyro measurement
  printf("\n");
  printf("\n");
  printf("Gyro:");
  printf("x=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.x));
  printf("y=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.y));
  printf("z=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.z));
  printf("\n");
  printf("estGyroBias=(x =%6.3f, y=%6.3f, z=%6.3f)\n" , double(estGyroBias.x), double(estGyroBias.y),double(estGyroBias.z));
  printf("Commanded Angular Acceleration=(x =%6.3f, y=%6.3f, z=%6.3f)\n" ,
         double(lastMainLoopOutputs.telemetryOutputs_plusMinus100[3]),
         double(lastMainLoopOutputs.telemetryOutputs_plusMinus100[4]),
         double(lastMainLoopOutputs.telemetryOutputs_plusMinus100[5]));
  printf("\n");

  printf("EstRoll=%6.4f, ",double(estRoll));
  printf("EstPitch=%6.4f, ",double(estPitch));
  printf("EstYaw=%6.4f, ",double(estYaw));
  printf("\n");
  printf("\n");

  printf("Last range = %6.3fm, ",
         double(lastMainLoopInputs.heightSensor.value));
  printf("Last flow: x=%6.3f, y=%6.3f\n",
         double(lastMainLoopInputs.opticalFlowSensor.value_x),
         double(lastMainLoopInputs.opticalFlowSensor.value_y));


  //printf("Example variable values:\n");
  //printf("  exampleVariable_int = %d\n", exampleVariable_int);
  //Note that it is somewhat annoying to print float variables.
  //  We need to cast the variable as double, and we need to specify
  //  the number of digits we want (if you used simply "%f", it would
  //  truncate to an integer.
  //  Here, we print 6 digits, with three digits after the period.
  //printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //We print the Vec3f by printing it's three components independently:
  //printf("  exampleVariable_Vec3f = (%6.3f, %6.3f, %6.3f)\n",
         //double(exampleVariable_Vec3f.x), double(exampleVariable_Vec3f.y),
         //double(exampleVariable_Vec3f.z));

  //just an example of how we would inspect the last main loop inputs and outputs:
  //printf("Last main loop inputs:\n");
  //printf("  batt voltage = %6.3f\n",
         //double(lastMainLoopInputs.batteryVoltage.value));
  //printf("  JS buttons: ");
  //if (lastMainLoopInputs.userInput.buttonRed)
    //printf("buttonRed ");
  //if (lastMainLoopInputs.userInput.buttonGreen)
    //printf("buttonGreen ");
  //if (lastMainLoopInputs.userInput.buttonBlue)
    //printf("buttonBlue ");
  //if (lastMainLoopInputs.userInput.buttonYellow)
    //printf("buttonYellow ");
  //if (lastMainLoopInputs.userInput.buttonArm)
    //printf("buttonArm ");
  //printf("\n");
  //printf("Last main loop outputs:\n");
  printf("  motor commands: = %6.3f\t%6.3f\t%6.3f\t%6.3f\t\n",
         double(lastMainLoopOutputs.motorCommand1),
         double(lastMainLoopOutputs.motorCommand2),
         double(lastMainLoopOutputs.motorCommand3),
         double(lastMainLoopOutputs.motorCommand4));
}
