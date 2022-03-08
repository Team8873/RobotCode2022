// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <AHRS.h>
#include <math.h>
#include <frc/AddressableLED.h>
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>


using namespace rev;
#define FEET_PER_COUNT ((6*M_PI/360)/12.0) //Math for encoder
#define RADIANS_PER_DEGREE (M_PI/180) //Math for the gyroscope

AHRS *ahrs; //Intiating gyroscope
float y_pos; //Forward and backward position of robot
float x_pos; //Left and right position of robot
float rightlastvalue; //Last value of the right encoder
float leftlastvalue; //Last value of the left encoder
float angleoffset; //Error of the robot angle
float pitchoffset;
float r_pos;
float x_target = 6;
float y_target = 6; 




VictorSPX m_intake(1); //iniating the intake
CANSparkMax m_elevator(5, CANSparkMaxLowLevel::MotorType::kBrushless); // iniating elevator

CANSparkMax m_rlead(1, CANSparkMaxLowLevel::MotorType::kBrushless); //intiating right lead drive motor
CANSparkMax m_rfollow(2, CANSparkMaxLowLevel::MotorType::kBrushless); //intiating right follow drive motor
CANSparkMax m_llead(3, CANSparkMaxLowLevel::MotorType::kBrushless); //intiating left lead drive motor
CANSparkMax m_lfollow(4, CANSparkMaxLowLevel::MotorType::kBrushless); //intiating left follow drive motor
frc::SpeedControllerGroup m_left(m_llead, m_lfollow); //binding the left side together
frc::SpeedControllerGroup m_right(m_rlead, m_rfollow); //binding the right side together
frc::Joystick m_drive1(0); //Joystick 1
frc::Joystick m_drive2(1); //Joystick 2
frc::Joystick m_drive3(2); //Joystick 3
frc::DigitalInput m_isensor(0);
SparkMaxRelativeEncoder m_rencoder = m_rfollow.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);
SparkMaxRelativeEncoder m_lencoder = m_lfollow.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);
SparkMaxRelativeEncoder m_eencoder = m_elevator.GetEncoder();
float rcurrent = m_rencoder.GetPosition(); //* FEET_PER_COUNT;
float lcurrent = m_lencoder.GetPosition() * FEET_PER_COUNT;
float ecurrent = m_eencoder.GetPosition() * FEET_PER_COUNT;
//float encoderrest = m_rencoder.SetPosition(0);

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  rightlastvalue = 0;
  
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);
  //rcurrent*0;

 
 
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
    
 } else {
    // Default Auto goes here
  }
}


void Robot::AutonomousPeriodic() {
  
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
m_rlead.RestoreFactoryDefaults();
}
{
     //float rcurrent = m_rencoder.GetPosition() * FEET_PER_COUNT;
   
    // Default Auto goes here
   
    /*
     if (rcurrent < .25)
    {
      m_left.Set(-.2);
      m_right.Set(.2);
     
    } else {
      m_left.Set(0);
      m_right.Set(0);
      }
      */
    frc::SmartDashboard::PutNumber("right encoder",rcurrent);
    frc::SmartDashboard::PutNumber("left encoder",lcurrent);
  
   if (ecurrent < 1)
   m_elevator.Set(.5);
   else if ( m_isensor.Get())
{
    m_elevator.Set(.5);
    m_left.Set(.2);
    m_right.Set(-.2);
}
  else if ( rcurrent < .25)
    {
      m_left.Set(-.2);
      m_right.Set(.2);
      m_intake.Set(VictorSPXControlMode::PercentOutput, 0.5);
     
    } else {
      m_left.Set(0);
      m_right.Set(0);
      } 
  } //else {
    m_rlead.RestoreFactoryDefaults();
    
  
 } 
  


void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

float x_axis = m_drive1.GetRawAxis(2) / 5;
float y_axis = m_drive1.GetRawAxis(1) / 5;
m_left.Set(x_axis + y_axis); 
m_right.Set(x_axis - y_axis); 
/*float l_side = m_drive1.GetRawAxis(1) / 5;
float r_side = m_drive2.GetRawAxis(1) / 5;
m_left.Set(l_side);
m_right.Set(-r_side);
*/
float intake = m_drive3.GetRawAxis(1);
float elevator = m_drive3.GetRawAxis(2);

m_intake.Set(VictorSPXControlMode::PercentOutput, intake);
m_elevator.Set(elevator);
/*
 if(m_drive3.GetRawButton(1)) {
  m_intake.Set(.5);
} else {
  m_intake.Set( 0);
} 
if(m_drive3.GetRawButton(2)) {
  m_elevator.Set(.5);
} else {
  m_elevator.Set(0);
  
} 
*/




}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
