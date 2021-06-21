// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

//////////////////////////////////////////////////////////////////
void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom1, kAutoNameCustom1);
  m_chooser.AddOption(kAutoNameCustom2, kAutoNameCustom2);
  m_chooser.AddOption(kAutoNameCustom3, kAutoNameCustom3);
  m_chooser.AddOption(kAutoNameCustom4, kAutoNameCustom4);
  m_chooser.AddOption(kAutoNameCustom4, kAutoNameCustom4);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  left_encoder.SetDistancePerPulse( (3.14159 *  3.75) / 256);
  right_encoder.SetDistancePerPulse( (3.14159 *  3.75) / 256);

  
  conveyor_motor.SetInverted(true);
  shooter_motor1.SetInverted(true);
  shooter_motor2.SetInverted(true);
  
}

//////////////////////////////////////////////////////////////////
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("ia: ", ia);
  frc::SmartDashboard::PutNumber("it: ", it);

  frc::SmartDashboard::PutBoolean("LB : ", leftbump);
  frc::SmartDashboard::PutBoolean("RB : ", rightbump);

  frc::SmartDashboard::PutBoolean("LT : ", lefttrigger);
  frc::SmartDashboard::PutBoolean("RT : ", righttrigger);

  frc::SmartDashboard::PutBoolean("bA : ", bA);
  frc::SmartDashboard::PutBoolean("bB : ", bB);
  frc::SmartDashboard::PutBoolean("bX : ", bX);
  frc::SmartDashboard::PutBoolean("bY : ", bY);

  frc::SmartDashboard::PutNumber("left x : ", left_x);
  frc::SmartDashboard::PutNumber("left y : ", left_y);
  frc::SmartDashboard::PutNumber("right x : ", right_x);
  frc::SmartDashboard::PutNumber("right y : ", right_y);

  frc::SmartDashboard::PutBoolean("limit_switch : ", limit_switch_value);

  frc::SmartDashboard::PutNumber("ai raw :", ai_raw);
  frc::SmartDashboard::PutNumber("ai voltage :", ai_voltage);
  frc::SmartDashboard::PutNumber("ultra distance :", distance);
  frc::SmartDashboard::PutNumber("ultra raw :", ultra_raw);

  frc::SmartDashboard::PutNumber("left enc count :", encoder_count);
  frc::SmartDashboard::PutNumber("left enc dist :", encoder_distance);
  frc::SmartDashboard::PutNumber("right enc count :", encoder_count);
  frc::SmartDashboard::PutNumber("right enc dist :", encoder_distance);

}

//////////////////////////////////////////////////////////////////
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom1) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
  ia = 0;

  left_encoder.Reset();
  right_encoder.Reset();

}

//////////////////////////////////////////////////////////////////
void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom1) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

  frc::SmartDashboard::PutNumber("ia: ", ia);
  ia++;

  encoder_count = left_encoder.Get();
  encoder_distance = left_encoder.GetDistance();
  encoder_count = right_encoder.Get();
  encoder_distance = right_encoder.GetDistance();

}

//////////////////////////////////////////////////////////////////
void Robot::TeleopInit() {
  it = 0;


}

//////////////////////////////////////////////////////////////////
void Robot::TeleopPeriodic() {
  frc::SmartDashboard::PutNumber("it: ", it);
  it++;

  m_robotDrive.TankDrive(
    left_y = m_driverController.GetY(frc::GenericHID::JoystickHand::kLeftHand),
    right_y = m_driverController.GetY(frc::GenericHID::JoystickHand::kRightHand), true);

 
  
  left_x = m_driverController.GetX(frc::GenericHID::JoystickHand::kLeftHand);
  right_x = m_driverController.GetX(frc::GenericHID::JoystickHand::kRightHand);

  leftbump = m_driverController.GetBumper(frc::GenericHID::kLeftHand);
  rightbump = m_driverController.GetBumper(frc::GenericHID::kRightHand);

  lefttrigger = m_driverController.GetTriggerAxis(frc::GenericHID::kLeftHand);
  righttrigger = m_driverController.GetTriggerAxis(frc::GenericHID::kRightHand);

  bA = m_driverController.GetAButton();
  bB = m_driverController.GetBButton();
  bX = m_driverController.GetXButton();
  bY = m_driverController.GetYButton();



  limit_switch_value = limit_switch.Get();

  ai_raw = ai.GetValue();
  ai_voltage = ai.GetVoltage();

  ultra_raw = ultra.GetValue();
  distance = (double) ultra_raw * adc_to_mm ;

// 4 motor drivetrain
 
  m_robotDrive.SetMaxOutput(-0.5); 
  m_robotDrive.SetDeadband(0.02);
  m_robotDrive.TankDrive(left_y, right_y, true);
  

// intake
  intake_motor.Set(bX);
  

  
  conveyor_motor.Set(bY);
  

  shooter_motor1.Set(lefttrigger);
  shooter_motor2.Set(righttrigger);


 



}

//////////////////////////////////////////////////////////////////
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
