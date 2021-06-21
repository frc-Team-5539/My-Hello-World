// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>
#include <frc/GenericHID.h>
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>
#include <frc/Spark.h>
#include <frc/Encoder.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>



class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom1 = "My Auto";
  const std::string kAutoNameCustom2 = "My Bike";
  const std::string kAutoNameCustom3 = "My Truck";
  const std::string kAutoNameCustom4 = "My Prius";
  const std::string kAutoNameCustom5 = "My Feet";
  std::string m_autoSelected;

  int it = 0;
  int ia = 0;

  frc::XboxController m_driverController{0};

  //Digital and Analog Sensors
  frc::DigitalInput   limit_switch{9};
  frc::AnalogInput    ai{0};
  frc::AnalogInput    ultra{1};
  frc::Encoder        left_encoder{2,3};
  frc::Encoder        right_encoder{1,0};
  
  // PMW
  frc::Spark          left_motor1{0};
  frc::Spark          left_motor2{1};
  frc::Spark          right_motor1{2};
  frc::Spark          right_motor2{3};

  frc::Spark          intake_motor{4};
  frc::Spark          conveyor_motor{5};

  frc::Spark          shooter_motor1{6};
  frc::Spark          shooter_motor2{7};

  
  frc::SpeedControllerGroup m_left{left_motor1, left_motor2};
  frc::SpeedControllerGroup m_right{right_motor1, right_motor2};
  frc::DifferentialDrive m_robotDrive {m_left, m_right};




  bool leftbump = false;
  bool rightbump = false;
  bool lefttrigger = false;
  bool righttrigger = false;
  bool bA = false;
  bool bB = false;
  bool bX = false;
  bool bY = false;

  double left_x = 0.0;
  double left_y = 0.0;
  double right_x = 0.0;
  double right_y = 0.0;

  bool limit_switch_value = false;

  int ai_raw  = 0;
  double ai_voltage = 0.0;

  int ultra_raw = 0;
  double distance = 0.0;
  double adc_to_mm = 1.25 / 1000.0;

  int encoder_count = 0;
  double encoder_distance = 0.0;

  



};
