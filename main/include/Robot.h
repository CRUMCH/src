
#pragma once

#include <string>
#include <iostream>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/WPILib.h>
#include <ctre/phoenix.h>

class Robot : public frc::TimedRobot 
{
 public:
  
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  
  frc::SendableChooser<std::string> m_chooser;
  
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  
  std::string m_autoSelected;

  WPI_TalonSRX Wheel1;
  WPI_TalonSRX Wheel2;
  WPI_TalonSRX Wheel3;
  WPI_TalonSRX Wheel4;

  frc::MecanumDrive Mecanums;

  frc::VictorSP Elevator;
  frc::VictorSP DriveArm;

  WPI_VictorSPX BallIntakeLeft;
  WPI_VictorSPX BallIntakeRight;

  frc::Solenoid ClawOpen;
  frc::Solenoid ClawClose;

  frc::Solenoid FrontOpen;
  frc::Solenoid FrontClose;

  frc::Solenoid BackOpen;
  frc::Solenoid BackClose;

  frc::Joystick Xbox;
  frc::Joystick Yoke;

  frc::AnalogInput PressureSensor;

  int BallControl;

  bool FrontExt;
  bool BackExt;

};
