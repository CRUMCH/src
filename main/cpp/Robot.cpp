
#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix.h>
#include <frc/WPILib.h>

WPI_TalonSRX Wheel1 {1};
WPI_TalonSRX Wheel2 {2};
WPI_TalonSRX Wheel3 {3};
WPI_TalonSRX Wheel4 {4};

frc::MecanumDrive Mecanums {Wheel4 , Wheel3 , Wheel2 , Wheel1};

frc::VictorSP Elevator {0};
frc::VictorSP DriveArm {1};

WPI_VictorSPX BallIntakeLeft {1};
WPI_VictorSPX BallIntakeRight {2};

frc::Solenoid ClawOpen {4};
frc::Solenoid ClawClose {5};

frc::Solenoid FrontOpen {0};
frc::Solenoid FrontClose {1};

frc::Solenoid BackOpen {2};
frc::Solenoid BackClose {3};

frc::Joystick Xbox {0};
frc::Joystick Yoke {1};

frc::AnalogInput PressureSensor {0};

int BallControl = 0;

bool FrontExt = false;
bool BackExt = false;

void Robot::RobotInit() 
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  
  ClawOpen.SetPulseDuration(0.1);
  ClawClose.SetPulseDuration(0.1);
  
  FrontOpen.SetPulseDuration(0.1);
  FrontClose.SetPulseDuration(0.1);
  
  BackOpen.SetPulseDuration(0.1);
  BackClose.SetPulseDuration(0.1);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() 
{
  double Pressure = PressureSensor.GetValue();
  Pressure = (Pressure - 400.0);
  Pressure = (Pressure / 16.0);
  std::string pressureOutput = "" + std::to_string(Pressure) + " PSI";
  frc::SmartDashboard::PutString("Pressure" , pressureOutput);
}

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
void Robot::AutonomousInit() 
{
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) 
  {
    // Custom Auto goes here
  } 
  else 
  {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() 
{
  if (m_autoSelected == kAutoNameCustom) 
  {
    // Custom Auto goes here
  } 
  else 
  {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() 
{

}

void Robot::TeleopPeriodic() 
{
  /*########################################################################################################################
      
                Drivetrain Program

  #########################################################################################################################*/
  double xboxLX = Xbox.GetRawAxis(0);
  double xboxRX = Xbox.GetRawAxis(4);
  double xboxRY = Xbox.GetRawAxis(5);
  double speedwheel = (Yoke.GetRawAxis(2) * -1);
  
  if ((xboxRY > -0.1) && (xboxRY < 0.1))
  {
    xboxRY = 0;
  }

  if ((xboxRX > -0.1) && (xboxRX < 0.1))
  {
    xboxRX = 0;
  }

  if ((xboxLX > -0.1) && (xboxLX < 0.1))
  {
    xboxLX = 0;
  }

  speedwheel = ((speedwheel + (5/3)) * 0.375);
  
  xboxRX = xboxRX * speedwheel;
  xboxRY = xboxRY * speedwheel;
  xboxLX = xboxLX * speedwheel;
  
  Mecanums.DriveCartesian(xboxRX , xboxRY , xboxLX); 

  /*########################################################################################################################
      
                 Pnuematic Drive Arm Program

  #########################################################################################################################*/
 
  double xboxRT = Xbox.GetRawAxis(3);
  double xboxLT = Xbox.GetRawAxis(2);
  double xboxDriveArm = xboxRT + xboxLT;
  
  DriveArm.Set(xboxDriveArm);

  /*########################################################################################################################
      
                Ball Intake/Outtake Program

  #########################################################################################################################*/

  if (Xbox.GetRawButtonPressed(5))
  {
    if (BallControl == -1)
    {
      BallControl = 0;
    }
    else
    {
      BallControl = -1;
    }
    
  }

  if (Xbox.GetRawButtonPressed(6))
  {
    if (BallControl == 1)
    {
      BallControl = 0;
    }
    else
    {
      BallControl = 1;
    }
    
  }

  if (BallControl == 1)
  {
    BallIntakeLeft.Set(1);
    BallIntakeRight.Set(-1);
  }
  else if (BallControl == -1)
  {
    BallIntakeLeft.Set(-1);
    BallIntakeRight.Set(1);
  }
  else
  {
    BallIntakeLeft.Set(0);
    BallIntakeRight.Set(0);
  }

  /*########################################################################################################################
      
                Pneumatic Lifting System Program

  #########################################################################################################################*/

  long xboxDPad = Xbox.GetPOV();

  if (xboxDPad == 0)
  {
    FrontOpen.StartPulse();
    FrontExt = true;

    BackOpen.StartPulse();
    BackExt = true;
  }
  
  if (xboxDPad == 90)
  {
      FrontClose.StartPulse();
      FrontExt = false;
  }
  
  if (xboxDPad == 180)
  {
    FrontClose.StartPulse();
    FrontExt = false;

    BackClose.StartPulse();
    BackExt = false;
  }

  if (xboxDPad == 270)
  {
    BackClose.StartPulse();
    BackExt = false;
  }

  if (Xbox.GetRawButtonPressed(7))
  {
    if (!BackExt)
    {
      BackOpen.StartPulse();
      BackExt = true;
    }
    else if (BackExt)
    {
      BackClose.StartPulse();
      BackExt = false;
    }
  }

  if (Xbox.GetRawButtonPressed(8))
  {
    if (!FrontExt)
    {
      FrontOpen.StartPulse();
      FrontExt = true;
    }
    else if (FrontExt)
    {
      FrontClose.StartPulse();
      FrontExt = false;
    }
  }

  /*########################################################################################################################
      
                Pneumatic Claw Program

  #########################################################################################################################*/

  if (Xbox.GetRawButtonPressed(1))
  {
    ClawClose.StartPulse();
  }

  if (Xbox.GetRawButtonPressed(2))
  {
    ClawOpen.StartPulse();
  }

  /*########################################################################################################################
      
                Arm Elevator Program

  #########################################################################################################################*/

  double yokeY = (Yoke.GetRawAxis(1) * -1);

  if ((yokeY > -0.05) && (yokeY < 0.05))
  {
    yokeY = 0;
  }

  Elevator.Set(yokeY);
}

void Robot::TestPeriodic() 
{

}

#ifndef RUNNING_FRC_TESTS
int main() 
{ 
  return frc::StartRobot<Robot>(); 
}

#endif
