#pragma once
#include <frc/WPIlib.h> 
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <iostream>
#include <string>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/Smartdashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <sstream>
#include <frc/VictorSP.h>
#include <frc/XboxController.h>
#include <AHRS.h>
#include <frc/controller/PIDController.h>
#include <frc/MedianFilter.h>
#include <frc/Timer.h>

class Robot : public frc::TimedRobot {
 public:

WPI_TalonSRX Shooter = WPI_TalonSRX(5); 
//WPI_TalonSRX ShooterFollow = WPI_TalonSRX(8); 
WPI_VictorSPX ShooterFollow{6};

WPI_VictorSPX IntakeWheels{10};

WPI_VictorSPX CPMWheel{13};

WPI_VictorSPX IndexerT{8};
WPI_VictorSPX IndexerB{9};

WPI_VictorSPX ClimberLift{7};

WPI_VictorSPX Climber {11};

WPI_TalonFX RightLead{3};
WPI_TalonFX RightFollow{4};

WPI_TalonFX LeftLead{1};
WPI_TalonFX LeftFollow{2};

frc::DifferentialDrive myRobot{RightLead, LeftLead};

frc::DoubleSolenoid ShooterStop {0,1};
frc::DoubleSolenoid IntakeArm {2,3};
frc::DoubleSolenoid CPM {4,5};

frc::DigitalInput IntakeLaser1 {0};  // eye back by shooter to prevent james *jams
frc::DigitalInput IntakeLaser2 {1};  // eye near front of indexer to prevent jams going that way
frc::DigitalInput IntakeLaser3 {2};  // eye at intake, to trigger indexer and slurp ball in

bool LaserState1 = 0;
bool LaserState2 = 0;
bool LaserState3 = 0;

frc::AnalogInput Ultrasonic{0};

// thinks calculated each tick
double Distance = 0.0;    // the ranger distance
float tx = 0.0;           // limelight angle off left/right
int tv = 0;               // does the limelight have a target?
double gyroAngle = 0.0;   // What is the angle from the gyro?
double RPM = 0.0;

// auto logic stuff
bool FIRST = true;
bool FIRST2 = true;
bool TestFIRST = true;

frc::Timer TikTok;
  // pointer to limelight stuff
std::shared_ptr<NetworkTable> table;

AHRS ahrs{frc::SPI::Port::kMXP};

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void TestInit()override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  frc::XboxController Xbox{0};

//PID for turning
  double kP = 0.07; //tune This to start ocolating.  0.2 made 0.667s periods .12
  double kI = 0; //Then tune this to stop the osolating .36
  double kD = 0.005; //Finaly tune this to fix final error  .01

  frc2::PIDController TurnToAngle{kP, kI, kD};

//PID for ultrasonic 
  //double kP2 = 0.01; //tune This to start ocolating
 // double kI2 = 0.005; //Then tune this to stop the osolating
  //double kD2 = 0.0; //Finaly tune this to fix final error  

  //frc::MedianFilter<double> distancefilter {5};
  
  //frc2::PIDController DistancePID{kP2, kI2, kD2};
};
