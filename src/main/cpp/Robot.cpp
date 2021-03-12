#include "Robot.h"

#include <iostream> 

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() { 
  m_chooser.SetDefaultOption(kAutoNameCustom, kAutoNameCustom);
  m_chooser.AddOption(kAutoNameDefault, kAutoNameDefault);//kAutoNameCustom, kAutoNameCustom
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  //frc::SmartDashboard::PutData("TurnToAngle", TurnToAngle);
  //frc::SmartDashboard::PutData("DistancePID", DistancePID);

  Distance = Ultrasonic.GetVoltage()*1000.0*(1.0/0.977)*(1.0/25.4);
  frc::SmartDashboard::PutNumber("Distance", Distance);
  LaserState1 = IntakeLaser1.Get();
  frc::SmartDashboard::PutBoolean("Laser1", LaserState1);
  LaserState2 = IntakeLaser2.Get();
  frc::SmartDashboard::PutBoolean("Laser2", LaserState2);
  LaserState3 = IntakeLaser2.Get();
  frc::SmartDashboard::PutBoolean("Laser3", LaserState3);

  RightLead.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
  LeftLead.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
  
  //gyroAngle = ahrs.GetYaw();
  //frc::SmartDashboard::PutNumber("gyroAngle", gyroAngle);

//PID stuff for shooter
  Shooter.ConfigFactoryDefault();

  Shooter.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10); 

  Shooter.SetSensorPhase(true);

  Shooter.ConfigNominalOutputForward(0,10);
  Shooter.ConfigNominalOutputReverse(0,10);
  Shooter.ConfigPeakOutputForward(1,10);
  Shooter.ConfigPeakOutputReverse(-1,10);
  
  Shooter.Config_kF(0, 0.1097, 10);
  Shooter.Config_kP(0, 0.22, 10);
  Shooter.Config_kI(0, 0.0, 10);
  Shooter.Config_kD(0, 0.0, 10);

  // pointer to limelight stuff
  table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

//Gyro PID
  //TurnToAngle.SetTolerance(6.0, 3.0); //room for error
  TurnToAngle.SetTolerance(0.05); //room for error
  TurnToAngle.EnableContinuousInput(-180.0f,  180.0f); 
  ahrs.ZeroYaw();


//Drive train motor grouping start
  RightFollow.Follow(RightLead);
  LeftFollow.Follow(LeftLead);

//shooter motors 
  ShooterFollow.Follow(Shooter);

//Indexer motors 
  IndexerT.Follow(IndexerB);
  IndexerT.SetInverted(true); 

// Ultrasonic ranging.  
//DistancePID.SetSetpoint(60.0);
//DistancePID.SetTolerance(3);
} 


void Robot::RobotPeriodic() {

  // get the ultrasonic range
  Distance = Ultrasonic.GetVoltage()*1000.0*(1.0/0.977)*(1.0/25.4);
  frc::SmartDashboard::PutNumber("Distance", Distance);


  rEncoder = -RightLead.GetSelectedSensorPosition();
  frc::SmartDashboard::PutNumber("Right Encoder", rEncoder);
  lEncoder = LeftLead.GetSelectedSensorPosition();
  frc::SmartDashboard::PutNumber("Left Encoder", lEncoder);


// Get Shooter RPM
  RPM = Shooter.GetSelectedSensorVelocity(0);
  frc::SmartDashboard::PutNumber("Distance", (RPM*600/4096));

// get the electric eye statuses
  LaserState1 = IntakeLaser1.Get();
  frc::SmartDashboard::PutBoolean("Laser1", LaserState1);
  LaserState2 = IntakeLaser2.Get();
  frc::SmartDashboard::PutBoolean("Laser2", LaserState2);
  LaserState3 = IntakeLaser3.Get();
  frc::SmartDashboard::PutBoolean("Laser3", LaserState3);

  // Get current gyro info
  gyroAngle = ahrs.GetYaw();
  frc::SmartDashboard::PutNumber("gyroAngle", gyroAngle);

  // Get limelight stuff
  tx = table->GetNumber("tx",0.0); 
  //float ty = table->GetNumber("ty",0.0); 
  //float ta = table-> GetNumber("ta",0.0);
  //float ts = table-> GetNumber("ts", 0.0);
  tv = table-> GetNumber("tv", 0); //tv is 0 when box is not in view 
}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  TikTok.Reset();
  TikTok.Start();
  FIRST = true;
  FIRST2 = true;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  }else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {

  
// this is a pretty janky way to do a timed state machine
  // but it works
  // With Falcons and encoders: we "go for some distance" instead of "go for some time"
  // and we can really really clean this up

  double now = TikTok.Get();  // how long since auto started

  if (m_autoSelected == kAutoNameCustom) {

  }else {
    // standard auto here
    if (rEncoder <= 360000 && lEncoder <= 360000) {
      myRobot.ArcadeDrive( -0.3, 0.0, true);
    } else {
      myRobot.ArcadeDrive( 0.0, 0.0, true);
    }
    //if (now <= 1.0) {
      // Do something for the first second, say, drive backwards at 0.6 speed
      /*myRobot.ArcadeDrive( 0.6, 0.0, true);  // Backwards is plus. Really.
    } else if ((now > 1.0) && (now <= 3.0)) {
        if (FIRST){
      myRobot.ArcadeDrive( 0.0, 0.0, true);
        FIRST=false;
        } 
      // do stuff for the rest of the auto, say, lock in and shoot
      table->PutNumber("pipeline", 0); //shooting pipe line
      if (tv == 1) {
        TurnToAngle.SetSetpoint(gyroAngle + tx);  
        myRobot.ArcadeDrive( 0.0, TurnToAngle.Calculate(gyroAngle), true);
      }
    } else if ((now > 3.0) && (now <= 4.0)) {
      // rev up shooting motor for 1s.  We could/should check that it's AtSetpoint() and on target here
      Shooter.Set(ControlMode::Velocity, 2500);
    } else if ((now > 4.0) && (now <= 5.0)) {
      // run indexer to shoot balls, for 1s
      IndexerB.Set(ControlMode::PercentOutput, -0.5);
    } else if ((now > 5.0) && (now <= 8.0)){
      if (FIRST2){
      // do stuff for the rest of the auto, like stop all the motions and get ready for teleop
      Shooter.Set(ControlMode::PercentOutput, 0.0);
      IndexerB.Set(ControlMode::PercentOutput, 0.0);
      table->PutNumber("pipeline", 1); //camera pipe line
      double TargetAngle = gyroAngle + 180.0; 
      FIRST2 = false;
      if ( TargetAngle >= 180.0) TargetAngle -= 360.0;
      if ( TargetAngle <= -180.0) TargetAngle += 360.0;

      TurnToAngle.SetSetpoint(TargetAngle);
      FIRST2= false;
      }
       myRobot.ArcadeDrive( 0.0, TurnToAngle.Calculate(gyroAngle), true);

    } else {
      TurnToAngle.Reset();
      myRobot.ArcadeDrive( 0.0, 0.0, true);
    }*/

  }
 
}

void Robot::TeleopInit() {} 

void Robot::TeleopPeriodic() {

//Driving Values
   double deadzone = 0.2; 
   double XboxY;
   double XboxX;

//Ultrasonic after turn to angle
  //double output2 = 0.0;
  double output = 0.0;
  //if (tx == 0){
  //  output2 = DistancePID.Calculate(distancefilter.Calculate(Distance));
 // }
    

if (IsOperatorControl() && IsEnabled()) {

  // spin up shooter motor
   if (Xbox.GetBButton()) { 
       Shooter.Set(ControlMode::Velocity, 2500);
       // If we're too close, let driver know with a rumble 
       if (Distance<85.0) Xbox.SetRumble(frc::GenericHID::RumbleType::kLeftRumble,1.0);
     }else{
       Shooter.Set(ControlMode::PercentOutput, 0.0);
       Xbox.SetRumble(frc::GenericHID::RumbleType::kLeftRumble,0.0);
     }
    
//Driving: get xbox joystick info
    if(Xbox.GetX(frc::XboxController::JoystickHand(0)) > deadzone || Xbox.GetX(frc::XboxController::JoystickHand(0)) < -deadzone){
      XboxX = Xbox.GetX(frc::XboxController::JoystickHand(0));
    }else{
      XboxX = 0.0;
    }
    if(Xbox.GetY(frc::XboxController::JoystickHand(0)) > deadzone || Xbox.GetY(frc::XboxController::JoystickHand(0)) < -deadzone){
      XboxY = Xbox.GetY(frc::XboxController::JoystickHand(0));
    }else{
      XboxY = 0.0;
    }

//Firing and Intaking
    if(Xbox.GetAButton()){                             // extend intake arm
      IntakeArm.Set(frc::DoubleSolenoid::Value(2));
      IntakeWheels.Set(ControlMode::PercentOutput, 0.7);
     // IndexerB.Set(ControlMode::PercentOutput, -0.5);
    } else{                                             // retract, stop indexer
      IntakeArm.Set(frc::DoubleSolenoid::Value(1));
      IntakeWheels.Set(ControlMode::PercentOutput, 0.0);
    }

     if(Xbox.GetXButton() && true/*LaserState1*/){        // advance indexer unless beam is broken
      // TODO: rather than using X button to slurp in ball from intake
      // "if (!LaserState3)" would notice the ball there and could activate indexer too
      IndexerB.Set(ControlMode::PercentOutput, -0.5);
    }else { 
      IndexerB.Set(ControlMode::PercentOutput, 0.0);
    }

//CPM Deployment
  /*  if (Xbox.GetXButton() && CPM.Get()!=1){
      CPM.Set(frc::DoubleSolenoid::Value(1));
    }else if (Xbox.GetXButton() && CPM.Get()!=2){
      CPM.Set(frc::DoubleSolenoid::Value(2));
    }
//CPM movement 
    if (Xbox.GetTriggerAxis(frc::XboxController::JoystickHand(0))){
      CPMWheel.Set(ControlMode::PercentOutput, 0.5);
    }else{
      CPMWheel.Set(ControlMode::PercentOutput, 0.0);
    } */

//Climber Deployment and wench 
    if (Xbox.GetY(frc::XboxController::JoystickHand(1))>.2){
      ClimberLift.Set(ControlMode::PercentOutput, 0.35);//.35, .65
    }else if (Xbox.GetY(frc::XboxController::JoystickHand(1))<-.2){
      ClimberLift.Set(ControlMode::PercentOutput, -0.35);//-.35, .65
    }else if(Xbox.GetBumper(frc::XboxController::JoystickHand(0))){
      Climber.Set(ControlMode::PercentOutput, 0.55);//.5, .8
    } else if (Xbox.GetYButton()){
       Climber.Set(ControlMode::PercentOutput, 0.8);//.5, .8
    }else{
      Climber.Set(ControlMode::PercentOutput, 0.0);
      ClimberLift.Set(ControlMode::PercentOutput, 0.0);
    }

// limelight aim
    if (Xbox.GetBumper(frc::XboxController::JoystickHand(1))) { 
      Shooter.Set(ControlMode::Velocity, 2500);
      table->PutNumber("pipeline", 0); //shooting pipe line
      //Gyro turning with PID
      //if (!TurnToAngle.IsEnabled()) TurnToAngle.Enabled();
      if (tv == 1) {
        TurnToAngle.SetSetpoint(gyroAngle + tx);
      } else { 
        // we don't have a lock yet, just point where we're already pointing
        TurnToAngle.SetSetpoint(gyroAngle);
      }
      //Feed where we're at now as the input to the turning PID
      output = TurnToAngle.Calculate(gyroAngle);
      if (TurnToAngle.AtSetpoint()){   // If the PID thinks we're pointed correctly, no correction needed
        tx = 0;
      }
    }else{
      //Shooter.Set(ControlMode::PercentOutput, 0.0);
      table->PutNumber("pipeline", 1); //camera pipe line
      TurnToAngle.Reset();
    }

//Drive line with Limelight value used for turning (if/else statement added to combat Limelight branch)
    if (Xbox.GetBumper(frc::XboxController::JoystickHand(1))){
      myRobot.ArcadeDrive( XboxY, output, true);
//      myRobot.ArcadeDrive( XboxY + output2, output + Jack, true);
    } else {
      myRobot.ArcadeDrive( XboxY, XboxX/1.33, true);
    } 

//To unjam robot, manually move indexer backwards and reverse intake wheels
    if(Xbox.GetStartButton()){
      IndexerB.Set(ControlMode::PercentOutput, 0.5);
      Shooter.Set(ControlMode::PercentOutput, -0.5);
    }if(Xbox.GetBackButton()){
      IndexerB.Set(ControlMode::PercentOutput, 0.5);
      Shooter.Set(ControlMode::PercentOutput, -0.5);
      IntakeWheels.Set(ControlMode::PercentOutput, -0.7);
      IntakeArm.Set(frc::DoubleSolenoid::Value(2));
    }
}
}
void Robot::TestInit(){

TestFIRST = true;
TurnToAngle.Reset();

}

void Robot::TestPeriodic() {

  if ( TestFIRST){
      double TargetAngle = gyroAngle + 45.0;
      TurnToAngle.SetSetpoint(TargetAngle);
      TestFIRST= false;
      }
       if (!TurnToAngle.AtSetpoint()) myRobot.ArcadeDrive( 0.0, TurnToAngle.Calculate(gyroAngle), true);
    } 


#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
