// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.constants;
import frc.robot.subsystems.SwerveMod;
import frc.robot.subsystems.OECPigeionIMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;;

public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final Field2d m_field = new Field2d();
  private SwerveMod RFMod = new SwerveMod(5,6,11,false, constants.k_RFZERO,false,true);
  private SwerveMod RBMod = new SwerveMod(8,7,12,false,constants.k_RBZERO,true,true);
  private SwerveMod LBMod = new SwerveMod(1,2,9,false, constants.k_LBZERO,false,true);
  private SwerveMod LFMod = new SwerveMod(3,4,10,false, constants.k_LFZERO,true,true);
  //Robot Dimensions 27,305 by 29,845 (in inches)
  //Positions defined from a top down view
  private final Limelight m_limelight = new Limelight("limelight");
  public Pose2d tempSetpoint;
  private Translation2d frontLeftLocation = new Translation2d(0.29845, 0.27305);
  private Translation2d frontRightLocation = new Translation2d(0.29845, -0.27305);
  private Translation2d backLeftLocation = new Translation2d(-0.29845, 0.27305);
  private Translation2d backRightLocation = new Translation2d(-0.29845, -0.27304);

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation); 
  OECPigeionIMU gyro;
  SwerveDriveOdometry odometry;
  Trajectory trajectory;
  //String file = "D:/Temp Robotics/JavaSwerveDriveCommand-Imported/src/main/paths/output/Unnamed.wpilib.json";

  public Drivetrain(int gyroport){
      //Motor process:
      /*Decide front back, left and right and assign locations/module names accordingly
        Find the encoder offsets by reading the absolute encoders when all the wheels are lined up in a direction parallel to front
        Log each wheels distance traveled, move forward on the sticks and note any wheels that are moving in a negative distance
        Subtract PI from the absolute encoder offset of the wheels reported in a negative distance going forward on the input
        Utilize the DriveReverse parameter in the module constructors to ensure that each module is spinning in the correct direction
       * 
       */
      gyro = new OECPigeionIMU(gyroport);  
      //gyro.BootCalibrate();
      gyro.ResetYaw();
      //Odometry Initialization
      odometry =  new SwerveDriveOdometry(
            kinematics,
            this.getAngle(),
            new SwerveModulePosition[] {
              LFMod.GetPosition(),
              RFMod.GetPosition(),
              LBMod.GetPosition(),
              RBMod.GetPosition()
            });

          AutoBuilder.configureHolonomic(
            this::SwerveOdometryGetPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    constants.k_maxSpeed, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },this);
      //Add reset Code
  }
  public void Drive(double xSpeed,double ySpeed,double rot, boolean fieldRelative){


      SwerveModuleState[] states = kinematics.toSwerveModuleStates(
          fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                              xSpeed, ySpeed, rot, this.getAngle())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
    
      SwerveDriveKinematics.desaturateWheelSpeeds(states, constants.k_maxSpeed);
        //Wheel states as order declared in kinematics constructor
        RFMod.SetDesiredState(states[1]);//3]);
        RBMod.SetDesiredState(states[3]);//1]);
        LBMod.SetDesiredState(states[2]);//0]);
        LFMod.SetDesiredState(states[0]);//2]);
  }
  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, constants.k_maxSpeed);
    //Wheel states as order declared in kinematics constructor
    RFMod.SetDesiredState(targetStates[1]);//3]);
    RBMod.SetDesiredState(targetStates[3]);//1]);
    LBMod.SetDesiredState(targetStates[2]);//0]);
    LFMod.SetDesiredState(targetStates[0]);//2]);
  }
  public boolean OdometryAtSetpoint(){
    if(Math.abs(this.tempSetpoint.getX()-this.SwerveOdometryGetPose().getX())<0.1&&
       Math.abs(this.tempSetpoint.getY()-this.SwerveOdometryGetPose().getY())<0.1 &&
       Math.abs(this.tempSetpoint.getRotation().getRadians()-this.SwerveOdometryGetPose().getRotation().getRadians())<0.1){
      return true;
    }else{
      return false;
    }
    
  }
  public void setModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states, constants.k_maxSpeed);
    //Wheel states as order declared in kinematics constructor
        RFMod.SetDesiredState(states[1]);//3]);
        RBMod.SetDesiredState(states[3]);//1]);
        LBMod.SetDesiredState(states[2]);//0]);
        LFMod.SetDesiredState(states[0]);//2]);
  }

  public void UpdateOdometry() {
      //Wheel states as order declared in kinematics constructor
      odometry.update(this.getAngle(),
                        new SwerveModulePosition[]{
                        LFMod.GetPosition(), 
                        RFMod.GetPosition(),
                        LBMod.GetPosition(), 
                        RBMod.GetPosition()});
  }  
  public Pose2d SwerveOdometryGetPose(){
      return odometry.getPoseMeters();
  }
  public Rotation2d getAngle(){
      return Rotation2d.fromRadians(((gyro.GetYaw()*((constants.k_PI)/(180.0)))));
  }
  public Rotation2d getPitchRad(){
      return new Rotation2d((gyro.GetPitch()*((constants.k_PI)/(180.0))));
  }
  public Rotation2d getRolRad(){
      return new Rotation2d((gyro.GetRoll()*((constants.k_PI)/(180.0))));
  }
  public void ResetDrive(){
      LFMod.ResetEncoder();
      LBMod.ResetEncoder();
      RFMod.ResetEncoder();
      RBMod.ResetEncoder();
  }
  public double getVisionX() {
    return m_limelight.getBotX();
  }

public double getVisionY() {
    return m_limelight.getBotY();
}

public double getVisionZ() {
    return m_limelight.getBotZ();
}

public double getVisionYaw() {
    return m_limelight.getBotYaw();
}

public double getVisionPitch() {
    return m_limelight.getBotPitch();
}

public double getVisionRoll() {
    return m_limelight.getBotRoll();
}

public boolean visionHasTarget() {
    return m_limelight.hasTarget();
}
public SwerveModulePosition[] getModulePositions(){
  return new SwerveModulePosition[] {
                                  LFMod.GetPosition(),
                                  RFMod.GetPosition(),
                                  LBMod.GetPosition(),
                                  RBMod.GetPosition()};
}
public SwerveModuleState[] getModuleStates(){
  return new SwerveModuleState[] {
                                  LFMod.GetState(),
                                  RFMod.GetState(),
                                  LBMod.GetState(),
                                  RBMod.GetState()};
}
public void resetPose(Pose2d pose) {
  odometry.resetPosition(this.getAngle(),this.getModulePositions(), pose);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.UpdateOdometry();
    m_field.setRobotPose(this.SwerveOdometryGetPose());
    SmartDashboard.putNumber("x", this.SwerveOdometryGetPose().getX());
    SmartDashboard.putNumber("LFwheeltravel_dist", LFMod.GetPosition().distanceMeters);
    SmartDashboard.putNumber("LBwheeltravel_dist", LBMod.GetPosition().distanceMeters);
    SmartDashboard.putNumber("RFwheeltravel_dist", RFMod.GetPosition().distanceMeters);
    SmartDashboard.putNumber("RBwheeltravel_dist", RBMod.GetPosition().distanceMeters);
    SmartDashboard.putNumber("y", this.SwerveOdometryGetPose().getY());
    SmartDashboard.putNumber("Swerve Angle", this.SwerveOdometryGetPose().getRotation().getDegrees());
    SmartDashboard.putNumber("GyroAngle", this.getAngle().getDegrees());

    SmartDashboard.putNumber("LF", LFMod.GetAbsEncoderAngle());
    SmartDashboard.putNumber("RF", RFMod.GetAbsEncoderAngle());
    SmartDashboard.putNumber("LB", LBMod.GetAbsEncoderAngle());
    SmartDashboard.putNumber("RB", RBMod.GetAbsEncoderAngle());
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
