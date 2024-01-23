// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Consumer;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.VisionTargetCommand;
import frc.robot.subsystems.Arm;
//import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_robotDrive = new Drivetrain(13);
  private final Arm m_robotArm = new Arm(16,15,17,18,21);
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  CommandXboxController m_driverController = new CommandXboxController(0);

  Joystick stick1 = new Joystick(1);
  Joystick stick2 = new Joystick(2);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    NamedCommands.registerCommand("ArmUp",this.armGoToPose(constants.ShelfAlpha, constants.ShelfBeta, constants.ShelfGamma, 1.0));
    NamedCommands.registerCommand("ArmDown",this.armGoToPose(constants.StowAlpha, constants.StowBeta, constants.StowGamma, 1.0));
    configureButtonBindings();
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> 
                m_robotDrive.Drive(
                    MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.2)*constants.k_maxSpeed,
                    MathUtil.applyDeadband(-m_driverController.getLeftX(),0.2)*constants.k_maxSpeed,
                    MathUtil.applyDeadband(-m_driverController.getRightX(),0.2)*constants.k_maxRotSpeed,
                    true),
            m_robotDrive));

    m_robotArm.setDefaultCommand(
          new RunCommand(
            () -> m_robotArm.DriveStick(stick1,stick2), m_robotArm)
          );  
    
  }

  /*
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController.a().onTrue(new VisionTargetCommand(m_robotDrive, 5.2431, -1.26019, 0));
    new JoystickButton(stick1, 7)//7
    .whileTrue(
            Commands.runOnce(
                () -> {
                  m_robotArm.zeroEncoders();
                },
                m_robotArm));

    new JoystickButton(stick2, 4)//4
        .whileTrue(
            Commands.run(
                () -> {
                  m_robotArm.GoTo(constants.StowAlpha, constants.StowBeta, constants.StowGamma, 1.0);
                },
                m_robotArm));

    new JoystickButton(stick2, 5)//5
    .whileTrue(
        Commands.run(
            () -> {
              m_robotArm.GoTo(constants.ShelfAlpha, constants.ShelfBeta, constants.ShelfGamma, 1.0);
            },
            m_robotArm));

    new JoystickButton(stick2, 7)//7
    .whileTrue(
        Commands.run(
            () -> {
              m_robotArm.GoTo(constants.MedAlpha, constants.MedBeta, constants.MedGamma, 1.0);
            },
            m_robotArm));
        
    new JoystickButton(stick2,6)//6
    .whileTrue(
        Commands.run(
            () -> {
              m_robotArm.GoTo(constants.HIAlpha, constants.HIBeta, constants.HIGamma, 1.0);
            },
            m_robotArm));
    
    new JoystickButton(stick2, 8)//8
    .whileTrue(
        Commands.run(
            () -> {
              m_robotArm.GoTo(constants.FloorAlpha, constants.FloorBeta, constants.FloorGamma, 1.0);
            },
            m_robotArm));

    new JoystickButton(stick1, 11)//11
    .whileTrue(
        Commands.run(
            () -> {
              m_robotArm.GoTo(constants.AutoPlaceAlpha, constants.AutoPlaceBeta, constants.AutoPlaceGamma, 1.0);
            },
            m_robotArm));
        
        
        
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command armGoToPose(double alpha, double beta, double gamma, double multipier) {
    // implicitly require `this`
    return new RunCommand(() -> m_robotArm.GoTo(alpha,beta,gamma,multipier),m_robotArm);
  }
  public Command armMoveAtPoint(Pose2d setpoint, double alpha, double beta, double gamma, double multipier){
    m_robotDrive.tempSetpoint = setpoint;
    return new ConditionalCommand(this.armGoToPose(alpha,beta,gamma,multipier), this.armGoToPose(constants.StowAlpha,constants.StowBeta,constants.StowGamma,multipier), m_robotDrive::OdometryAtSetpoint);
  }
  
  public Command getSwerveMove(Trajectory exampleTrajectory){

    var angleController =
        new ProfiledPIDController(
            constants.k_AAP,constants.k_AAI, constants.k_AAD, constants.k_angleControllerConstriants);
    angleController.enableContinuousInput(-constants.k_PI, constants.k_PI);
    
    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::SwerveOdometryGetPose, 
            m_robotDrive.kinematics,
            new PIDController(constants.k_AXP, constants.k_AXI, constants.k_AXD),
            new PIDController(constants.k_AYP, constants.k_AYI, constants.k_AYD),
            angleController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
      return swerveControllerCommand.andThen(() -> m_robotDrive.Drive(0, 0, 0, true));

  }
  // public Command getAutonomousCommand() {
   
  //   TrajectoryConfig config = new TrajectoryConfig(constants.k_maxSpeed, constants.k_maxAcc).setKinematics(m_robotDrive.kinematics);
  //   Trigger poseTrigger = new Trigger(null);
  //   // An example trajectory to follow.  All units in meters.
  //   Trajectory exampleTrajectory =
  //       TrajectoryGenerator.generateTrajectory(
  //           // Start at the origin facing the +X direction
  //           new Pose2d(0, 0, new Rotation2d(0)),
  //           // Pass through these two interior waypoints, making an 's' curve path
  //           List.of(new Translation2d(.5, .5)),
  //           // End 3 meters straight ahead of where we started, facing forward
  //           new Pose2d(1, 1, new Rotation2d(0)),
  //           config);

  //     Command arm1 = this.armMoveAtPoint(new Pose2d(.5,.5,new Rotation2d(0)), constants.ShelfAlpha, constants.ShelfBeta, constants.ShelfGamma,1.0);
  //     Command trajmove1 = this.getSwerveMove(exampleTrajectory);
  //     Command auto = Commands.parallel(null,trajmove1);
  //     return this.getSwerveMove(exampleTrajectory);

    
  // }
  // public Command getAutonomousCommand() {
  //   //Odometry supplier
  //   Supplier<Pose2d> get_odometry = () -> m_robotDrive.SwerveOdometryGetPose();
  //   //set drive command consumer
  //   Consumer<SwerveModuleState[]> set_states= states -> m_robotDrive.setModuleStates(states);
  //   // Create config for trajectory
  //   Trajectory traj;
  //   String trajectoryJSON = "paths/Unnamed.wpilib.json";
  //   TrajectoryConfig config = new TrajectoryConfig(constants.k_maxSpeed, constants.k_maxAcc).setKinematics(m_robotDrive.kinematics);
  //   Trigger poseTrigger = new Trigger(m_robotDrive::OdometryAtSetpoint);
    
  //   Trajectory exampleTrajectory =
  //       TrajectoryGenerator.generateTrajectory(
  //           // Start at the origin facing the +X direction
  //           new Pose2d(0, 0, new Rotation2d(0)),
  //           // Pass through these two interior waypoints, making an 's' curve path
  //           List.of(new Translation2d(.5, .5)),
  //           // End 3 meters straight ahead of where we started, facing forward
  //           new Pose2d(1, 1, new Rotation2d(0)),
  //           config);
  //   try {
  //       Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
  //       exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  //   } catch (IOException ex) {
  //           DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
  //   }
          
  //   var angleController =
  //       new ProfiledPIDController(
  //           constants.k_AAP,constants.k_AAI, constants.k_AAD, constants.k_angleControllerConstriants);
  //   angleController.enableContinuousInput(-constants.k_PI, constants.k_PI);
    
  //   SwerveControllerCommand swerveControllerCommand =
  //       new SwerveControllerCommand(
  //           exampleTrajectory,
  //           m_robotDrive::SwerveOdometryGetPose, 
  //           m_robotDrive.kinematics,
  //           new PIDController(constants.k_AXP, constants.k_AXI, constants.k_AXD),
  //           new PIDController(constants.k_AYP, constants.k_AYI, constants.k_AYD),
  //           angleController,
  //           m_robotDrive::setModuleStates,
  //           m_robotDrive);

  //     Command arm =  this.armMoveAtPoint(new Pose2d(0,0,new Rotation2d(0)), constants.ShelfAlpha, constants.ShelfBeta, constants.ShelfGamma,1.0);
  //     Command driveCommand = swerveControllerCommand.andThen(() -> m_robotDrive.Drive(0, 0, 0, true));
  //     Command driveCommand2 = new VisionTargetCommand(m_robotDrive,5.2431, -1.26019, 0).andThen(() -> m_robotDrive.Drive(0, 0, 0, true));
  //     return driveCommand2.alongWith(arm);

    
  // }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Test Auto");
    }
}






























