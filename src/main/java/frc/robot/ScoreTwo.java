// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreTwo extends SequentialCommandGroup {
  /** Creates a new ScoreTwo. */
  public ScoreTwo(Swerve s_Swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    TrajectoryConfig config = new TrajectoryConfig(
      SwerveConstants.AutoConstants.kMaxSpeedMetersPerSecond,
      SwerveConstants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(SwerveConstants.Swerve.swerveKinematics);

    Trajectory goBack = 
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0, new Rotation2d(0)), 
        List.of(new Translation2d(-2,0)), 
        new Pose2d(-5,0, new Rotation2d(0)), 
        config.setReversed(true));
    
    Trajectory goForward = 
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0, new Rotation2d(0)), 
        List.of(new Translation2d(2,0)), 
        new Pose2d(5,0, new Rotation2d(0)), 
        config.setReversed(false));
    
    Trajectory goRight =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0, new Rotation2d(0)), 
        List.of(new Translation2d(0,1)), 
        new Pose2d(0,2, new Rotation2d(Math.toRadians(-90))), 
        config.setReversed(false));

    Trajectory goLeft =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0, new Rotation2d(0)), 
        List.of(new Translation2d(0,-1)), 
        new Pose2d(0,-2, new Rotation2d(Math.toRadians(90))), 
        config.setReversed(true));
    
    var thetaController =
      new ProfiledPIDController(
          SwerveConstants.AutoConstants.kPThetaController, 0, 0, SwerveConstants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    SwerveControllerCommand goBackCommand =
    new SwerveControllerCommand(
        goBack,
        s_Swerve::getPose,
        SwerveConstants.Swerve.swerveKinematics,
        new PIDController(SwerveConstants.AutoConstants.kPXController, 0, 0),
        new PIDController(SwerveConstants.AutoConstants.kPYController, 0, 0),
        thetaController,
        s_Swerve::setModuleStates,
        s_Swerve);
    SwerveControllerCommand goForwardCommand =
    new SwerveControllerCommand(
        goForward,
        s_Swerve::getPose,
        SwerveConstants.Swerve.swerveKinematics,
        new PIDController(SwerveConstants.AutoConstants.kPXController, 0, 0),
        new PIDController(SwerveConstants.AutoConstants.kPYController, 0, 0),
        thetaController,
        s_Swerve::setModuleStates,
        s_Swerve);
    SwerveControllerCommand goLeftCommand =
    new SwerveControllerCommand(
        goLeft,
        s_Swerve::getPose,
        SwerveConstants.Swerve.swerveKinematics,
        new PIDController(SwerveConstants.AutoConstants.kPXController, 0, 0),
        new PIDController(SwerveConstants.AutoConstants.kPYController, 0, 0),
        thetaController,
        s_Swerve::setModuleStates,
        s_Swerve);
    SwerveControllerCommand goRightCommand =
    new SwerveControllerCommand(
        goRight,
        s_Swerve::getPose,
        SwerveConstants.Swerve.swerveKinematics,
        new PIDController(SwerveConstants.AutoConstants.kPXController, 0, 0),
        new PIDController(SwerveConstants.AutoConstants.kPYController, 0, 0),
        thetaController,
        s_Swerve::setModuleStates,
        s_Swerve);
    addCommands(
      new InstantCommand(() -> s_Swerve.resetOdometry(goForward.getInitialPose())),
      goBackCommand,
      goForwardCommand,
      goBackCommand,
      goRightCommand,
      goLeftCommand,
      goForwardCommand
    );
  }
}
