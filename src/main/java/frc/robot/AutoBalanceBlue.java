// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.constants.SwerveConstants;
import frc.robot.state.ElevatorState;
import frc.robot.state.PivotyState;
import frc.robot.state.RobotState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PivotySubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.AutoBalanceCommand;
// import frc.robot.commands.ZeroIntakeCommand;
import frc.robot.commands.GroundPickup;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class AutoBalanceBlue extends SequentialCommandGroup {
    public AutoBalanceBlue(Swerve s_Swerve,RobotState robotState,PivotySubsystem pivoty, PivotyState pivotyState, Elevator elevator, ElevatorState elevatorState, Intake intake){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    // SwerveConstants.AutoConstants.kMaxSpeedMetersPerSecond,
                    // SwerveConstants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    2,
                    3)
                .setKinematics(SwerveConstants.Swerve.swerveKinematics);
        TrajectoryConfig configSlow =
                new TrajectoryConfig(
                        // SwerveConstants.AutoConstants.kMaxSpeedMetersPerSecond,
                        // SwerveConstants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        1,
                        1)
                    .setKinematics(SwerveConstants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory goForward =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(-2,-0.35),new Translation2d(-4, -0.4)),
               //  new Pose2d(-SmartDashboard.getNumber("auto length", 4.7), SmartDashboard.getNumber("auto width", .7), new Rotation2d(Math.PI)),
                new Pose2d(-4.3, -.6, new Rotation2d(-Math.PI)),
                config.setReversed(true));

        Trajectory goBack = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(.3, 0)),
                new Pose2d(.6, 0, new Rotation2d(-Math.PI)),
                configSlow.setReversed(false));
        Trajectory spin = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(-.3, 0)),
                new Pose2d(-.6, 0, new Rotation2d(-Math.PI)),
                config.setReversed(true));

        var thetaController =
            new ProfiledPIDController(
                SwerveConstants.AutoConstants.kPThetaController, 0, 0, SwerveConstants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                goForward,
                s_Swerve::getPose,
                SwerveConstants.Swerve.swerveKinematics,
                new PIDController(SwerveConstants.AutoConstants.kPXController, 0, 0),
                new PIDController(SwerveConstants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand swerveControllerCommand3 =
            new SwerveControllerCommand(
                goBack,
                s_Swerve::getPose,
                SwerveConstants.Swerve.swerveKinematics,
                new PIDController(SwerveConstants.AutoConstants.kPXController, 0, 0),
                new PIDController(SwerveConstants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        SwerveControllerCommand spinCommand =
                new SwerveControllerCommand(
                    spin,
                    s_Swerve::getPose,
                    SwerveConstants.Swerve.swerveKinematics,
                    new PIDController(SwerveConstants.AutoConstants.kPXController, 0, 0),
                    new PIDController(SwerveConstants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);
    

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(goForward.getInitialPose())),
            new OnePiece(pivoty, robotState.pivotyState, elevator, robotState.elevatorState, intake),
            swerveControllerCommand,
            new GroundPickup(pivoty, pivotyState, elevator, elevatorState, intake),
            // swerveControllerCommand2,
            swerveControllerCommand3,
            // new OnePiece(pivoty, robotState.pivotyState, elevator, robotState.elevatorState, intake),
            // spinCommand,
            new InstantCommand(() -> s_Swerve.zeroGyro())

            // new AutoBalanceCommand(s_Swerve, robotState, false).repeatedly()
            
        );
    }
}