package frc.robot;

import frc.robot.commands.AutoBalanceCommand;
import frc.robot.constants.SwerveConstants;
import frc.robot.state.RobotState;
import frc.robot.subsystems.Swerve;

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
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class AutoBalance extends SequentialCommandGroup {
    public AutoBalance(Swerve s_Swerve,RobotState robotState){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    // SwerveConstants.AutoConstants.kMaxSpeedMetersPerSecond,
                    // SwerveConstants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    2,
                    2)
                .setKinematics(SwerveConstants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory goForward =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(-2,0),new Translation2d(-4, 0)),
                new Pose2d(-6, 0, new Rotation2d(0)),
                config.setReversed(true));

        Trajectory goBack = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(.5, 0)),
                new Pose2d(1, 0, new Rotation2d(0)),
                config.setReversed(false));

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

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(goForward.getInitialPose())),
            swerveControllerCommand,
            // swerveControllerCommand2,
            swerveControllerCommand3,
            new AutoBalanceCommand(s_Swerve, robotState, false).repeatedly()
            
        );
    }
}