// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// import edu.wpi.first.wpilibj2.command.Command.*;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.constants.SwerveConstants;
// import frc.robot.subsystems.Swerve;

// /** Add your docs here. */
// public class PathPlanners {
//     Swerve swerve;
//     PathPlannerTrajectory examplePath = PathPlanner.loadPath("ThreePieceLeft", new PathConstraints(4, 3));
//     PathPlanners(Swerve swerve){
//         this.swerve = swerve;
//     }
//     SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
//     swerve::getPose, // Pose2d supplier
//     swerve::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
//     SwerveConstants.Swerve.swerveKinematics, // SwerveDriveKinematics
//     new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
//     new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
//     swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
//     eventMap,
//     true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
//     swerve // The drive subsystem. Used to properly set the requirements of path following commands
// );
// public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
//     return new SequentialCommandGroup(
//          new InstantCommand(() -> {
//            // Reset odometry for the first path you run during auto
//            if(isFirstPath){
//                swerve.resetOdometry(traj.getInitialHolonomicPose());
//            }
//          }),
//          new PPSwerveControllerCommand(
//              traj, 
//              swerve::getPose, // Pose supplier
//              SwerveConstants.Swerve.swerveKinematics, // SwerveDriveKinematics
//              new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
//              new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
//              new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
//              swerve::setModuleStates, // Module states consumer
//              true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
//              swerve // Requires this drive subsystem
//          )
//      );
//  }

// Command fullAuto = autoBuilder.fullAuto(examplePath);
// }
