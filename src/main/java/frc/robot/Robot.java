package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.libraries.NewSwerve.CTREConfigs;
import frc.robot.libraries.external.math.RigidTransform2;
import frc.robot.libraries.external.math.Rotation2;
import frc.robot.libraries.external.robot.UpdateManager;
import frc.robot.libraries.external.robot.drivers.Limelight;

public class Robot extends TimedRobot {
    private static Robot instance = null;
    public static CTREConfigs ctreConfigs;

    private Command m_autonomousCommand;

    private RobotContainer robotContainer = new RobotContainer();
    private UpdateManager updateManager = new UpdateManager(
            // robotContainer.getDrivetrainSubsystem()
    );

    public Robot() {
        instance = this;
    }

    public static Robot getInstance() {
        return instance;
    }

    @Override
    public void robotInit() {
        updateManager.startLoop(5.0e-3);
        // robotContainer.getVisionSubsystem().setLedMode(Limelight.LedMode.OFF);
        ctreConfigs = new CTREConfigs();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        // robotContainer.getDrivetrainSubsystem().resetPose(RigidTransform2.ZERO);
        // robotContainer.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO);

        // robotContainer.getAutonomousCommand().schedule();
        
    }

    @Override
    public void disabledPeriodic() {
        // robotContainer.getVisionSubsystem().setLedMode(Limelight.LedMode.OFF);
    }

    @Override
    public void teleopInit() {
        // robotContainer.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO);
    }
}
