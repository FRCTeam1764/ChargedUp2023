package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.libraries.external.util.AutonomousChooser;
import frc.robot.libraries.external.control.Trajectory;
import frc.robot.libraries.external.math.Rotation2;
import frc.robot.libraries.external.robot.input.Axis;
import frc.robot.libraries.external.robot.input.XboxController;
import frc.robot.libraries.external.robot.input.DPadButton.Direction;

public class RobotContainer {
    private final XboxController primaryController = new XboxController(0);

    private final Superstructure superstructure = new Superstructure();

    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrainSubsystem);
    private final Elevator elevator = new Elevator();
    private final GrabberSubsystem grabber = new GrabberSubsystem();
    private final GrabberSubsystemCone grabberCone = new GrabberSubsystemCone();
    private final PivotySubsystem pivoty = new PivotySubsystem();
    private Trajectory[] trajectories;
    private final AutonomousChooser autonomousChooser;

    public RobotContainer() {
        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        CommandScheduler.getInstance().registerSubsystem(visionSubsystem);
        CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);

        CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));
        
        setTrajectories();
        configurePilotButtonBindings();
        configureCoPilotButtonBindings();
        autonomousChooser = new AutonomousChooser(trajectories);
    }

    private void setTrajectories() {
        trajectories = Trajectories.getTrajectories();
    }

    private void configurePilotButtonBindings() {
<<<<<<< HEAD
        primaryController.getBackButton().whenPressed(
                () -> drivetrainSubsystem.resetGyroAngle(Rotation2.ZERO)
        );
        
=======
        primaryController.getBackButton().onTrue(new ResetGyroCommand(drivetrainSubsystem));
>>>>>>> 2023_Update
    }

    private void configureCoPilotButtonBindings() {
        primaryController.getDPadButton(Direction.UP).onTrue(new ElevatorCommand(elevator,0.7));
        primaryController.getDPadButton(Direction.DOWN).onTrue(new ElevatorCommand(elevator,-0.7));
        primaryController.getAButton().onTrue(new GrabberCommand(grabber, 0.7));
        primaryController.getBButton().onTrue(new GrabberCommand(grabber, -.07));
        primaryController.getXButton().onTrue(new GrabberCommandCone(grabberCone, 0.7));
        primaryController.getYButton().OnTrue(new GrabberCommandCone(grabberCone, -.07));
        primaryController.getDPadButton(Direction.LEFT).onTrue(new PivotyCommand(pivoty,0.7));
        primaryController.getDPadButton(Direction.RIGHT).onTrue(new PivotyCommand(pivoty,-0.7));
    }

    public Command getAutonomousCommand() {
        return autonomousChooser.getCommand(this);
    }

    private Axis getDriveForwardAxis() {
        return primaryController.getLeftYAxis();
    }

    private Axis getDriveStrafeAxis() {
        return primaryController.getLeftXAxis();
    }

    private Axis getDriveRotationAxis() {
        return primaryController.getRightXAxis();
    }

    public DrivetrainSubsystem getDrivetrainSubsystem() {
        return drivetrainSubsystem;
    }

    public Superstructure getSuperstructure() {
        return superstructure;
    }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    public XboxController getPrimaryController() {
        return primaryController;
    }

    public AutonomousChooser getAutonomousChooser() {
        return autonomousChooser;
    }

    public Trajectory[] getTrajectories() {
        return trajectories;
    }
}