package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
// import frc.robot.libraries.external.util.AutonomousChooser;
import frc.robot.libraries.external.control.Trajectory;
import frc.robot.libraries.external.math.Rotation2;
import frc.robot.libraries.external.robot.input.Axis;
// import frc.robot.libraries.external.robot.input.XboxController;
import frc.robot.libraries.external.robot.input.DPadButton.Direction;
import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {
    private final XboxController driver = new XboxController(0);
    private final XboxController secondaryController = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    // private final XboxController secondaryController = new XboxController(0);

    private final Superstructure superstructure = new Superstructure();

    // private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    // private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrainSubsystem);
    private final Elevator elevator = new Elevator();
    private final GrabberSubsystem grabber = new GrabberSubsystem();
    private final GrabberSubsystemCone grabberCone = new GrabberSubsystemCone();
    private final PivotySubsystem pivoty = new PivotySubsystem();
    private Trajectory[] trajectories;
    // private final AutonomousChooser autonomousChooser;

    public RobotContainer() {
        // secondaryController.getLeftXAxis().setInverted(true);
        // secondaryController.getRightXAxis().setInverted(true);

        // CommandScheduler.getInstance().registerSubsystem(visionSubsystem);
        // CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);

        // CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));
        
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        setTrajectories();
        configurePilotButtonBindings();
        configureCoPilotButtonBindings();
        // autonomousChooser = new AutonomousChooser(trajectories);
    }

    private void setTrajectories() {
        trajectories = Trajectories.getTrajectories();
    }

    private void configurePilotButtonBindings() {
        // secondaryController.getBackButton().onTrue(new ResetGyroCommand(drivetrainSubsystem));
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }

    private void configureCoPilotButtonBindings() {
        secondaryController.getDPadButton(Direction.UP).onTrue(new ElevatorCommand(elevator,0.7));
        secondaryController.getDPadButton(Direction.DOWN).onTrue(new ElevatorCommand(elevator,-0.7));
        secondaryController.getAButton().onTrue(new GrabberCommand(grabber, 0.7));
        secondaryController.getBButton().onTrue(new GrabberCommand(grabber, -.07));
        secondaryController.getXButton().onTrue(new GrabberCommandCone(grabberCone, 0.7));
        secondaryController.getYButton().onTrue(new GrabberCommandCone(grabberCone, -.07));
        secondaryController.getDPadButton(Direction.LEFT).onTrue(new PivotyCommand(pivoty,0.7));
        secondaryController.getDPadButton(Direction.RIGHT).onTrue(new PivotyCommand(pivoty,-0.7));
    }

    // public Command getAutonomousCommand() {
    //     return autonomousChooser.getCommand(this);
    // }
    // // public Command getAutonomousCommand() {
    //     // An ExampleCommand will run in autonomous
    //     return new exampleAuto(s_Swerve);
    // }

    // private Axis getDriveForwardAxis() {
    //     return secondaryController.getLeftYAxis();
    // }

    // private Axis getDriveStrafeAxis() {
    //     return secondaryController.getLeftXAxis();
    // }

    // private Axis getDriveRotationAxis() {
    //     return secondaryController.getRightXAxis();
    // }

    // public DrivetrainSubsystem getDrivetrainSubsystem() {
    //     return drivetrainSubsystem;
    // }

    public Superstructure getSuperstructure() {
        return superstructure;
    }

    // public VisionSubsystem getVisionSubsystem() {
    //     return visionSubsystem;
    // }

    public XboxController getsecondaryController() {
        return secondaryController;
    }

    // public AutonomousChooser getAutonomousChooser() {
    //     return autonomousChooser;
    // }

    public Trajectory[] getTrajectories() {
        return trajectories;
    }
}