package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.networktables.NetworkTableInstance;
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
import frc.robot.state.RobotState;
import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton limelight0 = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton limelight1 = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton limelight2 = new JoystickButton(driver, XboxController.Button.kX.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final XboxController primaryController = new XboxController(0);
    RobotState robotState;
    private final LimelightSubsystem limelight = new LimelightSubsystem(NetworkTableInstance.getDefault().getTable("limelight"));
    private final Superstructure superstructure = new Superstructure();

    // private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    // private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrainSubsystem);
    // private final Elevator elevator = new Elevator();
    // private final GrabberSubsystem grabber = new GrabberSubsystem();
    // private final GrabberSubsystemCone grabberCone = new GrabberSubsystemCone();
    // private final PivotySubsystem pivoty = new PivotySubsystem();
    private Trajectory[] trajectories;
    // private final AutonomousChooser autonomousChooser;

    public RobotContainer() {
        // primaryController.getLeftXAxis().setInverted(true);
        // primaryController.getRightXAxis().setInverted(true);

        // CommandScheduler.getInstance().registerSubsystem(visionSubsystem);
        // CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);

        // CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));
        robotState = new RobotState(driver);
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                robotState
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
        // primaryController.getBackButton().onTrue(new ResetGyroCommand(drivetrainSubsystem));
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        limelight0.onTrue(new LimelightCommand(limelight, 0));
        limelight1.onTrue(new LimelightCommand(limelight, 1));
        limelight2.onTrue(new LimelightCommand(limelight, 2));
    }

    private void configureCoPilotButtonBindings() {

    //     primaryController.getDPadButton(Direction.UP).onTrue(new ElevatorCommand(elevator,0.7));
    //     primaryController.getDPadButton(Direction.DOWN).onTrue(new ElevatorCommand(elevator,-0.7));
    //     primaryController.getAButton().onTrue(new GrabberCommand(grabber, 0.7));
    //     primaryController.getBButton().onTrue(new GrabberCommand(grabber, -.07));
    //     primaryController.getXButton().onTrue(new GrabberCommandCone(grabberCone, 0.7));
    //     primaryController.getYButton().onTrue(new GrabberCommandCone(grabberCone, -.07));
    //     primaryController.getDPadButton(Direction.LEFT).onTrue(new PivotyCommand(pivoty,0.7));
    //     primaryController.getDPadButton(Direction.RIGHT).onTrue(new PivotyCommand(pivoty,-0.7));
    }

    // public Command getAutonomousCommand() {
    //     return autonomousChooser.getCommand(this);
    // }
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("ThreePieceLeft", new PathConstraints(4, 3));
        return new FollowPath(s_Swerve, examplePath);
    }


    // private Axis getDriveForwardAxis() {
    //     return primaryController.getLeftYAxis();
    // }

    // private Axis getDriveStrafeAxis() {
    //     return primaryController.getLeftXAxis();
    // }

    // private Axis getDriveRotationAxis() {
    //     return primaryController.getRightXAxis();
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

    public XboxController getPrimaryController() {
        return primaryController;
    }

    // public AutonomousChooser getAutonomousChooser() {
    //     return autonomousChooser;
    // }

    public Trajectory[] getTrajectories() {
        return trajectories;
    }
}