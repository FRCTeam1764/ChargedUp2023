package frc.robot;

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
import frc.robot.libraries.external.robot.input.JoystickAxis;
// import frc.robot.libraries.external.robot.input.XboxController;
import frc.robot.libraries.external.robot.input.DPadButton.Direction;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {
    private final Joystick driver = new Joystick(0);
    private final Joystick secondaryController = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton blinkinButton = new JoystickButton(driver, XboxController.Button.kY.value);

    /* CoPilot Buttons */
    private final JoystickButton highRung = new JoystickButton(secondaryController, XboxController.Button.kY.value);
    private final JoystickButton midRung = new JoystickButton(secondaryController, XboxController.Button.kX.value);
    private final JoystickButton lowRung = new JoystickButton(secondaryController, XboxController.Button.kA.value);
    private final JoystickButton elevatorUp = new JoystickButton(secondaryController, XboxController.Button.kRightBumper.value);
    private final JoystickButton elevatorDown = new JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton elevatorLeft = new JoystickButton(secondaryController, XboxController.Button.kBack.value);
    private final JoystickButton elevatorRight = new JoystickButton(secondaryController, XboxController.Button.kStart.value);
    private final JoystickAxis intakeIn = new JoystickAxis(secondaryController, XboxController.Axis.kRightTrigger.value);
    private final JoystickAxis intakeOff = new JoystickAxis(secondaryController, XboxController.Axis.kLeftTrigger.value);
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    // private final XboxController secondaryController = new XboxController(0);


    private final Superstructure superstructure = new Superstructure();

    // private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    // private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrainSubsystem);
    private final Elevator elevator = new Elevator();
    public final intakeSubsystem intake = new intakeSubsystem();
  //  private final GrabberSubsystem grabber = new GrabberSubsystem();
   // private final GrabberSubsystemCone grabberCone = new GrabberSubsystemCone();
    private final PivotySubsystem pivoty = new PivotySubsystem();
    private final BlinkinSubsystem blinkin = new BlinkinSubsystem();
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
        blinkinButton.whileTrue(new BlinkinCommand( blinkin));
    }
//do new button bindings
    private void configureCoPilotButtonBindings() {
        highRung.onTrue(new ElevatorPivotyCommandGroup(elevator, .8, pivoty, .8, 1234, 3));
        midRung.onTrue(new ElevatorPivotyCommandGroup(elevator, .8, pivoty, .8, 1000, 2));
        lowRung.onTrue(new ElevatorPivotyCommandGroup(elevator, .8, pivoty, .8, 500, 1));
       // secondaryController.getDPadButton(Direction.UP).onTrue(new ElevatorCommand(elevator,0.7)); maybe find dpadbuttons
       // secondaryController.getDPadButton(Direction.DOWN).onTrue(new ElevatorCommand(elevator,-0.7));
        // secondaryController.getAButton().onTrue(new GrabberCommand(grabber, 0.7));
        // secondaryController.getBButton().onTrue(new GrabberCommand(grabber, -.07));
        // secondaryController.getXButton().onTrue(new GrabberCommandCone(grabberCone, 0.7));
        // secondaryController.getYButton().onTrue(new GrabberCommandCone(grabberCone, -.07));
        //secondaryController.getDPadButton(Direction.LEFT).onTrue(new PivotyCommand(pivoty,0.7));maybe find dpadbuttons
        //secondaryController.getDPadButton(Direction.RIGHT).onTrue(new PivotyCommand(pivoty,-0.7));

        elevatorUp.whileTrue(new ElevatorCommand(elevator, .8 , 3));
        elevatorDown.whileTrue(new ElevatorCommand(elevator, 0.8, 1));
        elevatorLeft.whileTrue(new PivotyCommand(pivoty, 0.8, 69420));
        elevatorRight.whileTrue(new PivotyCommand(pivoty, -.8, -69420));
        
        intakeIn
    }

     public Command getAutonomousCommand() {
    //     return autonomousChooser.getCommand(this);
    // }
    // // public Command getAutonomousCommand() {
         // An ExampleCommand will run in autonomous
         return new exampleAuto(s_Swerve);
     }

    // private Axis getDriveForwardAxis() {
    //     return secondaryController.getLeftYAxis();
    // }

    // private Axis getDriveStrafeAxis() {
    //     return secondaryController.getLeftXAxis();
    // }

    // private Axis getDriveRotationAxis() {
    //     return secondaryController.getRightXAxis();
    // }

     public Swerve getDrivetrainSubsystem() {
         return s_Swerve;
     }

   public Superstructure getSuperstructure() {
       return superstructure;
   }


    // public VisionSubsystem getVisionSubsystem() {
    //     return visionSubsystem;
    // }

    public Joystick getsecondaryController() {
        return secondaryController;
    }

    // public AutonomousChooser getAutonomousChooser() {
    //     return autonomousChooser;
    // }

   public Trajectory[] getTrajectories() {
       return trajectories;
   }
    

public intakeSubsystem getIntakeSubsystem() {
    return intake;
}
   public BlinkinSubsystem getBlinkinSubsystem() {
    return blinkin;
   }
}
