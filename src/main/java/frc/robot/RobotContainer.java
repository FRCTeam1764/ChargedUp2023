package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.libraries.external.control.Trajectory;
import frc.robot.state.RobotState;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    private final JoystickButton elevatorButtonUp =  new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton elevatorButtonDown =  new JoystickButton(driver, XboxController.Button.kB.value);
    
    //private final JoystickButton limelight1 = new JoystickButton(driver, XboxController.Button.kA.value);
    //private final JoystickButton limelight2 = new JoystickButton(driver, XboxController.Button.kB.value);

    /* CoPilot Buttons */
    private final JoystickButton highRung = new JoystickButton(secondaryController, XboxController.Button.kY.value);
    private final JoystickButton PivotyDown = new JoystickButton(secondaryController, XboxController.Button.kX.value);
    private final JoystickButton PivotyUp = new JoystickButton(secondaryController, XboxController.Button.kA.value);
    private final JoystickButton openIntake = new JoystickButton(secondaryController, XboxController.Button.kRightBumper.value);
    private final JoystickButton closeIntake = new JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton toggleDriveTrainAutoBalance =  new JoystickButton(secondaryController, XboxController.Button.kStart.value);
    /* Subsystems */
    public RobotState robotState = new RobotState();
    private final Swerve s_Swerve = new Swerve();
    private final Superstructure superstructure = new Superstructure();
    private final Elevator elevator = new Elevator();
    private final PivotySubsystem pivoty = new PivotySubsystem();
    private final BlinkinSubsystem blinkin = new BlinkinSubsystem();
    public final intakeSubsystem intake = new intakeSubsystem(robotState.IntakeState);
    private final LimelightSubsystem limelight = new LimelightSubsystem(NetworkTableInstance.getDefault().getTable("limelight"));
    // private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    // private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrainSubsystem);

    private Trajectory[] trajectories;
    // private final AutonomousChooser autonomousChooser;

    public RobotContainer() {
        // secondaryController.getLeftXAxis().setInverted(true);
        // secondaryController.getRightXAxis().setInverted(true);


        // CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));
        
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
        // secondaryController.getBackButton().onTrue(new ResetGyroCommand(drivetrainSubsystem));
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        blinkinButton.whileTrue(new BlinkinCommand( blinkin));
    }
//do new button bindings
   private void configureCoPilotButtonBindings() {

        elevatorButtonDown.onTrue(new ElevatorCommand(elevator, 0.6,3 ));
        elevatorButtonUp.onTrue(new ElevatorCommand(elevator, 0.6,1 ));
       // highRung.onTrue(new ScoringCommand(elevator, 0.8, pivoty, 0.8, 3, intake, 0.2, false, limelight));
       // lowerAndGrab.onTrue(new ScoringCommand(elevator, 0.8, pivoty, 0.8, 1, intake, 0.2, true, limelight));
        openIntake.onTrue(new intakeCommand(intake, 0.2, false, 1));
        closeIntake.onTrue(new intakeCommand(intake, 0.2, true, 1));
        PivotyDown.onTrue(new PivotyCommand(pivoty, 0.8, 20));
        PivotyUp.onTrue(new PivotyCommand(pivoty, 0.8, 349));

        toggleDriveTrainAutoBalance.onTrue(new toggleSwerveState(robotState)); // set it up for a toggleontrue later
       // elevatorUp.whileTrue(new ElevatorCommand(elevator, .8 , 3));
        //elevatorDown.whileTrue(new ElevatorCommand(elevator, 0.8, 1));
        //elevatorLeft.whileTrue(new PivotyCommand(pivoty, 0.8, 69420));
        //elevatorRight.whileTrue(new PivotyCommand(pivoty, -.8, -69420));
        // secondaryController.getDPadButton(Direction.UP).onTrue(new ElevatorCommand(elevator,0.7)); maybe find dpadbuttons
       // secondaryController.getDPadButton(Direction.DOWN).onTrue(new ElevatorCommand(elevator,-0.7));
        //secondaryController.getDPadButton(Direction.LEFT).onTrue(new PivotyCommand(pivoty,0.7));maybe find dpadbuttons
        //secondaryController.getDPadButton(Direction.RIGHT).onTrue(new PivotyCommand(pivoty,-0.7));

    }

     public Command getAutonomousCommand() {
    //     return autonomousChooser.getCommand(this);
    // }
    // // public Command getAutonomousCommand() {
         // An ExampleCommand will run in autonomous
         return new exampleAuto(s_Swerve);
     }


     public Swerve getDrivetrainSubsystem() {
         return s_Swerve;
     }
     public intakeSubsystem getIntakeSubsystem() {
        return intake;
    }
    public BlinkinSubsystem getBlinkinSubsystem() {
         return blinkin;
    }

   public Superstructure getSuperstructure() {
       return superstructure;
   }



    public Joystick getsecondaryController() {
        return secondaryController;
    }

   public Trajectory[] getTrajectories() {
       return trajectories;
   }
    

    // public AutonomousChooser getAutonomousChooser() {
    //     return autonomousChooser;
    // }


    // public VisionSubsystem getVisionSubsystem() {
    //     return visionSubsystem;
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


    // public VisionSubsystem getVisionSubsystem() {
    //     return visionSubsystem;
    // }


    // public AutonomousChooser getAutonomousChooser() {
    //     return autonomousChooser;
    // }





}
