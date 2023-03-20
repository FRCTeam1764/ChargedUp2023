package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.libraries.external.control.Trajectory;
import frc.robot.state.PivotyState;
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
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton blinkinButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton elevatorTest = new JoystickButton(driver, XboxController.Button.kA.value);
    // private final JoystickButton highButton = new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton lowButton = new JoystickButton(driver, XboxController.Button.kA.value);
    // private final JoystickButton midButton = new JoystickButton(driver, XboxController.Button.kX.value);
    //private final JoystickButton limelight1 = new JoystickButton(driver, XboxController.Button.kA.value);
    //private final JoystickButton limelight2 = new JoystickButton(driver, XboxController.Button.kB.value);

    /* CoPilot Buttons */
    private final JoystickButton highRung = new JoystickButton(secondaryController, XboxController.Button.kY.value);
    private final JoystickButton midRung = new JoystickButton(secondaryController, XboxController.Button.kX.value);
    private final JoystickButton lowRung = new JoystickButton(secondaryController, XboxController.Button.kA.value);
    private final JoystickButton intakeClose = new JoystickButton(secondaryController, XboxController.Button.kRightBumper.value);
    private final JoystickButton intakeZero = new JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value);

    private final JoystickButton playerStation = new JoystickButton(secondaryController, XboxController.Button.kB.value);

    /* Subsystems */
    public RobotState robotState = new RobotState(driver);
    private final Swerve s_Swerve = new Swerve();
    private final Superstructure superstructure = new Superstructure();
    private final Elevator elevator = new Elevator();
    private final PivotySubsystem pivoty = new PivotySubsystem();
    private final BlinkinSubsystem blinkin = new BlinkinSubsystem();
    private final Claw claw = new Claw();
    private final SideRollers sideRollers = new SideRollers();
    private final BackRollers backRollers = new BackRollers();

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

        configurePilotButtonBindings();
        configureCoPilotButtonBindings();
        // autonomousChooser = new AutonomousChooser(trajectories);
    }


    private void configurePilotButtonBindings() {
        // secondaryController.getBackButton().onTrue(new ResetGyroCommand(drivetrainSubsystem));
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        blinkinButton.toggleOnTrue(new BlinkinCommand(blinkin));
        elevatorTest.toggleOnTrue(new ElevatorCommand(elevator, .6, 3));

        // highButton.onTrue(new ElevatorCommand(elevator, .6, 3));
        // midButton.onTrue(new ElevatorCommand(elevator, .6, 2));
        // lowButton.onTrue(new ElevatorCommand(elevator, .6, 1));
        // pivotyUp.whileTrue(new PivotyCommand(pivoty, 16, robotState.pivotyState));
        // pivotyDown.whileTrue(new PivotyCommand(pivoty, 7, robotState.pivotyState));
    }
//do new button bindings
   private void configureCoPilotButtonBindings() {

        // lowRung.onTrue(new ScoringCommand(elevator, 0.8, pivoty, 0.8, 1, intake, 0.2, false, limelight, robotState.pivotyState));
        
        // midRung.onTrue(new ScoringCommand(elevator, 0.8, pivoty, 0.8, 2, intake, 0.2, false, limelight, robotState.pivotyState));
        // highRung.onTrue(new ScoringCommand(elevator, 0.8, pivoty, 0.8, 3, intake, 0.2, false, limelight, robotState.pivotyState));
        // lowerAndGrab.onTrue(new ScoringCommand(elevator, 0.8, pivoty, 0.8, 1, intake, 0.2, true, limelight, robotState.pivotyState));
        // openIntake.onTrue(new intakeCommand(intake, 0.2, false, 1));
        // intakeClose.onTrue(new IntakeCommand(claw, -.2, sideRollers, backRollers, -1, blinkin));
        intakeClose.onTrue(new OutakeCommandGroup(backRollers, sideRollers, claw, 1));
        intakeClose.onFalse(new IntakeCommand(claw, sideRollers, backRollers, -1.0, blinkin));
        intakeZero.whileTrue(new ZeroIntakeCommand(claw, .2));
        // intakeOpen.onTrue(new OutakeCommand(claw, .2, sideRollers, backRollers, 1));
        lowRung.toggleOnTrue(new ElevatorPivotyCommandGroup(pivoty, 130000, robotState.pivotyState, elevator, .6, 1,sideRollers,backRollers,-1));
        midRung.toggleOnTrue(new ElevatorPivotyCommandGroup(pivoty, 50000, robotState.pivotyState, elevator, .6, 2,sideRollers,backRollers,-1));
        highRung.toggleOnTrue(new ElevatorPivotyCommandGroup(pivoty, 50000, robotState.pivotyState, elevator, .6, 3,sideRollers,backRollers,-1));//previosuly 75k
        // playerStation.toggleOnTrue(new ElevatorPivotyCommandGroup(elevator, .6, pivoty, 4, robotState.pivotyState));
        playerStation.toggleOnTrue(new ElevatorPivotyCommandGroup(pivoty, 50000, robotState.pivotyState, elevator, .6, 1,sideRollers,backRollers,-1));
        // toggleDriveTrainAutoBalance.onTrue(new toggleSwerveState(robotState)); // set it up for a toggleontrue later
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
         return new AutoBalance(s_Swerve, robotState,claw);
     }


     public Swerve getDrivetrainSubsystem() {
         return s_Swerve;
     }

    public BlinkinSubsystem getBlinkinSubsystem() {
         return blinkin;
    }

   public Superstructure getSuperstructure() {
       return superstructure;
   }

   public Elevator getElevatorSubsystem(){
    return elevator;
   }

   public PivotySubsystem getPivotySubsystem(){
    return pivoty;
   }


    public Joystick getsecondaryController() {
        return secondaryController;
    }

    public Joystick getPrimaryController() {
        return driver;
    }

   public Trajectory[] getTrajectories() {
       return trajectories;
   }
   public Claw getClaw(){
    return claw;
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
