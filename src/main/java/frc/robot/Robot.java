package frc.robot;




import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.libraries.NewSwerve.CTREConfigs;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.commands.BlinkinCommand;
// import frc.robot.commands.intakeCommand;
// import frc.robot.libraries.external.math.RigidTransform2;
// import frc.robot.libraries.external.math.Rotation2;
import frc.robot.libraries.external.robot.UpdateManager;
// import frc.robot.libraries.external.robot.drivers.Limelight;
import frc.robot.constants.Constants;


public class Robot extends TimedRobot {
   private static Robot instance = null;
   
  
   // (^ v ^) colors :D

    public static CTREConfigs ctreConfigs;


    //private Command m_autonomousCommand;
    CANSparkMax intakeMotor;
    public RobotContainer robotContainer = new RobotContainer();
    private UpdateManager updateManager = new UpdateManager(
            // robotContainer.getDrivetrainSubsystem()
    );

   public Robot() {
        intakeMotor = new CANSparkMax(Constants.INTAKE_OPENER_MOTOR, MotorType.kBrushless);
       instance = this;
   }


   public static Robot getInstance() {
       return instance;
   }


   @Override
   public void robotInit() {
       //grabberCone = new robotContainer.getGrabberConeSubsystem();
       //grabber = new robotContainer.getGrabberSubsystem();

        //CommandScheduler.getInstance().schedule(new BlinkinCommand(-.95, robotContainer.getBlinkinSubsystem()));


       updateManager.startLoop(5.0e-3);
       // robotContainer.getVisionSubsystem().setLedMode(Limelight.LedMode.OFF);
        ctreConfigs = new CTREConfigs();
        
        // CommandScheduler.getInstance().schedule(new BlinkinCommand(-.95, robotContainer.getBlinkinSubsystem()));
   }


   @Override
   public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        //handles intake motor clamping down
        if(robotContainer.getsecondaryController().getRawButton(XboxController.Button.kB.value)){
            intakeMotor.set(.2);
        }
        
        else if(robotContainer.getsecondaryController().getRawButton(XboxController.Button.kY.value)){
            intakeMotor.set(-0.2);
        }
        else{
            intakeMotor.set(0.0);
        }
/* */
         
   }


   @Override
   public void autonomousInit() {
    //    robotContainer.getDrivetrainSubsystem().resetPose(RigidTransform2.ZERO);
    //    robotContainer.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO);


       robotContainer.getAutonomousCommand().schedule();
   }


    @Override
    public void disabledPeriodic() {
        // robotContainer.getVisionSubsystem().setLedMode(Limelight.LedMode.OFF);
    }

   @Override
   public void teleopInit() {
   }
}
