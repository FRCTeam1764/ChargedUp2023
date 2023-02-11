package frc.robot;




import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.libraries.NewSwerve.CTREConfigs;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.BlinkinCommand;
import frc.robot.libraries.external.math.RigidTransform2;
import frc.robot.libraries.external.math.Rotation2;
import frc.robot.libraries.external.robot.UpdateManager;
import frc.robot.libraries.external.robot.drivers.Limelight;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.GrabberSubsystemCone;


public class Robot extends TimedRobot {
   private static Robot instance = null;
   private GrabberSubsystemCone grabberCone;
   private GrabberSubsystem grabber;
  
   DigitalInput color = new DigitalInput(5); //5 is just a place holder
   int cubeColor = 0;
   int coneColor = 1;
   // (^ v ^) colors :D

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
       color.get();



       if(color = cubeColor){
           grabberCone.grabberConeOn(kDefaultPeriod);


       }
       else if(!color.get()){
           grabber.grabberOn(kDefaultPeriod);
       }
      
   }


   @Override
   public void autonomousInit() {
       robotContainer.getDrivetrainSubsystem().resetPose(RigidTransform2.ZERO);
       robotContainer.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO);


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
