package frc.robot;


import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;


import edu.wpi.first.wpilibj.I2C;
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
        
        CommandScheduler.getInstance().schedule(new BlinkinCommand(-.95, robotContainer.getBlinkinSubsystem()));
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
//    private static Robot instance = null;
//    private GrabberSubsystemCone grabberCone;
//    private GrabberSubsystem grabber;
//    //changing I2c port to match connection of color sensor
//    private final I2C.Port i2cPort = I2C.Port.kOnboard;
//    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
//    private final ColorMatch m_colorMatcher = new ColorMatch();
  
//    private final Color kYellowTarget = new Color(0.255, 0.211, 0.033);
//    private final Color kPurpleTarget = new Color(0.096, 0.033, 0.255);
   // (^ v ^) colors :D


//    private RobotContainer robotContainer = new RobotContainer();
//    private UpdateManager updateManager = new UpdateManager(
//         //    robotContainer.getDrivetrainSubsystem()      
//    );


//    public Robot() {
//        instance = this;
//    }


//    public static Robot getInstance() {
//        return instance;
//    }


//    @Override
//    public void robotInit() {
//        //grabberCone = new robotContainer.getGrabberConeSubsystem();
//        //grabber = new robotContainer.getGrabberSubsystem();



//        updateManager.startLoop(5.0e-3);
//        robotContainer.getVisionSubsystem().setLedMode(Limelight.LedMode.OFF);
//        m_colorMatcher.addColorMatch(kYellowTarget);
//        m_colorMatcher.addColorMatch(kPurpleTarget);
//    }


//    @Override
//    public void robotPeriodic() {
//        CommandScheduler.getInstance().run();
//        Color detectedColor = m_colorSensor.getColor();


//        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);


//        if(match.color == kYellowTarget){
//            grabberCone.grabberConeOn(kDefaultPeriod);


//        }
//        else if(match.color == kPurpleTarget){
//            grabber.grabberOn(kDefaultPeriod);
//        }
      
//    }


//    @Override
//    public void autonomousInit() {
//        robotContainer.getDrivetrainSubsystem().resetPose(RigidTransform2.ZERO);
//        robotContainer.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO);


//        robotContainer.getAutonomousCommand().schedule();
//    }


//    @Override
//    public void disabledPeriodic() {
//        robotContainer.getVisionSubsystem().setLedMode(Limelight.LedMode.OFF);
//    }


//    @Override
//    public void teleopInit() {
//    }
}
