package frc.robot;




import com.fasterxml.jackson.databind.util.RootNameLookup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
   Spark blinkin;
   
  
   // (^ v ^) colors :D

    public static CTREConfigs ctreConfigs;


    //private Command m_autonomousCommand;
    public RobotContainer robotContainer;
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
       RobotController.setBrownoutVoltage(7.0);
         robotContainer = new RobotContainer();
         blinkin = new Spark(Constants.BLINKIN_SPARK);
          SmartDashboard.putNumber("auto length", 4.7);
          SmartDashboard.putNumber("auto width", .7);
          SmartDashboard.putNumber("auto extend", 20000);

        //  SmartDashboard.putNumber("elevator Kp", 0);
        //  SmartDashboard.putNumber("elevator Kd", 0);
        //  SmartDashboard.putNumber("elevator Ki", 0);
//          SmartDashboard.putNumber("intake Kp", 0);
//          SmartDashboard.putNumber("intake Kd", 0);
//          SmartDashboard.putNumber("intake Ki", 0);
//          SmartDashboard.putNumber("Feed forward velo", 0);
//          SmartDashboard.putNumber("Wrist setpoint", 0);
// SmartDashboard.putNumber("IntakeOffset", 0);

//            SmartDashboard.putNumber("HighPivoty", 0);
        //   SmartDashboard.putNumber("HPSPivoty", 0);
        //   SmartDashboard.putNumber("MidPivoty", 0);
        //   SmartDashboard.putNumber("LowPivoty", 0);

        //   SmartDashboard.putNumber("High Elevator", 0);
        //   SmartDashboard.putNumber("HPS Elevator", 0);
        //   SmartDashboard.putNumber("Mid Elevator", 0);
        //   SmartDashboard.putNumber("Low elevator", 0);
        

        //CommandScheduler.getInstance().schedule(new BlinkinCommand(-.95, robotContainer.getBlinkinSubsystem()));
        robotContainer.getDrivetrainSubsystem().getNavx().calibrate();
        robotContainer.getDrivetrainSubsystem().zeroGyro();

       updateManager.startLoop(5.0e-3);
       robotContainer.getPivotySubsystem().zeroEncoder();
       robotContainer.getElevatorSubsystem().zeroEncoder();

      // robotContainer.robotState.elevatorState.setEncoderValue(-4000);
       // robotContainer.getVisionSubsystem().setLedMode(Limelight.LedMode.OFF);

        
        // CommandScheduler.getInstance().schedule(new BlinkinCommand(-.95, robotContainer.getBlinkinSubsystem()));
   }


   @Override
   public void robotPeriodic() {

        CommandScheduler.getInstance().run();
        if(!robotContainer.getPivotySubsystem().breakBeamOne.get()){
            robotContainer.getPivotySubsystem().zeroEncoder();
        }

        runPivoty();
        runElevator();

        blinkin.set(robotContainer.getBlinkinSubsystem().getLEDColor());

        if(!robotContainer.getPivotySubsystem().breakBeamOne.get()){
             robotContainer.getPivotySubsystem().zeroEncoder();
         }

         if(!robotContainer.getElevatorSubsystem().limitSwitch.get() || !robotContainer.getElevatorSubsystem().limitSwitch2.get() ){
            robotContainer.getElevatorSubsystem().zeroEncoder();
        }

        // SmartDashboard.putBoolean("mid", robotContainer.getElevatorSubsystem().midExtend.get() );
        // SmartDashboard.putBoolean("max", robotContainer.getElevatorSubsystem().maxExtend.get() );
        // SmartDashboard.putBoolean("min", robotContainer.getElevatorSubsystem().minExtend.get() );
        SmartDashboard.putBoolean("pivoty", robotContainer.getPivotySubsystem().getBrkBeam());
        // SmartDashboard.putBoolean("elevator", robotContainer.getElevatorSubsystem().getLimitSwitch());
        // SmartDashboard.putNumber("elevator encoder", robotContainer.getElevatorSubsystem().getEncoderValue());
        // SmartDashboard.putNumber("DesiredPivoty", robotContainer.robotState.pivotyState.getEncoderValue());

    //     SmartDashboard.putNumber("velocity", robotContainer.robotState.elevatorState.getVelocity());
    //     SmartDashboard.putBoolean("elevator2", robotContainer.getElevatorSubsystem().getLimitSwitch2());
    //     // System.out.println(robotContainer.getPivotySubsystem().getBrkBeam());
    //     SmartDashboard.putNumber("pivoty encoder", robotContainer.getPivotySubsystem().getEncoderValue());
    //    // SmartDashboard.putNumber("Intake Encoder", robotContainer.getClaw().getEncoderValue());
    //     SmartDashboard.putNumber("desired elevator", robotContainer.robotState.elevatorState.getEncoderValue());
        // System.out.println(robotContainer.robotState.pivotyState.getEncoderValue());

/* */
         
   }


   @Override
   public void autonomousInit() {
    //    robotContainer.getDrivetrainSubsystem().resetPose(RigidTransform2.ZERO);
    //    robotContainer.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO);


        robotContainer.getAutonomousCommand().schedule();//fix later
   }


    @Override
    public void disabledPeriodic() {
        // robotContainer.getVisionSubsystem().setLedMode(Limelight.LedMode.OFF);
    }

   @Override
   public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
   }

   public void runPivoty(){
 //   robotContainer.getPivotySubsystem().pivotyOn(robotContainer.robotState.pivotyState.getEncoderValue(), robotContainer.getElevatorSubsystem());
   }

   public void runElevator(){
    robotContainer.getElevatorSubsystem().elevatorOn(robotContainer.robotState.elevatorState.getEncoderValue(),robotContainer.robotState.elevatorState.getVelocity());

   }
}
