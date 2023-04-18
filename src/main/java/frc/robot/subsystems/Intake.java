// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import com.fasterxml.jackson.annotation.JacksonInject.Value;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.libraries.external.control.PidController;
import frc.robot.libraries.internal.RevThroughBoreEncoder;
// import frc.robot.constants.GlobalConstants.IntakeConstants;
// import frc.robot.trobot5013lib.RevThroughBoreEncoder;
import frc.robot.RobotContainer;
//0.44298 Ks
//

/** Add your docs here. */
public class Intake extends SubsystemBase {
    private final CANSparkMax m_flexMotor = new CANSparkMax(Constants.WRIST_MOTOR.id, MotorType.kBrushless);
    private final CANSparkMax m_intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR.id, MotorType.kBrushless);
    // private final PIDController m_flexPIDController = new PIDController(1.1, 0, 0.05);
    private final SparkMaxAbsoluteEncoder m_angleEncoder = m_flexMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);// new RevThroughBoreEncoder(Constants.WRIST_ANGLE_ENCODER);
private PIDController m_flexPidController;
    private ArmFeedforward m_feedForward = new ArmFeedforward(0.11202, 0.11202,2.0024); 
    private ArmFeedforward down_Feedforward = new ArmFeedforward(0.15488,7.1406E+15 , 1.9548);
    private ArmFeedforward up_Feedforward = new ArmFeedforward(0.15245, 0.16345, 1.9801);
    private PivotySubsystem pivoty;
    private DigitalInput breakBeamCube;
    private DigitalInput breakBeamCone;

    public Intake(PivotySubsystem pivoty) {
        super();
        m_intakeMotor.restoreFactoryDefaults();
        m_flexMotor.restoreFactoryDefaults();
        m_flexMotor.setInverted(false);
        // m_flexMotor.setInverted(true);
        m_flexMotor.setIdleMode(IdleMode.kBrake);
        m_angleEncoder.setPositionConversionFactor(360);
        m_angleEncoder.setZeroOffset(140);//150
        m_angleEncoder.setInverted(true);
        this.pivoty = pivoty;
        breakBeamCube = new DigitalInput(Constants.INTAKE_BREAK_BEAM1);
        breakBeamCone = new DigitalInput(Constants.INTAKE_BREAK_BEAM2);

Timer timer = new Timer();
    } 

    public double getAngleRadians() {
        return ((m_angleEncoder.getPosition()*Math.PI)/180);
    }
    double negative;
    public void wheelsCube(double speed) {
        
        if(speed<0){
            negative=-1;
        }
        else{
            negative=1;
        }
        // if(speed<0 && (!breakBeamCone.get() || !breakBeamCube.get())){
        //     m_intakeMotor.set(-.2);
        // }

        // else if(!breakBeamCube.get()||!breakBeamCone.get() ){
        //     m_intakeMotor.set(.05);
        // }
        SmartDashboard.putNumber("intake speed", m_intakeMotor.get());
        if(!breakBeamCube.get()){
            m_intakeMotor.set(.04*negative);
        }
        else{
        m_intakeMotor.set(speed);
        }

    }
    public void wheelsCone(double speed){
        
        if(speed<0){
            negative=-1;
        }
        else{
            negative=1;
        }
        // if(speed<0 && (!breakBeamCone.get() || !breakBeamCube.get())){
        //     m_intakeMotor.set(-.2);
        // }

        // else if(!breakBeamCube.get()||!breakBeamCone.get() ){
        //     m_intakeMotor.set(.05);
        // }
        SmartDashboard.putNumber("intake speed", m_intakeMotor.get());
        if(!breakBeamCone.get()){
            m_intakeMotor.set(-.15*negative);
        }
        else{
        m_intakeMotor.set(-speed);
        }

    }


    public void run(double speed){
    m_intakeMotor.set(speed);
}
   
    public void stop() {
        m_intakeMotor.set(0);
    }
    private PIDController pid;
    double pidValue;
    double setpoint;
    public void flexClosedLoop() {
       // pid = new PIDController(SmartDashboard.getNumber("intake Kp", 0), SmartDashboard.getNumber("intake Ki", 0), SmartDashboard.getNumber("intake Kd", 0));
       pid = new PIDController(12, 0, 0.001);
       // pid.enableContinuousInput(-Math.PI,  Math.PI);
        //0.0008 D, 012 P, 0.1 V, 140 Offset
        // SmartDashboard.putNumber("Flex velocity", velocity);
        // SmartDashboard.putNumber("pivoty Radian", pivoty.getEncoderRadiansIntake());
        // System.out.println(17.5-((pivoty.getEncoderRadiansIntake()*180)/Math.PI));
        
        
        // double upfeedForward = m_feedForward.calculate(((213*Math.PI/180+((pivoty.getEncoderRadiansIntake())))) ,SmartDashboard.getNumber("Feed forward velo", 0));
        double upfeedForward = m_feedForward.calculate(getAngleRadians(),0.1);
        // double downfeedForward = down_Feedforward.calculate(((85.7+((pivoty.getEncoderRadiansIntake())))) ,SmartDashboard.getNumber("Feed forward velo", 0));

        // double interoplated = interpolate(upfeedForward, downfeedForward, pivoty.getEncoderValue());
// SmartDashboard.putNumber("interoplated feedfoward", interoplated);
        setpoint = (((213*Math.PI)/180)-pivoty.getEncoderRadiansIntake());
        SmartDashboard.putNumber("setpoint", setpoint*180/Math.PI);
        pid.setSetpoint(setpoint);
        SmartDashboard.putNumber("getAngleRadians", getAngleRadians());
  
        pidValue = pid.calculate(getAngleRadians()); //calculate feed forward
        if(Math.abs(setpoint - getAngleRadians())< .0175){
         pidValue = 0;   
        }
        SmartDashboard.putNumber("intakePID", pidValue);
  SmartDashboard.putNumber("totalMotorSet", pidValue+upfeedForward);
        m_flexMotor.setVoltage(Math.min(pidValue+upfeedForward,6));
    }
    //234902788.15639588
//160, 0.4 - p, 0.005-d, 0.8 velo
    public double getGroundRelativeWristPossitionRadians(){
        return  getAngleRadians()+pivoty.getEncoderRadiansIntake();
    }
    public double interpolate(double min, double max, double currentEncoder ){
        return (currentEncoder/140000)*(max-min)+min;
      }
//m_angleEncoder.isConnected() && pivoty.isArmEncoderConnected() && 


// public DigitalInput getBreakBeam1(){


// }

    @Override
    public void periodic(){
        //  PIDController m_flexPIDController = new PIDController(SmartDashboard.getNumber("intake Kp", 0), SmartDashboard.getNumber("intake Ki", 0), SmartDashboard.getNumber("intake Kd", 0));
        //  m_flexPIDController.enableContinuousInput(0, 2 * Math.PI);
        //m_angleEncoder.setZeroOffset(SmartDashboard.getNumber("IntakeOffset",0));
m_angleEncoder.setZeroOffset(140); // do not remove 
//SmartDashboard.putBoolean("intake1", breakBeamCone.get());
        // SmartDashboard.putBoolean("intake2", breakBeamCube.get());
        // SmartDashboard.putNumber("pivotyEncoderRadians", pivoty.getEncoderRadiansIntake());
        //  SmartDashboard.putNumber("WristAngle",((m_angleEncoder.getPosition())));
        // // System.out.println()

  //       SmartDashboard.putNumber("WriseAngleGround",Units.radiansToDegrees(getGroundRelativeWristPossitionRadians())); 
        // if (
        //    pivoty.getEncoderRadians() < Units.degreesToRadians(85) || pivoty.getEncoderRadians() > Units.degreesToRadians(200) ){
            // m_flexPIDController.setTolerance(Rotation2d.fromDegrees(2).getRadians());
            // m_flexPIDController.setSetpoint(Rotation2d.fromDegrees(17).getRadians());

            flexClosedLoop();
       // }
    }
    
 }
//there was a "isconnected" for m_angleencoder