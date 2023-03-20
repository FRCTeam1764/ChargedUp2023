// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.libraries.external.control.FeedforwardConstraint;
// import frc.robot.libraries.external.control.PidController;
import frc.robot.libraries.external.util.InterpolatingDouble;
import frc.robot.libraries.internal.LazyTalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer.InterpolateFunction;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class PivotySubsystem extends SubsystemBase {
  /** Creates a new Elevator. */
  LazyTalonFX pivotyMotor1;
  LazyTalonFX pivotyMotor2;
  double pivotySpeed;
  ArmFeedforward minFeedForward;
  ArmFeedforward maxFeedForward;
  public DigitalInput breakBeamOne;
  int encoderOffset;
  double minFeedforwardVelo;
  double maxFeedforwardVelo;
  double feedforward;
  PIDController pidController;
  
  public PivotySubsystem(){
    pidController = new PIDController(.00015, 0, .00003);
    pivotyMotor1 = new LazyTalonFX(Constants.PIVOTY_MOTOR.id, Constants.PIVOTY_MOTOR.busName);
    pivotyMotor2 = new LazyTalonFX(Constants.PIVOTY_MOTOR_2.id, Constants.PIVOTY_MOTOR_2.busName);
    // pivotyMotor1.config_kP(0, .002);
    // pivotyMotor2.config_kP(0, .002);
    // pivotyMotor1.config_kD(0, .0001);
    // pivotyMotor2.config_kD(0, .0001);
    minFeedForward = new ArmFeedforward(0.098267, 0.072146, 3.7608);
    maxFeedForward = new ArmFeedforward(0.12106, 0.1171, 3.6673);
    breakBeamOne = new DigitalInput(Constants.PIVOTY_BREAK_BEAM);
    encoderOffset = 115000;

    
  //  breakBeamTwo = new DigitalInput(6);
  }
  public void pivotyOn(int desiredEncoderValue, Elevator elevator){
    // if(desiredEncoderValue == 0) {
    //   pivotyMotor1.config_kP(0, .125);
    //   pivotyMotor2.config_kP(0, .125);
    //   pivotyMotor1.config_kD(0, 30.00);
    //   pivotyMotor2.config_kD(0, 30.00);
     
    // }
    // else if (desiredEncoderValue == 133000) {
    //   pivotyMotor1.config_kP(0, .121);
    //   pivotyMotor2.config_kP(0, .121);
    //   pivotyMotor1.config_kD(0, 120.00);  //previously 140 
    //   pivotyMotor2.config_kD(0, 120.00);
    // }
    // else {
    //   pivotyMotor1.config_kP(0, .145);
    //   pivotyMotor2.config_kP(0, .145); //previously .125
    //   pivotyMotor1.config_kD(0, 50.00);
    //   pivotyMotor2.config_kD(0, 50.00);
    // }
    
    minFeedforwardVelo = minFeedForward.calculate(getEncoderRadians(), .0002); //previously .0005
    maxFeedforwardVelo = maxFeedForward.calculate(getEncoderRadians(), .0002);
    feedforward= interpolate(minFeedforwardVelo, maxFeedforwardVelo, elevator.getEncoderValue());

    pivotyMotor1.setVoltage(feedforward+pidController.calculate(pivotyMotor1.getSelectedSensorPosition(), desiredEncoderValue));
    pivotyMotor2.setVoltage(feedforward+pidController.calculate(pivotyMotor1.getSelectedSensorPosition(), desiredEncoderValue));
    //System.out.println(pivotyMotor1.get()+", "+desiredEncoderValue);

    


 
    /* 
    if(pivotySpeed > 0 && breakBeamOne.get()){
    pivotyMotor.set(pivotySpeed);
    }else if(pivotySpeed < 0 && (thruboreEncoder.get() <= 106)){
    }else if(pivotySpeed < 0 && (thruboreEncoder.get() <= 106)){
      pivotyMotor.set(pivotySpeed);
    }else{
      pivotyMotor.set(0);
    }
    */
  }
  public void pivotyBreakMode(){
    pivotyMotor1.setNeutralMode(NeutralMode.Brake);
    pivotyMotor2.setNeutralMode(NeutralMode.Brake);
  }
  public void pivotyOff(){
    pivotyMotor1.set(ControlMode.PercentOutput, 0);
    pivotyMotor2.set(0);
    pivotyMotor1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 0, 0));
    pivotyMotor2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 0, 0));

  }
  public double getEncoderValue(){
    return pivotyMotor1.getSelectedSensorPosition();
  }
  public boolean getBrkBeam(){
    return breakBeamOne.get();
  }
  public void zeroEncoder(){
    pivotyMotor1.getSensorCollection().setIntegratedSensorPosition(0.0,0);
    pivotyMotor2.getSensorCollection().setIntegratedSensorPosition(0.0,0);
  }
  public double interpolate(double min, double max, double currentEncoder ){
    return (currentEncoder/120000)*(max-min)+min;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  public double getEncoderRadians(){
    return ((pivotyMotor1.getSelectedSensorPosition()-encoderOffset)*224)/(2*Math.PI);
  }
}

