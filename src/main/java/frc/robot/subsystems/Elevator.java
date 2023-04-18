// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.equation.VariableMatrix;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.libraries.internal.LazyTalonFX;
import frc.robot.state.ElevatorState;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  public LazyTalonFX elevatorMotor1;
  public LazyTalonFX elevatorMotor2;
  public PIDController pidController;
  public DigitalInput limitSwitch;
  public ArmFeedforward feedforward; 
  public ElevatorState elevatorState;
  public DigitalInput limitSwitch2;
  double elevatorOffset;
  int MaxElevatorEncoder =-120000;
  int negative;
//30.5 inches

  public Elevator(ElevatorState elevatorState) {
    //I dont know what im doing - aidan

    
    elevatorMotor1 = new LazyTalonFX(Constants.ELEVATOR_MOTOR.id, Constants.ELEVATOR_MOTOR.busName);
    elevatorMotor2 = new LazyTalonFX(Constants.ELAVATOR_MOTOR_2.id, Constants.ELAVATOR_MOTOR_2.busName);
    elevatorMotor2.follow(elevatorMotor1);
    limitSwitch = new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH);
    limitSwitch2 = new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH2);
    negative =1;

    this.elevatorState = elevatorState;
    //feedforward = new ArmFeedforward(0.1, 0.1,0.1 );//needs characterization

  }
  double desiredEncoder;
  public void elevatorOn(double desiredEncoderValue,double velocity){
    desiredEncoder = desiredEncoderValue;

   // double velocity = feedforward.calculate(getEncoderValueOffset(), 0.02);

//velocity +
//0.000002, 0, 1
//0.00025 - P VALUE
//0.00002 - P VALUE
//pidController = new PIDController(SmartDashboard.getNumber("Kp", 0), SmartDashboard.getNumber("Ki", 0), SmartDashboard.getNumber("Kd", 0));
//pidController = new PIDController(SmartDashboard.getNumber("elevator Kp", 0), SmartDashboard.getNumber("elevator Ki", 0), SmartDashboard.getNumber("elevator Kd", 0));
pidController = new PIDController(0.00002, 0, 0);
double variable = pidController.calculate(getEncoderValue(),desiredEncoderValue);
if(variable<0){
  negative = -1;
}
else{
  negative = 1;
}

 variable = negative*Math.min(7.2, Math.abs(variable));
//0.00002 P, NO OTHER VALUES
SmartDashboard.putNumber("elevatorPID",variable);
SmartDashboard.putNumber("elevatorSetpoint", desiredEncoderValue);
// elevatorMotor1.neutralOutput();
// elevatorMotor2.neutralOutput();
// elevatorMotor1.setNeutralMode(NeutralMode.Coast);
// elevatorMotor2.setNeutralMode(NeutralMode.Coast);
      elevatorMotor1.set(ControlMode.PercentOutput,variable);  
      elevatorMotor2.set(ControlMode.PercentOutput,variable);  
    
  }
  public double getDesiredEncoder(){
    return desiredEncoder;
  }

  public void zeroEncoder(){
    elevatorMotor1.getSensorCollection().setIntegratedSensorPosition(0.0,0);
    elevatorMotor2.getSensorCollection().setIntegratedSensorPosition(0.0,0);
  }
  
  public boolean getLimitSwitch(){
    return limitSwitch.get();
  }
  public boolean getLimitSwitch2(){
    return limitSwitch2.get();
  }
  public void elevatorOff(){


    // elevatorMotor1.set(ControlMode.PercentOutput, 0);
    // elevatorMotor2.set(0);
    // elevatorMotor1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 0, 0));
    // elevatorMotor2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 0, 0));

  }




  public double getEncoderValue(){
     return elevatorMotor1.getSelectedSensorPosition();
   }

   //8.597094609770005

   
   public double getEncoderValueTranslatedToInches(){
    return (elevatorMotor1.getSelectedSensorPosition() -  elevatorOffset) * 2048*9*(1/(8.597094609770005*Math.PI)); //figure out offset one day 
  }

  public double getEncoderValueOffset(){
     return elevatorMotor1.getSelectedSensorPosition() -  elevatorOffset; //may need to convert to a unit?
   }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
