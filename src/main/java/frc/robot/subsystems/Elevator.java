// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
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
  double elevatorOffset;
  int MaxElevatorEncoder =-120000;

  public Elevator(ElevatorState elevatorState) {
    //I dont know what im doing - aidan
    
    elevatorMotor1 = new LazyTalonFX(Constants.ELEVATOR_MOTOR.id, Constants.ELEVATOR_MOTOR.busName);
    elevatorMotor2 = new LazyTalonFX(Constants.ELAVATOR_MOTOR_2.id, Constants.ELAVATOR_MOTOR_2.busName);
    elevatorMotor2.follow(elevatorMotor1);
    limitSwitch = new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH);

    this.elevatorState = elevatorState;

    pidController = new PIDController(.001, 0, .001); //need tunin
    feedforward = new ArmFeedforward(0.1, 0.1,0.1 );//needs characterization

  }

  public void elevatorOn(double desiredEncoderValue){


    double velocity = feedforward.calculate(getEncoderValueOffset(), 0.02);


    elevatorMotor1.setVoltage(velocity + pidController.calculate(getEncoderValue(),-desiredEncoderValue));
    elevatorMotor2.setVoltage(velocity + pidController.calculate(getEncoderValue(),-desiredEncoderValue));
    
  }

  public void zeroEncoder(){
    elevatorMotor1.getSensorCollection().setIntegratedSensorPosition(0.0,0);
    elevatorMotor2.getSensorCollection().setIntegratedSensorPosition(0.0,0);
  }
  
  public boolean getLimitSwitch(){
    return limitSwitch.get();
  }
  public void elevatorOff(){
    elevatorMotor1.set(ControlMode.PercentOutput, 0);
    elevatorMotor2.set(0);
    elevatorMotor1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 0, 0));
    elevatorMotor2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 0, 0));

  }




  public double getEncoderValue(){
     return elevatorMotor1.getSelectedSensorPosition();
   }

  public double getEncoderValueOffset(){
     return elevatorMotor1.getSelectedSensorPosition() -  elevatorOffset; //may need to convert to a unit?
   }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
