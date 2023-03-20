// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.libraries.external.control.FeedforwardConstraint;
import frc.robot.libraries.external.control.PidController;
import frc.robot.libraries.internal.LazyTalonFX;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  public LazyTalonFX elevatorMotor1;
  public LazyTalonFX elevatorMotor2;
  public PidController pidController;
  public DigitalInput limitSwitch;
  public ArmFeedforward feedforward; 

  public Elevator() {
    elevatorMotor1 = new LazyTalonFX(Constants.ELEVATOR_MOTOR.id, Constants.ELEVATOR_MOTOR.busName);
    elevatorMotor2 = new LazyTalonFX(Constants.ELAVATOR_MOTOR_2.id, Constants.ELAVATOR_MOTOR_2.busName);
    elevatorMotor2.follow(elevatorMotor1);
    limitSwitch = new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH);


    
  }

  public void elevatorOn(){
    if(!limitSwitch.get()){
      elevatorMotor1.setSelectedSensorPosition(0);
    }
    
  }
  public void elevatorOff(){

  }




  public double getEncoderValue(){
     return elevatorMotor1.getSelectedSensorPosition();
   }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
