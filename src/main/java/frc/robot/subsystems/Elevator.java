// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libraries.internal.LazyTalonFX;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  LazyTalonFX elevatorMotor;
  double elevatorSpeed;
  Encoder encoder;

  public Elevator(){
    elevatorMotor = new LazyTalonFX(0);
    encoder = new Encoder(1,2);
  }
  
  public void elevatorOn(double elevatorSpeed, int desiredEncoderValue){
    if(encoder.get() <= desiredEncoderValue && desiredEncoderValue > 10000){
    elevatorMotor.set(elevatorSpeed);
    }else if(encoder.get() >= desiredEncoderValue){
    elevatorMotor.set(elevatorSpeed);
    }else {
      elevatorMotor.set(0);
    }
  }

  public void elevatorOff(){
    elevatorMotor.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
