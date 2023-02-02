// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libraries.internal.LazyTalonFX;
import static frc.robot.constants.Constants.*;

public class GrabberSubsystemCone extends SubsystemBase {
  /** Creates a new GrabberSubsystem. */
  LazyTalonFX grabberMotor;
  LazyTalonFX grabberMotorBack;
  // double grabberSpeed;

  public GrabberSubsystemCone() {
    grabberMotor = new LazyTalonFX(1,CANIVORE_NAME);
    grabberMotorBack = new LazyTalonFX(2,CANIVORE_NAME);
  }

  public void grabberConeOn(double grabberSpeed){
    grabberMotor.set(grabberSpeed);
    grabberMotorBack.set(grabberSpeed);
  }

  public void grabberConeOff(){
    grabberMotor.set(0);
    grabberMotorBack.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
