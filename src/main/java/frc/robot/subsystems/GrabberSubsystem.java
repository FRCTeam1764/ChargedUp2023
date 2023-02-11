// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.libraries.internal.LazyTalonFX;
import static frc.robot.constants.Constants.*;

public class GrabberSubsystem extends SubsystemBase {
  /** Creates a new GrabberSubsystem. */
  LazyTalonFX grabberMotor;
  // double grabberSpeed;

  //public GrabberSubsystem() {
    //grabberMotor = new LazyTalonFX(Constants.SIDE_INTAKE_MOTOR);
  //}

  public void grabberOn(double grabberSpeed){
    grabberMotor.set(grabberSpeed);
  }

  public void grabberOff(){
    grabberMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
