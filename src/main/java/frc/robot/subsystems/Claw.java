// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Claw extends SubsystemBase {
  CANSparkMax clawMotor;
  /** Creates a new Claw. */
  public Claw() {
    clawMotor = new CANSparkMax(Constants.INTAKE_OPENER_MOTOR.id, MotorType.kBrushless);
  }
  public void clawOpen(double speed){
    clawMotor.set(speed);
  }
  public void clawOff(){
    clawMotor.stopMotor();
  }
  public double getEncoderValue() {
    return clawMotor.getEncoder().getPosition();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
