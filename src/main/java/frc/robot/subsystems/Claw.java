// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Claw extends SubsystemBase {
  CANSparkMax clawMotor;
  private static final double INTAKE_OPENER_MOTOR_P = 0.0001;
  private static final double INTAKE_OPENER_MOTOR_D = 0.0000;
  private PIDController pidController = new PIDController(INTAKE_OPENER_MOTOR_P, 0, INTAKE_OPENER_MOTOR_D);

  /** Creates a new Claw. */
  public Claw() {
    clawMotor = new CANSparkMax(Constants.INTAKE_OPENER_MOTOR.id, MotorType.kBrushless);
    clawMotor.restoreFactoryDefaults();
    clawMotor.setSmartCurrentLimit(40);
    clawMotor.getEncoder().setPosition(0);
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
  public void zeroEncoder(){
    clawMotor.getEncoder().setPosition(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSetpoint(double setpoint) {
    pidController.setSetpoint(setpoint);
  }

  public void closeClaw() {
    clawMotor.set(pidController.calculate(getEncoderValue(), 1));
  }

  public void openClaw() {
    clawMotor.set(pidController.calculate(getEncoderValue(), 0));
  }
}
