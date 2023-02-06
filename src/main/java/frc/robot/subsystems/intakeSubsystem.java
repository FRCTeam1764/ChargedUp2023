// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class intakeSubsystem extends SubsystemBase {
  /** Creates a new intakeSubsystem. */
  SparkMaxAlternateEncoder intakeMotor;
  double openPosition;
  double closedPosition;

  
  public intakeSubsystem(double openPosition, double closedPosition) {
  this.intakeMotor = new SparkMaxAlternateEncoder(, , 42);
  this.openPosition = openPosition;
  this.losedPosition = openPosition;

  }
  public void intakeOn(double openPosition){
    intakeMotor.setPosition(openPosition);

  }
  public void intakeOff(double closePosition){
    intakeMotor.setPosition(closedPosition);

    //I DID IT :DDDDDDDDDDDDDDDDDD wahoo!
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
