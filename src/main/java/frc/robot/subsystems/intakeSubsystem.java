// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class intakeSubsystem extends SubsystemBase {
  /** Creates a new intakeSubsystem. */

   DoubleSolenoid intakeSolenoid;

  public intakeSubsystem() {

    intakeSolenoid = new DoubleSolenoid(1, null, Constants.INTAKE_SOLENOID_FORWARD, Constants.INTAKE_SOLENOID_REVERSE);
  }
  public void intakeOn(double speed){
    intakeSolenoid.set(Value.kForward);

  }
  public void intakeOff(){
    intakeSolenoid.set(Value.kReverse);

    //I DID IT :DDDDDDDDDDDD wahoo!
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
