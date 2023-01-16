// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libraries.internal.LazyTalonFX;

public class PivotySubsystem extends SubsystemBase {
  /** Creates a new Elevator. */
  LazyTalonFX pivotyMotor;
  double pivotySpeed;
  DigitalInput breakBeamOne;
  DigitalInput breakBeamTwo;

  public PivotySubsystem(){
    pivotyMotor = new LazyTalonFX(0);
    breakBeamOne = new DigitalInput(5);
    breakBeamTwo = new DigitalInput(6);
  }
  
  public void pivotyOn(double elevatorSpeed, int desiredEncoderValue){
    if(pivotySpeed > 0 && breakBeamOne.get()){
    pivotyMotor.set(pivotySpeed);
    }else if(pivotySpeed < 0 && breakBeamTwo.get()){
      pivotyMotor.set(pivotySpeed);
    }else{
      pivotyMotor.set(0);
    }
  }

  public void pivotyOff(){
    pivotyMotor.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
