// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libraries.internal.LazyTalonFX;
import edu.wpi.first.wpilibj.Encoder;
import static frc.robot.constants.Constants.*;

public class PivotySubsystem extends SubsystemBase {
  /** Creates a new Elevator. */
  LazyTalonFX pivotyMotor1;
  LazyTalonFX pivotyMotor2;
  double pivotySpeed;
  Encoder thruboreEncoder;
  //needs fixed
  public PivotySubsystem(){
    pivotyMotor1 = new LazyTalonFX(0);
    pivotyMotor2 = new LazyTalonFX(2);
    breakBeamOne = new DigitalInput(5);
  //  breakBeamTwo = new DigitalInput(6);
  }
  public void pivotyOn(double pivotySpeed, int desiredEncoderValue){
    if(!breakBeamOne.get()){
      pivotyMotor1.configIntegratedSensorOffset(0);
      pivotyMotor2.configIntegratedSensorOffset(0);

    }
    pivotyMotor1.set(pivotySpeed);
    pivotyMotor2.set(pivotySpeed);


 
    /* 
    if(pivotySpeed > 0 && breakBeamOne.get()){
    pivotyMotor.set(pivotySpeed);
    }else if(pivotySpeed < 0 && (thruboreEncoder.get() <= 106)){
    }else if(pivotySpeed < 0 && (thruboreEncoder.get() <= 106)){
      pivotyMotor.set(pivotySpeed);
    }else{
      pivotyMotor.set(0);
    }
    */
  }

  public void pivotyOff(){
    pivotyMotor1.set(0);
    pivotyMotor2.set(0);

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
