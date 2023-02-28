// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.libraries.internal.LazyTalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class PivotySubsystem extends SubsystemBase {
  /** Creates a new Elevator. */
  TalonFX pivotyMotor1;
  LazyTalonFX pivotyMotor2;
  double pivotySpeed;

  DigitalInput breakBeamOne;
  public PivotySubsystem(){
    pivotyMotor1 = new LazyTalonFX(0, Constants.CANIVORE_NAME);
    pivotyMotor2 = new LazyTalonFX(2, Constants.CANIVORE_NAME);
    breakBeamOne = new DigitalInput(Constants.PIVOTY_BREAK_BEAM);
  //  breakBeamTwo = new DigitalInput(6);
  }
  public void pivotyOn(double pivotySpeed, int desiredEncoderValue){
    // if(!breakBeamOne.get()){
    //   pivotyMotor1.getSensorCollection().setIntegratedSensorPosition(0.0,0);
    //   pivotyMotor2.getSensorCollection().setIntegratedSensorPosition(0.0,0);

    // }

    
    pivotyMotor1.set(ControlMode.Position, desiredEncoderValue);
    pivotyMotor2.set(ControlMode.Position, desiredEncoderValue);

    SmartDashboard.putNumber("pivoty encoder",pivotyMotor1.getSelectedSensorPosition());


 
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
    pivotyMotor1.set(ControlMode.PercentOutput, 0);
    pivotyMotor2.set(0);

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
