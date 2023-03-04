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
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class PivotySubsystem extends SubsystemBase {
  /** Creates a new Elevator. */
  LazyTalonFX pivotyMotor1;
  LazyTalonFX pivotyMotor2;
  double pivotySpeed;

  DigitalInput breakBeamOne;
  public PivotySubsystem(){
    pivotyMotor1 = new LazyTalonFX(Constants.PIVOTY_MOTOR.id, Constants.PIVOTY_MOTOR.busName);
    pivotyMotor2 = new LazyTalonFX(Constants.PIVOTY_MOTOR_2.id, Constants.PIVOTY_MOTOR_2.busName);
    breakBeamOne = new DigitalInput(Constants.PIVOTY_BREAK_BEAM);
  //  breakBeamTwo = new DigitalInput(6);

  }
  public void pivotyOn(double pivotySpeed){
    if(!breakBeamOne.get()){
      pivotyMotor1.getSensorCollection().setIntegratedSensorPosition(0.0,0);
      pivotyMotor2.getSensorCollection().setIntegratedSensorPosition(0.0,0);
      System.out.println("it gets here");
    }
  

    
    // pivotyMotor1.set(ControlMode.Position, desiredEncoderValue);
    // pivotyMotor2.set(ControlMode.Position, desiredEncoderValue);
    pivotyMotor1.set(ControlMode.PercentOutput, .8);
    pivotyMotor2.set(ControlMode.PercentOutput, .8);

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
  public void pivotyBreakMode(){
    pivotyMotor1.setNeutralMode(NeutralMode.Brake);
    pivotyMotor2.setNeutralMode(NeutralMode.Brake);
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
