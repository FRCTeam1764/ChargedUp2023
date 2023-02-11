// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.libraries.internal.LazyTalonFX;
import static frc.robot.constants.Constants.*;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  LazyTalonFX elevatorMotor1;

  LazyTalonFX elevatorMotor2;
  double elevatorSpeed;
  Encoder encoder;
  DigitalInput minExtend;
  DigitalInput maxExtend;
  DigitalInput midExtend;

  public Elevator(){
    elevatorMotor1 = new LazyTalonFX(0,Constants.CANIVORE_NAME);
    elevatorMotor2 = new LazyTalonFX(1, Constants.CANIVORE_NAME);
    elevatorMotor2.setInverted(true);
    elevatorMotor2.follow(elevatorMotor1);
    minExtend = new DigitalInput(Constants.MIN_EXTEND_BREAK_BEAM);
    maxExtend = new DigitalInput(Constants.MAX_EXTEND_BREAK_BEAM);
    midExtend = new DigitalInput(Constants.MID_EXTEND_BREAK_BEAM);

    
  }
  //needs fixed
  public void elevatorOn(double elevatorSpeed, int heightLevel){
    if(getBreakBeam(heightLevel).get()){
      elevatorMotor1.set(elevatorSpeed);
    }
    else{
      elevatorMotor1.set(0);
    }
  }

  public void elevatorOff(){
    elevatorMotor1.set(0);
    elevatorMotor2.set(0);

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  

  public DigitalInput getBreakBeam(int heightLevel){
    if(heightLevel==1){
      return minExtend;
    }
    if(heightLevel==2){
      return midExtend;
    }
    if(heightLevel==3){
      return maxExtend;
    }
    else{
      return null;
    }
  }
}
