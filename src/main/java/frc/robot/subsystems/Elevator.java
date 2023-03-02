// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.libraries.internal.LazyTalonFX;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  LazyTalonFX elevatorMotor1;

  LazyTalonFX elevatorMotor2;
  double elevatorSpeed;
  Encoder encoder;
  DigitalInput minExtend;
  DigitalInput maxExtend;
  DigitalInput midExtend;
  DigitalInput no;
  int previousHeightLevel;
  double Reverse = 1.0;
boolean BreakBeamOffOrOn = false;
  public Elevator(){
    elevatorMotor1 = new LazyTalonFX(0,Constants.CANIVORE_NAME);
    elevatorMotor2 = new LazyTalonFX(1, Constants.CANIVORE_NAME);
    elevatorMotor2.setInverted(true);
    elevatorMotor2.follow(elevatorMotor1);
    minExtend = new DigitalInput(Constants.MIN_EXTEND_BREAK_BEAM);
    maxExtend = new DigitalInput(Constants.MAX_EXTEND_BREAK_BEAM);
    midExtend = new DigitalInput(Constants.MID_EXTEND_BREAK_BEAM);

    
  }


  //Elevator goes to the breakbeam specified via "heightlevel", speed can be negative or positive
  public void elevatorOn(double elevatorSpeed){
    elevatorMotor1.set(elevatorSpeed);
    elevatorMotor2.set(elevatorSpeed);
 
  }

  public void elevatorOff(){
    elevatorMotor1.set(0);
    elevatorMotor2.set(0);
    System.out.println("min -"+minExtend.get()); 
    System.out.println("mid -"+midExtend.get()); 
    System.out.println("max -"+maxExtend.get()); 
  }
  

  public boolean IsWantedHeight(int heightLevel) {
    if (previousHeightLevel == heightLevel){
      return true;
    }
    return false;
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
      return no;
    }
  }
  //assigning height levels numbers, numbers cooralate to min, mid, and max.

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

}
