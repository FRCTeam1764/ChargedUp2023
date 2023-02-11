// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlinkinSubsystem extends SubsystemBase {
  /** Creates a new BlinkinSubsystem. */
  Spark blinkin;
  // double LEDColor;
  double LEDColor;

  public BlinkinSubsystem() {
    //:D wahoo!
    blinkin = new Spark(1);
    // this.LEDColor = LEDColor;
    
  }
  //zach is so very cool :D (better than aiden and sawyer) he's so cool :3
  public void setLEDs(){
    System.out.println("set LEDs");
    blinkin.set(getLEDColor());
  }
//i did it :DDDDDDDDDDDDDDDDDDD
  public void setLEDColor() {
      if(LEDColor == .67){
        LEDColor =.89;
      }
      if(LEDColor == .89){
        LEDColor = .67;
      }
  }
  public double getLEDColor(){
    if(LEDColor==.89){
      return .67;
    }
    if(LEDColor == .67){
      return .89;
    }
    else{
      return (Double) null;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
