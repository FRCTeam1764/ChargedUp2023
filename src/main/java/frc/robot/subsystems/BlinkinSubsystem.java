// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlinkinSubsystem extends SubsystemBase {
  /** Creates a new BlinkinSubsystem. */
  Spark blinkin;

  public BlinkinSubsystem() {
    //:D wahoo!
    blinkin = new Spark(1);
  }
  //zach is so very cool :D (better than aiden and sawyer) he's so cool :3
  public void setLEDs(double LEDColor){

    blinkin.set(LEDColor);
  }
//i did it :DDDDDDDDDDDDDDDDDDD

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
