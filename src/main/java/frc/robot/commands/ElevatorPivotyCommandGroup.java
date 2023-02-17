// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotySubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorPivotyCommandGroup extends ParallelCommandGroup {
  /** Creates a new ElevatorPivotyCommandGroup. */
  
  public ElevatorPivotyCommandGroup(
    Elevator elevator,
    double elevatorSpeed,
    PivotySubsystem pivoty,
    double pivotySpeed,
    int heightLevel
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new PivotyCommand(pivoty, pivotySpeed, getEncoderValue(heightLevel)),
      new ElevatorCommand(elevator, elevatorSpeed, heightLevel)
    );
  }
  private int getEncoderValue(int heightLevel){
    if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0) == 8){
      return 5;
    }
    else if(heightLevel==3){
      return 349;
    }
    else if(heightLevel==2){
      return 15;
    }
    else if(heightLevel==1){
      return 20;
    }
    else{
      return 22;
    }
  }
}
