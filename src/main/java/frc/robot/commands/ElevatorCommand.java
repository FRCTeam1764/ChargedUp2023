// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.state.ElevatorState;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends CommandBase {
  /** Creates a new ElevatorCommand. */
  //needs fixed
  Elevator elevator;
  double desiredEncoderValue;
  int heightLevel;
  ElevatorState elevatorState;
  boolean finish;
  double velo;
  public ElevatorCommand(Elevator elevator, double desiredEncoderValue,ElevatorState elevatorState,boolean finish,double velo) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.desiredEncoderValue = desiredEncoderValue;
    this.elevatorState = elevatorState;
    this.finish = finish;
    this.velo = velo;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorState.setEncoderValue(desiredEncoderValue);
    elevatorState.setVelocity(velo);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorState.setEncoderValue(desiredEncoderValue);
    elevatorState.setVelocity(velo);
  }

  // Called once the command ends or is interrupted.

  @Override
  public void end(boolean interrupted) {
    elevatorState.setEncoderValue(0);
    
    // elevator.elevatorOn(elevatorSpeed, 1);
    // while(!elevator.IsWantedHeight(1)){
    // elevator.elevatorOn(elevatorSpeed, 1);
  //  }
  }

  // // Returns true when the command should end.

  @Override
  public boolean isFinished() {
    if(finish){
      return elevator.getEncoderValue() <= desiredEncoderValue+4000 && elevator.getEncoderValue() >= desiredEncoderValue-4000;
    }
    return false;
}
}