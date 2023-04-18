// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.state.ElevatorState;
import frc.robot.subsystems.Elevator;

public class AutoElevatorCommand extends CommandBase {
  /** Creates a new AutoElevatorCommand. */
  ElevatorState elevatorState;
  Elevator elevator;
  double desiredEncoderValue;
  public AutoElevatorCommand(Elevator elevator,ElevatorState elevatorState, double desiredEncoderValue) {
this.elevator = elevator;
this.elevatorState = elevatorState;
this.desiredEncoderValue  = desiredEncoderValue;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorState.setEncoderValue(desiredEncoderValue);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorState.setEncoderValue(desiredEncoderValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return elevator.getEncoderValue() <= desiredEncoderValue+4000 && elevator.getEncoderValue() >= desiredEncoderValue-4000;
}
}
