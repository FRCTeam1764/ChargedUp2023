// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends CommandBase {
  /** Creates a new ElevatorCommand. */
  Elevator elevator;
  double elevatorSpeed;
  int desiredEncoderSpeed;
  public ElevatorCommand(Elevator elevator, double elevatorSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.elevatorSpeed = elevatorSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.elevatorOn(elevatorSpeed, desiredEncoderSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.elevatorOff();
  }

  // // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    //needs encoder stuff
    return false;
  }
}
