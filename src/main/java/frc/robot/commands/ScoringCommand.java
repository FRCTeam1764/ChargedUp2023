// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotySubsystem;
import frc.robot.subsystems.intakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoringCommand extends SequentialCommandGroup {
  /** Creates a new ScoringCommand. */
  public ScoringCommand(Elevator elevator, double elevatorSpeed, PivotySubsystem pivoty, double pivotySpeed, int heightLevel,
  intakeSubsystem intake, double intakeSpeed, boolean intakeClose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ElevatorPivotyCommandGroup(elevator, elevatorSpeed, pivoty, pivotySpeed, heightLevel),
      new intakeCommand(intake, intakeSpeed, intakeClose, heightLevel)
    );
  }
}
