// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.state.PivotyState;
import frc.robot.subsystems.BackRollers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotySubsystem;
import frc.robot.subsystems.SideRollers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorPivotyCommandGroup extends SequentialCommandGroup {
  /** Creates a new ElevatorPivotyCommandGroup. */
  public ElevatorPivotyCommandGroup(
    PivotySubsystem pivoty, int desiredEncoderValue, PivotyState pivotyState,
    Elevator elevator, double elevatorSpeed, int heightLevel, SideRollers sideRollers, BackRollers backRollers, double speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PivotyCommand(pivoty, desiredEncoderValue, pivotyState, true,sideRollers,backRollers,speed),
      new ParallelCommandGroup(
        new PivotyCommand(pivoty, desiredEncoderValue, pivotyState, false,sideRollers,backRollers,speed),
        new ElevatorCommand(elevator,elevatorSpeed, heightLevel))
    );
  }
}
