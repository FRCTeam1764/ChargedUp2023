// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.BackRollers;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SideRollers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCommand extends ParallelCommandGroup {
  /** Creates a new IntakeCommand. */
  public IntakeCommand(Claw claw, double clawSpeed, SideRollers sideRollers,BackRollers backRollers, double speed, BlinkinSubsystem blinkin) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addRequirements(claw);
    if(blinkin.getLEDColor()== .67){
      addCommands(new ClawCommand(claw, clawSpeed ),new SideRollerCommand(sideRollers, speed),new BackRollerCommand(backRollers, speed));
    }
    else{
      addCommands(new ClawCommand(claw, clawSpeed ),new SideRollerCommand(sideRollers, speed),new BackRollerCommand(backRollers, 0));
    }
  }
}
