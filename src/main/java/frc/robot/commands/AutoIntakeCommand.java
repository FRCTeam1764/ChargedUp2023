// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class AutoIntakeCommand extends CommandBase {
  /** Creates a new AutoIntakeCommand. */
boolean OnOff;
Intake intake;
String ConeCube;

  public AutoIntakeCommand(Intake intake, boolean OnOff,String ConeCube) {
    this.intake = intake;
    this.OnOff = OnOff;
    this.ConeCube = ConeCube;


    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(OnOff){
      if(ConeCube == "Cube"){
        intake.pickUpCube();

      }else if(ConeCube == "Cone"){
intake.pickUpCone();

      }


    }else{
      intake.stop();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
