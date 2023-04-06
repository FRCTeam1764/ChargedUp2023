// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.Intake;

public class intakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  boolean ConeCube;
  Intake intake;
  Timer timer;
  double speed;
  BlinkinSubsystem blinkin;
  public intakeCommand(boolean ConeCube, Intake intake, double speed, BlinkinSubsystem blinkin ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.ConeCube  = ConeCube;
    this.speed = speed;
    timer = new Timer();
    this.blinkin = blinkin;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(blinkin.getLEDColor()==.67){
      intake.wheelsCone(speed);
    }
    else{
      intake.wheelsCube(speed);
    }
    // if(ConeCube){
    //   intake.pickUpCone();
    // }else{
    //   intake.pickUpCube();
    // }

  }
  public double getSpeed(){
    if(blinkin.getLEDColor()== .67){
      return speed*-1;
    }
    else{
      return speed;
    }
  }

  // Called once the command ends or is interrupted.

  @Override
  public void end(boolean interrupted) {
    timer.restart();
    while(timer.get() <.75){
      if(blinkin.getLEDColor()==.67){
        intake.wheelsCone(-speed);
      }
      else{
        intake.wheelsCube(-speed);
      }
    }
    intake.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
