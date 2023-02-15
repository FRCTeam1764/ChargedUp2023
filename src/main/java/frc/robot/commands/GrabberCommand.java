// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.GrabberSubsystem;

// public class GrabberCommand extends CommandBase {
//   /** Creates a new GrabberCommand. */
//   GrabberSubsystem grabber;
//   double grabberSpeed;

//   public GrabberCommand(GrabberSubsystem grabber, double grabberSpeed) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.grabber = grabber;
//     this.grabberSpeed = grabberSpeed;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     grabber.grabberOn(grabberSpeed);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     grabber.grabberOff();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
