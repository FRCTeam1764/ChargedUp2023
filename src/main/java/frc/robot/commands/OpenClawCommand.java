// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Claw;

// public class OpenClawCommand extends CommandBase {
//   Claw claw;
//   double speed;
//   private double setpoint;
//   /** Creates a new ClawCommand. */
//   public OpenClawCommand(Claw claw, double setpoint) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.claw = claw;
//     addRequirements(claw);
//     this.speed = speed;
//     this.setpoint = setpoint;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
  
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     claw.openClaw(setpoint);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     // claw.openClaw(2);
//     claw.clawOpen(-1.0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return claw.getEncoderValue() >setpoint +.05 && claw.getEncoderValue() < setpoint +.05;
//   }
// }
