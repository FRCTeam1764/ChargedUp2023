// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Claw;

// public class ZeroIntakeCommand extends CommandBase {
//   /** Creates a new ZeroIntakeCommand. */
//   Claw claw;
//   double clawSpeed;
//   Timer timer;
//   public ZeroIntakeCommand(Claw claw, double clawSpeed) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.claw = claw;
//     this.clawSpeed = clawSpeed;
//     addRequirements(claw);
//     timer = new Timer();
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     timer.restart();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     claw.clawOpen(-clawSpeed);
//     claw.zeroEncoder();
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     claw.clawOpen(-.2);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return timer.hasElapsed(2);
//   }
// }
