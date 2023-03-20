// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.BackRollers;

// public class BackRollerCommand extends CommandBase {
//   /** Creates a new BackRollerCommand. */
//   BackRollers backRollers;
//   double speed;
//   Timer timer;
  
//   public BackRollerCommand(BackRollers backRollers, double speed) {
//     this.backRollers=backRollers;
//     // addRequirements(backRollers);
//     this.speed = speed;
//     timer = new Timer();
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     timer.reset();
//     timer.start();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
  
//   @Override
//   public void execute() {
//     backRollers.backRollerOn(speed);

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     backRollers.backRollerOff();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return timer.hasElapsed(1.0);
//   }
// }
