// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.intakeSubsystem;

// public class intakeCommand extends CommandBase {
//   /** Creates a new intakeCommand. */
//   intakeSubsystem intake;
//   double intakeSpeed;
//   boolean intakeClose;
//   int heightLevel;
//   public intakeCommand(intakeSubsystem intake, double intakeSpeed, boolean intakeClose, int heightLevel) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.intakeSpeed = intakeSpeed;
//     this.intake = intake;
//     this.intakeClose = intakeClose;
//     this.heightLevel = heightLevel;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(intakeClose){
//       intake.intakeClose(intakeSpeed);
//     }
//     else if(heightLevel==3){
//       intake.intakeOpen(-intakeSpeed);
//     }
//     else{
//       intake.intakeOpen(0);
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     // intake.intakeOpen(openSpeed);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     if(intake.getTimer()>30 || intake.getTimer2()>20){
//       return true;
//     }
//     return false;
//   }
  
// }
