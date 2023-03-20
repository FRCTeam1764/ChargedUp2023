// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.constants.Constants;

// public class BackRollers extends SubsystemBase {
//   CANSparkMax backRoller;
//   /** Creates a new BackRollers. */
//   public BackRollers() {
//     backRoller = new CANSparkMax(Constants.BACK_INTAKE_MOTOR.id, MotorType.kBrushless);
//   }
//   public void backRollerOn(double speed){
//     backRoller.set(speed);
//   }
//   public void backRollerOff(){
//     backRoller.set(0);
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
