// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.constants.Constants;
// import frc.robot.libraries.internal.LazyTalonFX;

// public class Elevator extends SubsystemBase {
//   /** Creates a new Elevator. */
//   LazyTalonFX elevatorMotor1;

//   LazyTalonFX elevatorMotor2;
//   double elevatorSpeed;
//   Encoder encoder;
//   // public DigitalInput minExtend;
//   // public DigitalInput maxExtend;
//   // public DigitalInput midExtend;

// public DigitalInput limitSwitch;
//   DigitalInput no;
//   int previousHeightLevel;
//   double Reverse = 1.0;
// boolean BreakBeamOffOrOn = false;
//   public Elevator(){
//     elevatorMotor1 = new LazyTalonFX(Constants.ELEVATOR_MOTOR.id, Constants.ELEVATOR_MOTOR.busName);
//     elevatorMotor2 = new LazyTalonFX(Constants.ELAVATOR_MOTOR_2.id, Constants.ELAVATOR_MOTOR_2.busName);
//     elevatorMotor2.follow(elevatorMotor1);
//     // minExtend = new DigitalInput(Constants.MIN_EXTEND_BREAK_BEAM);
//     // maxExtend = new DigitalInput(Constants.MAX_EXTEND_BREAK_BEAM);
//     // midExtend = new DigitalInput(Constants.MID_EXTEND_BREAK_BEAM);
//     previousHeightLevel=1;

    
//   }


//   //Elevator goes to the breakbeam specified via "heightlevel", speed can be negative or positive
//   public void elevatorOn(double elevatorSpeed, int heightLevel){
//     if (heightLevel < previousHeightLevel) { 
//       BreakBeamOffOrOn = true;
//       Reverse = 1.0;
//     }else {
//       BreakBeamOffOrOn = false;
//       Reverse = -1.0;
//     }

//     if(getBreakBeam(heightLevel).get() == BreakBeamOffOrOn){  // checks if desired breakbeam isn't broken\is broken
//       elevatorMotor1.set(elevatorSpeed * Reverse);
//       elevatorMotor2.set(elevatorSpeed * Reverse);
    
//     } else {           // if the breakbeam is broken it stops the motors and sets the desired height level as the current height level
//       elevatorMotor1.set(0);
//       elevatorMotor2.set(0);
//       // previousHeightLevel = heightLevel;
//     }
    

//   }
//   // public boolean stillRunning(int heightLevel){
//   //   return getBreakBeam(heightLevel).get() == BreakBeamOffOrOn;
//   // }
//   // public void setHeightLevel(){
//   //   previousHeightLevel = 1;
//   // }


//   public void elevatorOff(){
//     elevatorMotor1.set(0);
//     elevatorMotor2.set(0);

//   }
  

//   public boolean IsWantedHeight(int heightLevel) {
//     if (getBreakBeam(heightLevel).get() != BreakBeamOffOrOn){
//       return true;
//     }
//     return false;
//   }

//   // public DigitalInput getBreakBeam(int heightLevel){
//   //   if(heightLevel==1){
//   //     return minExtend;
//   //   }
//   //   if(heightLevel==2){
//   //     return midExtend;
//   //   }
//   //   if(heightLevel==3){
//   //     return maxExtend;
//   //   }
//   //   else{
//   //     return no;
//   //   }
//   // }


//   public double getEncoderValue(){
//     return elevatorMotor1.getSelectedSensorPosition();
//   }
//   //assigning height levels numbers, numbers cooralate to min, mid, and max.

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run

//   }

// }