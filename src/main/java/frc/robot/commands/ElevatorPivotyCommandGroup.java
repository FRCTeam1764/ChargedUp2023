// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.state.ElevatorState;
// import frc.robot.state.PivotyState;

// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.PivotySubsystem;


// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class ElevatorPivotyCommandGroup extends SequentialCommandGroup {
//   /** Creates a new ElevatorPivotyCommandGroup. */
//  ElevatorFeedforward elevatorFeedforward;

//  public double getkV(double x){
//    double velo = Math.pow(2.44, -6)*x + -0.1;
//    return velo;
//  }


//  public double getks(double x){
//    double velo = Math.pow(-3.94, -5)*x + 26.6;
//    return velo;
//  }

//  public double getKg(double x){
//    double velo = Math.pow(1.94, -6)*x + -0.233;
//    return velo;
//  }


//   public ElevatorPivotyCommandGroup(
//     PivotySubsystem pivoty, double desiredEncoderValue, PivotyState pivotyState,
//     Elevator elevator, double encoderValue, ElevatorState elevatorState) {

//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     double velo = new ElevatorFeedforward(getkV(desiredEncoderValue), getKg(desiredEncoderValue), getks(desiredEncoderValue)).calculate(0.000001);
    

//     addCommands(
//       new PivotyCommand(pivoty, desiredEncoderValue, pivotyState, true),
//       new ParallelCommandGroup(
//         new PivotyCommand(pivoty, desiredEncoderValue, pivotyState, false),
//         new ElevatorCommand(elevator,encoderValue,elevatorState,false,velo))
//     );
//   }
// }




