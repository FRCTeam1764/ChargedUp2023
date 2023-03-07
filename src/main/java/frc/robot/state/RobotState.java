// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.state;

/** Add your docs here. */
public class RobotState {
    public LimelightState limelightState;
    public IntakeState IntakeState;
    public SwerveState swerveState;
    public RobotState() {
        this.limelightState = new LimelightState();
        this.IntakeState = new IntakeState();
        this.swerveState = new SwerveState();
    }
}