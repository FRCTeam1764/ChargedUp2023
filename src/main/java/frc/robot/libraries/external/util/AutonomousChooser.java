package frc.robot.libraries.external.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.libraries.external.control.Trajectory;
import frc.robot.state.RobotState;
import frc.robot.subsystems.Swerve;
import frc.robot.AutoBalance;
import frc.robot.RobotContainer;
import frc.robot.ScoreTwo;

public class AutonomousChooser {
    private final Trajectory[] trajectories;

    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    private enum AutonomousMode {
        DEFAULT,
        OPTION1
    }

    public AutonomousChooser(Trajectory[] trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption("Auto Balance", AutonomousMode.DEFAULT);
        autonomousModeChooser.addOption("Score 3", AutonomousMode.OPTION1);
    }

    public SendableChooser<AutonomousMode> getAutonomousModeChooser() {
        return autonomousModeChooser;
    }

    private Command getDefaultAutoCommand(Swerve swerve, RobotState robotState) {
        return new AutoBalance(swerve, robotState);
    }

    private Command getOption1AutoCommand(Swerve swerve) {
        return new ScoreTwo(swerve);
    }

    public Command getCommand(Swerve swerve, RobotState robotState) {
        switch (autonomousModeChooser.getSelected()) {
            case DEFAULT:
                return getDefaultAutoCommand(swerve, robotState);
            case OPTION1:
                return getOption1AutoCommand(swerve);
        }

        return getDefaultAutoCommand(swerve, robotState);
    }
}