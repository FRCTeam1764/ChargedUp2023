package frc.robot.commands;

import frc.robot.constants.SwerveConstants;
import frc.robot.state.LimelightState;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private LimelightState limelightState;
    private LimelightSubsystem limelightSubsystem;
    private double strafeVal;
    double offsetToleranceProprtion;

    //needs limelight stuff
    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        offsetToleranceProprtion = 0.1;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    public double moveLeftOrRight() {
        if (limelightState.getLimelightState() == true && limelightSubsystem.isThereTarget ==1) {
            strafeVal = MathUtil.applyDeadband(limelightSubsystem.whereToMove(), SwerveConstants.stickDeadband);
        } else {
            strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), SwerveConstants.stickDeadband);
        }
        return strafeVal;
    }

    
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), SwerveConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), SwerveConstants.stickDeadband);
        

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, moveLeftOrRight()).times(SwerveConstants.Swerve.maxSpeed), 
            rotationVal * SwerveConstants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}