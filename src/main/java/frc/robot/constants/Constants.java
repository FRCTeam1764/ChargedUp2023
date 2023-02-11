package frc.robot.constants;

// import com.swervedrivespecialties.swervelib.CanPort;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 1.0; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 1.0; // FIXME Measure and set wheelbase

    // public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID
    // needs fixed
    public static final double FRONT_LEFT_MODULE_DRIVE_MOTOR =   (6);
    public static final double FRONT_RIGHT_MODULE_DRIVE_MOTOR =   (7);
    public static final double BACK_LEFT_MODULE_DRIVE_MOTOR =   (9);
    public static final double BACK_RIGHT_MODULE_DRIVE_MOTOR =   (8);

    public static final double FRONT_LEFT_MODULE_STEER_MOTOR =   (10);
    public static final double FRONT_RIGHT_MODULE_STEER_MOTOR =   (11);
    public static final double BACK_LEFT_MODULE_STEER_MOTOR =   (13);
    public static final double BACK_RIGHT_MODULE_STEER_MOTOR =   (12);

    public static final double FRONT_LEFT_MODULE_STEER_ENCODER =   (15);
    public static final double FRONT_RIGHT_MODULE_STEER_ENCODER =   (16);
    public static final double BACK_LEFT_MODULE_STEER_ENCODER =   (18);
    public static final double BACK_RIGHT_MODULE_STEER_ENCODER =   (17);

    public static final double BACK_INTAKE_MOTOR =  (21);
    public static final double SIDE_INTAKE_MOTOR =  (22);
    public static final double INTAKE_OPENER_MOTOR =  (23);

    // public static final doublePIVOTY_THRUBORE_ENCODER =  (31);
    public static final double PIVOTY_MOTOR =  (32);

    // public static final doubleELEVEATOR_ENCODER =  (41);
    public static final double ELEVATOR_MOTOR =  (42);

    // In degrees
    // increasing turns clockwise
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(-11.46);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(-300.42);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(-194.77);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(-332.06);

    public static final String CANIVORE_NAME = "1764 canivore";
    
    public static final int PRESSURE_SENSOR_PORT = 0;

    // HI :D
    //public static final int INTAKE_SOLENOID_FORWARD = 0;
    //public static final int INTAKE_SOLENOID_REVERSE = 1;

}
