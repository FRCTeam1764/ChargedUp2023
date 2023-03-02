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

    public static final int BACK_INTAKE_MOTOR =     21;
    public static final int SIDE_INTAKE_MOTOR =     22;
    public static final int INTAKE_OPENER_MOTOR =   23;

    public static final int PIVOTY_MOTOR =          36;
    public static final int PIVOTY_MOTOR_2 =        20;

    public static final int ELEVATOR_MOTOR =        32;
    public static final int ELAVATOR_MOTOR_2 =      33;

    public static final int MIN_EXTEND_BREAK_BEAM =  0;
    public static final int MAX_EXTEND_BREAK_BEAM =  2;
    public static final int MID_EXTEND_BREAK_BEAM =  4;
    public static final int PIVOTY_BREAK_BEAM =      6;


    public static final int COLOR_SENSOR =           8;
    public static final int COLOR_SENSOR2 =           8;
    public static final int BLINKIN_SPARK =          1;
    public static final int PRESSURE_SENSOR_PORT =   0;



    public static final String CANIVORE_NAME = "1764 canivore";
    
   




    // unused

        /**
     * The left-to-right distance between the drivetrain wheels
     *    //  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 1.0; 
     * Should be measured from center to center.
     */
    /**
     * The front-to-back distance between the drivetrain wheels.
          //  public static final double DRIVETRAIN_WHEELBASE_METERS = 1.0; 
     * Should be measured from center to center.
     */


    // public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR =   (6);
    // public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR =   (7);
    // public static final int BACK_LEFT_MODULE_DRIVE_MOTOR =   (9);
    // public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR =   (8);

    // public static final int FRONT_LEFT_MODULE_STEER_MOTOR =   (10);
    // public static final int FRONT_RIGHT_MODULE_STEER_MOTOR =   (11);
    // public static final int BACK_LEFT_MODULE_STEER_MOTOR =   (13);
    // public static final int BACK_RIGHT_MODULE_STEER_MOTOR =   (12);

    // public static final int FRONT_LEFT_MODULE_STEER_ENCODER =   (15);
    // public static final int FRONT_RIGHT_MODULE_STEER_ENCODER =   (16);
    // public static final int BACK_LEFT_MODULE_STEER_ENCODER =   (18);
    // public static final int BACK_RIGHT_MODULE_STEER_ENCODER =   (17);

    // In degrees
    // increasing turns clockwise
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(-11.46);
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(-300.42);
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(-194.77);
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(-332.06);


}