package frc.robot.constants;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterConstants {

    public static final double STATOR_LIMIT = 60;
    public static final boolean INVERT_TOP_MOTOR = false;
    public static final boolean INVERT_BOTTOM_MOTOR = true;
    public static final boolean BRAKE_MODE = false;

    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.096;
    public static final double kS = 0.5;
    public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(2);
}