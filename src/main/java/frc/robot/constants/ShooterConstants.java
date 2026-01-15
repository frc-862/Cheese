package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class ShooterConstants {

    public static final double STATOR_LIMIT = 80d;
    public static final boolean INVERT_TOP_MOTOR = false;
    public static final boolean INVERT_BOTTOM_MOTOR = true;
    public static final boolean BRAKE_MODE = false;

    public static final double COAST_POWER = -0.05;

    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.096;
    public static final double kS = 0.5;
    public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(2);
    public static final Current THRESHHOLD = Amps.of(20);
}
