package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.constants.DrivetrainConstants.TunerConstants;

public class AutonomousConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(50, 0, 0); // TODO: Tune
    public static final PIDConstants ROTATION_PID = new PIDConstants(5, 0, 0);

    private static final Mass ROBOT_MASS = Pounds.of(88); // 83.716 Calculated via OnShape; add 5 for electrical components
    private static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(3.3927854218); // Calculated via OnShape; Doesn't account for electrical components or bolts; for rotation around z-axis
    private static final double COF = 0.899; // Colsons

    private static final ModuleConfig MODULE_CONFIG = new ModuleConfig(
        TunerConstants.kWheelRadius, TunerConstants.kSpeedAt12Volts,
        COF, DCMotor.getFalcon500Foc(1).withReduction(TunerConstants.kDriveGearRatio),
        Amps.of(120), 1);

    public static final RobotConfig getConfig(Translation2d... moduleLocations) {
        return new RobotConfig(ROBOT_MASS, ROBOT_MOI, MODULE_CONFIG, moduleLocations);
    }
}
