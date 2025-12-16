package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.constants.DrivetrainConstants.TunerConstants;

public class AutonomousConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(50, 0, 0);
        public static final PIDConstants ROTATION_PID = new PIDConstants(5, 0, 0);

        private static final double TRACK_WIDTH = Units.inchesToMeters(22);
        private static final Mass ROBOT_MASS = Pounds.of(82.555);
        private static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(5.2268411); // TODO: update from CAD
        private static final double COF = 0.899; // Colsons

        private static final ModuleConfig MODULE_CONFIG = new ModuleConfig(
                TunerConstants.kWheelRadius, TunerConstants.kSpeedAt12Volts,
                COF, DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.kDriveGearRatio),
                Amps.of(120), 1);

        public static final RobotConfig CONFIG = new RobotConfig(ROBOT_MASS, ROBOT_MOI, MODULE_CONFIG,
                new Translation2d[] { new Translation2d(TRACK_WIDTH / 2, TRACK_WIDTH / 2),
                        new Translation2d(TRACK_WIDTH / 2, -TRACK_WIDTH / 2),
                        new Translation2d(-TRACK_WIDTH / 2, TRACK_WIDTH / 2),
                        new Translation2d(-TRACK_WIDTH / 2, -TRACK_WIDTH / 2) });
    }
