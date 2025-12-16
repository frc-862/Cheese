package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Swerve;

public class DrivetrainConstants {
    public class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs() // TODO: tune with SysId
            .withKP(100).withKI(0).withKD(0.5)
            .withKS(0.1).withKV(2.66).withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.1).withKI(0).withKD(0)
            .withKS(0).withKV(0.124);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The type of motor used for the drive motor
        private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        // The type of motor used for the drive motor
        private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
        private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final Current kSlipCurrent = Amps.of(120.0); // TODO: tune this

        // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a relatively low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
            );
        private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        private static final Pigeon2Configuration pigeonConfigs = null;

        // CAN bus that the devices are located on;
        // All swerve devices must share the same CAN bus
        public static final CANBus kCANBus = new CANBus("Canivore", "./logs/example.hoot");

        // Theoretical free speed (m/s) at 12 V applied output;
        // This needs to be tuned to your individual robot
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.73); // TODO: tune this with SysId

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;

        public static final double kDriveGearRatio = 6.746031746031747;
        private static final double kSteerGearRatio = 21.428571428571427;
        public static final Distance kWheelRadius = Inches.of(2);

        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        private static final int kPigeonId = 23;

        // These are only used for simulation
        private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
        private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
        // Simulated voltage necessary to overcome friction
        private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(kCANBus.getName())
                .withPigeon2Id(kPigeonId)
                .withPigeon2Configs(pigeonConfigs);

        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withDriveMotorType(kDriveMotorType)
                .withSteerMotorType(kSteerMotorType)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);


        // Front Left
        private static final int kFrontLeftDriveMotorId = 7;
        private static final int kFrontLeftSteerMotorId = 8;
        private static final int kFrontLeftEncoderId = 34;
        private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.285888671875);
        private static final boolean kFrontLeftSteerMotorInverted = true;
        private static final boolean kFrontLeftEncoderInverted = false;

        private static final Distance kFrontLeftXPos = Inches.of(11);
        private static final Distance kFrontLeftYPos = Inches.of(11);

        // Front Right
        private static final int kFrontRightDriveMotorId = 5;
        private static final int kFrontRightSteerMotorId = 6;
        private static final int kFrontRightEncoderId = 33;
        private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.051513671875);
        private static final boolean kFrontRightSteerMotorInverted = true;
        private static final boolean kFrontRightEncoderInverted = false;

        private static final Distance kFrontRightXPos = Inches.of(11);
        private static final Distance kFrontRightYPos = Inches.of(-11);

        // Back Left
        private static final int kBackLeftDriveMotorId = 1;
        private static final int kBackLeftSteerMotorId = 2;
        private static final int kBackLeftEncoderId = 31;
        private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.410888671875);
        private static final boolean kBackLeftSteerMotorInverted = true;
        private static final boolean kBackLeftEncoderInverted = false;

        private static final Distance kBackLeftXPos = Inches.of(-11);
        private static final Distance kBackLeftYPos = Inches.of(11);

        // Back Right
        private static final int kBackRightDriveMotorId = 3;
        private static final int kBackRightSteerMotorId = 4;
        private static final int kBackRightEncoderId = 32;
        private static final Angle kBackRightEncoderOffset = Rotations.of(0.421875);
        private static final boolean kBackRightSteerMotorInverted = true;
        private static final boolean kBackRightEncoderInverted = false;

        private static final Distance kBackRightXPos = Inches.of(-11);
        private static final Distance kBackRightYPos = Inches.of(-11);


        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
            ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
            ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
                kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
            ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
            ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
            );

        /**
         * Creates a CommandSwerveDrivetrain instance.
         * This should only be called once in your robot program,.
         */
        public static Swerve createDrivetrain() {
            return new Swerve(
                DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
            );
        }


        /**
         * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
         */
        public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
            /**
             * Constructs a CTRE SwerveDrivetrain using the specified constants.
             * <p>
             * This constructs the underlying hardware devices, so users should not construct
             * the devices themselves. If they need the devices, they can access them through
             * getters in the classes.
             *
             * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
             * @param modules               Constants for each specific module
             */
            public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                SwerveModuleConstants<?, ?, ?>... modules
            ) {
                super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, modules
                );
            }

            /**
             * Constructs a CTRE SwerveDrivetrain using the specified constants.
             * <p>
             * This constructs the underlying hardware devices, so users should not construct
             * the devices themselves. If they need the devices, they can access them through
             * getters in the classes.
             *
             * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
             * @param odometryUpdateFrequency The frequency to run the odometry loop. If
             *                                unspecified or set to 0 Hz, this is 250 Hz on
             *                                CAN FD, and 100 Hz on CAN 2.0.
             * @param modules                 Constants for each specific module
             */
            public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                SwerveModuleConstants<?, ?, ?>... modules
            ) {
                super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency, modules
                );
            }

            /**
             * Constructs a CTRE SwerveDrivetrain using the specified constants.
             * <p>
             * This constructs the underlying hardware devices, so users should not construct
             * the devices themselves. If they need the devices, they can access them through
             * getters in the classes.
             *
             * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
             * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
             *                                  unspecified or set to 0 Hz, this is 250 Hz on
             *                                  CAN FD, and 100 Hz on CAN 2.0.
             * @param odometryStandardDeviation The standard deviation for odometry calculation
             *                                  in the form [x, y, theta]ᵀ, with units in meters
             *                                  and radians
             * @param visionStandardDeviation   The standard deviation for vision calculation
             *                                  in the form [x, y, theta]ᵀ, with units in meters
             *                                  and radians
             * @param modules                   Constants for each specific module
             */
            public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                Matrix<N3, N1> odometryStandardDeviation,
                Matrix<N3, N1> visionStandardDeviation,
                SwerveModuleConstants<?, ?, ?>... modules
            ) {
                super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency,
                    odometryStandardDeviation, visionStandardDeviation, modules
                );
            }
        }
    }

    public static class DriveRequests {

        public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                                                                                // desired
                                                                                                // top
                                                                                                // speed
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a
                                                                                                        // rotation per
                                                                                                        // second max
                                                                                                        // angular
                                                                                                        // velocity

        public static final double SLOW_SPEED_MULT = 0.4;
        public static final double SLOW_TURN_MULT = 0.7;
        public static final double[] NORMAL_SPEED_MULTS = {1.0, 1.0};
        public static final double[] SLOW_MULTS = {SLOW_SPEED_MULT, SLOW_TURN_MULT};

        private static final SwerveRequest.FieldCentric DRIVE = new SwerveRequest.FieldCentric();
        private static final SwerveRequest.RobotCentric ROBO_CENTRIC = new SwerveRequest.RobotCentric();
        private static final SwerveRequest.SwerveDriveBrake BRAKE = new SwerveRequest.SwerveDriveBrake();
        public static final SwerveRequest.ApplyRobotSpeeds AUTON = new SwerveRequest.ApplyRobotSpeeds();

        public static Supplier<SwerveRequest> getDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
            return () -> DRIVE
                .withVelocityX(x.getAsDouble() * MAX_SPEED) // Drive forward with negative Y
                .withVelocityY(y.getAsDouble() * MAX_SPEED) // Drive left with negative X
                .withRotationalRate(rot.getAsDouble() * MAX_ANGULAR_RATE)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withDeadband(ControllerConstants.DEADBAND)
                .withRotationalDeadband(ControllerConstants.DEADBAND)
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective); // Drive counterclockwise with negative
                                                                            // X (left)
                                                                    
        }

        public static Supplier<SwerveRequest> getRobotCentric(DoubleSupplier x, DoubleSupplier y,
                DoubleSupplier rot) {
            return () -> ROBO_CENTRIC
                .withVelocityX(y.getAsDouble() * MAX_SPEED) // Drive forward with negative Y
                                                                                // (forward)
                .withVelocityY(x.getAsDouble() * MAX_SPEED) // Drive left with negative X
                                                                                // (left)
                .withRotationalRate(rot.getAsDouble() * MAX_ANGULAR_RATE) // Drive counterclockwise with negative
                                                                            // X (left)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withDeadband(ControllerConstants.DEADBAND)
                .withRotationalDeadband(ControllerConstants.DEADBAND);
        }

        public static Supplier<SwerveRequest> getBrake() {
            return () -> BRAKE;
        }
    }
}