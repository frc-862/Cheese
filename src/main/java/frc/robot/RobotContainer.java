// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ExtraSmartShoot;
import frc.robot.commands.SmartCollect;
import frc.robot.commands.otf.MinimalAimAtTarget;
import frc.robot.commands.otf.ShootAtTarget;
import frc.robot.constants.CollectorConstants;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.DrivetrainConstants.DriveRequests;
import frc.robot.constants.LEDConstants;
import frc.robot.constants.LEDConstants.LED_STATES;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.util.LightningContainer;
import frc.util.leds.Color;
import frc.util.leds.LEDBehaviorFactory;
import frc.util.leds.LEDSubsystem;
import frc.util.shuffleboard.DemoShuffleboard;

public class RobotContainer extends LightningContainer {

    private Collector collector;
    private Indexer indexer;
    private Shooter shooter;
    private Swerve  drivetrain;
    private LEDSubsystem leds;
    private PhotonVision vision;

    private XboxController driver;
    private XboxController copilot;

    private XboxController storedCopilot;

    private DoubleSubscriber shooterPowerMultiplier;
    private DoubleSubscriber shooterRPS;
    private DoubleSubscriber driveMultiplier;
    private BooleanSubscriber useSingleController;

    @Override
    protected void initializeHardware() {
        drivetrain = DrivetrainConstants.TunerConstants.createDrivetrain();

        // telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.magnitude());

        vision = new PhotonVision(drivetrain);
        
        collector = new Collector();
        indexer = new Indexer();
        shooter = new Shooter();

        leds = new LEDSubsystem(LED_STATES.values().length, LEDConstants.LED_LENGTH, LEDConstants.LED_PWM_PORT);
        
        driver = new XboxController(ControllerConstants.DRIVER);
        copilot = storedCopilot = new XboxController(ControllerConstants.COPILOT);

        shooterPowerMultiplier = DemoShuffleboard.subscribeToDouble("Shooter Power Multiplier", 0.4);
        driveMultiplier = DemoShuffleboard.subscribeToDouble("Drive Multiplier", 0.4);
        useSingleController = DemoShuffleboard.subscribeToBoolean("Use Single Controller", false);
        shooterRPS = DemoShuffleboard.subscribeToDouble("Shooter RPS", 40);
    }

    @Override
    protected void configureDefaultCommands() {
        
        // default drive
        drivetrain.setDefaultCommand(drivetrain.applyRequest(DriveRequests.getDrive(
            () -> -Math.pow(MathUtil.applyDeadband(driver.getLeftY(), ControllerConstants.DEADBAND), 3) * driveMultiplier.get(), 
            () -> -Math.pow(MathUtil.applyDeadband(driver.getLeftX(), ControllerConstants.DEADBAND), 3)  * driveMultiplier.get(), 
            () -> -Math.pow(MathUtil.applyDeadband(driver.getRightX(), ControllerConstants.DEADBAND), 3) * driveMultiplier.get())));

        // coast shooter in
        shooter.setDefaultCommand(shooter.applyPower(ShooterConstants.COAST_POWER));

    }

    @Override
    protected void configureButtonBindings() {
        // // demo collect & index
        // new Trigger(copilot::getLeftBumperButton).onTrue(collector.applyPower(CollectorConstants.DEFAULT_POWER))
        //     .onFalse(collector.applyStop())
        //     .whileTrue(leds.enableState(LED_STATES.COLLECTING.ID()));

        // new Trigger(copilot::getRightBumperButton).onTrue(collector.applyPower(-CollectorConstants.DEFAULT_POWER))
        //     .onFalse(collector.applyStop())
        //     .whileTrue(leds.enableState(LED_STATES.COLLECTING.ID()));

        // // demo shoot
        // new Trigger(() -> Math.abs(getCopilotTriggerDifference()) > ControllerConstants.DEADBAND)
        //     .whileTrue(shooter.applyPower(() -> getCopilotTriggerDifference() * shooterPowerMultiplier.get())
        //     .deadlineFor(leds.enableState(LED_STATES.SHOOTING.ID())));

        // manual index
        new Trigger(copilot::getXButton).onTrue(indexer.applyPower(CollectorConstants.DEFAULT_POWER))
            .onFalse(indexer.applyStop())
            .whileTrue(leds.enableState(LED_STATES.COLLECTING.ID()));

        new Trigger(copilot::getBButton).onTrue(indexer.applyPower(-CollectorConstants.DEFAULT_POWER))
            .onFalse(indexer.applyStop())
            .whileTrue(leds.enableState(LED_STATES.COLLECTING.ID()));
        
        // // Extra Smart Shoot
        // new Trigger(copilot::getYButton)
        //     .whileTrue(new ExtraSmartShoot(indexer, shooter, RotationsPerSecond.of(shooterRPS.get()))
        //     .onSuccess(leds.enableStateWithTimeout(LED_STATES.SHOT.ID(), 5))
        //     .deadlineFor(leds.enableState(LED_STATES.COLLECTING.ID())));

        // keep flyweheel spun up
        // new Trigger(copilot::getAButton)
        //     .whileTrue(new RunCommand(() -> shooter.setVelocity(RotationsPerSecond.of(shooterRPS.get()))) // will be autostopped by coast power
        //     .deadlineFor(leds.enableState(LED_STATES.SHOOTING.ID()))); // use run command to avoid requiring shooter

        // // robot centric
        // new Trigger(() -> (driver.getLeftTriggerAxis()) > ControllerConstants.DEADBAND).whileTrue(drivetrain.applyRequest(DriveRequests.getRobotCentric(
        //     () -> -Math.pow(MathUtil.applyDeadband(driver.getLeftX(), ControllerConstants.DEADBAND), 3) * driveMultiplier.get(), 
        //     () -> -Math.pow(MathUtil.applyDeadband(driver.getLeftY(), ControllerConstants.DEADBAND), 3) * driveMultiplier.get(), 
        //     () -> -Math.pow(MathUtil.applyDeadband(driver.getRightX(), ControllerConstants.DEADBAND), 3) * driveMultiplier.get()))).whileTrue(leds.enableState(LED_STATES.ERROR.ID()));

        // // slowmode
        // new Trigger(() -> (driver.getRightTriggerAxis()) > ControllerConstants.DEADBAND).whileTrue(drivetrain.applyRequest(DriveRequests.getRobotCentric(
        //     () -> -Math.pow(MathUtil.applyDeadband(driver.getLeftX(), ControllerConstants.DEADBAND), 3) * driveMultiplier.get() * DrivetrainConstants.SLOWMODE_MULTIPLIER,
        //     () -> -Math.pow(MathUtil.applyDeadband(driver.getLeftY(), ControllerConstants.DEADBAND), 3) * driveMultiplier.get() * DrivetrainConstants.SLOWMODE_MULTIPLIER, 
        //     () -> -Math.pow(MathUtil.applyDeadband(driver.getRightX(), ControllerConstants.DEADBAND), 3) * driveMultiplier.get())));

        // brake
        new Trigger(() -> useSingleController.get() ? false : driver.getXButton()).whileTrue(drivetrain.applyRequest(DriveRequests.getBrake()));

        // reset field forward
        new Trigger(() -> driver.getStartButton() && driver.getBackButton()).onTrue(drivetrain.commandResetFieldForward()).whileTrue(leds.enableState(LED_STATES.ERROR.ID()));

        // // switch to single controller mode when enabled
        // new Trigger(useSingleController::get)
        //     .onTrue(new InstantCommand(() -> copilot = driver))
        //     .onFalse(new InstantCommand(() -> copilot = storedCopilot))
        //     .whileTrue(leds.enableState(LED_STATES.SINGLE_CONTROLLER.ID()));
        
        new Trigger(() -> driver.getAButton()).whileTrue(new ShootAtTarget(drivetrain, shooter, indexer, new Translation3d(11.915394, 4.034536, 0)));
        new Trigger(() -> driver.getBButton()).whileTrue(new MinimalAimAtTarget(drivetrain, new Translation2d(11.915394, 4.034536)));
    }

    @Override
    protected void configureLEDs() {
        leds.setDefaultBehavior(LEDBehaviorFactory.SwirlBehabior(LEDConstants.allLEDs, 10, 5, Color.BLUE, Color.ORANGE));

        leds.setBehavior(LED_STATES.ERROR.ID(), LEDBehaviorFactory.BlinkColorBehavior(LEDConstants.allLEDs, 2, Color.RED));

        leds.setBehavior(LED_STATES.SINGLE_CONTROLLER.ID(), LEDBehaviorFactory.SolidColorBehavior(LEDConstants.strip2, Color.YELLOW).and(LEDBehaviorFactory.SolidColorBehavior(LEDConstants.strip4, Color.YELLOW)));

        leds.setBehavior(LED_STATES.COLLECTED.ID(), LEDBehaviorFactory.BlinkColorBehavior(LEDConstants.strip1, 4, Color.GREEN));
        leds.setBehavior(LED_STATES.SHOT.ID(), LEDBehaviorFactory.BlinkColorBehavior(LEDConstants.strip3, 4, Color.GREEN));

		leds.setBehavior(LED_STATES.COLLECTING.ID(), LEDBehaviorFactory.pulseColorBehavior(LEDConstants.allLEDs, 8, Color.ORANGE));
        leds.setBehavior(LED_STATES.SHOOTING.ID(), LEDBehaviorFactory.pulseColorBehavior(LEDConstants.allLEDs, 8, Color.PURPLE));

		leds.setBehavior(LED_STATES.AUTO.ID(), LEDBehaviorFactory.RainbowBehavior(LEDConstants.allLEDs, 3));
		leds.setBehavior(LED_STATES.TEST.ID(), LEDBehaviorFactory.TestStripBehavior(0, 
			() -> driver.getAButton(),
			() -> driver.getBButton(),
			() -> driver.getXButton(), 
			() -> driver.getYButton()));

        new Trigger(DriverStation::isTest).whileTrue(leds.enableState(LED_STATES.TEST.ID()));

        new Trigger(() -> DriverStation.isAutonomous() && DriverStation.isEnabled()).whileTrue(leds.enableState(LED_STATES.AUTO.ID()));
    }

    @Override
    protected void initializeNamedCommands() {}

    @Override
    protected Command getAutonomousCommand() {
        // TODO: Test

        // drive back, collect, shoot
        return (drivetrain.applyRequest(DriveRequests.getDrive(()-> -0.1, () -> 0, () -> 0)).withTimeout(3)
            .withDeadline(new SmartCollect(indexer, collector)))
            .andThen(drivetrain.applyRequest(DriveRequests.getDrive(() -> 0, () -> 0, () -> 0))
            .alongWith(new ExtraSmartShoot(indexer, shooter, RotationsPerSecond.of(40))));
    }

    private double getCopilotTriggerDifference() {
        // left trigger is disabled when in single controller mode
        return (copilot.getRightTriggerAxis() - (useSingleController.get() ? 0 : copilot.getLeftTriggerAxis()));
    }
}
