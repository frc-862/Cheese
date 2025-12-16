// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.constants.DrivetrainConstants.DriveRequests;
import frc.robot.constants.DrivetrainConstants.TunerConstants;
import frc.robot.constants.LEDConstants;
import frc.robot.constants.LEDConstants.LED_STATES;
import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.Swerve;
import frc.util.leds.Color;
import frc.util.leds.LEDBehaviorFactory;
import frc.util.leds.LEDSubsystem;
import frc.util.LightningContainer;
import frc.util.shuffleboard.LightningShuffleboard;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer extends LightningContainer {

    private SendableChooser<Command> autoChooser;
    private Swerve drivetrain;
    private XboxController driver;
    private Telemetry logger;

    private LEDSubsystem leds;
 
    @Override
    protected void initializeHardware() {
        drivetrain = TunerConstants.createDrivetrain();

        leds = new LEDSubsystem(LED_STATES.values().length, LEDConstants.LED_LENGTH, LEDConstants.LED_PWM_PORT);

        driver = new XboxController(ControllerConstants.DRIVER);

        logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    @Override
    protected void configureDefaultCommands() {

        drivetrain.setDefaultCommand(drivetrain.applyRequest(DriveRequests.getDrive(
                () -> -driver.getLeftX() * drivetrain.getSpeedMults()[0],
                () -> -driver.getLeftY() * drivetrain.getSpeedMults()[0],
                () -> -driver.getRightX() * drivetrain.getSpeedMults()[1])));
    }

    @Override
    protected void configureLEDs() {
        leds.setDefaultBehavior(LEDBehaviorFactory.SwirlBehabior(LEDConstants.allLEDs, 10, 5, Color.BLUE, Color.ORANGE));

		leds.setBehavior(LED_STATES.A.ID(), LEDBehaviorFactory.BlinkColorBehavior(LEDConstants.strip1, 4,  Color.YELLOW)); 
		leds.setBehavior(LED_STATES.B.ID(), LEDBehaviorFactory.pulseColorBehavior(LEDConstants.strip2, 1, Color.PINK)); 
		leds.setBehavior(LED_STATES.X.ID(), LEDBehaviorFactory.RainbowBehavior(LEDConstants.strip3, 1));
		leds.setBehavior(LED_STATES.Y.ID(), LEDBehaviorFactory.SolidColorBehavior(LEDConstants.strip4, Color.GREEN));
		leds.setBehavior(LED_STATES.AUTO.ID(), LEDBehaviorFactory.RainbowBehavior(LEDConstants.allLEDs, 3));
		leds.setBehavior(LED_STATES.TEST.ID(), LEDBehaviorFactory.TestStripBehavior(34, 
			() -> driver.getAButton(),
			() -> driver.getBButton(), 
			() -> driver.getXButton(), 
			() -> driver.getYButton()));
    }

    @Override
    protected void configureButtonBindings() {

        // Slow Mode
        new Trigger(() -> driver.getRightTriggerAxis() > 0.25).onTrue(drivetrain.applySlowMode(true))
            .onFalse(drivetrain.applySlowMode(false));

        // Robot Centric
        new Trigger(() -> driver.getLeftTriggerAxis() > 0.25)
            .whileTrue(drivetrain.applyRequest(DriveRequests.getRobotCentric(
                () -> -driver.getLeftX() * drivetrain.getSpeedMults()[0],
                () -> -driver.getLeftY() * drivetrain.getSpeedMults()[0],
                () -> -driver.getRightX() * drivetrain.getSpeedMults()[1])));

        // brake
        new Trigger(driver::getXButton).whileTrue(drivetrain.applyRequest(DriveRequests.getBrake()));

        // reset field forward
        new Trigger(() -> driver.getStartButton() && driver.getBackButton()).onTrue(drivetrain.commandResetFieldForward());

        // SysId command
        // new Trigger(driver::getYButton).whileTrue(drivetrain.sysId());
    }

    @Override
    protected void initializeNamedCommands() {
        autoChooser = AutoBuilder.buildAutoChooser();
        LightningShuffleboard.send("Auton", "Auto Chooser", autoChooser);
    }

    @Override
    protected Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
