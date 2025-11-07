// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.Constants.DrivetrainConstants.TunerConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Swerve;
import frc.util.LightningContainer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer extends LightningContainer {

    private SendableChooser<Command> autoChooser;
    private Swerve drivetrain;
    private XboxController driver = new XboxController(ControllerConstants.DRIVER_PORT);
 
    @Override
    protected void initializeSubsystems() {
        drivetrain = TunerConstants.createDrivetrain();
    }

    @Override
    protected void configureDefaultCommands() {

        drivetrain.setDefaultCommand(drivetrain.applyRequest(DriveRequests.getDrive(
                () -> -MathUtil.applyDeadband(-driver.getLeftY(), ControllerConstants.DEADBAND)  * Swerve.getSpeedMults()[0],
                () -> -MathUtil.applyDeadband(-driver.getLeftX(), ControllerConstants.DEADBAND) * Swerve.getSpeedMults()[0],
                () -> -MathUtil.applyDeadband(-driver.getRightX(), ControllerConstants.DEADBAND) * Swerve.getSpeedMults()[1])));
    }

    @Override
    protected void configureButtonBindings() {

        // Slow Mode
        new Trigger(() -> driver.getRightTriggerAxis() > 0.25).onTrue(Swerve.applySlowMode(true))
            .onFalse(Swerve.applySlowMode(false));

        // Robot Centric
        new Trigger(() -> driver.getLeftTriggerAxis() > 0.25)
            .whileTrue(drivetrain.applyRequest(DriveRequests.getRobotCentric(
                () -> -MathUtil.applyDeadband(-driver.getLeftY(), ControllerConstants.DEADBAND)  * Swerve.getSpeedMults()[0],
                () -> -MathUtil.applyDeadband(-driver.getLeftX(), ControllerConstants.DEADBAND) * Swerve.getSpeedMults()[0],
                () -> -MathUtil.applyDeadband(-driver.getRightX(), ControllerConstants.DEADBAND) * Swerve.getSpeedMults()[1])));

        // brake
        new Trigger(() -> driver.getXButton()).whileTrue(drivetrain.applyRequest(DriveRequests.getBrake()));
    }

    @Override
    protected void initializeNamedCommands() {
        // autoChooser = AutoBuilder.buildAutoChooser();
        // LightningShuffleboard.send("Auton", "Auto Chooser", autoChooser);
    }

    @Override
    protected Command getAutonomousCommand() {
        // return autoChooser.getSelected();
        return new InstantCommand();
    }
}
