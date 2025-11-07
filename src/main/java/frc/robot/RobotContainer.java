// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.Constants.DrivetrainConstants.TunerConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Swerve;
import frc.util.LightningContainer;
import frc.util.shuffleboard.LightningShuffleboard;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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
        drivetrain.setDefaultCommand(drivetrain.applyRequest(DriveRequests.getDrive(() -> -driver.getLeftY(),
                () -> -driver.getLeftX(), () -> -driver.getRightX())));
    }

    @Override
    protected void configureButtonBindings() {}

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
