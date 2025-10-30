// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.util.LightningContainer;
import frc.util.shuffleboard.LightningShuffleboard;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer extends LightningContainer {

    private SendableChooser<Command> autoChooser;
 
    @Override
    protected void initializeSubsystems() {}

    @Override
    protected void configureDefaultCommands() {}

    @Override
    protected void configureButtonBindings() {}

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
