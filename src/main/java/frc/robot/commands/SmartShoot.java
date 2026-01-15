// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.util.leds.LEDCommand;

public class SmartShoot extends LEDCommand {

    protected Indexer indexer;
    protected Shooter shooter;

    protected AngularVelocity velocity;
    protected boolean isShooting;
    
    public SmartShoot(Indexer indexer, Shooter shooter, AngularVelocity velocity) {
        this.indexer = indexer;
        this.shooter = shooter;
        this.velocity = velocity;

        addRequirements(indexer, shooter);
    }

    @Override
    public void initialize() {

        isShooting = false;

        shooter.setVelocity(velocity);
    }

    @Override
    public void execute() {

        if (shooter.onTarget()) {
            indexer.setPower(IndexerConstants.DEFAULT_POWER);
        }

        if (!isShooting && indexer.getShooterBeamBreak()) {
            isShooting = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        indexer.stop();

        succeeded(true);
    }

    @Override
    public boolean isFinished() {
        return isShooting && !indexer.getShooterBeamBreak(); //TODO: I dont think current hit is necessary
    }
}

