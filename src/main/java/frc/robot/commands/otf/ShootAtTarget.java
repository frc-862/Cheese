// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.otf;

import java.util.function.DoubleSupplier;


import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;  

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.DrivetrainConstants.DriveRequests;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootAtTarget extends Command {
    // Subsystems
    Swerve swerve;
    Shooter shooter;
    Indexer indexer;

    // Target angle stuff
    Angle robotTargetAngle;
    PIDController anglePID;

    // Some constnats to add later
    Angle angleTolerance;
    Distance flyWheelRadius;
    Angle verticalShootingAngle;

    // Target shooter velocity
    AngularVelocity shooterTargetVelocity;

    // The target to shoot at
    Translation3d target;

    // Movement suppliers if added
    DoubleSupplier xMovement;
    DoubleSupplier yMovement;
    
    /** Creates a new ShootAnywhere. */
    public ShootAtTarget(Swerve swerve, Shooter shooter, Indexer indexer, Translation3d target) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.indexer = indexer;

        this.target = target;
        // TODO: Add constants later
        anglePID = new PIDController(0.1, 0, 0);

        // The constnats
        angleTolerance = Degrees.of(5);
        flyWheelRadius = Meters.of(0.2); // in meters (est.)
        verticalShootingAngle = Degrees.of(15);

        // Keep movement at null for now
        xMovement = null;
        yMovement = null;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve, shooter, indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Vector<N2> stationaryShootVelocityVector = getStationaryShootingVector();

        // Just set them for stationary shooting
        shooterTargetVelocity = RadiansPerSecond.of(stationaryShootVelocityVector.norm()/flyWheelRadius.in(Meters));
        robotTargetAngle = Degrees.of(new Translation2d(stationaryShootVelocityVector).getAngle().getDegrees());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Calculate turret position for on the fly?
        if (xMovement != null && yMovement != null) {
            // Create the known vectors
            Vector<N2> stationaryShootVelocityVector = getStationaryShootingVector();
            Vector<N2> robotVelocityVector = swerve.getFieldRelativeVelocity();

            // Solve for the target vector
            // TODO: Compensate for angular rate if we need to
            Vector<N2> shooterTargetVector = stationaryShootVelocityVector.minus(robotVelocityVector);
            
            // Set the values based on the target vector
            shooterTargetVelocity = RadiansPerSecond.of(shooterTargetVector.norm()/flyWheelRadius.in(Meters));
            robotTargetAngle = Degrees.of(new Translation2d(shooterTargetVector).getAngle().getDegrees());
        }

        Rotation2d swerveAngle = swerve.getPose().getRotation();

        double clippedAngularRate = Math.max(-1, Math.min(1, anglePID.calculate(swerveAngle.getDegrees(), robotTargetAngle.in(Degrees))));

        swerve.setControl(DriveRequests.getAutoDriveInstance(
            yMovement == null ? 0 : -yMovement.getAsDouble(), 
            xMovement == null ? 0 : -xMovement.getAsDouble(),
            -clippedAngularRate));

        // if (Math.abs(swerveAngle.getDegrees() - robotTargetAngle.in(Degrees)) < angleTolerance.in(Degrees) && clippedAngularRate < 0.3) {
        //     shooter.setVelocity(shooterTargetVelocity);
        //     indexer.setPower(IndexerConstants.DEFAULT_POWER);
        // } else {
        //     shooter.applyPower(ShooterConstants.COAST_POWER);
        //     indexer.stop();
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        swerve.setControl(new SwerveRequest.Idle());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private Vector<N2> getStationaryShootingVector() {
        // Get our robots target angle
        Translation2d deltaTranslation = target.toTranslation2d().minus(swerve.getTranslation2d());
        Rotation2d stationaryTargetAngle = new Rotation2d(Units.degreesToRadians(90)).minus(deltaTranslation.getAngle());

        // Get our target ball target velocity
        double velocity = Math.sqrt((deltaTranslation.getNorm()*9.81)/Math.sin(Math.toRadians(2*verticalShootingAngle.in(Radians))));

        // Create the known vectors
        return VecBuilder.fill(Math.cos(stationaryTargetAngle.getRadians()) * velocity, Math.sin(stationaryTargetAngle.getRadians()) * velocity);
    }

    /**
     * Add drive movement while continuing to aim and shoot at a target
     * @param x Movement in the x direction
     * @param y Movementn in teh y direction
     * @return this
     */
    public ShootAtTarget withMovement(DoubleSupplier x, DoubleSupplier y) {
        xMovement = x;
        yMovement = y;

        return this;
    }
}
