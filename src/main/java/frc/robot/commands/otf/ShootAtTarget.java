// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.otf;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.DrivetrainConstants.DriveRequests;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootAtTarget extends Command {
    // Subsystems
    Swerve swerve;
    Shooter shooter;

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
    Translation2d target;
    
    /** Creates a new ShootAnywhere. */
    public ShootAtTarget(Swerve swerve, Shooter shooter, Translation2d target) {
        this.swerve = swerve;
        this.shooter = shooter;

        this.target = target;
        // TODO: Add constants later
        anglePID = new PIDController(0, 0, 0);

        // The constnats
        angleTolerance = Degrees.of(5);
        flyWheelRadius = Meters.of(0.2); // in meters (est.)
        verticalShootingAngle = Degrees.of(15);
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve, shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Get our robots target angle
        Translation2d deltaTranslation = swerve.getTranslation2d().minus(target);
        robotTargetAngle = Degrees.of(deltaTranslation.getAngle().plus(new Rotation2d(Units.degreesToRadians(90))).getDegrees());

        // Get our target shooter target velocity
        double distance = deltaTranslation.getNorm(); 
        double velocity = Math.sqrt((distance*9.81)/Math.sin(Math.toRadians(2*verticalShootingAngle.baseUnitMagnitude())));

        shooterTargetVelocity = RadiansPerSecond.of(velocity/flyWheelRadius.in(Meters));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double swerveAngleDegrees = swerve.getPose().getRotation().getDegrees();

        if (Math.abs(swerveAngleDegrees - robotTargetAngle.in(Degrees)) > angleTolerance.in(Degrees)) {
            swerve.setControl(DriveRequests.getAutoRequest(0, 0, -anglePID.calculate(swerveAngleDegrees, robotTargetAngle.in(Degrees))));
        } else {
            shooter.setVelocity(shooterTargetVelocity);
        }
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
}
