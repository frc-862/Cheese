package frc.robot.commands.otf;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Swerve;
import frc.util.shuffleboard.LightningShuffleboard;

public class MinimalAimAtTarget extends Command {
    Translation2d target;
    double targetAngle;

    Swerve swerve;

    PIDController pid;

    DoubleSupplier xPower;
    DoubleSupplier yPower;

    public MinimalAimAtTarget(Swerve swerve, Translation2d target, DoubleSupplier xPower, DoubleSupplier yPower) {
        this.swerve = swerve;
        this.target = target;

        this.yPower = yPower;
        this.xPower = xPower;

        this.pid = new PIDController(0.002, 0.0, 0);
        pid.enableContinuousInput(-180, 180);
        pid.setTolerance(1.0);

        LightningShuffleboard.setDouble("Targeting", "kP", 0);
        LightningShuffleboard.setDouble("Targeting", "kD", 0);
        

    }  
    
    @Override
    public void initialize() {
        targetAngle = target.minus(swerve.getPose().getTranslation()).getAngle().getDegrees();
    }

    @Override 
    public void execute() {
        targetAngle = target.minus(swerve.getPose().getTranslation()).getAngle().getDegrees();

        double power = pid.calculate((swerve.getPose().getRotation().getDegrees()), targetAngle);

        swerve.setControl(DrivetrainConstants.DriveRequests.getAutoDriveInstance(-yPower.getAsDouble() * 0.1, -xPower.getAsDouble() * 0.1, power));
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    } 
}
