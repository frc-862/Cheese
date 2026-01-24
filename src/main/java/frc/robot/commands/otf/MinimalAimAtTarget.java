package frc.robot.commands.otf;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Swerve;

public class MinimalAimAtTarget extends Command {
    Translation2d target;
    double targetAngle;

    Swerve swerve;

    PIDController pid;

    public MinimalAimAtTarget(Swerve swerve, Translation2d target) {
        this.swerve = swerve;
        this.target = target;

        this.pid = new PIDController(0.02, 0.0, 0.001);
        pid.enableContinuousInput(-180, 180);
        pid.setTolerance(1.0);

        // LightningShuffleboard.setDouble("Targeting", "kP", 0);

        // LightningShuffleboard.setDouble("Targeting", "kP", 0);
        

    }  
    
    @Override
    public void initialize() {
         targetAngle = target.minus(swerve.getPose().getTranslation()).getAngle().getDegrees();
    }

    @Override 
    public void execute() {
        //  pid.setP(LightningShuffleboard.getDouble("Targeting", "kP", 0));

        double power = pid.calculate((swerve.getPose().getRotation().getDegrees()), targetAngle +180);

        swerve.setControl(DrivetrainConstants.DriveRequests.getAutoDriveInstance(0, 0, power));
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    } 
}
