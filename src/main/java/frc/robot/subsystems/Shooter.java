// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;
import frc.robot.constants.ShooterConstants;
import frc.util.hardware.ThunderBird;

public class Shooter extends SubsystemBase {

    private ThunderBird shooterMotorBottom;
    private ThunderBird shooterMotorTop;

    private DutyCycleOut shooterDutyCycle;
    private VelocityVoltage velocityPID;

    private AngularVelocity bottomTargetVelocity;
    private AngularVelocity topTargetVelocity;
    
    public Shooter() {

        // Initialize the shooter motors with their configuration
        shooterMotorBottom = new ThunderBird(RobotMap.SHOOTER_MOTOR_BOTTOM_ID, RobotMap.CANIVORE_CAN_NAME,
            ShooterConstants.INVERT_TOP_MOTOR, ShooterConstants.STATOR_LIMIT, ShooterConstants.BRAKE_MODE);

        shooterMotorTop = new ThunderBird(RobotMap.SHOOTER_MOTOR_TOP_ID, RobotMap.CANIVORE_CAN_NAME,
            ShooterConstants.INVERT_BOTTOM_MOTOR, ShooterConstants.STATOR_LIMIT, ShooterConstants.BRAKE_MODE);

        shooterDutyCycle = new DutyCycleOut(0d);
        velocityPID = new VelocityVoltage(0d);

        TalonFXConfiguration bottomConfig = shooterMotorBottom.getConfig();
        bottomConfig.Slot0.kP = ShooterConstants.kP;
        bottomConfig.Slot0.kI = ShooterConstants.kI;
        bottomConfig.Slot0.kD = ShooterConstants.kD;
        bottomConfig.Slot0.kV = ShooterConstants.kV;
        bottomConfig.Slot0.kS = ShooterConstants.kS;
        shooterMotorBottom.applyConfig(bottomConfig);

        TalonFXConfiguration topConfig = shooterMotorTop.getConfig();
        topConfig.Slot0.kP = ShooterConstants.kP;
        topConfig.Slot0.kI = ShooterConstants.kI;
        topConfig.Slot0.kD = ShooterConstants.kD;
        topConfig.Slot0.kV = ShooterConstants.kV;
        topConfig.Slot0.kS = ShooterConstants.kS;
        shooterMotorTop.applyConfig(topConfig);

    }

    /**
     * sets the power for the shooter motors
     * @param power
     */
    public void setPowerTop(double power) {
        shooterMotorTop.setControl(shooterDutyCycle.withOutput(power));
    }

    /**
     * sets the power for the shooter motors
     * @param power
     */
    public void setPowerBottom(double power) {
        shooterMotorBottom.setControl(shooterDutyCycle.withOutput(power));
    }


    public void setPower(double power) {
        setPower(power, power);
    }
    
    public void setPower(double powerTop, double powerBottom) {
        setPowerTop(powerTop);
        setPowerBottom(powerBottom);
    }

    public void stop() {
        setPower(0);
    }

    public Command applyStop() {
        return runOnce(this::stop);
    }

    /**
     * @param power
     * @return instantCommand that sets the power
     */
    public Command applyPower(double power) {
        return runOnce(() -> setPower(power));
    }

    /**
     * @param powerTop
     * @param powerBottom
     * @return instantCommand that sets the power
     */
    public Command applyPower(double powerTop, double powerBottom) {
        return runOnce(() -> setPower(powerTop, powerBottom));
    }

    /**
     * @param power
     * @return runCommand that sets the power
     */
    public Command applyPower(DoubleSupplier power){
        return startEnd(() -> setPower(power.getAsDouble()), () -> stop());
    }

    /**
     * @param powerTop
     * @param powerBottom
     * @return runCommand that sets the power
     */
    public Command applyPower(DoubleSupplier powerTop, DoubleSupplier powerBottom){
        return startEnd(() -> setPower(powerTop.getAsDouble(), powerBottom.getAsDouble()), () -> stop());
    }
    
    public void setVelocityTop(AngularVelocity velocity) {
        this.topTargetVelocity = velocity;
        shooterMotorTop.setControl(velocityPID.withVelocity(velocity));
    }

    public void setVelocityBottom(AngularVelocity velocity) {
        this.bottomTargetVelocity = velocity;
        shooterMotorBottom.setControl(velocityPID.withVelocity(velocity));
    }

    public void setVelocity(AngularVelocity velocity) {
        setVelocity(velocity, velocity);
    }

    public void setVelocity(AngularVelocity velocityTop, AngularVelocity velocityBottom) {
        setVelocityTop(velocityTop);
        setVelocityBottom(velocityBottom);
    }

    public Command applyVelocity(AngularVelocity velocity) {
        return runOnce(() -> setVelocity(velocity));
    }

    public Command applyVelocity(Supplier<AngularVelocity> velocityTop, Supplier<AngularVelocity> velocityBottom) {
        return startEnd(() -> setVelocity(velocityTop.get(), velocityBottom.get()), () -> stop());
    }

    public Command applyVelocity(Supplier<AngularVelocity> velocity) {
        return startEnd(() -> setVelocity(velocity.get()), () -> stop());
    }

    public AngularVelocity getBottomVelocity() {
        return shooterMotorBottom.getVelocity().getValue();
    }

    public AngularVelocity getTopVelocity() {
        return shooterMotorTop.getVelocity().getValue();
    }

    public AngularVelocity getAverageVelocity() {
        return getBottomVelocity().plus(getTopVelocity()).div(2);
    }

    public boolean bottomOnTarget(){
        return getBottomVelocity().isNear(bottomTargetVelocity, ShooterConstants.TOLERANCE);
    }

    public boolean topOnTarget(){
        return getTopVelocity().isNear(topTargetVelocity, ShooterConstants.TOLERANCE);
    }

    public boolean onTarget(){
        return bottomOnTarget() && topOnTarget();
    }

    public boolean getShooterCurrentHit(){
        return shooterMotorTop.getStatorCurrent().getValue().minus(ShooterConstants.THRESHHOLD).magnitude() > 0
            && shooterMotorBottom.getStatorCurrent().getValue().minus(ShooterConstants.THRESHHOLD).magnitude() > 0;
    }
}
