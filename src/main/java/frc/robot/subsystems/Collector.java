// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CollectorConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;

public class Collector extends SubsystemBase {
    
    private ThunderBird collectorMotor;
    private final DutyCycleOut collectorDutyCycle = new DutyCycleOut(0d);
    
    public Collector() {
        // Initialize the collector motor with its configuration
        collectorMotor = new ThunderBird(RobotMap.COLLECTOR_MOTOR_ID, RobotMap.CANIVORE_CAN_NAME, CollectorConstants.INVERT,
            CollectorConstants.STATOR_LIMIT, CollectorConstants.BRAKE_MODE);
    }

    /**
     * sets the power for the collector motor
     * @param power
     */
    public void setPower(double power) {
        collectorMotor.setControl(collectorDutyCycle.withOutput(power));
    }

    public void stop() {
        collectorMotor.setControl(collectorDutyCycle.withOutput(0));
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
     * @param power
     * @return runCommand that sets the power
     */
    public Command applyPower(DoubleSupplier power){
        return startEnd(() -> setPower(power.getAsDouble()), () -> stop());
    }
}
