// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems.hood;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.team157.robot.Constants.HoodConstants;
import org.team157.robot.Constants.ModelConstants;
import org.team157.robot.Constants.TelemetryConstants;
import org.team157.robot.RobotContainer;
import org.team157.robot.subsystems.FlywheelSystem;
import org.team157.utilities.PosUtils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class Hood extends SubsystemBase {
    private HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood() {
    }

    public void setIO(HoodIO io){
        this.io = io;
    }

    public Command setDefault() {
        return io.stop();
    }

    /**
     * Set the target angle of the hood.
     * 
     * @param angle Angle to go to.
     */
    public Command setAngle(Angle angle) {
        return io.setTargetAngle(angle);
    }

    public Command setAngle(Supplier<Angle> angle) {
        return io.setTargetAngle(angle);
    }

/**
     * Set the duty cycle output of the hood motor.
     * Primarily used for manual control
     * 
     * @param dutycycle The power to be applied to the motor.
     */
    public Command set(double dutycycle) {
        return io.set(dutycycle);
    }

    public Command setDynamicHoodAngle() {
        return setAngle(FlywheelSystem::getDesiredHoodAngle);
    }

    /**
     * Get the current angle of the hood, based on the YAMS pivot system, relative
     * to the simulated zero position.
     * 
     * @return The angle of the hood, in degrees, from -180 to 180, using the YAMS
     *         pivot system.
     */
    public double getScaledPosAngleSim() {
        return PosUtils.mapRange(inputs.angleDegrees, HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE,
                HoodConstants.MAX_ANGLE, HoodConstants.MIN_ANGLE) - 40;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
        // This method will be called once per scheduler run
        // Send values to NT to display on Elastic.
        if (TelemetryConstants.TELEMETRY_VERBOSITY == TelemetryVerbosity.HIGH) {
            SmartDashboard.putNumber("Hood Pos", inputs.encoderPositionRotations);
            SmartDashboard.putNumber("Scaled Hood Pos", inputs.scaledEncoderPosition);
            SmartDashboard.putNumber("Hood Angle (Encoder)", inputs.angleFromEncoderDegrees);
            SmartDashboard.putNumber("Hood Angle (YAMS)", inputs.angleDegrees);
        }

        
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        // Updates the intake pivot simulation's values,
        io.simIterate();
    }

    // TODO: make a Mechanism3d class distinct from each subsystem and RobotContainer for storing poses for the model
    public Pose3d getHoodPose() {
        return new Pose3d(ModelConstants.ORIGIN_TO_HOOD_PIVOT_POINT_OFFSET,
                new Rotation3d(0, -(inputs.angleDegrees), 0));
    }

}