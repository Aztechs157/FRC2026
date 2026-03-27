// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems.hood;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team157.robot.Constants.ModelConstants;
import org.team157.robot.subsystems.flywheel.FlywheelSystem;
import org.team157.utilities.PosUtils;

import org.littletonrobotics.junction.Logger;

/** 
 * Represents the Hood subsystem, which adjusts the angle 
 * of the shot in order to reach a certain position. 
 */
public class Hood extends SubsystemBase {

    // The IO interface for interacting with the hood's motor.
    private HoodIO io;

    // Inputs from the motor, encoder, and mechanism, to be updated periodically and logged.
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    /** Creates a Hood. */
    public Hood() {

    }

    /** Specifies the IO implementation to be used for the hood.
     * 
     * @param io An implementation of the Hood's IO layer, i.e. HoodIOTalonFX
     */
    public void setIO(HoodIO io){
        this.io = io;
    }

    /** Sets the default command of the hood, stopping motor output when no other commands are running.
     * 
     * @return Command setting the duty cycle output of the hood's motor to 0
     */
    public Command getDefault() {
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

    /**
     * Set the target angle of the hood, with a Supplier angle.
     * 
     * @param angle Angle to go to.
     */
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

    /**
     * Set the dynamic angle of the hood, for targeting the hub or a passing point.
     * 
     * @return A Command setting the angle of the Hood to the desired angle, 
     *         determined by the FlywheelSystem's ballistics calculations.
     */
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
        // This method will be called once per scheduler run
        // Updates the inputs to be logged by AdvantageKit and writes them to the Logger
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        io.simIterate();
    }

    // TODO: make a Mechanism3d class distinct from each subsystem and RobotContainer for storing poses for the model
    public Pose3d getHoodPose() {
        return new Pose3d(ModelConstants.ORIGIN_TO_HOOD_PIVOT_POINT_OFFSET,
                new Rotation3d(0, -(inputs.angleDegrees), 0));
    }

}