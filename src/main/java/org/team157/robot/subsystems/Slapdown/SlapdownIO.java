package org.team157.robot.subsystems.Slapdown;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** 
 * Defines the input data to be logged by AdvantageKit, along with methods and
 * {@link Command}s which an implementation of this IO interface must have. 
 */
public interface SlapdownIO {
    
    /** 
     * Represents the set of inputs which are to be logged by AdvantageKit 
     * and updated by an implementation of the {@link SlapdownIO} interface. 
     */
    @AutoLog
    public static class SlapdownIOInputs {
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double appliedVolts = 0.0;
        public double temperatureCelsius = 0.0;
        public double targetAngleDegrees = 0.0;
        public double angleDegrees = 0.0;
        public double encoderPositionRotations = 0.0;
        public double scaledEncoderPosition = 0.0;
        public double angleFromEncoderDegrees = 0.0;
        public double mechanismVelocityDegreesPerSecond = 0.0;
    }

    /** 
     * Updates the inputs to be logged by AdvantageKit. 
     * 
     * @param inputs The set of inputs to be logged, including information on the motor, encoder, and mechanism.
    */
    default void updateInputs(SlapdownIOInputs inputs) {}

    /** Updates the values for the simulated version of the slapdown mechanism. */
    default void simIterate() {}

    /** 
     * Sets the target angle of the slapdown mechanism. 
     * 
     * @param angle Angle to go to
     * @return a {@link Command} setting the target angle of the slapdown to the specified angle.
     */
    default Command setTargetAngle(Angle angle) {
        return Commands.none();
    }

    /** 
     * Sets the target angle of the slapdown mechanism, using a Angle Supplier.  
     * 
     * @param angle Angle to go to
     * @return a {@link Command} setting the target angle of the slapdown to the specified angle.
     */
    default Command setTargetAngle(Supplier<Angle> angle){
        return Commands.none();
    }

    /** 
     * Stops the slapdown pivot.
     * @return a {@link Command} setting the motor's output power to 0.
     */
    default Command stop() {
        return Commands.none();
    }

    /** 
     * Directly sets the output power of the slapdown's motor
     * 
     * @param dutycycle Power to be applied to the motor, from 1 to -1.
     * @return a {@link Command} applying the specified power to the motor.
     */
    default Command set(double dutycycle) {
        return Commands.none();
    } 

}
