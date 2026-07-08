package org.team157.robot.subsystems.uptake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team157.robot.subsystems.turret.Turret;

/**
 * Represents the Uptake subsystem, which feeds balls from the hopper up into the flywheel for
 * shooting.
 */
public class Uptake extends SubsystemBase {

    // The IO interface for interacting with the uptake's motor.
    private UptakeIO io;
    private Turret turret;

    // Inputs from the motor and mechanism, to be updated periodically and logged.
    private final UptakeIOInputsAutoLogged inputs = new UptakeIOInputsAutoLogged();

    /** Creates a new Uptake. */
    public Uptake() {}

    /**
     * Specifies the IO implementation to be used for the Uptake.
     *
     * @param io An implementation of the Uptake's IO layer, i.e. UptakeIOTalonFX
     */
    public void setIO(UptakeIO io, Turret turret) {
        this.io = io;
        this.turret = turret;
    }

    /**
     * Sets the default command of the uptake, stopping motor output when no other commands are
     * running.
     *
     * @return Command setting the duty cycle output of the uptake's motor to 0
     */
    public Command setDefault() {
        return io.stop();
    }

    /**
     * Set the duty cycle of the uptake roller motors.
     *
     * @param dutyCycle The power to be applied to the motors, between -1 and 1.
     * @return {@link Command} setting the duty cycle of the uptake roller motors.
     */
    public Command set(double dutyCycle) {
        return io.set(dutyCycle);
    }

    /**
     * Determines the duty cycle setpoint of the uptake. Runs backwards when turret is not within
     * tolerance, forwards otherwise.
     */
    public double getDutyCycleSetpoint() {
        if (!turret.isWithinTolerance(15.7)) {
            return UptakeConstants.RUN_SPEED_LOW;
        } else {
            return UptakeConstants.RUN_SPEED_HIGH;
        }
    }

    /**
     * Runs the uptake at a dynamic speed based on the current turret state, as determined in
     * setDutyCycleSetpoint()
     *
     * @return a {@link Command} running the uptake at the desired speed
     */
    public Command runUptake() {
        return io.set(this::getDutyCycleSetpoint);
    }

    /**
     * Runs the uptake at a set speed, ignoring the tolerance state of the turret. For use with angu
     *
     * @return aa {@link Command} running the uptake at maximum speed.
     */
    public Command runUptakeIgnoringTolerance() {
        return io.set(UptakeConstants.RUN_SPEED_HIGH);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Update the uptake's setpoint based on the turret's tolerance status
        // Updates the inputs to be logged by AdvantageKit and writes them to the Logger
        io.updateInputs(inputs);
        Logger.processInputs("Uptake", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        io.simIterate();
    }
}
