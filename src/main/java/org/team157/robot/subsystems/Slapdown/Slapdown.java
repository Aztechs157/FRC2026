package org.team157.robot.subsystems.Slapdown;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;
import org.team157.robot.Constants.IntakeConstants;
import org.team157.robot.Constants.ModelConstants;
import org.team157.utilities.PosUtils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Slapdown extends SubsystemBase {
    
    // The IO interface for interacting with the hood's motor.
    private SlapdownIO io;

    // Inputs from the motor, encoder, and mechanism, to be updated periodically and logged.
    private final SlapdownIOInputsAutoLogged inputs = new SlapdownIOInputsAutoLogged();


    /** Creates a new Slapdown. */
    public Slapdown() {
        
    }

    /** 
     * Specifies the IO implementation to be used for the Slapdown.
     * 
     * @param io An implementation of the Slapdown's IO layer, i.e. SlapdownIOTalonFX
     */
    public void setIO(SlapdownIO io){
        this.io = io;
    }

    /**
     * Set the target angle of the slapdown.
     * 
     * @param angle Angle to go to.
     */
    public Command setAngle(Angle angle) {
        return io.setTargetAngle(angle).finallyDo(() -> io.stop());
    }

    /**
     * Set the target angle of the slapdown, and stop once the pivot is oscillating about its setpoint.
     * 
     * @param angle Angle to go to.
     */
    public Command setAngleThenStop(Angle angle) {
        return setAngle(angle)
            .until(() -> PosUtils.isOscillating(angle.in(Degrees), inputs.angleDegrees, 2.0, 0.0, 1.0));

    }

    /**
     * Deploys the slapdown intake.
     * 
     * @return a {@link Command} setting the slapdown's target angle to 0°.
     */
    public Command deployIntake() {
        return setAngleThenStop(Degrees.of(0));
    }

    public Command deployIntakeAndHold() {
        return setAngle(Degrees.of(0)); //TODO: confirm weither we still need the hold current and/or/also consider setting the angle to a negative to constantly press intake down and have pid at the same time
    }

    /**
     * Retracts the slapdown intake.
     * 
     * @return a {@link Command} setting the slapdown's target angle to 78°.
     */
    public Command retractIntake() {
        return setAngleThenStop(Degrees.of(78));
    }

    /** 
     * Quickly moves the slapdown up and down to agitate fuel.
     * 
     * @return a {@link Command} setting the slapdown's target angle to 40°, then back down to 0°.
     */
    public Command wiggleIntake() {
        return setAngleThenStop(Degrees.of(40)).andThen(setAngleThenStop(Degrees.of(0)));
    }

    /**
     * Set the duty cycle output of the slapdown motor.
     * Primarily used for manual control
     * 
     * @param dutycycle The power to be applied to the motor.
     */
    public Command set(double dutycycle) {
        return io.set(dutycycle);
    }


    public Command getDefault() {
        return io.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Updates the inputs to be logged by AdvantageKit and writes them to the Logger
        io.updateInputs(inputs);
        Logger.processInputs("Slapdown", inputs);

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation. 
        io.simIterate();
    }

    public double getHopperWallsPosition() {
        return PosUtils.mapRange(inputs.angleFromEncoderDegrees, IntakeConstants.MIN_ANGLE, IntakeConstants.MAX_ANGLE, 0.3048, 0);
    }

    public Pose3d getHopperWallsPose() {
        return new Pose3d(getHopperWallsPosition(), 0, 0, new Rotation3d());
    }

    public Pose3d getIntakePivotPose() {
        return new Pose3d(ModelConstants.ORIGIN_TO_INTAKE_PIVOT_POINT_OFFSET,
            new Rotation3d(0, -Math.toRadians(inputs.angleDegrees), 0));
    }
}
