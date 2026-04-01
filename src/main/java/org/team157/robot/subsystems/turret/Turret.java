package org.team157.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;

import org.team157.robot.Constants.ModelConstants;
import org.team157.robot.Robot;
import org.team157.robot.subsystems.vision.VisionSystem;

/**
 * Represents the Turret subsystem, which rotates horizontally
 * to aim the hood and flywheel at a target.
 */
public class Turret extends SubsystemBase {

    // The IO interface for interacting with the turret's motor and encoder.
    private TurretIO io;

    // Inputs from the motor, encoder, and mechanism, to be updated periodically and logged.
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    // The current angle the turret is tracking towards.
    public Angle trackingAngle = Degrees.of(0);

    // Reference to the vision system for target tracking.
    private VisionSystem visionSystem;

    /** Creates a new Turret. */
    public Turret() {}

    /**
     * Specifies the IO implementation and vision system to be used for the Turret.
     *
     * @param io An implementation of the Turret's IO layer, i.e. TurretIOTalonFX
     * @param visionSystem The vision system used for target tracking
     */
    public void setIO(TurretIO io, VisionSystem visionSystem) {
        this.io = io;
        this.visionSystem = visionSystem;
    }

    /**
     * Sets the default command of the turret, stopping
     * motor output when no other commands are running.
     *
     * @return Command setting the duty cycle output of the turret's motor to 0
     */
    public Command getDefault() {
        return io.set(0);
    }

    /**
     * Set the target angle of the turret.
     *
     * @param angle Angle to go to.
     * @return {@link Command} setting the target angle of the turret.
     */
    public Command setAngle(Angle angle) {
        return io.setTargetAngle(angle);
    }

    /**
     * Set the duty cycle output of the turret motor.
     * Primarily used for manual control.
     *
     * @param dutycycle The power to be applied to the motor, between -1 and 1.
     * @return {@link Command} setting the duty cycle of the turret motor.
     */
    public Command set(double dutycycle) {
        return io.set(dutycycle);
    }

    /**
     * Tracks the target tag based on the globally-computed relative angle.
     *
     * @return {@link Command} continuously updating the turret angle to track the target.
     */
    public Command trackTagGlobalRelative() {
        return io.setTargetAngle(this::getTrackingAngle);
    }

    /**
     * Gets the current angle to the turret's target.
     *
     * @return The current tracking angle as an {@link Angle}.
     */
    public Angle getTrackingAngle() {
        return trackingAngle;
    }

    /**
     * Calculates and updates the angle the turret needs to rotate to face the target.
     *
     * @param targetPose The target position on the field as a {@link Pose2d}.
     * @param robotPose  The current robot position on the field as a {@link Pose2d}.
     */
    public void updateRelativeAngleToTarget(Pose2d targetPose, Pose2d robotPose) {
        visionSystem.setTargetParams(targetPose, robotPose);
        double turretToRobotAngleOffset = VisionSystem.angleToTargetFromTurret + TurretConstants.TURRET_ANGLE_OFFSET;
        if (Robot.isReal()) {
            if (turretToRobotAngleOffset > 180) {
                turretToRobotAngleOffset -= 360;
            }
            if (turretToRobotAngleOffset < -180) {
                turretToRobotAngleOffset += 360;
            }
            trackingAngle = Degrees.of(turretToRobotAngleOffset);
        } else {
            // Disable turret offset in simulation, as simulated 0 is forward.
            trackingAngle = Degrees.of(VisionSystem.angleToTargetFromTurret);
        }
    }

    /**
     * Gets the 3D pose of the turret base for mechanism visualization.
     *
     * @return The {@link Pose3d} of the turret base.
     */
    public Pose3d getBasePose() {
        return new Pose3d(ModelConstants.ORIGIN_TO_TURRET_BASE_OFFSET,
            new Rotation3d(0, 0, Math.toRadians(inputs.angleDegrees)));
    }

    /**
     * Gets the 3D transform from the turret base to the hood pivot point.
     *
     * @return The {@link Transform3d} from the turret base to the hood pivot.
     */
    public Transform3d getHoodPivotLocation() {
        return new Transform3d(
            0.1245 * Math.cos(Math.toRadians(inputs.angleDegrees)),
            0.1245 * Math.sin(Math.toRadians(inputs.angleDegrees)),
            0.070,
            new Rotation3d(0, 0, Math.toRadians(inputs.angleDegrees)));
    }

    /**
     * Gets the 3D pose of the hood pivot point after applying an additional transform.
     *
     * @param rotation The additional {@link Transform3d} to apply to the hood pivot location.
     * @return The resulting {@link Pose3d} of the hood pivot.
     */
    public Pose3d getHoodPivotPose(Transform3d rotation) {
        return new Pose3d(ModelConstants.ORIGIN_TO_TURRET_BASE_OFFSET, new Rotation3d())
            .transformBy(getHoodPivotLocation())
            .transformBy(rotation);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simIterate();
    }
}
