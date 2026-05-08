package org.team157.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team157.robot.subsystems.hood.Hood;
import org.team157.robot.subsystems.slapdown.Slapdown;
import org.team157.robot.subsystems.slapdown.SlapdownConstants;
import org.team157.robot.subsystems.turret.Turret;
import org.team157.utilities.PosUtils;

/**
 * Contains 3D mechanism offset constants for Sunstone V2 and methods to animate the AdvantageScope
 * CAD model.
 */
public class SunstoneV2Mechanism3D extends SubsystemBase {

    private Turret turret;
    private Hood hood;
    private Slapdown slapdown;

    /**
     * Initializes the 3D mechanisms for SunstoneV2, and specifies the mechanisms to extract poses
     * from.
     */
    public SunstoneV2Mechanism3D(Turret turret, Hood hood, Slapdown slapdown) {
        this.turret = turret;
        this.hood = hood;
        this.slapdown = slapdown;
    }

    /**
     * 3D offsets from the robot's origin (center of rotation, roughly equivalent to the location of
     * the Pigeon) to various key points on the robot, used for ballistic calculations and mechanism
     * visualization on the AdvantageScope model.
     */
    public class Mechanism3DConstants {

        /** 3D offset from the robot origin to the turret base, in meters. */
        public static final Translation3d ORIGIN_TO_TURRET_BASE_OFFSET =
                new Translation3d(-0.12192, -0.11557, 0.396535);

        /** 3D offset from the robot origin to the slapdown's pivot point, in meters. */
        public static final Translation3d ORIGIN_TO_SLAPDOWN_PIVOT_POINT_OFFSET =
                new Translation3d(0.146050, 0, 0.197803);

        /** 2D offset from the robot's origin to the turret base, used in ballistic calculations. */
        public static final Transform2d XY_ORIGIN_TO_TURRET_BASE_OFFSET =
                new Transform2d(-0.12192, -0.11557, new Rotation2d());
    }

    /**
     * Gets the 3D pose of the turret base for mechanism visualization.
     *
     * @return The {@link Pose3d} of the turret base.
     */
    public Pose3d getTurretBasePose() {
        return new Pose3d(
                Mechanism3DConstants.ORIGIN_TO_TURRET_BASE_OFFSET,
                new Rotation3d(0, 0, turret.getTurretRotation().in(Radians)));
    }

    /**
     * Gets the 3D transform from the turret base to the hood pivot point.
     *
     * @return The {@link Transform3d} from the turret base to the hood pivot.
     */
    public Transform3d getHoodPivotLocation() {
        return new Transform3d(
                0.1 * Math.cos(turret.getTurretRotation().in(Radians)),
                0.1 * Math.sin(turret.getTurretRotation().in(Radians)),
                0.064,
                new Rotation3d(0, 0, turret.getTurretRotation().in(Radians)));
    }

    /**
     * Gets the 3D pose of the hood pivot point after applying an additional transform.
     *
     * @param rotation The additional {@link Transform3d} to apply to the hood pivot location.
     * @return The resulting {@link Pose3d} of the hood pivot.
     */
    public Pose3d getHoodPivotPose(Transform3d rotation) {
        return new Pose3d(Mechanism3DConstants.ORIGIN_TO_TURRET_BASE_OFFSET, new Rotation3d())
                .transformBy(getHoodPivotLocation())
                .transformBy(rotation);
    }

    /**
     * Gets the current 3D pose of the hood for mechanism visualization.
     *
     * @return a {@link Pose3d} containing the hood's position relative to the robot origin.
     */
    public Pose3d getHoodPose() {
        return getHoodPivotPose(
                new Transform3d(
                        0,
                        0,
                        0,
                        new Rotation3d(0, Math.toRadians(hood.getScaledPosAngleSim()), 0)));
    }

    /**
     * Gets the current position of the hopper walls.
     *
     * @return the current displacement of the hopper walls from their stowed position, in meters.
     */
    public double getHopperWallsPosition() {
        return PosUtils.mapRange(
                slapdown.getSlapdownAngle().in(Radians),
                SlapdownConstants.MIN_ANGLE,
                SlapdownConstants.MAX_ANGLE,
                0.3048,
                0);
    }

    /**
     * Gets the current 3D pose of the hopper walls for mechanism visualization.
     *
     * @return a {@link Pose3d} containing the position of the hopper walls relative to the robot
     *     origin
     */
    public Pose3d getHopperWallsPose() {
        return new Pose3d(getHopperWallsPosition(), 0, 0, new Rotation3d());
    }

    /**
     * Gets the current 3D pose of the slapdown for mechanism visualization.
     *
     * @return a {@link Pose3d} containing the position of the hopper walls relative to the robot
     *     origin
     */
    public Pose3d getSlapdownPose() {
        return new Pose3d(
                Mechanism3DConstants.ORIGIN_TO_SLAPDOWN_PIVOT_POINT_OFFSET,
                new Rotation3d(0, -slapdown.getSlapdownAngle().in(Radians), 0));
    }

    /**
     * Gets the poses of all mechanisms to log via AdvantageKit for mechanism visualization.
     *
     * @return a {@link Pose3d} array containing the poses of the turret, hood, slapdown, and hopper
     *     walls.
     */
    public Pose3d[] getMechanismPoses() {
        return new Pose3d[] {
            getTurretBasePose(), //
            getHoodPose(), //
            getSlapdownPose(), //
            getHopperWallsPose()
        };
    }

    /**
     * Gets an array of blank poses for model calibration
     *
     * @return a {@link Pose3d} array containing blank Pose3d objects.
     */
    public Pose3d[] getCalibrationPoses() {
        return new Pose3d[] {
            new Pose3d(), //
            new Pose3d(), //
            new Pose3d(), //
            new Pose3d()
        };
    }

    @Override
    public void periodic() {
        // Publish poses to NT and/or save them to the log file.
        Logger.recordOutput("Mechanism Poses", getMechanismPoses());
        Logger.recordOutput("Calibration Poses", getCalibrationPoses());
    }
}
