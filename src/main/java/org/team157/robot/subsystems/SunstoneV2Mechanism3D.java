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
 * model.
 */
public class SunstoneV2Mechanism3D extends SubsystemBase {

  private Turret turret;
  private Hood hood;
  private Slapdown slapdown;

  public SunstoneV2Mechanism3D(Turret turret, Hood hood, Slapdown slapdown) {
    this.turret = turret;
    this.hood = hood;
    this.slapdown = slapdown;
  }

  public class Mechanism3DConstants {

    // 3D offsets from the robot's origin (center of rotation) to various key points
    // on the robot, used for mechanism visualization on the AdvantageScope model.
    public static final Translation3d ORIGIN_TO_TURRET_BASE_OFFSET =
        new Translation3d(-0.12192, -0.11557, 0.396535);

    public static final Translation3d ORIGIN_TO_HOOD_PIVOT_POINT_OFFSET =
        new Translation3d(-0.0465, 0, 0.530);

    public static final Translation3d ORIGIN_TO_INTAKE_PIVOT_POINT_OFFSET =
        new Translation3d(0.146050, 0, 0.197803);
    /**
     * 2D offset from the robot's origin to the turret base, used in position-based dynamic shooting
     * calculations.
     */
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
        0.1245 * Math.cos(turret.getTurretRotation().in(Radians)),
        0.1245 * Math.sin(turret.getTurretRotation().in(Radians)),
        0.070,
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
   * Gets the 3D pose of the hood for mechanism visualization.
   *
   * @return The {@link Pose3d} of the hood.
   */
  public Pose3d getHoodPose() {
    return getHoodPivotPose(
        new Transform3d(
            0, 0, 0, new Rotation3d(0, Math.toRadians(hood.getScaledPosAngleSim()), 0)));
  }

  public Pose3d[] getMechanismPoses() {
    return new Pose3d[] {
      getTurretBasePose(), getHoodPose(), getSlapdownPose(), getHopperWallsPose()
    };
  }

  public double getHopperWallsPosition() {
    return PosUtils.mapRange(
        slapdown.getSlapdownAngle().in(Radians),
        SlapdownConstants.MIN_ANGLE,
        SlapdownConstants.MAX_ANGLE,
        0.3048,
        0);
  }

  public Pose3d getHopperWallsPose() {
    return new Pose3d(getHopperWallsPosition(), 0, 0, new Rotation3d());
  }

  public Pose3d getSlapdownPose() {
    return new Pose3d(
        Mechanism3DConstants.ORIGIN_TO_INTAKE_PIVOT_POINT_OFFSET,
        new Rotation3d(0, -slapdown.getSlapdownAngle().in(Radians), 0));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Mechanism Poses", getMechanismPoses());
  }
}
