// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.team157.robot.subsystems.vision;

import static org.team157.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.team157.robot.Constants.FieldConstants;
import org.team157.robot.Constants.ModelConstants;
import org.team157.robot.subsystems.drive.Drive;
import org.team157.robot.subsystems.flywheel.Flywheel;
import org.team157.robot.subsystems.turret.Turret;
import org.team157.robot.subsystems.vision.VisionIO.PoseObservationType;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private boolean isBlueAlliance = true;

    public double angleToTarget = 0;
  public double distanceToTarget = 0;
  public static double distanceToTargetFromTurret = 0;
  public static double angleToTargetFromTurret = 0;


  private double driveLinearVelocityX;
  private double driveLinearVelocityY;
  private double driveRotationalVelocity;
  private double driveFieldRotation;
  private double ballTOF;
  private Supplier<Pose2d> currentPose;

  public Vision(Supplier<Pose2d> currentPose, VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;
    this.currentPose = currentPose;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

    public Command setDefault(Drive drivetrain, Turret turret) {
    return run(
        () -> {
          turret.updateRelativeAngleToTarget(
              FieldConstants.positionDetails.getTargetPose2d(drivetrain.getPose(), isBlueAlliance),
              drivetrain.getPose());
          driveLinearVelocityX = drivetrain.getChassisSpeeds().vxMetersPerSecond;
          driveLinearVelocityY = drivetrain.getChassisSpeeds().vyMetersPerSecond;
          driveRotationalVelocity = drivetrain.getChassisSpeeds().omegaRadiansPerSecond;
          driveFieldRotation = drivetrain.getPose().getRotation().getRadians();
          ballTOF = Flywheel.getBallTimeOfFlight();
        });
  }

  /**
   * Gets the aiming target of the turret, based on the current alliance, and the robot's current
   * location on the field.
   *
   * @return the target point on the field the turret should be aiming at, as a Pose2d.
   */
  public Pose2d getDesiredPose() {
    return FieldConstants.positionDetails.getTargetPose2d(currentPose.get(), isBlueAlliance);
  }

  public void updateAlliance() {
    isBlueAlliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Blue;
    SmartDashboard.putBoolean("Is Blue Alliance", isBlueAlliance);
  }

    /**
   * Calculate the angle and distance to a certain target from the robot's pose.
   *
   * @param targetPose the target Pose2d to calculate angle/distance to
   * @param robotPose the current Pose2d of the robot to calculate angle/distance from
   */
  public void setTargetParams(Pose2d targetPose, Pose2d robotPose) {

    // beginning vector math for momentum shooting
    double turretToRobotTheta =
        Math.atan(
            ModelConstants.XY_ORIGIN_TO_TURRET_BASE_OFFSET.getY()
                / ModelConstants.XY_ORIGIN_TO_TURRET_BASE_OFFSET.getX());
    SimpleMatrix turretToRobotThetaMatrix =
        new SimpleMatrix(2, 1, true, -Math.sin(turretToRobotTheta), Math.cos(turretToRobotTheta));
    double dOffsetRobot =
        Math.hypot(
            ModelConstants.XY_ORIGIN_TO_TURRET_BASE_OFFSET.getX(),
            ModelConstants.XY_ORIGIN_TO_TURRET_BASE_OFFSET.getY());

    SimpleMatrix vRotationRobot =
        turretToRobotThetaMatrix.scale(driveRotationalVelocity * dOffsetRobot);

    SimpleMatrix robotRotationMatrix =
        new SimpleMatrix(
            2,
            2,
            true,
            new double[] {
              Math.cos(driveFieldRotation),
              -Math.sin(driveFieldRotation),
              Math.sin(driveFieldRotation),
              Math.cos(driveFieldRotation)
            });

    SimpleMatrix vRotationField = robotRotationMatrix.mult(vRotationRobot);

    SimpleMatrix vShooter =
        vRotationField.plus(
            new SimpleMatrix(2, 1, true, driveLinearVelocityX, driveLinearVelocityY));

    SimpleMatrix adjustedTargetPoseMatrix =
        new SimpleMatrix(2, 1, true, targetPose.getX(), targetPose.getY())
            .minus(vShooter.scale(ballTOF));

    Pose2d adjustedTargetPose =
        new Pose2d(
            adjustedTargetPoseMatrix.get(0, 0),
            adjustedTargetPoseMatrix.get(1, 0),
            targetPose.getRotation());

    // If the hub is the target, rotate the target about the hub by the drive orientation
    if (Math.round(targetPose.getY()) == Math.round(FieldConstants.FIELD_WIDTH.magnitude() / 2)) {
      adjustedTargetPose =
          adjustedTargetPose.rotateAround(
              targetPose.getTranslation(), new Rotation2d(driveFieldRotation));
    } else {
      adjustedTargetPose =
          adjustedTargetPose.rotateAround(
              targetPose.getTranslation(), new Rotation2d(Math.PI - driveFieldRotation));
    }
    distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, adjustedTargetPose);
    distanceToTargetFromTurret =
        PhotonUtils.getDistanceToPose(
            robotPose.plus(ModelConstants.XY_ORIGIN_TO_TURRET_BASE_OFFSET), adjustedTargetPose);

    angleToTarget = PhotonUtils.getYawToPose(robotPose, adjustedTargetPose).getDegrees();
    angleToTargetFromTurret =
        PhotonUtils.getYawToPose(
                robotPose.plus(ModelConstants.XY_ORIGIN_TO_TURRET_BASE_OFFSET), adjustedTargetPose)
            .getDegrees();

    Logger.recordOutput("Adjusted Target Pose", adjustedTargetPose);
    Logger.recordOutput("Distance to Target", distanceToTarget);
    Logger.recordOutput("Angle to Target", angleToTarget);
    Logger.recordOutput("Distance to Target from Turret", distanceToTargetFromTurret);
    Logger.recordOutput("Angle to Target from Turret", angleToTargetFromTurret);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
