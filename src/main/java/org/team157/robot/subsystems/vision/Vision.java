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
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.team157.robot.Constants.FieldConstants;
import org.team157.robot.subsystems.SunstoneMechanism3D.Mechanism3DConstants;
import org.team157.robot.subsystems.drive.Drive;
import org.team157.robot.subsystems.flywheel.Flywheel;
import org.team157.robot.subsystems.turret.Turret;
import org.team157.robot.subsystems.vision.VisionIO.PoseObservationType;

public class Vision extends SubsystemBase {
    // Pre-computed turret-to-robot-center geometry. These depend only on
    // Mechanism3DConstants.XY_ORIGIN_TO_TURRET_BASE_OFFSET, so they're constant per match.
    // Replaces the per-tick atan/sin/cos/hypot calls that used to happen inside
    // setTargetParams() alongside the SimpleMatrix allocations.
    private static final double TURRET_TO_ROBOT_THETA =
            Math.atan(
                    Mechanism3DConstants.XY_ORIGIN_TO_TURRET_BASE_OFFSET.getY()
                            / Mechanism3DConstants.XY_ORIGIN_TO_TURRET_BASE_OFFSET.getX());
    private static final double TURRET_TO_ROBOT_NEG_SIN_THETA = -Math.sin(TURRET_TO_ROBOT_THETA);
    private static final double TURRET_TO_ROBOT_COS_THETA = Math.cos(TURRET_TO_ROBOT_THETA);
    private static final double D_OFFSET_ROBOT =
            Math.hypot(
                    Mechanism3DConstants.XY_ORIGIN_TO_TURRET_BASE_OFFSET.getX(),
                    Mechanism3DConstants.XY_ORIGIN_TO_TURRET_BASE_OFFSET.getY());

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

    // Reusable scratch lists for periodic() — avoids 16 LinkedList allocations per loop tick
    // (4 method-scope + 4 per-camera × 3 cameras). Cleared at the start of each scope.
    private final List<Pose3d> allTagPoses = new ArrayList<>();
    private final List<Pose3d> allRobotPoses = new ArrayList<>();
    private final List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
    private final List<Pose3d> allRobotPosesRejected = new ArrayList<>();
    private final List<Pose3d> cameraTagPoses = new ArrayList<>();
    private final List<Pose3d> cameraRobotPoses = new ArrayList<>();
    private final List<Pose3d> cameraRobotPosesAccepted = new ArrayList<>();
    private final List<Pose3d> cameraRobotPosesRejected = new ArrayList<>();

    public Vision(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

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
                            "Vision camera " + Integer.toString(i) + " is disconnected.",
                            AlertType.kWarning);
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
                            FieldConstants.positionDetails.getTargetPose2d(
                                    drivetrain.getPose(), isBlueAlliance),
                            drivetrain.getPose());
                    driveLinearVelocityX = drivetrain.getChassisSpeeds().vxMetersPerSecond;
                    driveLinearVelocityY = drivetrain.getChassisSpeeds().vyMetersPerSecond;
                    driveRotationalVelocity = drivetrain.getChassisSpeeds().omegaRadiansPerSecond;
                    driveFieldRotation = drivetrain.getPose().getRotation().getRadians();
                    ballTOF = Flywheel.getBallTimeOfFlight();
                    Logger.recordOutput(
                            "Targeting/Target Pose", getDesiredPose(drivetrain.getPose()));
                });
    }

    /**
     * Gets the aiming target of the turret, based on the current alliance, and the robot's current
     * location on the field.
     *
     * @return the target point on the field the turret should be aiming at, as a Pose2d.
     */
    public Pose2d getDesiredPose(Pose2d robotPose) {
        return FieldConstants.positionDetails.getTargetPose2d(robotPose, isBlueAlliance);
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

        // Momentum-shooting math, expanded from the original SimpleMatrix formulation into
        // primitive doubles. Each step below corresponds to one matrix operation in the old
        // code — see git history for the original vector form.

        // Turret tangential velocity in robot frame, from chassis rotation about robot center.
        double rotationalLinearSpeed = driveRotationalVelocity * D_OFFSET_ROBOT;
        double vRotationRobotX = TURRET_TO_ROBOT_NEG_SIN_THETA * rotationalLinearSpeed;
        double vRotationRobotY = TURRET_TO_ROBOT_COS_THETA * rotationalLinearSpeed;

        // Rotate that velocity into field frame using the robot heading.
        double cosFieldRot = Math.cos(driveFieldRotation);
        double sinFieldRot = Math.sin(driveFieldRotation);
        double vRotationFieldX = cosFieldRot * vRotationRobotX - sinFieldRot * vRotationRobotY;
        double vRotationFieldY = sinFieldRot * vRotationRobotX + cosFieldRot * vRotationRobotY;

        // Total shooter velocity in field frame = rotational contribution + chassis linear.
        double vShooterX = vRotationFieldX + driveLinearVelocityX;
        double vShooterY = vRotationFieldY + driveLinearVelocityY;

        // Lead the target by ballTOF seconds: subtract shooter velocity × TOF from target.
        double adjustedX = targetPose.getX() - vShooterX * ballTOF;
        double adjustedY = targetPose.getY() - vShooterY * ballTOF;

        Pose2d adjustedTargetPose = new Pose2d(adjustedX, adjustedY, targetPose.getRotation());

        // If the hub is the target, rotate the target about the hub by the drive orientation
        if (Math.round(targetPose.getY())
                == Math.round(FieldConstants.FIELD_WIDTH.magnitude() / 2)) {
            adjustedTargetPose =
                    adjustedTargetPose.rotateAround(
                            targetPose.getTranslation(), new Rotation2d(driveFieldRotation));
        } else {
            adjustedTargetPose =
                    adjustedTargetPose.rotateAround(
                            targetPose.getTranslation(),
                            new Rotation2d(Math.PI - driveFieldRotation));
        }
        distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, adjustedTargetPose);
        distanceToTargetFromTurret =
                PhotonUtils.getDistanceToPose(
                        robotPose.plus(Mechanism3DConstants.XY_ORIGIN_TO_TURRET_BASE_OFFSET),
                        adjustedTargetPose);

        angleToTarget = PhotonUtils.getYawToPose(robotPose, adjustedTargetPose).getDegrees();
        angleToTargetFromTurret =
                PhotonUtils.getYawToPose(
                                robotPose.plus(
                                        Mechanism3DConstants.XY_ORIGIN_TO_TURRET_BASE_OFFSET),
                                adjustedTargetPose)
                        .getDegrees();

        Logger.recordOutput("Targeting/Adjusted Target Pose", adjustedTargetPose);
        Logger.recordOutput("Targeting/Distance to Target", distanceToTarget);
        Logger.recordOutput("Targeting/Angle to Target", angleToTarget);
        Logger.recordOutput("Targeting/Distance to Target from Turret", distanceToTargetFromTurret);
        Logger.recordOutput("Targeting/Angle to Target from Turret", angleToTargetFromTurret);
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        // Reset summary scratch lists for this loop
        allTagPoses.clear();
        allRobotPoses.clear();
        allRobotPosesAccepted.clear();
        allRobotPosesRejected.clear();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Reset per-camera scratch lists for this camera
            cameraTagPoses.clear();
            cameraRobotPoses.clear();
            cameraRobotPosesAccepted.clear();
            cameraRobotPosesRejected.clear();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    cameraTagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose =
                        observation.tagCount() == 0 // Must have at least one tag
                                || (observation.tagCount() == 1
                                        && observation.ambiguity()
                                                > maxAmbiguity) // Cannot be high ambiguity
                                || Math.abs(observation.pose().getZ())
                                        > maxZError // Must have realistic Z coordinate

                                // Must be within the field boundaries
                                || observation.pose().getX() < 0.0
                                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                                || observation.pose().getY() < 0.0
                                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                cameraRobotPoses.add(observation.pose());
                if (rejectPose) {
                    cameraRobotPosesRejected.add(observation.pose());
                } else {
                    cameraRobotPosesAccepted.add(observation.pose());
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
                    cameraTagPoses.toArray(new Pose3d[0]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                    cameraRobotPoses.toArray(new Pose3d[0]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                    cameraRobotPosesAccepted.toArray(new Pose3d[0]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                    cameraRobotPosesRejected.toArray(new Pose3d[0]));
            allTagPoses.addAll(cameraTagPoses);
            allRobotPoses.addAll(cameraRobotPoses);
            allRobotPosesAccepted.addAll(cameraRobotPosesAccepted);
            allRobotPosesRejected.addAll(cameraRobotPosesRejected);
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
