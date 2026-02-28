// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.swerve.SwerveDrive;

import org.team157.robot.Constants.FieldConstants;
import org.team157.robot.Constants.ModelConstants;
import org.team157.robot.Constants.TurretConstants;
import org.team157.robot.Constants.VisionConstants;
import org.team157.robot.Robot;

@Logged(strategy = Strategy.OPT_OUT)
public class VisionSystem extends SubsystemBase {

  // Publishes the turret's target point to NT for field zoning testing.
  public StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Target Pose", Pose2d.struct).publish();

  public boolean hasTag = false;

  public static double angleToTarget = 0;
  public static double distanceToTarget = 0;
  public static double distanceToTargetFromTurret = 0;
  public static double angleToTargetFromTurret = 0;
  public static double lastDistanceToTarget = 0;
  public static double lastTrackedTime = 0;

  // TODO: move to constants.java
  final PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
  final PoseStrategy fallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY;

  PhotonCamera frontRightCamera;
  PhotonCamera frontLeftCamera;
  PhotonCamera topBackCamera;
  PhotonCamera turretCamera;
  public static AprilTagFieldLayout fieldLayout;
  PhotonPoseEstimator poseEstimatorFrontRight;
  PhotonPoseEstimator poseEstimatorFrontLeft;
  PhotonPoseEstimator poseEstimatorTopBack;
  PhotonPoseEstimator poseEstimatorTurret;
  // EstimatedRobotPose currentEstimatedPose = new EstimatedRobotPose(new Pose3d(), getTimeStamp(), new ArrayList<>(),
  //     poseStrategy);
  PhotonPipelineResult latestResult;
  public List<PhotonPipelineResult> resultsList = new ArrayList<>();

  // private LEDSystem prettyLights;

  public Optional<Pose2d> desiredPose = Optional.empty();
  public Field2d field2d;
  private final double maximumAmbiguity = 0.25;
  private double longDistangePoseEstimationCount = 0;
  private Supplier<Pose2d> currentPose;

  boolean isBlueAlliance = true;
  
  
    /** Creates a new vision. */
    public VisionSystem(Supplier<Pose2d> currentPose, Field2d field) {
  
      this.currentPose = currentPose;
      this.field2d = field;
      Shuffleboard.getTab("vision").add("vision based field", field2d).withWidget(BuiltInWidgets.kField);
  
      try {
        fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltAndymark.m_resourceFile);
      } catch (IOException exception) {
        frontRightCamera.close();
        frontLeftCamera.close();
        topBackCamera.close();
        turretCamera.close();
        throw new RuntimeException(exception);
      }
    
      PortForwarder.add(5800, "photonvision1.local", 5800);
      PortForwarder.add(5800, "photonvision2.local", 5800);
      setDefaultCommand(getDefaultCommand());
    }
  
    public Command getDefaultCommand(DriveSystem drivetrain, TurretSystem turret) {
      return run(() -> {
        updatePoseEstimation(drivetrain);
        turret.updateRelativeAngleToTag(FieldConstants.positionDetails.targetPose2d(drivetrain.getPose(), isBlueAlliance), drivetrain.getPose());
        // turret.updateRelativeAngleToTag(26, drivetrain.getPose());
  
  
      });
      
    }
    /** 
     * Gets the aiming target of the turret, based on the current alliance, and the robot's current location on the field.
     * @return the target point on the field the turret should be aiming at, as a Pose2d.
     */
    public Pose2d getDesiredPose() {
      return FieldConstants.positionDetails.targetPose2d(currentPose.get(), isBlueAlliance);
    }
  
    public void updateAlliance() {
      isBlueAlliance = DriverStation.getAlliance()
        .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
      SmartDashboard.putBoolean("Is Blue Alliance", isBlueAlliance);
    }
  
      /**
     * Calculates a target pose relative to an AprilTag on the field.
     *
     * @param aprilTag    The ID of the AprilTag.
     * @param robotOffset The offset {@link Transform2d} of the robot to apply to the pose for the robot to position
     *                    itself correctly.
     * @return The target pose of the AprilTag.
     */
    public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
      Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
      if (aprilTagPose3d.isPresent()) {
        return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
      } else {
        throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
      }
  
    }
  
      /**
     * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
     *
     * @param swerveDrive {@link SwerveDrive} instance.
     */
    public void updatePoseEstimation(DriveSystem swerveDrive) {
      for (Cameras camera : Cameras.values()) {
         // ignore turretCamera for global positioning
        if(!camera.useForPositioning) {
          continue;
        }
        Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
        if (poseEst.isPresent()) {
          Optional<EstimatedRobotPose> filteredPose = filterPose(poseEst);
          if (filteredPose.isPresent()) {
            var pose = filteredPose.get();
          
            swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                                            pose.timestampSeconds,
                                            camera.curStdDevs);
            field2d.getObject("Vision").setPose(pose.estimatedPose.toPose2d()); // photon's percieved pose
            field2d.setRobotPose(swerveDrive.getPose()); // photon's pose combined with robot's known pose
          }
        }
      }
  
    }
  
  
    
      /**
     * Reset the pose estimation inside of {@link SwerveDrive} with all of the given poses.
     *
     * @param swerveDrive {@link SwerveDrive} instance.
     */
    public void resetPoseEstimation(DriveSystem swerveDrive) {
      for (Cameras camera : Cameras.values()) {
        // ignore turretCamera for global positioning
        if(!camera.useForPositioning) { 
          continue;
        }
        Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
        if (poseEst.isPresent()) {
          Optional<EstimatedRobotPose> filteredPose = filterPose(poseEst);
          if (filteredPose.isPresent()) {
            var pose = filteredPose.get();
  
            swerveDrive.resetPose(pose.estimatedPose.toPose2d());
  
            field2d.getObject("Vision").setPose(pose.estimatedPose.toPose2d()); // photon's percieved pose
            field2d.setRobotPose(swerveDrive.getPose()); // photon's pose combined with robot's known pose
  
            break;
          }
        }
      }
  
    }
      /**
     * Generates the estimated robot pose. Returns empty if:
     * <ul>
     *  <li> No Pose Estimates could be generated</li>
     * <li> The generated pose estimate was considered not accurate</li>
     * </ul>
     *
     * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to create the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
      Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
      return poseEst;
    }
  
    /**
     * Filter pose via the ambiguity and find best estimate between all of the camera's throwing out distances more than
     * 10m for a short amount of time.
     *
     * @param pose Estimated robot pose.
     * @return Could be empty if there isn't a good reading.
     */
    private Optional<EstimatedRobotPose> filterPose(Optional<EstimatedRobotPose> pose) {
      if (pose.isPresent()) {
        double bestTargetAmbiguity = 1; // 1 is max ambiguity
        for (PhotonTrackedTarget target : pose.get().targetsUsed) {
          double ambiguity = target.getPoseAmbiguity();
          if (ambiguity != -1 && ambiguity < bestTargetAmbiguity) {
            bestTargetAmbiguity = ambiguity;
          }
        }
        //ambiguity to high dont use estimate
        if (bestTargetAmbiguity > maximumAmbiguity) {
          return Optional.empty();
        }
  
        //est pose is very far from recorded robot pose
        if (PhotonUtils.getDistanceToPose(currentPose.get(), pose.get().estimatedPose.toPose2d()) > 1) {
          longDistangePoseEstimationCount++;
  
          //if it calculates that were 10 meter away for more than 10 times in a row its probably right
          if (longDistangePoseEstimationCount < 10) {
            return Optional.empty();
          }
        } else {
          longDistangePoseEstimationCount = 0;
        }
        return pose;
      }
      return Optional.empty();
    }
  
    /**
     * Get distance of the robot from the AprilTag pose.
     *
     * @param id AprilTag ID
     * @return Distance
     */
    public double getDistanceFromAprilTag(int id) {
      Optional<Pose3d> tag = fieldLayout.getTagPose(id);
      return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
    }
  
    /**
     * Get tracked target from a camera of AprilTagID
     *
     * @param id     AprilTag ID
     * @param camera Camera to check.
     * @return Tracked target.
     */
    public PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
      PhotonTrackedTarget target = null;
      for (PhotonPipelineResult result : camera.resultsList) {
        if (result.hasTargets()) {
          for (PhotonTrackedTarget i : result.getTargets()) {
            if (i.getFiducialId() == id) {
              return i;
            }
          }
        }
      }
      return target;
  
    }
  
    /**
     * Update the {@link Field2d} to include tracked targets/
     */
    public void updateVisionField() {
  
      List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
      for (Cameras camera : Cameras.values()) {
         // ignore turretCamera for global positioning
        if(!camera.useForPositioning) {
          continue;
        }
        if (!camera.resultsList.isEmpty()) {
          PhotonPipelineResult latest = camera.resultsList.get(0);
          if (latest.hasTargets()) {
            targets.addAll(latest.targets);
          }
        }
      }
  
      List<Pose2d> poses = new ArrayList<>();
      for (PhotonTrackedTarget target : targets) {
        if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
          Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
          poses.add(targetPose);
        }
      }
  
      field2d.getObject("tracked targets").setPoses(poses);
    }
      /**
     * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
     *
     * @param swerveDrive {@link SwerveDrive} instance.
     */
    public double getHubTagYawFromTurretCam() {
      Cameras.TURRET_CAM.updateUnreadResults();
      PhotonTrackedTarget target = getTargetFromId(26, Cameras.TURRET_CAM);
      if(target == null) {
        // arbitrary number indicating no target
        return 157357;
      }
  
      return target.yaw;
    }
  
    /**
     * Get a tag's 2d location on the field, and calculate the angle and distance to it from the robot's pose.
     * @param id the tag ID to extract a Pose2d from
     * @param robotPose the current Pose2d of the robot to calculate angle/distance from
     */
    public void setTargetParams(int id, Pose2d robotPose) {
      Pose2d tagPose = fieldLayout.getTagPose(id).get().toPose2d();
  
      distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, tagPose);
      distanceToTargetFromTurret = PhotonUtils.getDistanceToPose(robotPose.plus(ModelConstants.XY_ORIGIN_TO_TURRET_BASE_OFFSET), tagPose);
      angleToTarget = PhotonUtils.getYawToPose(robotPose, tagPose).getDegrees();
      // angleToTargetFromTurret = PhotonUtils.getYawToPose(robotPose.plus(ModelConstants.XY_ORIGIN_TO_TURRET_BASE_OFFSET), tagPose).getDegrees();
      angleToTargetFromTurret = PhotonUtils.getYawToPose(robotPose.plus(ModelConstants.XY_ORIGIN_TO_TURRET_BASE_OFFSET), tagPose).getDegrees();
  
    }
    /**
     * Calculate the angle and distance to a certain target from the robot's pose.
     * @param targetPose the target Pose2d to calculate angle/distance to
     * @param robotPose the current Pose2d of the robot to calculate angle/distance from
     */
    public void setTargetParams(Pose2d targetPose, Pose2d robotPose) {
      distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, targetPose);
      distanceToTargetFromTurret = PhotonUtils.getDistanceToPose(robotPose.plus(ModelConstants.XY_ORIGIN_TO_TURRET_BASE_OFFSET), targetPose);
      // lastDistanceToTarget = distanceToTarget;
      // lastTrackedTime = NetworkTablesJNI.now();
      angleToTarget = PhotonUtils.getYawToPose(robotPose, targetPose).getDegrees();
      angleToTargetFromTurret = PhotonUtils.getYawToPose(robotPose.plus(ModelConstants.XY_ORIGIN_TO_TURRET_BASE_OFFSET), targetPose).getDegrees();
  }

 /**
   * Camera Enum to select each camera
   */
  enum Cameras {
    /**
     * Left Camera
     */
    LEFT_CAM(VisionConstants.FRONTLEFT_CAMERA_NICKNAME,
             VisionConstants.FRONTLEFT_CAMERA_ROTATION,
             VisionConstants.FRONTLEFT_CAMERA_TRANSLATION,
             VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),
    /**
     * Right Camera
     */
    RIGHT_CAM(VisionConstants.FRONTRIGHT_CAMERA_NICKNAME,
              VisionConstants.FRONTRIGHT_CAMERA_ROTATION,
              VisionConstants.FRONTRIGHT_CAMERA_TRANSLATION,
              VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),
    /**
     * Back Camera
     */
    BACK_CAM(VisionConstants.BACK_CAMERA_NICKNAME,
               VisionConstants.BACK_CAMERA_ROTATION,
               VisionConstants.BACK_CAMERA_TRANSLATION,
               VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),
    /**
     * Turret Camera
     */
    TURRET_CAM(VisionConstants.TURRET_CAMERA_NICKNAME,
               VisionConstants.TURRET_CAMERA_ROTATION,
               VisionConstants.TURRET_CAMERA_TRANSLATION,
               VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1));

      /**
     * flag to ignore the turret cam for positioning
     */        
    private boolean useForPositioning = true;

    /**
     * Latency alert to use when high latency is detected.
     */
    public final Alert latencyAlert;
    /**
     * Camera instance for comms.
     */
    public final PhotonCamera camera;
    /**
     * Pose estimator for camera.
     */
    public final PhotonPoseEstimator poseEstimator;
    /**
     * Standard Deviation for single tag readings for pose estimation.
     */
    private final Matrix<N3, N1> singleTagStdDevs;
    /**
     * Standard deviation for multi-tag readings for pose estimation.
     */
    private final Matrix<N3, N1> multiTagStdDevs;
    /**
     * Transform of the camera rotation and translation relative to the center of the robot
     */
    private final Transform3d robotToCamTransform;
    /**
     * Current standard deviations used.
     */
    public Matrix<N3, N1> curStdDevs;
    /**
     * Estimated robot pose.
     */
    public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

    /**
     * Simulated camera instance which only exists during simulations.
     */
    public PhotonCameraSim cameraSim;
    /**
     * Results list to be updated periodically and cached to avoid unnecessary queries.
     */
    public List<PhotonPipelineResult> resultsList = new ArrayList<>();
    /**
     * Last read from the camera timestamp to prevent lag due to slow data fetches.
     */
    private double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

    /**
     * Construct a Photon Camera class with help. Standard deviations are fake values, experiment and determine
     * estimation noise on an actual robot.
     *
     * @param name Name of the PhotonVision camera found in the PV UI.
     * @param robotToCamRotation {@link Rotation3d} of the camera.
     * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
     * @param singleTagStdDevs Single AprilTag standard deviations of estimated poses from the camera.
     * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated poses from the camera.
     */
    Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
            Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix) {
      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

      camera = new PhotonCamera(name);

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator = new PhotonPoseEstimator(VisionSystem.fieldLayout,
                                              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                              robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      if(name.equals(VisionConstants.TURRET_CAMERA_NICKNAME)) {
        this.useForPositioning = false;
      } 

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;
    }

    /**
     * Get the result with the least ambiguity from the best tracked target within the Cache. This may not be the most
     * recent result!
     *
     * @return The result in the cache with the least ambiguous best tracked target. This is not the most recent result!
     */
    public Optional<PhotonPipelineResult> getBestResult() {
      if (resultsList.isEmpty()) {
        return Optional.empty();
      }

      PhotonPipelineResult bestResult = resultsList.get(0);
      double amiguity = bestResult.getBestTarget().getPoseAmbiguity();
      double currentAmbiguity = 0;
      for (PhotonPipelineResult result : resultsList) {
        currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
        if (currentAmbiguity < amiguity && currentAmbiguity > 0) {
          bestResult = result;
          amiguity = currentAmbiguity;
        }
      }
      return Optional.of(bestResult);
    }

    /**
     * Get the latest result from the current cache.
     *
     * @return Empty optional if nothing is found. Latest result if something is there.
     */
    public Optional<PhotonPipelineResult> getLatestResult() {
      return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
    }

    /**
     * Get the estimated robot pose. Updates the current robot pose estimation, standard deviations, and flushes the
     * cache of results.
     *
     * @return Estimated pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      updateUnreadResults();
      return estimatedRobotPose;
    }

    /**
     * Update the latest results, cached with a maximum refresh rate of 1req/15ms. Sorts the list by timestamp.
     */
    private void updateUnreadResults() {
      double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
      double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
      double debounceTime = Milliseconds.of(15).in(Seconds);
      for (PhotonPipelineResult result : resultsList) {
        mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
      }

        // resultsList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
        resultsList = Robot.isReal() ? camera.getAllUnreadResults() : new ArrayList<PhotonPipelineResult>(); // ß
        lastReadTimestamp = currentTimestamp;
        resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
          return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
        });
        if (!resultsList.isEmpty()) {
          updateEstimatedGlobalPose();
        }

    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should only be called once
     * per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link Cameras#updateEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets used for
     * estimation.
     */
    private void updateEstimatedGlobalPose() {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : resultsList) {
        visionEst = poseEstimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());
      }
      estimatedRobotPose = visionEst;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard deviations based
     * on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      if (estimatedPose.isEmpty()) {
        // No pose input. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;

      } else {
        // Pose present. Start running Heuristic
        var estStdDevs = singleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets) {
          var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) {
            continue;
          }
          numTags++;
          avgDist +=
              tagPose
                  .get()
                  .toPose2d()
                  .getTranslation()
                  .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
          // No tags visible. Default to single-tag std devs
          curStdDevs = singleTagStdDevs;
        } else {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1) {
            estStdDevs = multiTagStdDevs;
          }
          // Increase std devs based on (average) distance
          if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
          curStdDevs = estStdDevs;
        }
      }
    }


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Publish the turret's target point to NT for field zoning testing.
    targetPosePublisher.set(getDesiredPose());
    updateAlliance();
  }
}