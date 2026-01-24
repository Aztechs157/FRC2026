// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team157.robot.Constants.VisionConstants;
import org.team157.robot.Robot;

@Logged(strategy = Strategy.OPT_OUT)
public class VisionSystem extends SubsystemBase {

  public boolean hasTag = false;

  // TODO: move to constants.java
  final PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
  final PoseStrategy fallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY;

  PhotonCamera frontRightCamera;
  PhotonCamera frontLeftCamera;
  PhotonCamera topBackCamera;
  public static AprilTagFieldLayout fieldLayout;
  PhotonPoseEstimator poseEstimatorFrontRight;
  PhotonPoseEstimator poseEstimatorFrontLeft;
  PhotonPoseEstimator poseEstimatorTopBack;
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

  boolean blueAlliance = true;

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
      throw new RuntimeException(exception);
    }
    
    
    
    
    
    // // this.prettyLights = prettyLights;
    // frontRightCamera = new PhotonCamera(VisionConstants.FRONTRIGHT_CAMERA_NICKNAME);
    // frontLeftCamera = new PhotonCamera(VisionConstants.FRONTLEFT_CAMERA_NICKNAME);
    // topBackCamera = new PhotonCamera(VisionConstants.BACK_CAMERA_NICKNAME);
    PortForwarder.add(5800, "photonvision1.local", 5800);
    PortForwarder.add(5800, "photonvision2.local", 5800);

    

    // poseEstimatorFrontRight = new PhotonPoseEstimator(fieldLayout, poseStrategy,
    //     VisionConstants.FRONTRIGHT_CAMERA_PLACEMENT); // TODO: decide which pose strategy to use
    // poseEstimatorFrontLeft = new PhotonPoseEstimator(fieldLayout, poseStrategy,
    //     VisionConstants.FRONTLEFT_CAMERA_PLACEMENT); // TODO: decide which pose strategy to use
    // poseEstimatorTopBack = new PhotonPoseEstimator(fieldLayout, poseStrategy,
    //     VisionConstants.BACK_CAMERA_PLACEMENT); // TODO: decide which pose strategy to use

    // Shuffleboard.getTab("vision").add("vision based field", vision_field).withWidget(BuiltInWidgets.kField);
    // Shuffleboard.getTab("vision").add("Desired Position", desiredField).withWidget(BuiltInWidgets.kField);
    // Shuffleboard.getTab("vision").addDouble("bumper to tag", this::getDistanceToTag)
    //     .withWidget(BuiltInWidgets.kNumberBar);
    // periodic();
  }

  public void updateAlliance() {
    var alliance = DriverStation.getAlliance();
    blueAlliance = alliance.get() == DriverStation.Alliance.Blue;
  }

    /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to the pose for the robot to position
   *                    itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset)
  {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent())
    {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else
    {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }

  }

    /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(CommandSwerveDrivetrain swerveDrive)
  {
    for (Cameras camera : Cameras.values())
    {
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (poseEst.isPresent())
      {
        Optional<EstimatedRobotPose> filteredPose = filterPose(poseEst);
        if (filteredPose.isPresent())
        {
          var pose = filteredPose.get();
        
          swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                                          pose.timestampSeconds,
                                          camera.curStdDevs);
          field2d.setRobotPose(pose.estimatedPose.toPose2d()); // photon's percieved pose
          // field2d.setRobotPose(swerveDrive.getPose()); // photon's pose combined with robot's known pose
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
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera)
  {
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
  private Optional<EstimatedRobotPose> filterPose(Optional<EstimatedRobotPose> pose)
  {
    if (pose.isPresent())
    {
      double bestTargetAmbiguity = 1; // 1 is max ambiguity
      for (PhotonTrackedTarget target : pose.get().targetsUsed)
      {
        double ambiguity = target.getPoseAmbiguity();
        if (ambiguity != -1 && ambiguity < bestTargetAmbiguity)
        {
          bestTargetAmbiguity = ambiguity;
        }
      }
      //ambiguity to high dont use estimate
      if (bestTargetAmbiguity > maximumAmbiguity)
      {
        return Optional.empty();
      }

      //est pose is very far from recorded robot pose
      if (PhotonUtils.getDistanceToPose(currentPose.get(), pose.get().estimatedPose.toPose2d()) > 1)
      {
        longDistangePoseEstimationCount++;

        //if it calculates that were 10 meter away for more than 10 times in a row its probably right
        if (longDistangePoseEstimationCount < 10)
        {
          return Optional.empty();
        }
      } else
      {
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
  public double getDistanceFromAprilTag(int id)
  {
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
  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera)
  {
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.resultsList)
    {
      if (result.hasTargets())
      {
        for (PhotonTrackedTarget i : result.getTargets())
        {
          if (i.getFiducialId() == id)
          {
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
  public void updateVisionField()
  {

    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (Cameras c : Cameras.values())
    {
      if (!c.resultsList.isEmpty())
      {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets())
        {
          targets.addAll(latest.targets);
        }
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets)
    {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent())
      {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }

















 /**
   * Camera Enum to select each camera
   */
  enum Cameras
  {
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
               VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1));

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
            Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix)
    {
      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

      camera = new PhotonCamera(name);

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator = new PhotonPoseEstimator(VisionSystem.fieldLayout,
                                              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                              robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;
    }

    /**
     * Get the result with the least ambiguity from the best tracked target within the Cache. This may not be the most
     * recent result!
     *
     * @return The result in the cache with the least ambiguous best tracked target. This is not the most recent result!
     */
    public Optional<PhotonPipelineResult> getBestResult()
    {
      if (resultsList.isEmpty())
      {
        return Optional.empty();
      }

      PhotonPipelineResult bestResult = resultsList.get(0);
      double amiguity = bestResult.getBestTarget().getPoseAmbiguity();
      double currentAmbiguity = 0;
      for (PhotonPipelineResult result : resultsList)
      {
        currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
        if (currentAmbiguity < amiguity && currentAmbiguity > 0)
        {
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
    public Optional<PhotonPipelineResult> getLatestResult()
    {
      return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
    }

    /**
     * Get the estimated robot pose. Updates the current robot pose estimation, standard deviations, and flushes the
     * cache of results.
     *
     * @return Estimated pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose()
    {
      updateUnreadResults();
      return estimatedRobotPose;
    }

    /**
     * Update the latest results, cached with a maximum refresh rate of 1req/15ms. Sorts the list by timestamp.
     */
    private void updateUnreadResults()
    {
      double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
      double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
      double debounceTime = Milliseconds.of(15).in(Seconds);
      for (PhotonPipelineResult result : resultsList)
      {
        mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
      }

        resultsList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
        lastReadTimestamp = currentTimestamp;
        resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
          return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
        });
        if (!resultsList.isEmpty())
        {
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
    private void updateEstimatedGlobalPose()
    {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : resultsList)
      {
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
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets)
    {
      if (estimatedPose.isEmpty())
      {
        // No pose input. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;

      } else
      {
        // Pose present. Start running Heuristic
        var estStdDevs = singleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets)
        {
          var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty())
          {
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

        if (numTags == 0)
        {
          // No tags visible. Default to single-tag std devs
          curStdDevs = singleTagStdDevs;
        } else
        {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1)
          {
            estStdDevs = multiTagStdDevs;
          }
          // Increase std devs based on (average) distance
          if (numTags == 1 && avgDist > 4)
          {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else
          {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
          curStdDevs = estStdDevs;
        }
      }
    }


  }
}


//   @NotLogged
//   public PhotonTrackedTarget findBestTargetReef() {
//     if (latestResult.hasTargets()) {
//       var results = new ArrayList<PhotonTrackedTarget>(latestResult.getTargets());
//       results.sort((target1, target2) -> Double.compare(target1.area, target2.area));

//       return results.get(results.size() - 1);
//     }

//     return null;
//   }

//   public double getDistanceToTag() {
//     if (hasTag) {
//       if (latestResult.targets.size() > 0) {
//         var tagID = latestResult.targets.get(0).fiducialId;
//         var tagPos = tagLayout.getTagPose(tagID).get();
//         var estimatedPos = getEstimatedGlobalPose2d();
//         var distance = estimatedPos.getTranslation().getDistance(tagPos.toPose2d().getTranslation());
//         var bumperLen = 0.45085;
//         return distance - bumperLen;
//       }
//     }

//     return 0;
//   }

//   /*
//    * Gets the yaw of the target in degrees (positive right).
//    */

//   public double getTargetYaw(PhotonTrackedTarget target) {
//     return target.getYaw();
//   }

//   /*
//    * Get the pitch of the target in degrees (positive up).
//    */

//   public double getTargetPitch(PhotonTrackedTarget target) {
//     return target.getPitch();
//   }

//   /*
//    * Get the area of the target (how much of the camera feed the bounding box
//    * takes up) as a percent (0-100).
//    */

//   public double getTargetArea(PhotonTrackedTarget target) {
//     return target.getArea();
//   }

//   /*
//    * Get the skew of the target (the skew of the target in degrees
//    * counter-clockwise positive).
//    */

//   public double getTargetSkew(PhotonTrackedTarget target) {
//     return target.getSkew();
//   }

//   // /* TODO: Added in docs but not in library
//   // * Get 4 corners of the minimum bounding box rectagle
//   // */

//   // public List<TargetCorner> getTargetCorners(PhotonTrackedTarget target) {
//   // return target.getCorners();
//   // }

//   /*
//    * Get the ID of the detected fiducial marker.
//    */

//   public int getTargetID(PhotonTrackedTarget target) {
//     return target.getFiducialId();
//   }

//   /*
//    *
//    */

//   public double getPoseAmbiguity(PhotonTrackedTarget target) {
//     return target.getPoseAmbiguity();
//   }

//   // /* TODO: Included in docs but not in library
//   // * Get the transform that maps camera space (X = forward, Y = left, Z = up) to
//   // * object/fiducial tag space (X forward, Y left, Z up) with the lowest
//   // * reprojection error.
//   // */

//   // public Transform2d getCameraToTarget(PhotonTrackedTarget target) {
//   // return target.getCameraToTarget();
//   // }

//   /*
//    * Get the transform that maps camera space (X = forward, Y = left, Z = up) to
//    * object/fiducial tag space (X forward, Y left, Z up) with the lowest
//    * reprojection error.
//    */

//   public Transform3d getBestPathToTarget(PhotonTrackedTarget target) {
//     return target.getBestCameraToTarget();
//   }

//   /*
//    * Get the transform that maps camera space (X = forward, Y = left, Z = up) to
//    * object/fiducial tag space (X forward, Y left, Z up) with the highest
//    * reprojection error.
//    */

//   public Transform3d getOtherPathToTarget(PhotonTrackedTarget target) {
//     return target.getAlternateCameraToTarget();
//   }

//   // @NotLogged
//   // public Optional<EstimatedRobotPose> getEstimatedGlobalPoseTopRight() {
//   // // TODO: if re-added, then implement like `getEstimatedGlobalPose`.
//   // }

//   // @NotLogged
//   // public EstimatedRobotPose getEstimatedGlobalPoseBottom() {
//   // return getEstimatedGlobalPose();
//   // }

//   @NotLogged
//   public double getTimeStamp() {
//     if (getEstimatedGlobalPose() != null) {
//       return getEstimatedGlobalPose().timestampSeconds;
//     } else {
//       return 0.0;
//     }
//   }

//   /*
//    * Estimate the position of the robot relitive to the field.
//    */
//   @NotLogged
//   public EstimatedRobotPose getEstimatedGlobalPose() {
//     return currentEstimatedPose;
//   }

//   public Pose2d getEstimatedGlobalPose2d() {
//     return getEstimatedGlobalPose().estimatedPose.toPose2d();
//   }

//   /*
//    * Get the position of the tag relitive to the field.
//    */

//   public Optional<Pose3d> getTagPose(int targetID) {
//     return tagLayout.getTagPose(targetID); // TODO: make this return a non-optional Pose3d
//   }

//   /*
//    * Get the angle of the tag on the field in radians
//    */
//   public double getTagFieldAngle(int targetID) {
//     return tagLayout.getTagPose(targetID).get().getRotation().getAngle();
//   }

//   /*
//    * Calculate your robot’s Pose3d on the field using the pose of the AprilTag
//    * relative to the camera, pose of the AprilTag relative to the field, and the
//    * transform from the camera to the origin of the robot.
//    */
//   // TODO: Only use function if
//   // (tagLayout.getTagPose(target.getFiducialId()).isPresent())

//   public Pose3d getFieldRelativePose(Pose3d tagPose, Transform3d cameraToTarget) {
//     return PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, tagPose, VisionConstants.FRONTLEFT_CAMERA_PLACEMENT);
//   }

//   // TODO: Define 2d version of camera placement to use this function
//   // public Pose2d getFieldRelativePose( Pose2d tagPose, Transform2d
//   // cameraToTarget) {
//   // return PhotonUtils.estimateFieldToRobot(cameraToTarget, tagPose,
//   // VisionConstants.LEFT_CAMERA_PLACEMENT);
//   // }

//   /*
//    * Calculate the distance to the target based on the hieght of the camera off of
//    * the ground, the hieght of the target off of the ground, the camera’s pitch,
//    * and the pitch to the target.
//    */

//   public double getDistanceToTarget(double targetHeight, double cameraPitch,
//       double targetPitch) {
//     return PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.FRONTRIGHT_CAMERA_PLACEMENT.getY(), targetHeight,
//         cameraPitch,
//         Units.degreesToRadians(targetPitch)); // TODO: convert cameraPitch to use a constant
//   }

//   /*
//    * Calculate the distance between two poses. This is useful when using
//    * AprilTags, given that there may not be an AprilTag directly on the target.
//    */

//   public double getDistanceToPose(Pose2d robotPose, Pose2d targetPose) {
//     return PhotonUtils.getDistanceToPose(robotPose, targetPose);
//   }

//   /*
//    * Calculate translation to the target based on the distance to the target and
//    * angle to the target (yaw).
//    */

//   public Translation2d getTranslationToTarget(double distanceToTarget, double targetYaw) {
//     return PhotonUtils.estimateCameraToTargetTranslation(distanceToTarget, Rotation2d.fromDegrees(-targetYaw));
//   }

//   /*
//    * Calculate the Rotation2d between your robot and a target. This is useful when
//    * turning towards an arbitrary target on the field.
//    */

//   public Rotation2d getYawToPose(Pose2d robotPose, Pose2d targetPose) {
//     return PhotonUtils.getYawToPose(robotPose, targetPose);
//   }

//   /*
//    * Toggle driver mode on or off. Driver mode is an unfiltered/normal view of the
//    * camera to be used while driving the robot.
//    */

//   public void driverModeToggle(boolean toggleOn) {
//     frontLeftCamera.setDriverMode(toggleOn);
//   }

//   /*
//    * Set the pipeline used by the camera.
//    */

//   public void setPipelineIndex(int index) {
//     frontLeftCamera.setPipelineIndex(index);
//   }

//   public void setDesiredPose(Pose2d pose) {
//     desiredPose = pose == null ? Optional.empty() : Optional.of(pose);
//     if (pose != null) {
//       desiredField.setRobotPose(pose);
//     } else {
//       desiredField.setRobotPose(new Pose2d());
//     }
//   }

//   // /* TODO: Docs say we don't care
//   // * Get the latency of the pipeline in miliseconds.
//   // */

//   // public double getPipelineLatency() {
//   // var visionFrame = leftCamera.getLatestResult();
//   // return visionFrame.getLatencyMillis();
//   // }

//   /*
//    * Set the mode of the camera LED(s).
//    */

//   public void setLED(VisionLEDMode LEDMode) {
//     frontLeftCamera.setLED(LEDMode);
//   }

//   void updatePhotonPipelineResult(PhotonPipelineResult pipelineResult, boolean useTopRight) {
//     latestResult = pipelineResult;
//     if(useTopRight) {
//       var newPose = poseEstimatorFrontRight.update(pipelineResult);
//       if (newPose.isPresent()) {
//         currentEstimatedPose = newPose.get();
//       }
//     }
//     else {
//       var newPose = poseEstimatorTopBack.update(pipelineResult);
//       if (newPose.isPresent()) {
//         currentEstimatedPose = newPose.get();
//       }
//     }
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run

//     List<PhotonPipelineResult> pipelineResultsFrontLeft = frontLeftCamera.getAllUnreadResults();
//     List<PhotonPipelineResult> pipelineResultsFrontRight = frontRightCamera.getAllUnreadResults();
//     List<PhotonPipelineResult> pipelineResultsTopBack = topBackCamera.getAllUnreadResults();
//     topBackCamera.get

//     if(!pipelineResultsFrontLeft.isEmpty()) {
//       if(!pipelineResultsFrontRight.isEmpty()) {
//       if(pipelineResultsFrontRight.get(0).hasTargets()) {
//         hasBotTag = true;
//         for (var pipelineResult : pipelineResultsFrontRight) {
//             updatePhotonPipelineResult(pipelineResult, false);
//         }
//       }
//     }
//       else {
//         hasBotTag = false;
//         if(!pipelineResultsFrontRight.isEmpty()) {
//           if(pipelineResultsFrontRight.get(0).hasTargets()) {
//             hasTopTag = true;
//             for (var pipelineResult : pipelineResultsFrontRight) {
//               updatePhotonPipelineResult(pipelineResult, true);
//             }
//         }
//         else {
//           hasTopTag = false;
//         }
//       }
//     }
//   }
//   hasTag = hasBotTag || hasTopTag;

//     // TODO: verify that periodic doesn't run faster than photonvision, which could
//     // lead to this variable being toggled
//     // true/false rapidly in succession.
//     // if (pipelineResults.isEmpty()) {
//     // hasTag = false;
//     // }

//     var pose2d = getEstimatedGlobalPose().estimatedPose.toPose2d();
//     vision_field.setRobotPose(pose2d);
//     // TODO: might not be necessary?
//     SignalLogger.writeDoubleArray("visionOdometry",
//         new double[] { pose2d.getX(), pose2d.getY(), pose2d.getRotation().getDegrees() });
    
//     LEDPattern tagSeen = LEDPattern.solid(Color.kGreen);
//     LEDPattern tooClose = tagSeen.blink(Seconds.of(0.25));

//     // if (hasTag) {
//     //     // Flashing green pattern when the robot is too close to auto align.
//     //     if(getDistanceToTag() <= VisionConstants.MIN_DISTANCE_TO_TAG) {
//     //       prettyLights.removeTopPattern("Has Tag");
//     //       prettyLights.addTopPattern("Has Tag Too Close", 9, tooClose);
//     //     } else {
//     //       // Solid green pattern when the robot is at a far enough distance to auto align
//     //       prettyLights.removeTopPattern("Has Tag Too Close");
//     //       prettyLights.addTopPattern("Has Tag", 10, tagSeen);
          
//     //     }
//     // } else {
//     //   prettyLights.removeTopPattern("Has Tag Too Close");
//     //   prettyLights.removeTopPattern("Has Tag");
//     // }
//   }
// }
