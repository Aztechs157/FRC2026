// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.team157.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "frontLeftCam";
    public static String camera1Name = "frontRightCam";
    public static String camera2Name = "backCam";

    // Robot to camera transforms
    // Transform:
    //      x, y, z:
    //      All in meters, relative to the Pigeon IMU.
    // Rotation:
    //      roll (about x axis) pitch (about y axis), yaw (about z axis):
    //      All in Radians (convert from degrees), relative to the Pigeon IMU
    // Measure these values in Onshape by selecting the Pigeon
    //      (or origin, if that's easier, but account for the
    //      offset from origin -> pigeon for the linear transforms)
    //      and the lens of the camera you're measuring.
    // Onshape coords to robot coords:
    //      +X -> -Y
    //      +Y -> -X
    //      +Z -> +Z (the same)

    public static Transform3d robotToCamera0 =
            new Transform3d(-0.127, 0.3302, 0.3556, new Rotation3d(0.0, 0.0, Math.toRadians(65)));
    public static Transform3d robotToCamera1 =
            new Transform3d(-0.127, -0.3302, 0.3556, new Rotation3d(0.0, 0.0, Math.toRadians(-65)));
    public static Transform3d robotToCamera2 =
            new Transform3d(-0.32326, 0, 0.3556, new Rotation3d(0.0, 0.0, Math.PI));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
            new double[] {
                1.0, // Camera 0 (frontLeftCam)
                1.0, // Camera 1 (frontRightCam)
                1.0 // Camera 2 (backCam)
            };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
            Double.POSITIVE_INFINITY; // No rotation data available
}
