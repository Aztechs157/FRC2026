// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.parsing;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

/** 
 * The PositionDetails class is responsible for parsing a JSON file containing the coordinates 
 * of various important locations on the field, such as shooting targets and alliance zones. 
 * <br>
 * It provides methods to retrieve the appropriate target location for shooting based on the 
 * robot's current position and alliance color, as well as checking if the robot is under a trench.
 */
public class PositionDetails {
    /** Location of the JSON file containting position information, placed inside the deploy directory. */
    private final String jsonPath = "/positionDetails.json";

    /** Location representing an arbitrary point in the lower half of the blue alliance's passing zone. */
    public Location bluePassingZoneLowLocation;
    /** Location representing an arbitrary point in the upper half of the blue alliance's passing zone. */
    public Location bluePassingZoneHighLocation;
    /** Location representing an arbitrary point in the lower half of the red alliance's passing zone. */
    public Location redPassingZoneLowLocation;
    /** Location representing an arbitrary point in the upper half of the red alliance's passing zone. */
    public Location redPassingZoneHighLocation;
    /** Location representing the center of the blue alliance's Hub. */
    public Location blueHubLocation;
    /** Location representing the center of the red alliance's Hub. */
    public Location redHubLocation;
    /** Rectangle representing the red alliance's zone. */
	public Rectangle2d redAllianceZone;
    /** Rectangle representing the blue alliance's zone. */
	public Rectangle2d blueAllianceZone;
    /** Rectangle representing the lower half of the neutral zone. */
	public Rectangle2d neutralLowZone;
    /** Rectangle representing the upper half of the neutral zone. */
	public Rectangle2d neutralHighZone;
	/** Rectangle representing the area underneath the blue alliance's lower trench. */
    public Rectangle2d blueTrenchLow;
	/** Rectangle representing the area underneath the blue alliance's upper trench. */
	public Rectangle2d blueTrenchHigh;
	/** Rectangle representing the area underneath the red alliance's lower trench. */
	public Rectangle2d redTrenchLow;
	/** Rectangle representing the area underneath the red alliance's upper trench. */
	public Rectangle2d redTrenchHigh;
    
    /** The height of the current shooting target, in meters. */
    public static double height = 0.0;
    
    /**
     * The Location class represents a specific point on the field with x, y, and z coordinates.
     */
    public class Location {
        // The x, y, and z coordinates of the location, 
        private double x;
        private double y;
        private double z;
        // The Pose2d representing the location, containing the x and y coordinates and a blank rotation.
        private Pose2d pose;

        /**
         * Constructs a Location object from a given 
         * set of XYZ coordinates, parsed from the JSON file. 
         * The x and y coordinates are used to create a Pose2d for targeting, 
         * while the z coordinate is stored separately for height information.
         * 
         * @param locationJSON A JsonNode containing the coordinate values for the location, used to construct the Pose2d and store the z value.
         */
        public Location(JsonNode locationJSON) {
            // Parse the x, y, and z coordinates from the JSON node.
            this.x = locationJSON.get("x").asDouble();
            this.y = locationJSON.get("y").asDouble();
            this.z = locationJSON.get("z").asDouble();
            // Create a Pose2d using the x and y values with a blank rotation.
            this.pose = new Pose2d(x, y, new Rotation2d()); 
        }

        /**
         * Get the Pose2d of the location, which 
         * represents the x and y coordinates for targeting.
         * @return The Pose2d of the location, with the x and y
         *  coordinates from the JSON file, and a blank rotation.
         */
        public Pose2d getPose() {
            return this.pose;
        }

        /**
         * Get the Z coordinate of the location, which 
         * represents the height of the shooting target.
         * @return The Z coordinate of the location.
         */
        public double getZ() {
            return this.z;
        }
    }
    /** 
     * The Zone class represents a rectangular area on the field defined by minimum and maximum x and y coordinates.
    */
    public class Zone {
        //  The minimum and maximum X and Y coordinates, used to construct the zone's Rectangle2d.
        private double xMin;
        private double xMax;
        private double yMin;
        private double yMax;

        // The Rectangle2d representing the zone, created from the minimum and maximum x and y coordinates.
        private Rectangle2d rectangle;

        /** 
         * Constructs a Zone object from a given set of XY coordinates, parsed from the JSON file. 
         * 
         * @param zoneJSON A JsonNode containing the coordinate values for the zone, used to construct the Rectangle2d representing the zone.
         */
        public Zone(JsonNode zoneJSON) {
            // Parse the minimum and maximum x and y coordinates from the JSON node.
            this.xMin = zoneJSON.get("xMin").asDouble();
            this.xMax = zoneJSON.get("xMax").asDouble();
            this.yMin = zoneJSON.get("yMin").asDouble();
            this.yMax = zoneJSON.get("yMax").asDouble();
            // Create a Rectangle2d representing the zone, using the minimum and maximum x and y coordinates as opposite corners of the rectangle.
            this.rectangle = new Rectangle2d(
                new Translation2d(this.xMin, this.yMin),
                new Translation2d(this.xMax, this.yMax)
            );
        }

        /** Returns a Rectangle2d representing the zone.
         * 
         * @return a Rectangle2d, defined by the minimum and maximum x and y coordinates of the zone.
         */
        public Rectangle2d getRectangle() {
            return this.rectangle;
        }

    }

    public PositionDetails() {
       File file = new File(Filesystem.getDeployDirectory().toPath() + jsonPath); 
       ObjectMapper objectMapper = new ObjectMapper();

        try {
            // Specify the JSON file to read from, and parse it into a JsonNode object.
            JsonNode root = objectMapper.readTree(file);

            /// Target Locations ///
                // Hub Locations
            this.redHubLocation = new Location(root.get("red").get("hub"));
            this.blueHubLocation = new Location(root.get("blue").get("hub"));
                // Red Passing Targets
            this.redPassingZoneHighLocation = new Location(root.get("red").get("passingZoneHigh"));
            this.redPassingZoneLowLocation = new Location(root.get("red").get("passingZoneLow"));
                // Blue Passing Targets //
            this.bluePassingZoneHighLocation = new Location(root.get("blue").get("passingZoneHigh"));
            this.bluePassingZoneLowLocation = new Location(root.get("blue").get("passingZoneLow"));

            /// Targeting Zones ///
                // Alliance Zones //
            this.redAllianceZone = new Zone(root.get("zones").get("red")).getRectangle();
            this.blueAllianceZone = new Zone(root.get("zones").get("blue")).getRectangle();
                // Neutral Zones //
            this.neutralLowZone = new Zone(root.get("zones").get("neutralLow")).getRectangle();
            this.neutralHighZone = new Zone(root.get("zones").get("neutralHigh")).getRectangle();

            /// Trench Zones ///
                // Blue Trenches //
            this.blueTrenchLow = new Zone(root.get("zones").get("blueTrenchLow")).getRectangle();
            this.blueTrenchHigh = new Zone(root.get("zones").get("blueTrenchHigh")).getRectangle();
                // Red Trenches //
            this.redTrenchLow = new Zone(root.get("zones").get("redTrenchLow")).getRectangle();
            this.redTrenchHigh = new Zone(root.get("zones").get("redTrenchHigh")).getRectangle();

        } catch(IOException e) {
            // Throw an IO exception if the file cannot be found or read, 
            // and print the stack trace for debugging.
            e.printStackTrace();
            return;
        }


    }
    /** 
     * Gets the target Pose2d for the robot to shoot at, based on the robot's current pose and alliance color.
     * Also updates the height variable to the Z value of the target location.
     * 
     * @param currentPose The robot's current pose on the field.
     * @param isBlue Whether the robot is on the blue alliance or not.
     * @return The Pose2d of the target location for shooting.
     */
    public Pose2d targetPose2d(Pose2d currentPose, boolean isBlue) {
       if(isBlue) {
            if(blueAllianceZone.contains(currentPose.getTranslation())) {
                // Alliance Hub Target
                height = blueHubLocation.getZ();
                return blueHubLocation.getPose();
            } else if(neutralLowZone.contains(currentPose.getTranslation())) {
                // Neutral Zone Low Target
                height = bluePassingZoneLowLocation.getZ();
                return bluePassingZoneLowLocation.getPose();
            } else if(neutralHighZone.contains(currentPose.getTranslation())) {
                // Neutral Zone High Target
                height = bluePassingZoneHighLocation.getZ();
                return bluePassingZoneHighLocation.getPose();
            } else {
                // Target the hub by default if the robot is outside of all zones.
                height = blueHubLocation.getZ();
                return blueHubLocation.getPose();
            }
       } else {
            // Assume red alliance if not on blue, to avoid crashes when alliance is improperly set.
            if(redAllianceZone.contains(currentPose.getTranslation())) {
                // Alliance Hub Target
                height = redHubLocation.getZ();
                return redHubLocation.getPose();
            } else if(neutralLowZone.contains(currentPose.getTranslation())) {
                // Neutral Zone Low Target
                height = redPassingZoneLowLocation.getZ();
                return redPassingZoneLowLocation.getPose();
            } else if(neutralHighZone.contains(currentPose.getTranslation())) {
                // Neutral Zone High Target
                height = redPassingZoneHighLocation.getZ();
                return redPassingZoneHighLocation.getPose();
            } else {
                // Target the hub by default if the robot is outside of all zones.
                height = redHubLocation.getZ();
                return redHubLocation.getPose();
            }
       }
    }
    /** 
     * Gets the current height of the robot's shooting target.
     * 
     * @return The Z-axis height of the shooting target, in meters.
     */
    public double getTargetHeight() {
        return height;
    }

    /**
     * Checks if the robot's current pose is underneath any of the four trenches.
     * 
     * @param currentPose the robot's current pose
     * @return true if the pose is inside any trench zone, false otherwise
     */
    public boolean isUnderTrench(Pose2d currentPose) {
        return blueTrenchLow.contains(currentPose.getTranslation())
            || blueTrenchHigh.contains(currentPose.getTranslation())
            || redTrenchLow.contains(currentPose.getTranslation())
            || redTrenchHigh.contains(currentPose.getTranslation());
    }
    
    
}
