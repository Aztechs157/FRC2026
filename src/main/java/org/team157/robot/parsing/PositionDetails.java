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

public class PositionDetails {
    private final String jsonPath = "/positionDetails.json";
    public Location bluePassingZoneLowLocation;
    public Location bluePassingZoneHighLocation;
    public Location redPassingZoneLowLocation;
    public Location redPassingZoneHighLocation;
    public Location blueHubLocation;
    public Location redHubLocation;
	public Rectangle2d redZone;
	public Rectangle2d blueZone;
	public Rectangle2d neutralLowZone;
	public Rectangle2d neutralHighZone;
    
    public class Location {
        private double x;
        private double y;
        private Pose2d pose;

        public Location(JsonNode locationJSON) {
            this.x = locationJSON.get("x").asDouble();
            this.y = locationJSON.get("y").asDouble();
            this.pose = new Pose2d(x, y, new Rotation2d(0)); 
        }

        public Pose2d getPose() {
            return this.pose;
        }
    }

    public PositionDetails() {
       File file = new File(Filesystem.getDeployDirectory().toPath() + jsonPath); 
       ObjectMapper objectMapper = new ObjectMapper();

        try {
            JsonNode root = objectMapper.readTree(file);
            this.redHubLocation = new Location(root.get("red").get("hub"));
            this.blueHubLocation = new Location(root.get("blue").get("hub"));
            this.redPassingZoneHighLocation = new Location(root.get("red").get("passingZoneHigh"));
            this.redPassingZoneLowLocation = new Location(root.get("red").get("passingZoneLow"));
            this.bluePassingZoneHighLocation = new Location(root.get("blue").get("passingZoneHigh"));
            this.bluePassingZoneLowLocation = new Location(root.get("blue").get("passingZoneLow"));
            
            
            this.redZone = new Rectangle2d(
                new Translation2d(root.get("zones").get("red").get("xMin").asDouble(), root.get("zones").get("red").get("yMin").asDouble()),
                new Translation2d(root.get("zones").get("red").get("xMax").asDouble(), root.get("zones").get("red").get("yMax").asDouble())
            );
            this.blueZone = new Rectangle2d(
                new Translation2d(root.get("zones").get("blue").get("xMin").asDouble(), root.get("zones").get("blue").get("yMin").asDouble()),
                new Translation2d(root.get("zones").get("blue").get("xMax").asDouble(), root.get("zones").get("blue").get("yMax").asDouble())
            );
            this.neutralLowZone = new Rectangle2d(
                new Translation2d(root.get("zones").get("neutralLow").get("xMin").asDouble(), root.get("zones").get("neutralLow").get("yMin").asDouble()),
                new Translation2d(root.get("zones").get("neutralLow").get("xMax").asDouble(), root.get("zones").get("neutralLow").get("yMax").asDouble())
            );
            this.neutralHighZone = new Rectangle2d(
                new Translation2d(root.get("zones").get("neutralHigh").get("xMin").asDouble(), root.get("zones").get("neutralHigh").get("yMin").asDouble()),
                new Translation2d(root.get("zones").get("neutralHigh").get("xMax").asDouble(), root.get("zones").get("neutralHigh").get("yMax").asDouble())
            );

        } catch(IOException e) {
            System.out.println("Error reading position details JSON: " + e.getMessage());
            e.printStackTrace();
            return;
        }


    }

    public Pose2d targetPose2d(Pose2d currentPose, boolean isRed) {
       if(isRed) {
            if(redZone.contains(currentPose.getTranslation())) {
                return redHubLocation.getPose();
            } else if(neutralLowZone.contains(currentPose.getTranslation())) {
                return redPassingZoneLowLocation.getPose();
            } else if(neutralHighZone.contains(currentPose.getTranslation())) {
                return redPassingZoneHighLocation.getPose();
            } else {
                return redHubLocation.getPose();
            }
       } else {
            if(blueZone.contains(currentPose.getTranslation())) {
                return blueHubLocation.getPose();
            } else if(neutralLowZone.contains(currentPose.getTranslation())) {
                return bluePassingZoneLowLocation.getPose();
            } else if(neutralHighZone.contains(currentPose.getTranslation())) {
                return bluePassingZoneHighLocation.getPose();
            } else {
                return blueHubLocation.getPose();
            }
       }
    }
}
