// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.parsing;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

public class PositionDetails {
    private final String jsonPath = "/positionDetails.json";
    public Location bluePassingZone1Location;
    public Location bluePassingZone2Location;
    public Location redPassingZone2Location;
    public Location redPassingZone1Location;
    public Location blueHubLocation;
    public Location redHubLocation;
    
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
            this.redPassingZone1Location = new Location(root.get("red").get("passingZoneHP"));
            this.redPassingZone2Location = new Location(root.get("red").get("passingZoneDepot"));
            this.bluePassingZone1Location = new Location(root.get("blue").get("passingZoneHP"));
            this.bluePassingZone2Location = new Location(root.get("blue").get("passingZoneDepot"));

        } catch(IOException e) {
            System.out.println("Error reading position details JSON: " + e.getMessage());
            e.printStackTrace();
            return;
        }



    }
}
