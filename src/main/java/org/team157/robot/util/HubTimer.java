// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

/** Add your docs here. */
// TODO: consider extending SubsystemBase to automatically update the hub status on each
// CommandScheduler run independent from Robot.
public class HubTimer extends SubsystemBase {

    public enum Shift {
        INACTIVE,
        AUTO,
        TRANSITION,
        SHIFT1,
        SHIFT2,
        SHIFT3,
        SHIFT4,
        ENDGAME;
    }

    public Shift currentShift = Shift.INACTIVE;
    public boolean hubActive = true;
    public double timeUntilSwap = 0.0;

    public void updateHubStatus() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            hubActive = false;
            currentShift = Shift.INACTIVE;
        }

        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            hubActive = false;
            currentShift = Shift.INACTIVE;
        }

        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            hubActive = true;
            currentShift = Shift.AUTO;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();

        boolean redInactiveFirst = false;

        // If we have no game data, we cannot compute, assume hub is active, as its
        // likely early in teleop.
        if (gameData.isEmpty()) {
            hubActive = true;
        } else {
            switch (gameData.charAt(0)) {
                case 'R' -> redInactiveFirst = true;
                case 'B' -> redInactiveFirst = false;
                default -> {
                    // If we have invalid game data, assume hub is active.
                    hubActive = true;
                }
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active =
                switch (alliance.get()) {
                    case Red -> !redInactiveFirst;
                    case Blue -> redInactiveFirst;
                };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            hubActive = true;
            currentShift = Shift.TRANSITION;
            timeUntilSwap = matchTime - 130;
        } else if (matchTime > 105) {
            // Shift 1
            hubActive = shift1Active;
            currentShift = Shift.SHIFT1;
            timeUntilSwap = matchTime - 105;
        } else if (matchTime > 80) {
            // Shift 2
            hubActive = !shift1Active;
            currentShift = Shift.SHIFT2;
            timeUntilSwap = matchTime - 80;
        } else if (matchTime > 55) {
            // Shift 3
            hubActive = shift1Active;
            currentShift = Shift.SHIFT3;
            timeUntilSwap = matchTime - 55;
        } else if (matchTime > 30) {
            // Shift 4
            hubActive = !shift1Active;
            currentShift = Shift.SHIFT4;
            timeUntilSwap = matchTime - 30;
        } else if (matchTime > 0 && !DriverStation.isAutonomous()) {
            // End game, hub always active.
            hubActive = true;
            currentShift = Shift.ENDGAME;
            timeUntilSwap = matchTime;
        } else if (matchTime > 0 && DriverStation.isAutonomous()) {
            // Hub is always active in autonomous.
            hubActive = true;
            currentShift = Shift.AUTO;
            timeUntilSwap = matchTime;
        } else {
            // Match time is invalid, assume hub is active.
            hubActive = true;
            currentShift = Shift.INACTIVE;
            timeUntilSwap = -1.0;
        }
    }

    @Override
    public void periodic(){
        
    }
}
