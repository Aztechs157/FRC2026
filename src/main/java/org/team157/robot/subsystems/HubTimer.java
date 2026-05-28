// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * The HubTimer class keeps track of the current state of the hub based on the match time and our
 * current alliance, and provides utilities for determining what our current shift is, how much time
 * remains in that shift, and whether that shift is about to end.
 */
public class HubTimer extends SubsystemBase {

    /** Represents the possible shift states which the hub can possess. */
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

    /** The current state of the hub. */
    private Shift currentShift = Shift.INACTIVE;
    /** The current activity status of the hub. */
    private boolean hubActive = true;
    /** The time remaining until the next shift, in seconds. */
    private double timeUntilSwap = 0.0;
    /**
     * Whether the Red alliance is inactive first, based on the game data from the {@link
     * DriverStation}.
     */
    private boolean redInactiveFirst = false;
    /** Whether the first shift is active, based on whether or not our alliance won auto. */
    private boolean shift1Active = false;

    /**
     * Updates the status of the hub based on the current match time and game data. <br>
     * Taken almost directly from the WPILib documentation, with the addition of our Shift states.
     */
    public void updateHubStatus() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            hubActive = false;
            currentShift = Shift.INACTIVE;
        } else {

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
            shift1Active =
                    switch (alliance.get()) {
                        case Red -> !redInactiveFirst;
                        case Blue -> redInactiveFirst;
                        default -> !redInactiveFirst;
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

        // Logger outputs
        Logger.recordOutput("Shift/Time Until Next Swap", timeUntilSwap);
        Logger.recordOutput("Shift/Hub Active?", hubActive);
        Logger.recordOutput("Shift/Current Shift", currentShift.name());
    }

    /**
     * Determines if the current shift is about to end based on the remaining match time.
     *
     * @param threshold The amount of time, in seconds, remaining in the match to trigger the event
     * @return true if the shift is about to end, false otherwise
     */
    public boolean isShiftAboutToEnd(double threshold) {
        // If we're between 2 active shifts, our shooting time is not about to end.
        if ((currentShift == Shift.SHIFT4 && !shift1Active)
                || (currentShift == Shift.TRANSITION && shift1Active)) {
            return false;
        } else {
            return timeUntilSwap > 0 && timeUntilSwap < threshold;
        }
    }

    /**
     * @return true if the hub is currently active, false otherwise.
     */
    public boolean isHubActive() {
        return hubActive;
    }

    /**
     * @return the time remaining until the next shift change, in seconds.
     */
    public double getTimeUntilSwap() {
        return timeUntilSwap;
    }

    /**
     * @return the current {@link Shift} state of the hub.
     */
    public Shift getHubState() {
        return currentShift;
    }

    @Override
    public void periodic() {
        updateHubStatus();
    }
}
