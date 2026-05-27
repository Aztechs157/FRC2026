// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team157.robot.RobotContainer;
import org.team157.utilities.PriorityMap;

/** The LEDs subsystem controls our LED strip to display various patterns based on the hub's state and provide visual feedback to the driver. */
public class LEDs extends SubsystemBase {

    /// CONSTANTS ///

    /** PWM port for the LED strip */
    public static final int PWM_PORT = 9;
    /** Amount of LEDs in the strip */
    public static final int STRIP_LENGTH = 38;
    /** LEDs per meter */
    public static final int DENSITY = 60;

    /// LED STRIP CONTROL SETUP ///
    
    /**
     * Priority map for pattern stacking, mapping each LED pattern to a name (string) and priority
     * level. Patterns of higher priority (lower number) will display over those of lower priority.
     */
    private PriorityMap<String, LEDPattern> fullPatterns = new PriorityMap<String, LEDPattern>();
    /** LED strip controller */
    AddressableLED prettyLights;
    /** Pattern buffer for the LED strip */
    AddressableLEDBuffer prettyLightsBuffer;

    /// PATTERNS ///
    
    /** Idle pattern, a scrolling gradient in our team colors. */
    public LEDPattern idle =
            LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGold, Color.kBlue)
                    .scrollAtRelativeSpeed(Hertz.of(0.25));
    /** Active pattern, a scrolling rainbow. */
    public LEDPattern active = LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Hertz.of(0.5));
    /** Inactive pattern, solid white. */
    public LEDPattern inactive = LEDPattern.solid(Color.kWhite);
    public LEDPattern crunchTime = LEDPattern.solid(Color.kWhite).blink(Seconds.of(0.33));
    public LEDPattern shiftEnd = crunchTime.overlayOn(active);


    /** Creates the LEDs subsystem. */
    public LEDs() {

        prettyLights = new AddressableLED(PWM_PORT);
        prettyLights.setLength(STRIP_LENGTH);

        prettyLightsBuffer = new AddressableLEDBuffer(STRIP_LENGTH);

        prettyLights.start();

        // Adds the idle pattern to the buffer.
        addPattern("Idle", 157, idle);
    }

    /** Determines the current desired LED pattern based on the hub timer status.
     * 
     * @param hubTimer the hub timer to derive shift information from.
     * @return the desired LED pattern based on the current hub state.
     */
    public LEDPattern getDesiredPattern(HubTimer hubTimer) {
        if (!DriverStation.isEnabled()) {
            return idle;
        } else if (hubTimer.isShiftAboutToEnd(5)
                && hubTimer.isHubActive()) {
            return shiftEnd;
        } else if (hubTimer.isShiftAboutToEnd(5)) {
            return crunchTime;
        } else if (hubTimer.isHubActive()) {
            return active;
        } else {
            return inactive;
        }
    }

    /** Adds an LED pattern to the collection. */
    public void addPattern(String name, int priority, LEDPattern pattern) {
        fullPatterns.put(name, priority, pattern);
    }

    /** Removes an LED pattern from the collection. */
    public LEDPattern removePattern(String name) {
        return fullPatterns.remove(name);
    }

    /** @return true if the collection contains a pattern with the specified name, false otherwise. */
    public boolean hasPattern(String name) {
        return fullPatterns.containsKey(name);
    }

    @Override
    public void periodic() {
        fullPatterns.put("Desired Pattern", 5, getDesiredPattern(RobotContainer.hubStatus));
        fullPatterns.firstValue().applyTo(prettyLightsBuffer);
        prettyLights.setData(prettyLightsBuffer);
    }
}
