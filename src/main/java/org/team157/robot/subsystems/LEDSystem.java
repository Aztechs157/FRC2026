// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team157.utilities.PriorityMap;
import org.team157.robot.Constants.LEDConstants;


//@Logged(strategy = Strategy.OPT_OUT)
public class LEDSystem extends SubsystemBase {
  private PriorityMap<String, LEDPattern> fullPatterns = new PriorityMap<String, LEDPattern>();
  private PriorityMap<String, LEDPattern> topPatterns = new PriorityMap<String, LEDPattern>();
  private PriorityMap<String, LEDPattern> midPatterns = new PriorityMap<String, LEDPattern>();
  private PriorityMap<String, LEDPattern> botPatterns = new PriorityMap<String, LEDPattern>();

  AddressableLED prettyLights;
  AddressableLEDBuffer prettyLightsBuffer;

  AddressableLEDBufferView topBuffer;
  AddressableLEDBufferView midBuffer;
  AddressableLEDBufferView botBuffer;

  /** Creates a new LEDSystem. */
  public LEDSystem() {

    prettyLights = new AddressableLED(LEDConstants.PMW_PORT);
    prettyLightsBuffer = new AddressableLEDBuffer(LEDConstants.STRIP_LENGTH);

    topBuffer = prettyLightsBuffer.createView(12, 17);
    midBuffer = prettyLightsBuffer.createView(6, 11);
    botBuffer = prettyLightsBuffer.createView(0, 5);

    prettyLights.setLength(LEDConstants.STRIP_LENGTH);

    prettyLights.start();

    // fun assabet-y pattern
    LEDPattern assabet = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGold, Color.kBlue);
    // the same one but scrolling
    LEDPattern assabetScroll = assabet.scrollAtRelativeSpeed(Percent.per(Second).of(25))
        .atBrightness(Percent.of(20));
    addPattern("Assabet Scroll", 157, assabetScroll);

  }

  public void isFMS() {
    // runs the idle pattern without lowering the brightness, only when connected to
    // an FMS.
    LEDPattern assabet = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGold, Color.kBlue);
    LEDPattern assabetScroll = assabet.scrollAtRelativeSpeed(Percent.per(Second).of(25));

    fullPatterns.replace("Assabet Scroll", assabetScroll);
  }

  /*
   * for now, im using the same method as the fms checker,
   * but i would like to know why we're making a method
   * instead of just using DriverStation's isFMSAttached directly.
   */
  public void isEStop() {
    // make this pattern less pleasant
    LEDPattern unpleasant = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kSpringGreen,
        Color.kMagenta, Color.kSaddleBrown);
    // runs the pattern in place of the default scrolly pattern upon being
    // e-stopped.
    addPattern("unpleasant", 1, unpleasant);
  }

  public void batteryLow(boolean isLow) {
    // this pattern is run when the battery drops below a certain voltage
    LEDPattern low = LEDPattern.solid(Color.kRed);
    LEDPattern redFlash = low.blink(Seconds.of(0.5));
    if (isLow) {
      addBotPattern("Battery Low", 1, redFlash);
    } else {
      removeBotPattern("Battery Low");
    }

  }

  // full
  public void addPattern(String name, int priority, LEDPattern pattern) {
    fullPatterns.put(name, priority, pattern);
  }

  public LEDPattern removePattern(String name) {
    return fullPatterns.remove(name);
  }

  public boolean hasPattern(String name) {
    return fullPatterns.containsKey(name);
  }

  // bot
  public void addBotPattern(String name, int priority, LEDPattern pattern) {
    botPatterns.put(name, priority, pattern);
  }

  public LEDPattern removeBotPattern(String name) {
    return botPatterns.remove(name);
  }

  public boolean hasBotPattern(String name) {
    return botPatterns.containsKey(name);
  }

  // mid
  public void addMidPattern(String name, int priority, LEDPattern pattern) {
    midPatterns.put(name, priority, pattern);
  }

  public LEDPattern removeMidPattern(String name) {
    return midPatterns.remove(name);
  }

  public boolean hasMidPattern(String name) {
    return midPatterns.containsKey(name);
  }

  // top
  public void addTopPattern(String name, int priority, LEDPattern pattern) {
    topPatterns.put(name, priority, pattern);
  }

  public LEDPattern removeTopPattern(String name) {
    return topPatterns.remove(name);
  }

  public boolean hasTopPattern(String name) {
    return topPatterns.containsKey(name);
  }

  @Override
  public void periodic() {

    fullPatterns.firstValue().applyTo(prettyLightsBuffer);

    if (botPatterns.firstPriority() != null && botPatterns.firstPriority() <= fullPatterns.firstPriority()) {
      botPatterns.firstValue().applyTo(botBuffer);
    }

    if (midPatterns.firstPriority() != null && midPatterns.firstPriority() <= fullPatterns.firstPriority()) {
      midPatterns.firstValue().applyTo(midBuffer);
    }

    if (topPatterns.firstPriority() != null && topPatterns.firstPriority() <= fullPatterns.firstPriority()) {
      topPatterns.firstValue().applyTo(topBuffer);
    }

    prettyLights.setData(prettyLightsBuffer);
  }
}
