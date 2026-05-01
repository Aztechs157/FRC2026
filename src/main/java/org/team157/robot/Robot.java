// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.team157.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private String autoName, newAutoName;
  private Optional<Alliance> alliance, newAlliance;
  private Command autonomousCommand;

  private RobotContainer m_robotContainer;
  public static final Field2d m_field = new Field2d();

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Set the PDH's ID
    LoggedPowerDistribution.getInstance(51, ModuleType.kRev);

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    SmartDashboard.putData("Field", m_field);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);

    Logger.recordOutput(
        "FinalComponentPoses",
        new Pose3d[] {
          // turret base
          m_robotContainer.turret.getBasePose(),
          // turret hood
          m_robotContainer.turret.getHoodPivotPose(
              new Transform3d(
                  0,
                  0,
                  0,
                  new Rotation3d(
                      0, Math.toRadians(m_robotContainer.hood.getScaledPosAngleSim()), 0))),
          // intake pivot
          m_robotContainer.slapdown.getIntakePivotPose(),
          // hopper walls
          m_robotContainer.slapdown.getHopperWallsPose()
        });
    Logger.recordOutput("Manual Override", RobotContainer.manualOverride);
    // Gets the match time from the FMS to display for the driver.
    Logger.recordOutput("Match Time", Timer.getMatchTime());
    // Gets hub activity status to display on the dashboard.
    Logger.recordOutput("Hub Active?", m_robotContainer.isHubActive());
    Logger.recordOutput("Under Trench?", RobotContainer.drive.isUnderTrench());
    Logger.recordOutput("Robot Field Pose", RobotContainer.drive.getPose());
    m_field.setRobotPose(RobotContainer.drive.getPose());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_robotContainer.visionSystem.updatePoseEstimation(RobotContainer.drive);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    m_robotContainer.visionSystem.resetPoseEstimation(RobotContainer.drive);
    m_robotContainer.visionSystem.updatePoseEstimation(RobotContainer.drive);

    newAlliance = DriverStation.getAlliance();
    newAutoName = m_robotContainer.getAutonomousCommand().getName();
    if (autoName != newAutoName || alliance != newAlliance) {
      autoName = newAutoName;
      alliance = newAlliance;
      if (AutoBuilder.getAllAutoNames().contains(autoName)) {
        try {
          List<PathPlannerPath> pathPlannerPaths =
              PathPlannerAuto.getPathGroupFromAutoFile(autoName);
          List<Pose2d> poses = new ArrayList<>();
          for (PathPlannerPath path : pathPlannerPaths) {
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
              path = path.flipPath();
            }
            poses.addAll(
                path.getAllPathPoints().stream()
                    .map(
                        point ->
                            new Pose2d(
                                point.position.getX(), point.position.getY(), new Rotation2d()))
                    .collect(Collectors.toList()));
          }
          m_field.getObject("path").setPoses(poses);
        } catch (IOException | org.json.simple.parser.ParseException e) {
          e.printStackTrace();
        }
      }
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
