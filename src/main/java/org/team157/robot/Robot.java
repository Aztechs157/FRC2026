// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot;

import java.util.Optional;

import org.littletonrobotics.junction.LoggedRobot;
import org.team157.robot.Constants.VisionConstants;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.geometry.Pose3d;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically 
 * corresponding to each mode, as described in the TimedRobot documentation. 
 * If you change the name of this class or the package after 
 * creating this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private String autoName, newAutoName;
  private Optional<Alliance> alliance, newAlliance;
  private Command m_autonomousCommand;

  public static Pose3d[] zeroArray = new Pose3d[4];
  public static Pose3d[] finalArray = new Pose3d[4];
  public static Pose3d[] cameras = new Pose3d[3];
  // creates a publisher to send zeroed Pose3d values to NT for model calibration.
  public static StructArrayPublisher<Pose3d> zeroedPoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("ZeroedComponentPoses", Pose3d.struct).publish();
  public static StructArrayPublisher<Pose3d> finalPoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("FinalComponentPoses", Pose3d.struct).publish();

  public static StructArrayPublisher<Pose3d> cameraPoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("CameraPoses", Pose3d.struct).publish();

  public static final Field2d m_field = new Field2d();

  private final RobotContainer m_robotContainer;

  private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
      .withTimestampReplay()
      .withJoystickReplay();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform 
    //all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    SmartDashboard.putData("Field", m_field);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_timeAndJoystickReplay.update();
    // Runs the Scheduler. This is responsible 
    // for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, 
    // removing finished or interrupted commands,
    // and running subsystem periodic() methods. 
    //This must be called from the robot's periodic block 
    //in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // // these are just for model calibration
    // zeroArray = new Pose3d[] {
    // // turret base
    // new Pose3d(),
    // // turret hood
    // new Pose3d(),
    // // intake pivot
    // new Pose3d(),
    // // hopper walls
    // new Pose3d()
    // };
    // zeroedPoses.set(zeroArray);

    // Gets mechanism poses to be used for the AdvantageScope model.
    finalArray = new Pose3d[] {
        // turret base
        m_robotContainer.turret.getBasePose(),
        // turret hood
        m_robotContainer.turret.getHoodPivotPose(new Transform3d(0, 0, 0,
            new Rotation3d(0, Math.toRadians(m_robotContainer.hood.getScaledPosAngleSim()), 0))),
        // intake pivot
        m_robotContainer.intake.getIntakePivotPose(),
        // hopper walls
        m_robotContainer.intake.getHopperWallsPose()
    };
    // Send mechanism poses to NT.
    finalPoses.set(finalArray);

    // // used for model camera position calibration
    // cameras = new Pose3d[] {
    // new
    // Pose3d(m_robotContainer.drivetrain.getPose()).plus(VisionConstants.FRONTLEFT_CAMERA_PLACEMENT),
    // new
    // Pose3d(m_robotContainer.drivetrain.getPose()).plus(VisionConstants.FRONTRIGHT_CAMERA_PLACEMENT),
    // new
    // Pose3d(m_robotContainer.drivetrain.getPose()).plus(VisionConstants.BACK_CAMERA_PLACEMENT)
    // };
    // cameraPoses.set(cameras);

    // Gets the manual override status from RobotContainer to display on the
    // dashboard.
    SmartDashboard.putBoolean("Manual Override", RobotContainer.manualOverride);
    // Gets the match time from the FMS to display for the driver.
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
    // Gets hub activity status to display on the dashboard.
    SmartDashboard.putBoolean("Hub Active?", m_robotContainer.isHubActive());

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.visionSystem.updatePoseEstimation(m_robotContainer.drivetrain);
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.visionSystem.resetPoseEstimation(m_robotContainer.drivetrain);
    m_robotContainer.visionSystem.updatePoseEstimation(m_robotContainer.drivetrain);

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
