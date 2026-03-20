// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.team157.robot.Constants.ControllerConstants;
import org.team157.robot.Constants.ModifierConstants;
import org.team157.robot.generated.TunerConstants;
import org.team157.robot.subsystems.DriveSystem;
import org.team157.robot.subsystems.FlywheelSystem;
import org.team157.robot.subsystems.HopperSystem;
import org.team157.robot.subsystems.IntakeSystem;
import org.team157.robot.subsystems.HoodSystem;
import org.team157.robot.subsystems.TurretSystem;
import org.team157.robot.subsystems.UptakeSystem;
import org.team157.robot.subsystems.VisionSystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final TurretSystem turret;
    public final VisionSystem visionSystem;
    public final HoodSystem hood = new HoodSystem();
    public final IntakeSystem intake = new IntakeSystem();
    public final HopperSystem hopper = new HopperSystem();
    public final UptakeSystem uptake = new UptakeSystem();
    public final FlywheelSystem flywheel = new FlywheelSystem();
    public final DriveSystem drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;

    public final Trigger intakeDeployTrigger = new Trigger(() -> intake.getDeployState());

    public static boolean manualOverride = false; // When true, allows manual control of the turret, hood, and flywheel, disabling any dynamic control.

    public RobotContainer() {
        // Adjusts drive speed based on if the robot is in rookie/demo mode.
        if (ModifierConstants.DEMO_MODE) {
            MaxSpeed = MaxSpeed * ModifierConstants.DEMO_DRIVE_MODIFIER;
            MaxAngularRate = MaxAngularRate * ModifierConstants.DEMO_DRIVE_MODIFIER;
        } else if (ModifierConstants.ROOKIE_MODE) {
            MaxSpeed = MaxSpeed * ModifierConstants.ROOKIE_DRIVE_MODIFIER;

        }

        visionSystem = new VisionSystem(drivetrain::getPose, Robot.m_field);
        turret = new TurretSystem(visionSystem);

        NamedCommands.registerCommand("DeployIntake", intake.deployIntake());
        NamedCommands.registerCommand("RunIntake", intake.runIntake());
        NamedCommands.registerCommand("RunHopper", hopper.setRoller(0.5));
        NamedCommands.registerCommand("ShootBalls", uptake.setRoller(1));
        NamedCommands.registerCommand("Wiggle", intake.wiggleIntake());

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    private void configureBindings() {
        ////////////////////////
        /// DEFAULT COMMANDS ///
        ////////////////////////

        // Idle while the robot is disabled. This ensures the configured neutral mode is applied to the drive motors while disabled.

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
        // Telemetry for the drivetrain.
        drivetrain.registerTelemetry(logger::telemeterize);

        // Update the pose estimation and turret tracking angle while no other vision commands are running.
        visionSystem.setDefaultCommand(visionSystem.setDefault(drivetrain, turret));

        // Disable turret movement when no other turret commands are running.
        turret.setDefaultCommand(turret.setDefault());
        flywheel.setDefaultCommand(flywheel.setDefault());
        intake.setDefaultCommand(intake.setDefault());
        hopper.setDefaultCommand(hopper.setDefault());
        uptake.setDefaultCommand(uptake.setDefault());
        hood.setDefaultCommand(hood.setDefault());

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //////////////////////////////////////////////
        /// DRIVER COMMANDS ///
        //////////////////////////////////////////////

        // Note that X is defined as forward according to WPILib convention, and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(MathUtil.applyDeadband(-driverController.getLeftY(),
                                ControllerConstants.JOYSTICK_DEADBAND) * modifySpeed(MaxSpeed)) // Drive forward with
                                                                                                // negative Y
                        // (forward)
                        .withVelocityY(MathUtil.applyDeadband(-driverController.getLeftX(),
                                ControllerConstants.JOYSTICK_DEADBAND) * modifySpeed(MaxSpeed)) // Drive left with
                                                                                                // negative X (left)
                        .withRotationalRate(MathUtil.applyDeadband(-driverController.getRightX(),
                                ControllerConstants.JOYSTICK_DEADBAND) * modifySpeed(MaxAngularRate)) // Drive
                                                                                                      // counterclockwise
                                                                                                      // with
                // negative X (left)
                ));

        // Reset the field-centric heading on start and back button press.
        driverController.start().and(driverController.back()).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        // When the B button is held, the robot will brake in place,
        //holding its position against external forces.
        driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));

        ///////////////////////////
        /// FlYWHEEL HOOD ///
        //////////////////////////

        // Enables dynamic control of the flywheel and hood.
        driverController.a().toggleOnTrue(flywheel.setDynamicVelocity());
        driverController.a().toggleOnTrue(hood.setDynamicHoodAngle());

        ////////////////////////////
        /// INTAKE UPTAKE HOPPER ///
        ////////////////////////////

        // Swaps the intake and shooting triggers if Maya mode is enabled, per Maya's peference.
        if (ModifierConstants.MAYA_MODE) {
            // Shooting on left trigger, intake on right trigger
            driverController.leftTrigger().whileTrue(uptake.setRoller(1));
            driverController.rightTrigger().whileTrue(intake.runIntake());
            driverController.leftTrigger().whileTrue(hopper.setRoller(0.5));
        } else {
            // Shooting on right trigger, intake on left trigger
            driverController.rightTrigger().whileTrue(uptake.setRoller(1));
            driverController.leftTrigger().whileTrue(intake.runIntake());
            driverController.rightTrigger().whileTrue(hopper.setRoller(0.5));
        }
        // Runs the hopper, uptake, and intake backwards at a low speed to clear jams.
        driverController.y().whileTrue(forceOuttake());
        // Wiggles the intake up and down to free up stuck balls
        driverController.x().toggleOnTrue(intake.wiggleIntake());
        // Toggle manual override (on driver A for testing without controller)
        // driverController.a().onTrue(toggleManualOverride());

        //////////////////////////////////////////////////
        /// OPERATOR COMMANDS ///
        //////////////////////////////////////////////////

        // Toggle manual override with both sticks to prevent accidental activation during teleop.
        operatorController.leftStick().and(operatorController.rightStick()).onTrue(toggleManualOverride());

        // Disables automatic turret tracking when manual override is enabled,
        // allowing the operator to control the turret without interference from vision tracking.
        turretTrackingTrigger().whileTrue(turret.trackTagGlobalRelative());
        ;
        turretTrackingTrigger().whileTrue(flywheel.setDynamicVelocity());
        turretTrackingTrigger().whileTrue(hood.setDynamicHoodAngle());

        ///////////////////////
        /// MANUAL FLYWHEEL ///
        ///////////////////////

        // Only enable manual control of turret, hood and flywheel when manual override is enabled
        // Set the turret to preset robot-relative angles based on the D-Pad input of the Operator controller.
        operatorController.povUp().toggleOnTrue(turret.setAngle(Degrees.of(-50)));
        operatorController.povUpRight().toggleOnTrue(turret.setAngle(Degrees.of(-5)));
        operatorController.povRight().whileTrue(turret.setAngle(Degrees.of(40)));
        operatorController.povDownRight().toggleOnTrue(turret.setAngle(Degrees.of(85)));
        operatorController.povDown().toggleOnTrue(turret.setAngle(Degrees.of(130)));
        operatorController.povDownLeft().toggleOnTrue(turret.setAngle(Degrees.of(175)));
        operatorController.povLeft().whileTrue(turret.setAngle(Degrees.of(220)));
        operatorController.povUpLeft().toggleOnTrue(turret.setAngle(Degrees.of(265)));

        ////////////////////////
        /// MANUAl FLYWHEEL ///
        ////////////////////////

        // Set the flywheel to preset velocities based on the bumpers and triggers of the
        // Operator controller.
        operatorController.rightTrigger().toggleOnTrue(flywheel.setVelocity(RPM.of(4800)));
        operatorController.rightBumper().toggleOnTrue(flywheel.setVelocity(RPM.of(2800)));

        ////////////////////
        /// MANUAL HOOD ///
        ////////////////////

        // Set the hood to preset angles based on the bumpers and triggers of the
        // Operator controller.
        operatorController.leftTrigger().toggleOnTrue(hood.setAngle(Degrees.of(45)));
        operatorController.leftBumper().toggleOnTrue(hood.setAngle(Degrees.of(60)));

        ///////////////////////
        /// INTAKE COMMANDS ///
        ///////////////////////

        // Deploy and retract the intake with the A and Y buttons, but only when the
        // back button is held to prevent accidental activation during teleop.
        operatorController.a().and(operatorController.back()).toggleOnTrue(intake.deployIntake());
        operatorController.y().and(operatorController.back()).toggleOnTrue(intake.retractIntake());

    }

    ///////////////////////////////////////////////////////
    /// NON-CONTROL FUNCTIONS ///
    //////////////////////////////////////////////////////

    // If the A button is held, apply the precision modifier of 0.5x speed.
    public double modifySpeed(final double speed) {
        if (driverController.rightBumper().getAsBoolean()) {
            return speed * ModifierConstants.PRECISION_DRIVE_MODIFIER;
        } else {
            return speed;
        }
    }
    // Old trigger-based precision mode that scales speed based on how much the right trigger is pressed.
    // Replaced by button-based toggle due to lack of available triggers.
    // public double modifySpeed(final double speed) {
    // final var modifier = 1 - driverController.getRightTriggerAxis() *
    // ModifierConstants.PRECISION_DRIVE_MODIFIER;
    // return speed * modifier;
    // }

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its
        // likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Inverts the state of manual override, allowing the operator to toggle between
     * manual and dynamic control of the turret, hood, and flywheel.
     * 
     * @return {@link InstantCommand} that toggles manual override when executed.
     */
    private Command toggleManualOverride() {
        return new InstantCommand(() -> {
            manualOverride = !manualOverride;
        });
    }

    // A simple command that runs the intake, hopper, and uptake rollers in reverse
    // at a low speed to clear any jams.
    // TODO: remove from RobotContainer and into eventual Superstructure subsystem once it exists.
    private Command forceOuttake() {
        return uptake.setRoller(-0.5).alongWith(hopper.setRoller(-0.5)).alongWith(intake.setRoller(-0.5));
    }

    /**
     * Trigger used for tracking a target location with the turret
     * 
     * @return {@link Trigger} that is true when the robot is in teleop or autonomous and manual override
     * is not enabled, allowing the turret to track targets when those conditions are met.
     */
    private Trigger turretTrackingTrigger() {
        return new Trigger(
                () -> (RobotModeTriggers.teleop().getAsBoolean() || RobotModeTriggers.autonomous().getAsBoolean())
                        && !manualOverride);
    }
}