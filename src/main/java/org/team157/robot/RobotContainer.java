// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.team157.robot.Constants.ControllerConstants;
import org.team157.robot.Constants.Mode;
import org.team157.robot.Constants.ModifierConstants;
import org.team157.robot.commands.DriveCommands;
import org.team157.robot.generated.AKitTunerConstants;
import org.team157.robot.subsystems.drive.Drive;
import org.team157.robot.subsystems.drive.GyroIO;
import org.team157.robot.subsystems.drive.GyroIOPigeon2;
import org.team157.robot.subsystems.drive.ModuleIO;
import org.team157.robot.subsystems.drive.ModuleIOSim;
import org.team157.robot.subsystems.drive.ModuleIOTalonFX;
import org.team157.robot.subsystems.flywheel.Flywheel;
import org.team157.robot.subsystems.flywheel.FlywheelIOTalonFX;
import org.team157.robot.subsystems.hood.Hood;
import org.team157.robot.subsystems.hood.HoodIOTalonFX;
import org.team157.robot.subsystems.hopper.Hopper;
import org.team157.robot.subsystems.hopper.HopperIOTalonFX;
import org.team157.robot.subsystems.intake.Intake;
import org.team157.robot.subsystems.intake.IntakeIOTalonFX;
import org.team157.robot.subsystems.slapdown.Slapdown;
import org.team157.robot.subsystems.slapdown.SlapdownIOTalonFX;
import org.team157.robot.subsystems.uptake.Uptake;
import org.team157.robot.subsystems.uptake.UptakeIOTalonFX;
import org.team157.robot.subsystems.turret.Turret;
import org.team157.robot.subsystems.turret.TurretIOTalonFX;
import org.team157.robot.subsystems.vision.VisionSystem;

public class RobotContainer {
    private double MaxSpeed = AKitTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final Turret turret = new Turret();
    public final VisionSystem visionSystem;
    public static final Flywheel flywheel = new Flywheel();
    public static final Drive drivetrain;

    static {
        switch (Constants.currentMode) {
            case REAL:
                drivetrain = new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFX(AKitTunerConstants.FrontLeft),
                    new ModuleIOTalonFX(AKitTunerConstants.FrontRight),
                    new ModuleIOTalonFX(AKitTunerConstants.BackLeft),
                    new ModuleIOTalonFX(AKitTunerConstants.BackRight));
                break;
            case SIM:
                drivetrain = new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(AKitTunerConstants.FrontLeft),
                    new ModuleIOSim(AKitTunerConstants.FrontRight),
                    new ModuleIOSim(AKitTunerConstants.BackLeft),
                    new ModuleIOSim(AKitTunerConstants.BackRight));
                break;
            default:
                drivetrain = new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});
                break;
        }
    }

    

    public final Hood hood = new Hood();
    public final Intake intake = new Intake();
    public final Hopper hopper = new Hopper();
    public final Uptake uptake = new Uptake();
    public final Slapdown slapdown = new Slapdown();


    private final SendableChooser<Command> autoChooser;

    

    public static boolean manualOverride = false; // When true, allows manual control of the turret, hood, and flywheel, disabling any dynamic control.

    public RobotContainer() {
        // Adjusts drive speed based on if the robot is in rookie/demo mode.
        if (ModifierConstants.DEMO_MODE) {
            MaxSpeed = MaxSpeed * ModifierConstants.DEMO_DRIVE_MODIFIER;
            MaxAngularRate = MaxAngularRate * ModifierConstants.DEMO_DRIVE_MODIFIER;
        } else if (ModifierConstants.ROOKIE_MODE) {
            MaxSpeed = MaxSpeed * ModifierConstants.ROOKIE_DRIVE_MODIFIER;

        }

        intake.setIO(new IntakeIOTalonFX(intake));
        hood.setIO(new HoodIOTalonFX(hood));
        slapdown.setIO(new SlapdownIOTalonFX(slapdown));
        hopper.setIO(new HopperIOTalonFX(hopper));
        uptake.setIO(new UptakeIOTalonFX(uptake));
        flywheel.setIO(new FlywheelIOTalonFX(flywheel));
        visionSystem = new VisionSystem(drivetrain::getPose, Robot.m_field);
        turret.setIO(new TurretIOTalonFX(turret), visionSystem);

        NamedCommands.registerCommand("DeployIntake", slapdown.deployIntake());
        NamedCommands.registerCommand("RunIntake", intake.runIntake());
        NamedCommands.registerCommand("RunHopper", hopper.set(0.5));
        NamedCommands.registerCommand("ShootBalls", uptake.set(1));
        NamedCommands.registerCommand("Wiggle", slapdown.wiggleIntake());
        NamedCommands.registerCommand("WiggleCubed", slapdown.wiggleIntake().andThen(new WaitCommand(1)).andThen(slapdown.wiggleIntake()).andThen(new WaitCommand(1)).andThen(slapdown.wiggleIntake()));

        autoChooser = AutoBuilder.buildAutoChooser("New Auto");


    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drivetrain));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drivetrain));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));


        configureBindings();



        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    private void configureBindings() {
        ////////////////////////
        /// DEFAULT COMMANDS ///
        ////////////////////////

        // Update the pose estimation and turret tracking angle while no other vision commands are running. 
        visionSystem.setDefaultCommand(visionSystem.setDefault(drivetrain, turret));

        // Disable turret movement when no other turret commands are running.
        turret.setDefaultCommand(turret.getDefault());
        flywheel.setDefaultCommand(flywheel.getDefault());
        slapdown.setDefaultCommand(slapdown.getDefault());
        intake.setDefaultCommand(intake.getDefault());
        hopper.setDefaultCommand(hopper.getDefault());
        uptake.setDefaultCommand(uptake.setDefault());
        hood.setDefaultCommand(hood.getDefault(drivetrain));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //////////////////////////////////////////////
        ///             DRIVER COMMANDS            ///
        //////////////////////////////////////////////

        // Note that X is defined as forward according to WPILib convention, and Y is defined as to the left according to WPILib convention.
        // drivetrain.setDefaultCommand(
        //         drivetrain.run(() -> {
        //             // Field-centric driving with joystick inputs
        //             double xSpeed = MathUtil.applyDeadband(-driverController.getLeftY(),
        //                     ControllerConstants.JOYSTICK_DEADBAND) * modifySpeed(MaxSpeed);
        //             double ySpeed = MathUtil.applyDeadband(-driverController.getLeftX(),
        //                     ControllerConstants.JOYSTICK_DEADBAND) * modifySpeed(MaxSpeed);
        //             double rotSpeed = MathUtil.applyDeadband(-driverController.getRightX(),
        //                     ControllerConstants.JOYSTICK_DEADBAND) * modifySpeed(MaxAngularRate);

        //             drivetrain.runVelocity(
        //                 ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, drivetrain.getRotation()));
        //         }));


        drivetrain.setDefaultCommand(
            DriveCommands.joystickDrive(
                drivetrain,
                () -> MathUtil.applyDeadband(-driverController.getLeftY(), ControllerConstants.JOYSTICK_DEADBAND) * modifySpeed(MaxSpeed),
                () -> MathUtil.applyDeadband(-driverController.getLeftX(), ControllerConstants.JOYSTICK_DEADBAND) * modifySpeed(MaxSpeed),
                () -> MathUtil.applyDeadband(-driverController.getRightX(), ControllerConstants.JOYSTICK_DEADBAND) * modifySpeed(MaxAngularRate))
            );


        // Reset the field-centric heading by resetting the pose with the current position but zero rotation.
        driverController.start().and(driverController.back()).onTrue(drivetrain.runOnce(() -> {
            drivetrain.setPose(new Pose2d(drivetrain.getPose().getTranslation(), Rotation2d.kZero));
        }));
        // Reset the robot pose to the alliance-specific manual reset pose when both start and back are pressed together on both controllers
        operatorController.start().and(operatorController.back().and(driverController.start()
              .and(driverController.back()))).onTrue(drivetrain.resetPoseCommand());

        // When the B button is held, the robot will brake in place by commanding X-pattern wheel angles.
        driverController.b().whileTrue(drivetrain.run(() -> drivetrain.stopWithX()));



          /////////////////////
         /// FlYWHEEL HOOD ///
        /////////////////////
        // Enables dynamic control of the flywheel and hood.
        driverController.a().toggleOnTrue(flywheel.setDynamicVelocity());
        driverController.a().toggleOnTrue(hood.setDynamicHoodAngle());

          ////////////////////////////
         /// INTAKE UPTAKE HOPPER ///
        ////////////////////////////
        // Swaps the intake and shooting triggers if Maya mode is enabled, per Maya's preference.
        if(ModifierConstants.MAYA_MODE) {
            // Shooting on left trigger, intake on right trigger
            driverController.leftTrigger().whileTrue(uptake.set(1));
            driverController.rightTrigger().whileTrue(intake.runIntake());
            driverController.leftTrigger().whileTrue(hopper.set(1));
        } else {
            // Shooting on right trigger, intake on left trigger
            driverController.rightTrigger().whileTrue(uptake.set(1));
            driverController.leftTrigger().whileTrue(intake.runIntake());
            driverController.rightTrigger().whileTrue(hopper.set(1));
        }
        // Runs the hopper, uptake, and intake backwards at a low speed to clear jams.
        driverController.y().whileTrue(forceOuttake());
        // Wiggles the intake up and down to free up stuck balls
        driverController.x().toggleOnTrue(slapdown.wiggleIntake());
        // Toggle manual override (on driver A for testing without controller)
        // driverController.a().onTrue(toggleManualOverride());

        //////////////////////////////////////////////////
        ///             OPERATOR COMMANDS              ///
        //////////////////////////////////////////////////

        // Toggle manual override with both sticks to prevent accidental activation during teleop.
        operatorController.leftStick().and(operatorController.rightStick()).onTrue(toggleManualOverride());

        // Disables automatic turret tracking when manual override is enabled,
        // allowing the operator to control the turret without interference from vision tracking.
        turretTrackingTrigger().whileTrue(turret.trackTagGlobalRelative());
        turretTrackingTrigger().whileTrue(flywheel.setDynamicVelocity());
        turretTrackingTrigger().whileTrue(hood.setDynamicHoodAngle());

        ///////////////////////
        /// MANUAL FLYWHEEL ///
        ///////////////////////

        // Only enable manual control of turret, hood and flywheel when manual override is enabled
        // Set the turret to preset robot-relative angles based on the D-Pad input of the Operator controller.
        operatorController.povUp().toggleOnTrue(turret.setAngle(Degrees.of(168.5)));
        // operatorController.povUpRight().toggleOnTrue(turret.setAngle(Degrees.of(-5)));
        operatorController.povRight().whileTrue(turret.setAngle(Degrees.of(78.5)));
        // operatorController.povDownRight().toggleOnTrue(turret.setAngle(Degrees.of(85)));
        operatorController.povDown().toggleOnTrue(turret.setAngle(Degrees.of(-12.5)));
        // operatorController.povDownLeft().toggleOnTrue(turret.setAngle(Degrees.of(175)));
        operatorController.povLeft().whileTrue(turret.setAngle(Degrees.of(-102.5)));
        // operatorController.povUpLeft().toggleOnTrue(turret.setAngle(Degrees.of(265)));
        
          ///////////////////////
         /// MANUAl FLYWHEEL ///
        ///////////////////////
        // Set the flywheel to preset velocities based on the bumpers and triggers of the Operator controller.
        operatorController.rightTrigger().toggleOnTrue(flywheel.setVelocity(RPM.of(4800)));
        operatorController.rightBumper().toggleOnTrue(flywheel.setVelocity(RPM.of(2800)));

        
          ///////////////////
         /// MANUAL HOOD ///
        ///////////////////

        // Set the hood to preset angles based on the bumpers and triggers of the
        // Operator controller.
        operatorController.leftTrigger().toggleOnTrue(hood.setAngle(Degrees.of(45)));
        operatorController.leftBumper().toggleOnTrue(hood.setAngle(Degrees.of(60)));

        ///////////////////////
        /// INTAKE COMMANDS ///
        ///////////////////////

        // Deploy and retract the intake with the A and Y buttons, but only when the
        // back button is held to prevent accidental activation during teleop.
        operatorController.a().and(operatorController.back()).toggleOnTrue(slapdown.deployIntake());
        operatorController.y().and(operatorController.back()).toggleOnTrue(slapdown.retractIntake());

    }

          //////////////////////////////////////////////////////
         ///            NON-CONTROL FUNCTIONS               ///
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
        return uptake.set(-0.5).alongWith(hopper.set(-0.5)).alongWith(intake.set(-0.5));
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