package org.team157.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team157.robot.Constants.FieldConstants;
import org.team157.robot.Constants.Mode;
import org.team157.robot.Constants.ModifierConstants;
import org.team157.robot.Constants.ModifierConstants.DriveControlMode;
import org.team157.robot.commands.DriveCommands;
import org.team157.robot.generated.TunerConstants;
import org.team157.robot.subsystems.HubTimer;
import org.team157.robot.subsystems.LEDs;
import org.team157.robot.subsystems.SunstoneMechanism3D;
import org.team157.robot.subsystems.drive.Drive;
import org.team157.robot.subsystems.drive.GyroIO;
import org.team157.robot.subsystems.drive.GyroIOPigeon2;
import org.team157.robot.subsystems.drive.ModuleIO;
import org.team157.robot.subsystems.drive.ModuleIOSim;
import org.team157.robot.subsystems.drive.ModuleIOTalonFX;
import org.team157.robot.subsystems.flywheel.Flywheel;
import org.team157.robot.subsystems.flywheel.FlywheelIO;
import org.team157.robot.subsystems.flywheel.FlywheelIOTalonFX;
import org.team157.robot.subsystems.hood.Hood;
import org.team157.robot.subsystems.hood.HoodIO;
import org.team157.robot.subsystems.hood.HoodIOTalonFX;
import org.team157.robot.subsystems.hopper.Hopper;
import org.team157.robot.subsystems.hopper.HopperIO;
import org.team157.robot.subsystems.hopper.HopperIOTalonFX;
import org.team157.robot.subsystems.intake.Intake;
import org.team157.robot.subsystems.intake.IntakeIO;
import org.team157.robot.subsystems.intake.IntakeIOTalonFX;
import org.team157.robot.subsystems.slapdown.Slapdown;
import org.team157.robot.subsystems.slapdown.SlapdownIO;
import org.team157.robot.subsystems.slapdown.SlapdownIOTalonFX;
import org.team157.robot.subsystems.turret.Turret;
import org.team157.robot.subsystems.turret.TurretIO;
import org.team157.robot.subsystems.turret.TurretIOTalonFX;
import org.team157.robot.subsystems.uptake.Uptake;
import org.team157.robot.subsystems.uptake.UptakeIO;
import org.team157.robot.subsystems.uptake.UptakeIOTalonFX;
import org.team157.robot.subsystems.vision.Vision;
import org.team157.robot.subsystems.vision.VisionConstants;
import org.team157.robot.subsystems.vision.VisionIO;
import org.team157.robot.subsystems.vision.VisionIOPhotonVision;
import org.team157.robot.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /**
     * Speed factor used in flywheel ballistic equations, to be manually adjusted by the operator
     */
    public static double ballisticSpeedModifier = 1;

    // Subsystems
    public static Vision vision;
    public static Drive drive;
    public static SunstoneMechanism3D mechanism3D;
    public static final Turret turret = new Turret();
    public static final Flywheel flywheel = new Flywheel();
    public static final Hood hood = new Hood();
    public static final Intake intake = new Intake();
    public static final Hopper hopper = new Hopper();
    public static final Uptake uptake = new Uptake();
    public static final Slapdown slapdown = new Slapdown();
    public static final LEDs leds = new LEDs();

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Dashboard inputs (auto chooser)
    private final LoggedDashboardChooser<Command> autoChooser;
    public static final HubTimer hubStatus = new HubTimer();

    // Manual Override Status
    public static boolean manualOverride = false;
    // Turret Override (Dumper Mode) status
    public static boolean dumperMode = false;

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {

        mechanism3D = new SunstoneMechanism3D(turret, hood, slapdown);

        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // ModuleIOTalonFX is intended for modules with
                // TalonFX drive, TalonFX turn, and a CANcoder
                drive =
                        new Drive(
                                new GyroIOPigeon2(),
                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                new ModuleIOTalonFX(TunerConstants.BackRight));
                vision =
                        new Vision(
                                drive::addVisionMeasurement,
                                new VisionIOPhotonVision(
                                        VisionConstants.camera0Name,
                                        VisionConstants.robotToCamera0),
                                new VisionIOPhotonVision(
                                        VisionConstants.camera1Name,
                                        VisionConstants.robotToCamera1),
                                new VisionIOPhotonVision(
                                        VisionConstants.camera2Name,
                                        VisionConstants.robotToCamera2));
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive =
                        new Drive(
                                new GyroIO() {},
                                new ModuleIOSim(TunerConstants.FrontLeft),
                                new ModuleIOSim(TunerConstants.FrontRight),
                                new ModuleIOSim(TunerConstants.BackLeft),
                                new ModuleIOSim(TunerConstants.BackRight));
                vision =
                        new Vision(
                                drive::addVisionMeasurement,
                                new VisionIOPhotonVisionSim(
                                        VisionConstants.camera0Name,
                                        VisionConstants.robotToCamera0,
                                        drive::getPose),
                                new VisionIOPhotonVisionSim(
                                        VisionConstants.camera1Name,
                                        VisionConstants.robotToCamera1,
                                        drive::getPose),
                                new VisionIOPhotonVisionSim(
                                        VisionConstants.camera2Name,
                                        VisionConstants.robotToCamera2,
                                        drive::getPose));
                break;

            default:
                // Replayed robot, disable IO implementations
                drive =
                        new Drive(
                                new GyroIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {});
                vision =
                        new Vision(
                                drive::addVisionMeasurement,
                                new VisionIO() {},
                                new VisionIO() {},
                                new VisionIO() {});
                break;
        }

        // Specify the IO implementation to be used for each subsystem
        if (Constants.currentMode == Mode.REPLAY) {
            // Disable IO implementations during log REPLAY
            intake.setIO(new IntakeIO() {});
            hood.setIO(new HoodIO() {});
            slapdown.setIO(new SlapdownIO() {});
            hopper.setIO(new HopperIO() {});
            uptake.setIO(new UptakeIO() {});
            flywheel.setIO(new FlywheelIO() {}, vision);
            turret.setIO(new TurretIO() {}, vision);
        } else {
            // Use TalonFX IO implementations on REAL or SIM robot.
            // Not included in initial switch case, as the TalonFX
            // IO layers automatically switch between real and sim
            // implementations based on the curren mode.
            intake.setIO(new IntakeIOTalonFX(intake));
            hood.setIO(new HoodIOTalonFX(hood));
            slapdown.setIO(new SlapdownIOTalonFX(slapdown));
            hopper.setIO(new HopperIOTalonFX(hopper));
            uptake.setIO(new UptakeIOTalonFX(uptake));
            flywheel.setIO(new FlywheelIOTalonFX(flywheel), vision);
            turret.setIO(new TurretIOTalonFX(turret), vision);
        }

        NamedCommands.registerCommand("DeployIntake", slapdown.deployIntake());
        NamedCommands.registerCommand("RunIntake", intake.runIntake());
        NamedCommands.registerCommand("RunHopper", hopper.set(0.5));
        NamedCommands.registerCommand("ShootBalls", shootBalls());
        NamedCommands.registerCommand("Stop Shooting", stopShooter());
        NamedCommands.registerCommand("Wiggle", slapdown.wiggleIntake());
        NamedCommands.registerCommand(
                "WiggleCubed",
                slapdown.wiggleIntake()
                        .andThen(new WaitCommand(1))
                        .andThen(slapdown.wiggleIntake())
                        .andThen(new WaitCommand(1))
                        .andThen(slapdown.wiggleIntake()));

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        if (!DriverStation.isFMSAttached()) {
            // Set up SysId routines only when not connected to FMS
            autoChooser.addOption(
                    "Drive Wheel Radius Characterization",
                    DriveCommands.wheelRadiusCharacterization(drive));
            autoChooser.addOption(
                    "Drive Simple FF Characterization",
                    DriveCommands.feedforwardCharacterization(drive));
            autoChooser.addOption(
                    "Drive SysId (Quasistatic Forward)",
                    drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption(
                    "Drive SysId (Quasistatic Reverse)",
                    drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            autoChooser.addOption(
                    "Drive SysId (Dynamic Forward)",
                    drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption(
                    "Drive SysId (Dynamic Reverse)",
                    drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
            autoChooser.addOption(
                    "Flywheel SysId (Quasistatic Forward)",
                    flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption(
                    "Flywheel SysId (Quasistatic Reverse)",
                    flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            autoChooser.addOption(
                    "Flywheel SysId (Dynamic Forward)",
                    flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption(
                    "Flywheel SysId (Dynamic Reverse)",
                    flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));
            autoChooser.addOption(
                    "Turret SysId (Quasistatic Forward)",
                    turret.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption(
                    "Turret SysId (Quasistatic Reverse)",
                    turret.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            autoChooser.addOption(
                    "Turret SysId (Dynamic Forward)",
                    turret.sysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption(
                    "Turret SysId (Dynamic Reverse)",
                    turret.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }

        // Configure the button bindings
        configureBindings();

        RobotController.setBrownoutVoltage(6.0);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureBindings() {

        /////////////////////////////////////////////////
        ///             DEFAULT COMMANDS              ///
        /////////////////////////////////////////////////

        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> joystickModifier(-driverController.getLeftY()),
                        () -> joystickModifier(-driverController.getLeftX()),
                        () -> joystickModifier(-driverController.getRightX())));
        // Update the pose estimation and turret tracking angle while no other vision commands are
        // running.
        vision.setDefaultCommand(vision.setDefault(drive, turret));

        turret.setDefaultCommand(turret.getDefault());
        flywheel.setDefaultCommand(flywheel.getDefault());
        slapdown.setDefaultCommand(slapdown.getDefault());
        intake.setDefaultCommand(intake.getDefault());
        hopper.setDefaultCommand(hopper.getDefault());
        uptake.setDefaultCommand(uptake.setDefault());
        hood.setDefaultCommand(hood.getDefault(drive));

        //////////////////////////////////////////////
        ///             DRIVER COMMANDS            ///
        //////////////////////////////////////////////
        // Face hub when Dumper Mode (toggled by operator LT + RT)
        driverController
                .rightTrigger()
                .and(dumperModeTrigger())
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive, () -> 0, () -> 0, vision::getDriveAngleToFaceHub));

        // Reset gyro to 0° when start and back buttons are pressed
        driverController
                .start()
                .and(driverController.back())
                .onTrue(
                        Commands.runOnce(
                                        () ->
                                                drive.setPose(
                                                        new Pose2d(
                                                                drive.getPose().getTranslation(),
                                                                Rotation2d.kZero)),
                                        drive)
                                .ignoringDisable(true));

        /////////////////////
        /// FlYWHEEL HOOD ///
        /////////////////////
        // Enables dynamic control of the flywheel and hood.
        if (ModifierConstants.currentControlMode == DriveControlMode.DEMO) {
            driverController.a().toggleOnTrue(flywheel.setVelocity(RPM.of(2800)));
            driverController.b().toggleOnTrue(flywheel.setVelocity(RPM.of(800)));

        } else {
            driverController.a().toggleOnTrue(flywheel.setDynamicVelocity());
            driverController.b().onTrue(Commands.runOnce(drive::stopWithX, drive));
        }

        ////////////////////////////
        /// INTAKE UPTAKE HOPPER ///
        ////////////////////////////

        driverController.rightTrigger().whileTrue(uptake.set(1));
        driverController.rightTrigger().whileTrue(hopper.set(1));

        driverController.leftTrigger().whileTrue(intake.runIntake());

        // Runs the hopper, uptake, and intake backwards at a low speed to clear jams.
        driverController.y().whileTrue(forceOuttake());
        // Wiggles the intake up and down to free up stuck balls
        operatorController
                .x()
                .and(operatorController.start())
                .toggleOnTrue(slapdown.wiggleIntake());

        // (in/de)creases the ballistic modifier
        operatorController
                .y()
                .or(operatorController.a())
                .and(operatorController.back().negate())
                .onTrue(setModifier());
        //////////////////////////////////////////////////
        ///             OPERATOR COMMANDS              ///
        //////////////////////////////////////////////////

        // Toggle manual override with both sticks to prevent accidental activation during teleop.
        operatorController
                .leftStick()
                .and(operatorController.rightStick())
                .onTrue(toggleManualOverride());

        // Disables automatic turret tracking when manual override is enabled,
        // allowing the operator to control the turret without interference from vision tracking.
        turretTrackingTrigger()
                .and(dumperModeTrigger().negate())
                .whileTrue(turret.trackTagGlobalRelative());

        if (ModifierConstants.currentControlMode == DriveControlMode.STANDARD) {
            turretTrackingTrigger().whileTrue(flywheel.setDynamicVelocity());
        } else {
            turretTrackingTrigger().whileTrue(flywheel.setVelocity(RPM.of(2800)));
        }
        turretTrackingTrigger()
                .and(driverController.rightTrigger())
                .whileTrue(hood.setDynamicHoodAngle());

        ///////////////////////
        /// MANUAL FLYWHEEL ///
        ///////////////////////

        // Only enable manual control of turret, hood and flywheel when manual override is enabled
        // Set the turret to preset robot-relative angles based on the D-Pad input of the Operator
        // controller.
        operatorController
                .povUp()
                .and(manualOverrideTrigger().or(turretTrackingTrigger().negate()))
                .toggleOnTrue(turret.setAngle(Degrees.of(168.5)));
        // operatorController.povUpRight().toggleOnTrue(turret.setAngle(Degrees.of(-5)));
        operatorController
                .povRight()
                .and(manualOverrideTrigger().or(turretTrackingTrigger().negate()))
                .toggleOnTrue(turret.setAngle(Degrees.of(78.5)));
        // operatorController.povDownRight().toggleOnTrue(turret.setAngle(Degrees.of(85)));
        operatorController
                .povDown()
                .and(manualOverrideTrigger().or(turretTrackingTrigger().negate()))
                .toggleOnTrue(turret.setAngle(Degrees.of(-12.5)));
        // operatorController.povDownLeft().toggleOnTrue(turret.setAngle(Degrees.of(175)));
        operatorController
                .povLeft()
                .and(manualOverrideTrigger().or(turretTrackingTrigger().negate()))
                .toggleOnTrue(turret.setAngle(Degrees.of(-102.5)));
        // operatorController.povUpLeft().toggleOnTrue(turret.setAngle(Degrees.of(265)));

        ///////////////////////
        /// MANUAl FLYWHEEL ///
        ///////////////////////
        // Set the flywheel to preset velocities based on the bumpers and triggers of the Operator
        // controller.
        operatorController
                .rightTrigger()
                .and(manualOverrideTrigger().or(turretTrackingTrigger().negate()))
                .toggleOnTrue(flywheel.setVelocity(RPM.of(4800)));
        operatorController
                .rightBumper()
                .and(manualOverrideTrigger().or(turretTrackingTrigger().negate()))
                .toggleOnTrue(flywheel.setVelocity(RPM.of(2800)));

        ///////////////////
        /// MANUAL HOOD ///
        ///////////////////

        // Set the hood to preset angles based on the bumpers and triggers of the
        // Operator controller.
        operatorController
                .leftTrigger()
                .and(manualOverrideTrigger().or(turretTrackingTrigger().negate()))
                .toggleOnTrue(hood.setAngle(Degrees.of(45)));
        operatorController
                .leftBumper()
                .and(manualOverrideTrigger().or(turretTrackingTrigger().negate()))
                .toggleOnTrue(hood.setAngle(Degrees.of(65)));

        ///////////////////////
        /// INTAKE COMMANDS ///
        ///////////////////////

        // Deploy and retract the intake with the A and Y buttons, but only when the
        // back button is held to prevent accidental activation during teleop.
        operatorController //
                .a() //
                .and(operatorController.back()) //
                .toggleOnTrue(slapdown.deployIntake());
        operatorController
                .y()
                .and(operatorController.back())
                .toggleOnTrue(slapdown.retractIntake());

        // Enable Dumper Mode (align with drivebase rather than turret)
        operatorController.start().and(operatorController.back()).onTrue(toggleDumperMode());
        // Manual operator turret control in dumper/manual modes.
        operatorController
                .x()
                .and(manualOverrideTrigger().or(dumperModeTrigger()))
                .whileTrue(turret.set(0.05));
        operatorController
                .b()
                .and(manualOverrideTrigger().or(dumperModeTrigger()))
                .whileTrue(turret.set(-0.05));
    }

    /**
     * Applies speed modifiers based on the current control mode and the robot's current
     * position/state.
     */
    public double joystickModifier(final double speed) {
        double outputSpeed = speed;

        // Applies precision modifier if shooting from within alliance zone, or when right bumper is
        // held.
        if (driverController.rightBumper().getAsBoolean()
                || driverController.rightTrigger().getAsBoolean()
                        && FieldConstants.positionDetails.isInAllianceZone(
                                drive.getPose(), DriverStation.getAlliance())) {
            outputSpeed *= ModifierConstants.PRECISION_DRIVE_MODIFIER;
        } else if (driverController.rightTrigger().getAsBoolean()
                && !FieldConstants.positionDetails.isInAllianceZone(
                        drive.getPose(), DriverStation.getAlliance())) {
            // Applies neutral modifier when shooting from outside of alliance zone
            outputSpeed *= ModifierConstants.NEUTRAL_DRIVE_MODIFIER;
        } else if (drive.isUnderTrench()) {
            outputSpeed *= ModifierConstants.TRENCH_DRIVE_MODIFIER;
        }

        return outputSpeed;
    }

    /**
     * Enables controller rumble when 2 seconds remain in the current shift, or when the match is 7
     * seconds from ending (for BC dot).
     */
    public void setRumble() {
        if (hubStatus.isShiftAboutToEnd(2)
                || (hubStatus.isShiftAboutToEnd(7) && DriverStation.isTeleop())) {
            driverController.setRumble(RumbleType.kLeftRumble, 1);
            driverController.setRumble(RumbleType.kRightRumble, 1);
            operatorController.setRumble(RumbleType.kLeftRumble, 1);
            operatorController.setRumble(RumbleType.kRightRumble, 1);
        } else {
            cancelRumble();
        }
    }

    public void cancelRumble() {
        driverController.setRumble(RumbleType.kLeftRumble, 0);
        driverController.setRumble(RumbleType.kRightRumble, 0);
        operatorController.setRumble(RumbleType.kLeftRumble, 0);
        operatorController.setRumble(RumbleType.kRightRumble, 0);
    }

    /** Update the ballistic equation modifier based on the operator's button presses */
    public void setBallisticSpeedModifier() {
        if (operatorController.y().getAsBoolean()) {
            ballisticSpeedModifier = ballisticSpeedModifier + 0.05;
        } else if (operatorController.a().getAsBoolean()) {
            ballisticSpeedModifier = ballisticSpeedModifier - 0.05;
        }
    }

    public InstantCommand setModifier() {
        return new InstantCommand(() -> setBallisticSpeedModifier());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    /**
     * Inverts the state of manual override, allowing the operator to toggle between manual and
     * dynamic control of the turret, hood, and flywheel.
     *
     * @return {@link InstantCommand} that toggles manual override when executed.
     */
    private Command toggleManualOverride() {
        return new InstantCommand(
                () -> {
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
     * @return {@link Trigger} that is true when the robot is in teleop or autonomous and manual
     *     override is not enabled, allowing the turret to track targets when those conditions are
     *     met.
     */
    private Trigger turretTrackingTrigger() {
        return new Trigger(
                () ->
                        (RobotModeTriggers.teleop().getAsBoolean()
                                        || RobotModeTriggers.autonomous().getAsBoolean())
                                && !manualOverride);
    }

    /**
     * Returns the current state of Dumper Mode.
     *
     * @return a {@link Trigger} with the current state of Dumper Mode
     */
    private Trigger dumperModeTrigger() {
        return new Trigger(() -> (dumperMode));
    }

    /**
     * Returns the current state of Manual Override.
     *
     * @return a {@link Trigger} with the current state of Dumper Mode
     */
    private Trigger manualOverrideTrigger() {
        return new Trigger(() -> (manualOverride));
    }

    /**
     * Inverts the state of dumper mode, allowing for drivebase-centric targeting when true.
     *
     * @return an {@link InstantCommand} toggling the value of dumperMode
     */
    private Command toggleDumperMode() {
        return new InstantCommand(
                () -> {
                    dumperMode = !dumperMode;
                });
    }

    /** Enables the uptake and dynamic hood during auto to shoot balls. */
    private Command shootBalls() {
        return uptake.set(1).alongWith(hood.setDynamicHoodAngle().withTimeout(9)).withTimeout(9);
    }

    /** Stops the uptake and stows the hood during auto to ensure safe trench clearance. */
    private Command stopShooter() {

        return uptake.set(0).alongWith(hood.setAngle(Degrees.of(65)));
    }
}
