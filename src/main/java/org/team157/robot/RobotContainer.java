package org.team157.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team157.robot.Constants.ModifierConstants;
import org.team157.robot.commands.DriveCommands;
import org.team157.robot.generated.TunerConstants;
import org.team157.robot.subsystems.LEDs;
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
import org.team157.robot.subsystems.turret.Turret;
import org.team157.robot.subsystems.turret.TurretIOTalonFX;
import org.team157.robot.subsystems.uptake.Uptake;
import org.team157.robot.subsystems.uptake.UptakeIOTalonFX;
import org.team157.robot.subsystems.vision.VisionSystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private double MaxSpeed =
      1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  // Subsystems
  public static Drive drive;
  public final Turret turret = new Turret();
  public final VisionSystem visionSystem;
  public static final Flywheel flywheel = new Flywheel();
  public final Hood hood = new Hood();
  public final Intake intake = new Intake();
  public final Hopper hopper = new Hopper();
  public final Uptake uptake = new Uptake();
  public final Slapdown slapdown = new Slapdown();
  public final LEDs leds = new LEDs();

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs (auto chooser)
  private final LoggedDashboardChooser<Command> autoChooser;

  // Manual Override Status
  public static boolean manualOverride = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

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
        break;
    }

    // Adjusts drive speed based on if the robot is in rookie/demo mode.
    if (ModifierConstants.DEMO_MODE) {
      MaxSpeed = MaxSpeed * ModifierConstants.DEMO_DRIVE_MODIFIER;
      MaxAngularRate = MaxAngularRate * ModifierConstants.DEMO_DRIVE_MODIFIER;
    } else if (ModifierConstants.ROOKIE_MODE) {
      MaxSpeed = MaxSpeed * ModifierConstants.ROOKIE_DRIVE_MODIFIER;
    }

    // Specify the IO implementation to be used for each subsystem
    intake.setIO(new IntakeIOTalonFX(intake));
    hood.setIO(new HoodIOTalonFX(hood));
    slapdown.setIO(new SlapdownIOTalonFX(slapdown));
    hopper.setIO(new HopperIOTalonFX(hopper));
    uptake.setIO(new UptakeIOTalonFX(uptake));
    flywheel.setIO(new FlywheelIOTalonFX(flywheel));
    visionSystem = new VisionSystem(drive::getPose, Robot.m_field);
    turret.setIO(new TurretIOTalonFX(turret), visionSystem);

    NamedCommands.registerCommand("DeployIntake", slapdown.deployIntake());
    NamedCommands.registerCommand("RunIntake", intake.runIntake());
    NamedCommands.registerCommand("RunHopper", hopper.set(0.5));
    NamedCommands.registerCommand("ShootBalls", uptake.set(1));
    NamedCommands.registerCommand("Wiggle", slapdown.wiggleIntake());
    NamedCommands.registerCommand(
        "WiggleCubed",
        slapdown
            .wiggleIntake()
            .andThen(new WaitCommand(1))
            .andThen(slapdown.wiggleIntake())
            .andThen(new WaitCommand(1))
            .andThen(slapdown.wiggleIntake()));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureBindings();
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
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
    // Update the pose estimation and turret tracking angle while no other vision commands are
    // running.
    visionSystem.setDefaultCommand(visionSystem.setDefault(drive, turret));

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
    // Lock to 0° when A button is held
    // driverController
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> Rotation2d.kZero));

    // Reset gyro to 0° when start and back buttons are pressed
    driverController
        .start()
        .and(driverController.back())
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
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
    if (ModifierConstants.MAYA_MODE) {
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
    turretTrackingTrigger().whileTrue(turret.trackTagGlobalRelative());
    turretTrackingTrigger().whileTrue(flywheel.setDynamicVelocity());
    turretTrackingTrigger().whileTrue(hood.setDynamicHoodAngle());

    ///////////////////////
    /// MANUAL FLYWHEEL ///
    ///////////////////////

    // Only enable manual control of turret, hood and flywheel when manual override is enabled
    // Set the turret to preset robot-relative angles based on the D-Pad input of the Operator
    // controller.
    operatorController.povUp().toggleOnTrue(turret.setAngle(Degrees.of(168.5)));
    // operatorController.povUpRight().toggleOnTrue(turret.setAngle(Degrees.of(-5)));
    operatorController.povRight().toggleOnTrue(turret.setAngle(Degrees.of(78.5)));
    // operatorController.povDownRight().toggleOnTrue(turret.setAngle(Degrees.of(85)));
    operatorController.povDown().toggleOnTrue(turret.setAngle(Degrees.of(-12.5)));
    // operatorController.povDownLeft().toggleOnTrue(turret.setAngle(Degrees.of(175)));
    operatorController.povLeft().toggleOnTrue(turret.setAngle(Degrees.of(-102.5)));
    // operatorController.povUpLeft().toggleOnTrue(turret.setAngle(Degrees.of(265)));

    ///////////////////////
    /// MANUAl FLYWHEEL ///
    ///////////////////////
    // Set the flywheel to preset velocities based on the bumpers and triggers of the Operator
    // controller.
    operatorController.rightTrigger().toggleOnTrue(flywheel.setVelocity(RPM.of(4800)));
    operatorController.rightBumper().toggleOnTrue(flywheel.setVelocity(RPM.of(2800)));

    ///////////////////
    /// MANUAL HOOD ///
    ///////////////////

    // Set the hood to preset angles based on the bumpers and triggers of the
    // Operator controller.
    operatorController.leftTrigger().toggleOnTrue(hood.setAngle(Degrees.of(45)));
    operatorController.leftBumper().toggleOnTrue(hood.setAngle(Degrees.of(65)));

    ///////////////////////
    /// INTAKE COMMANDS ///
    ///////////////////////

    // Deploy and retract the intake with the A and Y buttons, but only when the
    // back button is held to prevent accidental activation during teleop.
    operatorController.a().and(operatorController.back()).toggleOnTrue(slapdown.deployIntake());
    operatorController.y().and(operatorController.back()).toggleOnTrue(slapdown.retractIntake());
  }

  // If the right bumper is held, apply the precision modifier of 0.5x speed.
  public double modifySpeed(final double speed) {
    if (driverController.rightBumper().getAsBoolean() || drive.isUnderTrench()) {
      return speed * ModifierConstants.PRECISION_DRIVE_MODIFIER;
    } else {
      return speed;
    }
  }

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
    boolean shift1Active =
        switch (alliance.get()) {
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
}
