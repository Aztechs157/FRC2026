// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.team157.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team157.robot.Constants.ModifierConstants;
import org.team157.robot.commands.DriveCommands;
import org.team157.robot.generated.TunerConstants;
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
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


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

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
    // Manual Override Status
    public static boolean manualOverride = false; // When true, allows manual control of the turret, hood, and flywheel, disabling any dynamic control.


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
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
        NamedCommands.registerCommand("WiggleCubed", slapdown.wiggleIntake().andThen(new WaitCommand(1)).andThen(slapdown.wiggleIntake()).andThen(new WaitCommand(1)).andThen(slapdown.wiggleIntake()));

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
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));


    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }


  // If the A button is held, apply the precision modifier of 0.5x speed.
    public double modifySpeed(final double speed) {
        if (driverController.rightBumper().getAsBoolean() || drivetrain.isUnderTrench()) {
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
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
