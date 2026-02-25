// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

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


    public final DriveSystem drivetrain = TunerConstants.createDrivetrain();

    public final FlywheelSystem flywheel = new FlywheelSystem();

    public final VisionSystem visionSystem;
    public final TurretSystem turret;
    public final HoodSystem hood = new HoodSystem();
    public final IntakeSystem intake = new IntakeSystem();
    public final HopperSystem hopper = new HopperSystem();
    public final UptakeSystem uptake = new UptakeSystem();


    private final SendableChooser<Command> autoChooser;

    public final Trigger intakeDeployTrigger = new Trigger(() -> intake.getDeployState());
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

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
                SmartDashboard.putData("Auto Chooser", autoChooser);


    }

    private void configureBindings() {
        ////////////////////////////////////////////////////
        /// DRIVETRAIN COMMANDS
        ///////////////////////////////////////////////////
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> drive
                .withVelocityX(MathUtil.applyDeadband(-driverController.getLeftY(),
                        ControllerConstants.LEFT_Y_DEADBAND) * modifySpeed(MaxSpeed)) // Drive forward with
                                                                                        // negative Y
                // (forward)
                .withVelocityY(MathUtil.applyDeadband(-driverController.getLeftX(),
                        ControllerConstants.LEFT_X_DEADBAND) * modifySpeed(MaxSpeed)) // Drive left with
                                                                                        // negative X (left)
                .withRotationalRate(MathUtil.applyDeadband(-driverController.getRightX(),
                        ControllerConstants.RIGHT_X_DEADBAND) * MaxAngularRate) // Drive counterclockwise with
                                                                                // negative X (left)
        ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on start button press.
        driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        

        
        ////////////////////////////////////////////////////
        /// VISION COMMANDS
        ///////////////////////////////////////////////////
        visionSystem.setDefaultCommand(visionSystem.getDefaultCommand(drivetrain, turret));



        ////////////////////////////////////////////////////
        /// TURRET COMMANDS
        ///////////////////////////////////////////////////
        turret.setDefaultCommand(turret.set(0));

        driverController.povUp().toggleOnTrue(turret.setAngle(Degrees.of(-50)));
        driverController.povDown().toggleOnTrue(turret.setAngle(Degrees.of(130)));
        operatorController.povLeft().whileTrue(turret.set(-0.25));
        operatorController.povRight().whileTrue(turret.set(0.25));
        // driverController.x().whileTrue(turret.set(0));

        // driverController.y().toggleOnTrue(turret.trackHubTag());
        // TODO: switch back to operator controller
        driverController.b().toggleOnTrue(turret.trackTagGlobalRelative());


        drivetrain.registerTelemetry(logger::telemeterize);
        
        

        ////////////////////////////////////////////////////////
        /// FLYWHEEL COMMANDS
        ///////////////////////////////////////////////////////
        flywheel.setDefaultCommand(flywheel.set(0));


        driverController.rightTrigger().toggleOnTrue(flywheel.setVelocity(RPM.of(6000)));
        driverController.leftTrigger().toggleOnTrue(flywheel.setVelocity(RPM.of(4500)));
        ////////////////////////////////////////////////////////
        /// INTAKE COMMANDS
        ///////////////////////////////////////////////////////
        intake.setDefaultCommand(intake.setDefault()); 
        driverController.rightBumper().toggleOnTrue(intake.setRoller(0.75));
        //intakeDeployTrigger.onTrue(intake.deployIntake()).onFalse(intake.retractIntake()); //TODO: decide on a button for this
        operatorController.a().toggleOnTrue(intake.deployIntake());
        operatorController.y().toggleOnTrue(intake.retractIntake());
       

        ////////////////////////////////////////////////////////
        /// HOPPER COMMANDS
        ///////////////////////////////////////////////////////
        hopper.setDefaultCommand(hopper.setDefault());

        driverController.leftBumper().toggleOnTrue(hopper.setRoller(0.5).alongWith(uptake.setRoller(1)));

        ////////////////////////////////////////////////////////
        /// UPTAKE COMMANDS
        ///////////////////////////////////////////////////////
        uptake.setDefaultCommand(uptake.setDefault());

        ////////////////////////////////////////////////////////
        /// HOOD COMMANDS
        ///////////////////////////////////////////////////////
        hood.setDefaultCommand(hood.setDefault());
        // driverController.a().toggleOnTrue(hood.setAngle(Degrees.of(60)));
        // driverController.y().toggleOnTrue(hood.setAngle(Degrees.of(48)));
        // Enables dynamic control of the flywheel and hood.
        driverController.a().toggleOnTrue(flywheel.setDynamicVelocity());
        driverController.y().toggleOnTrue(hood.setDynamicHoodAngle());


    }


    public double modifySpeed(final double speed) {
        final var modifier = 1 - driverController.getRightTriggerAxis() * ModifierConstants.PRECISION_DRIVE_MODIFIER;
        return speed * modifier;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}