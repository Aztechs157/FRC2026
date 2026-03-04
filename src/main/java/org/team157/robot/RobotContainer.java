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
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
                SmartDashboard.putData("Auto Chooser", autoChooser);


    }

    private void configureBindings() {
          ////////////////////////
         /// DEFAULT COMMANDS ///
        ////////////////////////
        
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
        final var idle = new SwerveRequest.Idle();
        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
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

          ///////////////////////
         /// DRIVER COMMANDS ///
        ///////////////////////
        
        // Reset the field-centric heading on start and back button press.
        driverController.start().and(driverController.back()).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        
        // Swaps the intake and shooting triggers if Maya mode is enabled, per Maya's preference.
        if(ModifierConstants.MAYA_MODE) {
            // Shooting on left trigger, intake on right trigger
            driverController.leftTrigger().toggleOnTrue(hopper.setRoller(0.5)
                    .alongWith(uptake.setRoller(1)));
            driverController.rightTrigger().toggleOnTrue(intake.setRoller(0.75));
        } else {
            // Shooting on right trigger, intake on left trigger
            driverController.rightTrigger().toggleOnTrue(hopper.setRoller(0.5)
                    .alongWith(uptake.setRoller(1)));
            driverController.leftTrigger().toggleOnTrue(intake.setRoller(0.75));
        }        
        
        driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));

          /////////////////////////
         /// OPERATOR COMMANDS ///
        /////////////////////////
        
        operatorController.leftStick().and(operatorController.rightStick()).onTrue(toggleManualOverride()); // Toggle manual override with both sticks
        driverController.a().onTrue(toggleManualOverride()); // Toggle manual override with both sticks (driver controller as well for redundancy in case of operator controller failure)

        // Disables automatic turret tracking when manual override is enabled, 
        // allowing the operator to control the turret without interference from vision tracking.
        manualOverrideTrigger().whileFalse(turret.trackTagGlobalRelative());

        if(!manualOverride) {
            // Set the turret to preset robot-relative angles based on the D-Pad input of the Operator controller.
            operatorController.povUp().toggleOnTrue(turret.setAngle(Degrees.of(-50)));
            operatorController.povUpRight().toggleOnTrue(turret.setAngle(Degrees.of(-5)));
            operatorController.povRight().whileTrue(turret.setAngle(Degrees.of(40)));
            operatorController.povDownRight().toggleOnTrue(turret.setAngle(Degrees.of(85)));
            operatorController.povDown().toggleOnTrue(turret.setAngle(Degrees.of(130)));
            operatorController.povDownLeft().toggleOnTrue(turret.setAngle(Degrees.of(175)));
            operatorController.povLeft().whileTrue(turret.setAngle(Degrees.of(220)));
            operatorController.povUpLeft().toggleOnTrue(turret.setAngle(Degrees.of(265)));
            
            // Set the flywheel to preset velocities based on the bumpers and triggers of the Operator controller.
            operatorController.rightTrigger().toggleOnTrue(flywheel.setVelocity(RPM.of(5800)));
            operatorController.rightBumper().toggleOnTrue(flywheel.setVelocity(RPM.of(3600)));

            // Set the hood to preset angles based on the bumpers and triggers of the Operator controller.
            operatorController.leftTrigger().toggleOnTrue(hood.set(0.1));
            operatorController.leftBumper().toggleOnTrue(hood.set(-0.1));
        }
       



        operatorController.a().toggleOnTrue(intake.deployIntake());
        operatorController.y().toggleOnTrue(intake.retractIntake());

        //intakeDeployTrigger.onTrue(intake.deployIntake()).onFalse(intake.retractIntake()); //TODO: decide on a button for this

        
        // Enables dynamic control of the turret.
        driverController.x().toggleOnTrue(turret.trackTagGlobalRelative());
        // Enables dynamic control of the flywheel and hood.
        driverController.a().toggleOnTrue(flywheel.setDynamicVelocity().alongWith(hood.setDynamicHoodAngle()));

    }

    public double modifySpeed(final double speed) {
        final var modifier = 1 - driverController.getRightTriggerAxis() * ModifierConstants.PRECISION_DRIVE_MODIFIER;
        return speed * modifier;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /** 
     * Inverts the state of manual override, allowing the operator to toggle between manual and dynamic control of the turret, hood, and flywheel.
     * @return {@link InstantCommand} that toggles manual override when executed.
     * */ 
    private Command toggleManualOverride() {
        return new InstantCommand(() -> {
            manualOverride = !manualOverride;
        });
    }

    private Trigger manualOverrideTrigger() {
        return new Trigger(() -> manualOverride);
    }
}