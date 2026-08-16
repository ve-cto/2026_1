// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Haiiiii :3

package frc.robot;

// Swerve
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
// import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// Pathplanner and Poses
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.networktables.NetworkTableInstance;

import static edu.wpi.first.units.Units.*;

// import java.util.function.DoubleSupplier;

// import java.util.function.BooleanSupplier;

// Command Setup and Controllers
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// Commands
// import frc.robot.commands.drive.DriveToApriltag;
// import frc.robot.commands.drive.DriveToPose;
// import frc.robot.commands.drive.PointToHub;
// import frc.robot.commands.drive.PointToAngle;
import frc.robot.subsystems.Stopper;
import frc.robot.Constants.Led.StatusList;
import frc.robot.commands.ShootAtAllianceFuel;
// import frc.robot.commands.drive.PointToPose;
// import frc.robot.commands.RunDebugMotors;
import frc.robot.commands.ShootAtTarget;
import frc.robot.commands.ShootAtTargetAutonomous;
// import frc.robot.commands.drive.PointToAllianceFuel;
// Subsystems
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeNeo;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.shooter.HoodedShooter;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.util.NetworkTablesIO;
import frc.robot.subsystems.util.TrajectoryCalculator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Feeder;

public class RobotContainer {
    // #region Swerve setup
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(Constants.Swerve.kMaxAngularRps).in(RadiansPerSecond); // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * Constants.Swerve.kDeadbandFraction)
        .withRotationalDeadband(MaxAngularRate * Constants.Swerve.kDeadbandFraction) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
    //     .withDeadband(MaxSpeed * Constants.Swerve.kDeadbandFraction)
    //     .withRotationalDeadband(MaxAngularRate * Constants.Swerve.kDeadbandFraction) // Add a `% deadband
    //     .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    // #endregion Swerve setup
    
    // private final CommandPS4Controller primaryJoystick = new CommandXboxController(Constants.Controller.kDriverControllerPort);
    // private final CommandPS4Controller secondaryJoystick = new CommandXboxController(Constants.Controller.kOperatorControllerPort);
    private final CommandXboxController primaryJoystick = new CommandXboxController(Constants.Controller.kDriverControllerPort);
    private final CommandXboxController secondaryJoystick = new CommandXboxController(Constants.Controller.kOperatorControllerPort);
    
    // #region Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final NetworkTablesIO m_networkTablesIO = new NetworkTablesIO(primaryJoystick, secondaryJoystick);
    public final Vision m_vision = new Vision(drivetrain);
    private final IntakeNeo m_intake = new IntakeNeo();
    private final Arm m_arm = new Arm();
    // private final DebugMotors m_DebugMotors = new DebugMotors();
    private final Led m_led = new Led();
    private final Shooter m_shooter = new Shooter();
    private final Feeder m_feeder = new Feeder();
    private final HoodedShooter m_hood = new HoodedShooter();
    private final Stopper m_stopper = new Stopper();
    private final TrajectoryCalculator m_trajectoryCalculator = new TrajectoryCalculator(m_networkTablesIO);
    // #endregion Subsystems

    // private final Command shootAtHub(DoubleSupplier x, DoubleSupplier y) {
    //     return new ShootAtTarget(
    //             () -> m_networkTablesIO.getOwnHubPose(), 
    //             x,
    //             y,
    //             m_shooter, 
    //             m_hood, 
    //             drivetrain, 
    //             m_feeder, 
    //             m_stopper,
    //             m_trajectoryCalculator
    //         );
    //     }

    private final SendableChooser<Command> autoChooser;

    private final Alert alertEstopped = new Alert("Robot has been EStopped and requires a restart or redeploy to resume operation.", AlertType.kError);

    public RobotContainer() {
        NamedCommands.registerCommand("ExtendIntake", m_arm.runVoltageCommand(Volts.of(-10.0)));
        NamedCommands.registerCommand("RetractIntake", m_arm.runVoltageCommand(Volts.of(10.5)));
        NamedCommands.registerCommand("RunIntake", m_intake.runIntakeCommand(-0.9));
        NamedCommands.registerCommand("ReverseIntake", m_intake.reverseIntakeCommand());
        // NamedCommands.registerCommand("StopIntake", m_intake.stop());
        // NamedCommands.registerCommand("HoodHub", m_hood.gotoAngleCommand(m_trajectoryCalculator.getRequiredHoodAngleHub()));
        // NamedCommands.registerCommand("ShooterHub", m_shooter.runRPMCommand(m_trajectoryCalculator.getRequiredShooterSpeedHub()));
        NamedCommands.registerCommand("Feed", m_feeder.feedCommand());
        // NamedCommands.registerCommand("ShooterStop", m_shooter.coastCommand());
        // NamedCommands.registerCommand("HomeHood", m_hood.resetHomeCommand());
        NamedCommands.registerCommand("ShootAtHub", new ShootAtTargetAutonomous(
                () -> m_networkTablesIO.getOwnHubPose(), 
                m_shooter, 
                m_hood, 
                drivetrain, 
                m_feeder, 
                m_stopper,
                m_trajectoryCalculator
            ));
        NamedCommands.registerCommand("ShootAtAllianceFuel", new ShootAtTargetAutonomous(
                m_trajectoryCalculator.getClosestAllianceFuel(() -> drivetrain.getStateCopy()), 
                m_shooter, 
                m_hood, 
                drivetrain, 
                m_feeder, 
                m_stopper,
                m_trajectoryCalculator
            ));
        NamedCommands.registerCommand("ExtendStopper", m_stopper.extendCommand());
        NamedCommands.registerCommand("HomeExtendStopper", m_stopper.homeExtendCommand());


        // NamedCommands.registerCommand("ShootAtHub", new ShootAtTarget(
        //         () -> m_networkTablesIO.getOwnHubPose(), 
        //         () -> 0.0,
        //         () -> 0.0,
        //         m_shooter, 
        //         m_hood, 
        //         drivetrain, 
        //         m_feeder, 
        //         m_stopper,
        //         m_trajectoryCalculator
        //     ));

        // Create our auto chooser
        // Pathplanner autos get populated into it automatically
        // for (String auto : AutoBuilder.getAllAutoNames()) {
        //     if (auto.contains("FLIP")) {
                
        //     }
        // }
        autoChooser = BetterAutoChooser.buildAutoChooser();
        // autoChooser = FlippingAutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Warm up on-the-fly path generation
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        // Bind the commands to the controller inputs.
        configureBindings();
    }

    private void configureBindings() {
        // #region Misc
        // Point the modules towards the direction of the left stick, without driving the robot. Note that this does not get updated while holding, only on initialize. (They aren't double suppliers)
        // primaryJoystick.triangle().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-primaryJoystick.getLeftY(), -primaryJoystick.getLeftX()))
        // ));

        // Make the drivetrain idle when robot is disabled. (note that this is called only once)
        final var swerveIdle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> swerveIdle).ignoringDisable(true)
        );
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> swerveIdle).ignoringDisable(true)
        );
        // Disable the drivetrain while in test mode to avoid noisy motors
        RobotModeTriggers.test().whileTrue(
            drivetrain.applyRequest(() -> swerveIdle).ignoringDisable(true)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // primaryJoystick.back().and(primaryJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // primaryJoystick.back().and(primaryJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // primaryJoystick.start().and(primaryJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // primaryJoystick.start().and(primaryJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Update the robot's odometry. 
        drivetrain.registerTelemetry(logger::telemeterize);

        // Drive robot-centric instead of field-centric while held.
        // primaryJoystick.leftBumper().whileTrue(
        //     drivetrain.applyRequest(() ->
        //         driveRobotCentric.withVelocityX(-primaryJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-primaryJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-primaryJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );
        
        // Brake while holding. When the robot brakes, the four drive motors stop and the modules point towards the center of the robot.
        // primaryJoystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
        // primaryJoystick.R1().whileTrue(drivetrain.applyRequest(() -> brake));
        // #endregion Misc
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                driveFieldCentric.withVelocityX(-primaryJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-primaryJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-primaryJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        m_intake.setDefaultCommand(
            m_intake.coastCommand()
        );
        m_arm.setDefaultCommand(
            m_arm.coastCommand()
        );
        m_feeder.setDefaultCommand(
            m_feeder.coastCommand()
        );
        m_shooter.setDefaultCommand(
            m_shooter.coastCommand()
            // m_shooter.runRPMCommand(() -> 1800)
        );
        m_hood.setDefaultCommand(
            m_hood.homeCommand()
        );
        m_stopper.setDefaultCommand(
            m_stopper.homeExtendCommand()
        );
        // Default to displaying the specific modes' pattern (disconn, disabl, auto, teleop)
        m_led.setDefaultCommand(
            Commands.either(
                m_led.display(StatusList.DEBUG),
                m_led.handleDefault(),
                m_networkTablesIO.secondaryJoystickEnabled().and(() -> DriverStation.isEnabled())
            ).ignoringDisable(true)
        );

        // --------------------------------------------------------------------------------------------------------------------------------
        // When we enable test mode, update the subsystem PID configs (kp ki kd) 
        RobotModeTriggers.test().whileTrue(m_shooter.updateMotorConfigsCommand().alongWith(m_hood.updateMotorConfigsCommand()));
        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop().or(RobotModeTriggers.test())).onChange(m_trajectoryCalculator.updateAllianceCommand().ignoringDisable(true));
        // If the robot is ESTOPPED, flash and alert
        RobotModeTriggers.disabled().and(() -> DriverStation.isEStopped()).whileTrue(m_led.estop().ignoringDisable(true));
        RobotModeTriggers.disabled().and(() -> DriverStation.isEStopped()).onTrue(Commands.runOnce(() -> alertEstopped.set(true)).ignoringDisable(true));
        m_networkTablesIO.radioReady().onTrue(m_led.flash(Constants.Led.StatusList.DSCONNECTED, 2, 0.5).ignoringDisable(true));
        m_networkTablesIO.DSAttached().onTrue(m_led.flash(Constants.Led.StatusList.DSCONNECTED, 1, 0.1).ignoringDisable(true));
        m_networkTablesIO.DSAttached().onFalse(m_led.flash(Constants.Led.StatusList.DSDISCONNECTED, 1, 0.1).ignoringDisable(true));

        // Reset the field-centric heading on button press. Note that this has limited effect during the actual game, as m_vision measurements will override it.
        primaryJoystick.povLeft().and(primaryJoystick.rightStick()).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // Disable vision processing for this match.
        primaryJoystick.povDown().and(primaryJoystick.rightStick()).whileTrue(
          m_vision.setVisionEnabled(false).asProxy()
        );
        // Re-enable vision processing for this match.
        primaryJoystick.povUp().and(primaryJoystick.rightStick()).whileTrue(
            m_vision.setVisionEnabled(true).asProxy()
        );
        // primaryJoystick.povUp().and(primaryJoystick.rightStick()).onTrue(
        //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        // );
        RobotModeTriggers.disabled().negate().and(m_vision.isVisionEnabled()).whileTrue( 
            m_vision.addVisionMeasurementCommand(() -> drivetrain.getStateCopy())
        );

        // --------------------------------------------------------------------------------------------------------------------------------
    
        // primaryJoystick.rightBumper().whileTrue(
        //     m_intake.runIntakeCommand()
        // );
        primaryJoystick.rightBumper().and(m_networkTablesIO.secondaryJoystickEnabled().negate()).whileTrue(
            m_intake.runIntakeCommand(-0.9)
        );

        primaryJoystick.leftTrigger().and(m_networkTablesIO.secondaryJoystickEnabled().negate()).whileTrue(
            m_arm.runVoltageCommand(Volts.of(-11))
        );

        primaryJoystick.leftBumper().and(m_networkTablesIO.secondaryJoystickEnabled().negate()).whileTrue(
            m_arm.runVoltageCommand(Volts.of(11))
        );

        // primaryJoystick.povLeft().whileTrue(
        //     m_stopper.homeCommand()
        // );
        // primaryJoystick.povRight().whileTrue(
        //     m_stopper.extendCommand()
        // );
        primaryJoystick.b().and(m_networkTablesIO.secondaryJoystickEnabled().negate()).whileTrue(m_shooter.runDashboard().alongWith(m_hood.gotoDashboard()));
        primaryJoystick.x().and(m_networkTablesIO.secondaryJoystickEnabled().negate()).whileTrue(m_hood.homeCommand());
        primaryJoystick.a().and(m_networkTablesIO.secondaryJoystickEnabled().negate()).whileTrue(m_feeder.feedCommand().alongWith(m_stopper.homeCommand()));

        primaryJoystick.rightTrigger().and(m_networkTablesIO.secondaryJoystickEnabled().negate()).and(() -> m_networkTablesIO.isInOwnAllianceZone()).whileTrue(
            new ShootAtTarget(
                () -> m_networkTablesIO.getOwnHubPose(), 
                () -> -primaryJoystick.getLeftY() * MaxSpeed, 
                () -> -primaryJoystick.getLeftX() * MaxSpeed, 
                m_shooter, 
                m_hood, 
                drivetrain, 
                m_feeder, 
                m_stopper,
                m_trajectoryCalculator
            )
            // .alongWith(
            //     Commands.runEnd(
            //         () -> primaryJoystick.setRumble(RumbleType.kBothRumble, 0.5),
            //         () -> primaryJoystick.setRumble(RumbleType.kBothRumble, 0.0)
            //     )
            // )
        );

        // primaryJoystick.rightTrigger().and(() -> !m_networkTablesIO.isInOwnAllianceZone()).whileTrue(
        //     new ShootAtTarget(
        //         m_trajectoryCalculator.getClosestAllianceFuel(() -> drivetrain.getStateCopy()), 
        //         () -> -primaryJoystick.getLeftY() * MaxSpeed, 
        //         () -> -primaryJoystick.getLeftX() * MaxSpeed, 
        //         m_shooter, 
        //         m_hood, 
        //         drivetrain, 
        //         m_feeder, 
        //         m_stopper,
        //         m_trajectoryCalculator
        //     )
        //     // .alongWith(
        //     //     Commands.runEnd(
        //     //         () -> primaryJoystick.setRumble(RumbleType.kBothRumble, 0.5),
        //     //         () -> primaryJoystick.setRumble(RumbleType.kBothRumble, 0.0)
        //     //     )
        //     // )
        // );
        primaryJoystick.rightTrigger().and(m_networkTablesIO.secondaryJoystickEnabled().negate()).and(() -> !m_networkTablesIO.isInOwnAllianceZone()).whileTrue(
            new ShootAtAllianceFuel(
                m_trajectoryCalculator.getClosestAllianceFuel(() -> drivetrain.getStateCopy()), 
                () -> -primaryJoystick.getLeftY() * MaxSpeed, 
                () -> -primaryJoystick.getLeftX() * MaxSpeed, 
                m_shooter, 
                m_hood, 
                drivetrain, 
                m_feeder, 
                m_stopper
            )
            // .alongWith(
            //     Commands.runEnd(
            //         () -> primaryJoystick.setRumble(RumbleType.kBothRumble, 0.5),
            //         () -> primaryJoystick.setRumble(RumbleType.kBothRumble, 0.0)
            //     )
            // )
        );

        primaryJoystick.rightTrigger().and(m_networkTablesIO.secondaryJoystickEnabled().negate()).and(primaryJoystick.rightStick()).whileTrue(
          m_shooter.runRPMCommand(() -> 1800).alongWith(m_hood.gotoAngleCommand(() -> 50)).alongWith(m_feeder.feedCommand()).alongWith(m_stopper.homeCommand())  
        );

        // primaryJoystick.rightTrigger().whileTrue(
        //     m_led.display(StatusList.PMSHOOTER).onlyWhile(m_stopper.isHomed().negate())  
        // );
        primaryJoystick.rightTrigger().and(m_networkTablesIO.secondaryJoystickEnabled().negate()).and(m_stopper.isHomed()).whileTrue(
            m_led.display(StatusList.SHOOTING)
        );
        m_networkTablesIO.hubActive().onChange( 
            m_led.flash(StatusList.SHOOTING, 1, 0.2)
        );

        // Rumble controller when the hub state switches
        m_networkTablesIO.hubActive().onChange(
            Commands.runOnce(
                    () -> primaryJoystick.setRumble(RumbleType.kBothRumble, 1.0)
                ).andThen(
                    Commands.waitSeconds(0.25)
                ).andThen(
                    () -> primaryJoystick.setRumble(RumbleType.kBothRumble, 0.0)
                ).andThen(
                    Commands.waitSeconds(0.1)
                ).andThen(
                    () -> primaryJoystick.setRumble(RumbleType.kBothRumble, 1.0)
                ).andThen(
                    Commands.waitSeconds(0.25)
                ).andThen(
                    () -> primaryJoystick.setRumble(RumbleType.kBothRumble, 0.0)
                ).ignoringDisable(true)
        );

        // --------------------------------------------------------------------------------------------------------------------------------
        // Enable secondary controller overrides
        primaryJoystick.back().and(primaryJoystick.start()).onTrue(
            m_networkTablesIO.setSecondaryJoystickEnabled(() -> !primaryJoystick.rightStick().getAsBoolean())  // allow disabling if modifier pressed (just grow extra fingers)
        );
        secondaryJoystick.back().and(secondaryJoystick.start()).onTrue(
            m_networkTablesIO.setSecondaryJoystickEnabled(() -> !secondaryJoystick.rightStick().getAsBoolean())  
        );

        //

        secondaryJoystick.rightTrigger().and(m_networkTablesIO.secondaryJoystickEnabled()).and(m_stopper.isHomed()).whileTrue(
            m_led.display(StatusList.SHOOTING)
        );

        m_networkTablesIO.secondaryJoystickEnabled().whileTrue(
            drivetrain.applyRequest(() ->
                driveFieldCentric.withVelocityX(-secondaryJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-secondaryJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-secondaryJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
            
        );

        secondaryJoystick.rightTrigger().and(m_networkTablesIO.secondaryJoystickEnabled()).and(() -> !m_networkTablesIO.isInOwnAllianceZone()).whileTrue(
            new ShootAtAllianceFuel(
                m_trajectoryCalculator.getClosestAllianceFuel(() -> drivetrain.getStateCopy()), 
                () -> -primaryJoystick.getLeftY() * MaxSpeed, 
                () -> -primaryJoystick.getLeftX() * MaxSpeed, 
                m_shooter, 
                m_hood, 
                drivetrain, 
                m_feeder, 
                m_stopper
            )
            // .alongWith(
            //     Commands.runEnd(
            //         () -> primaryJoystick.setRumble(RumbleType.kBothRumble, 0.5),
            //         () -> primaryJoystick.setRumble(RumbleType.kBothRumble, 0.0)
            //     )
            // )
        );

        secondaryJoystick.b().and(m_networkTablesIO.secondaryJoystickEnabled()).whileTrue(m_shooter.runDashboard().alongWith(m_hood.gotoDashboard()));
        secondaryJoystick.x().and(m_networkTablesIO.secondaryJoystickEnabled()).whileTrue(m_hood.homeCommand());
        secondaryJoystick.a().and(m_networkTablesIO.secondaryJoystickEnabled()).whileTrue(m_feeder.feedCommand().alongWith(m_stopper.homeCommand()));

        secondaryJoystick.rightTrigger().and(m_networkTablesIO.secondaryJoystickEnabled()).and(() -> m_networkTablesIO.isInOwnAllianceZone()).whileTrue(
            new ShootAtTarget(
                () -> m_networkTablesIO.getOwnHubPose(), 
                () -> -primaryJoystick.getLeftY() * MaxSpeed, 
                () -> -primaryJoystick.getLeftX() * MaxSpeed, 
                m_shooter, 
                m_hood, 
                drivetrain, 
                m_feeder, 
                m_stopper,
                m_trajectoryCalculator
            )
            // .alongWith(
            //     Commands.runEnd(
            //         () -> primaryJoystick.setRumble(RumbleType.kBothRumble, 0.5),
            //         () -> primaryJoystick.setRumble(RumbleType.kBothRumble, 0.0)
            //     )
            // )
        );

        // secondaryJoystick.rightBumper().and(m_networkTablesIO.secondaryJoystickEnabled()).whileTrue(
        //     // m_intake.runIntakeVoltageCommand(Volts.of(-11.5))
        // );

        secondaryJoystick.leftTrigger().and(m_networkTablesIO.secondaryJoystickEnabled()).whileTrue(
            m_arm.runVoltageCommand(Volts.of(-11))
        );

        secondaryJoystick.leftBumper().and(m_networkTablesIO.secondaryJoystickEnabled()).whileTrue(
            m_arm.runVoltageCommand(Volts.of(11))
        );
        
        // m_networkTablesIO.secondaryJoystickEnabled().and(RobotModeTriggers.teleop()).whileTrue(
        //     m_led.display(StatusList.DEBUG)
        // ); 

        // secondaryJoystick.b().and(m_networkTablesIO.secondaryJoystickEnabled()).whileTrue(
        //     m_feeder.feedCommand()
        // );
        // secondaryJoystick.b().and(m_networkTablesIO.secondaryJoystickEnabled()).whileTrue(
        //     m_stopper.homeCommand()
        // );

        // secondaryJoystick.povLeft().and(m_networkTablesIO.secondaryJoystickEnabled()).whileTrue(
        //     m_stopper.homeCommand()
        // );
        // secondaryJoystick.povRight().and(m_networkTablesIO.secondaryJoystickEnabled()).whileTrue(
        //     m_stopper.extendCommand()
        // );

        // secondaryJoystick.povUp().and(m_networkTablesIO.secondaryJoystickEnabled()).onTrue(
        //     m_hood.manualShiftPercentage(() -> 0.1)  
        // );
        // secondaryJoystick.povDown().and(m_networkTablesIO.secondaryJoystickEnabled()).onTrue(
        //     m_hood.manualShiftPercentage(() -> -0.1)  
        // );
        // secondaryJoystick.povUp().negate().and(secondaryJoystick.povDown().negate()).and(m_networkTablesIO.secondaryJoystickEnabled()).whileTrue(
        //     m_hood.manualShiftPercentage()
        // );

        // secondaryJoystick.a().and(m_networkTablesIO.secondaryJoystickEnabled()).whileTrue(
        //     m_shooter.runPercentageCommand(() -> secondaryJoystick.getRightTriggerAxis(), 2000, 4000)
        //     // .alongWith(
        //     //     m_hood.gotoPercentageCommand(() -> secondaryJoystick.getLeftTriggerAxis())
        //     // )
        // );

        // // Rumble controller when the hub state switches
        // m_networkTablesIO.hubActive().onChange(
        //     Commands.runOnce(
        //             () -> secondaryJoystick.setRumble(RumbleType.kBothRumble, 1.0)
        //         ).andThen(
        //             Commands.waitSeconds(0.25)
        //         ).andThen(
        //             () -> secondaryJoystick.setRumble(RumbleType.kBothRumble, 0.0)
        //         ).andThen(
        //             Commands.waitSeconds(0.1)
        //         ).andThen(
        //             () -> secondaryJoystick.setRumble(RumbleType.kBothRumble, 1.0)
        //         ).andThen(
        //             Commands.waitSeconds(0.25)
        //         ).andThen(
        //             () -> secondaryJoystick.setRumble(RumbleType.kBothRumble, 0.0)
        //         ).onlyIf(m_networkTablesIO.secondaryJoystickEnabled()).ignoringDisable(true)
        // );
    }

    public Command getAutonomousCommand() {
        // Return the auto selected by the chooser on SmartDashboard
        return autoChooser.getSelected();
        // .andThen(shootAtHub.withTimeout(5));
    }
}
