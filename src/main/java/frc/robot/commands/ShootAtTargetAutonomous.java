// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.shooter.HoodedShooter;
import frc.robot.subsystems.Stopper;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.util.TrajectoryCalculator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootAtTargetAutonomous extends Command {
  Supplier<Pose2d> targetSup;
  Pose2d target;
  Shooter shooter;
  HoodedShooter hood;
  CommandSwerveDrivetrain drivetrain;
  TrajectoryCalculator trajcalc;
  Feeder feeder;
  Stopper stopper;
  double ROTATION_DEADBAND = (10 * Math.PI)/180;
  /*
   * Shoot at a pose whilst in Autonomous Mode,
   */
  public ShootAtTargetAutonomous(Supplier<Pose2d> targetSup, Shooter shooter, HoodedShooter hood, CommandSwerveDrivetrain drivetrain, Feeder feeder, Stopper stopper, TrajectoryCalculator trajcalc) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetSup = targetSup;
    this.shooter = shooter;
    this.hood = hood;
    this.drivetrain = drivetrain;
    this.trajcalc = trajcalc;
    this.feeder = feeder;
    this.stopper = stopper;
    addRequirements(shooter, hood, feeder, stopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.reset();
    this.target = targetSup.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.target = targetSup.get();
    Object[] data = trajcalc.getHoodShooter(drivetrain.getStateCopy(), target);
    shooter.runRPM(() -> (Double) data[1]);
    hood.runClosedLoopAngle(() -> (Double) data[0]);
    // feeder.feed();
    if (DriverStation.isAutonomousEnabled()) {
      double[] p = drivetrain.calculatePointOffsetRotationRequirement((Translation2d) data[2]);
      PPHolonomicDriveController.overrideRotationFeedback(
        () -> p[0]
      );
      if (Math.abs(p[1]) < ROTATION_DEADBAND && shooter.isAtSetpoint()) {
        stopper.home();
        // feeder.run(0.9);
        feeder.feed();
      } else {
        stopper.extend();
      }
        return;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.reset();
    hood.stopreset();
    feeder.coast();
    stopper.extend();
    trajcalc.clearVirTarget();
    PPHolonomicDriveController.clearRotationFeedbackOverride();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
