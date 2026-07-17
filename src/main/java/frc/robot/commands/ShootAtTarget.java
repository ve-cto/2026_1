// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.shooter.HoodedShooter;
import frc.robot.subsystems.Stopper;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.util.TrajectoryCalculator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootAtTarget extends Command {
  Supplier<Pose2d> targetSup;
  Pose2d target;
  Shooter shooter;
  HoodedShooter hood;
  CommandSwerveDrivetrain drivetrain;
  TrajectoryCalculator trajcalc;
  Feeder feeder;
  Stopper stopper;
  DoubleSupplier velx;
  DoubleSupplier vely;
  double ROTATION_DEADBAND = (10 * Math.PI)/180;
  /** Creates a new ShootAtTarget. */
  public ShootAtTarget(Supplier<Pose2d> targetSup, DoubleSupplier velx, DoubleSupplier vely, Shooter shooter, HoodedShooter hood, CommandSwerveDrivetrain drivetrain, Feeder feeder, Stopper stopper, TrajectoryCalculator trajcalc) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetSup = targetSup;
    this.shooter = shooter;
    this.hood = hood;
    this.drivetrain = drivetrain;
    this.trajcalc = trajcalc;
    this.feeder = feeder;
    this.stopper = stopper;
    this.velx = velx;
    this.vely = vely;
    addRequirements(shooter, hood, drivetrain, feeder);
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
    double[] data = trajcalc.getHoodShooterAngle(target);
    shooter.runRPM(() -> data[0]);
    // hood.runClosedLoopAngle(() -> data[1]);
    feeder.feed();
    if (Math.abs(drivetrain.pointToPose(target, velx.getAsDouble(), vely.getAsDouble())) < ROTATION_DEADBAND && shooter.isAtSetpoint()) {
      stopper.home();
      feeder.run(0.9);
    } else {
      stopper.extend();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.reset();
    hood.stopreset();
    feeder.coast();
    stopper.extend();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
