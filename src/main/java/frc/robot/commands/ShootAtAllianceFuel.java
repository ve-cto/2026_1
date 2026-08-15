// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.shooter.HoodedShooter;
import frc.robot.subsystems.Stopper;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.util.TrajectoryCalculator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootAtAllianceFuel extends Command {
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
  public ShootAtAllianceFuel(Supplier<Pose2d> targetSup, DoubleSupplier velx, DoubleSupplier vely, Shooter shooter, HoodedShooter hood, CommandSwerveDrivetrain drivetrain, Feeder feeder, Stopper stopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetSup = targetSup;
    this.shooter = shooter;
    this.hood = hood;
    this.drivetrain = drivetrain;
    // this.trajcalc = trajcalc;
    this.feeder = feeder;
    this.stopper = stopper;
    this.velx = velx;
    this.vely = vely;
    addRequirements(shooter, hood, drivetrain, feeder, stopper);
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
    shooter.runRPM(() -> 3000);
    hood.runClosedLoopAngle(() -> 53);
    // Object[] data = trajcalc.getHoodShooter(drivetrain.getStateCopy(), target);
    // shooter.runRPM(() -> (Double) data[1]);
    // hood.runClosedLoopAngle(() -> (Double) data[0]);
    // feeder.feed();
    if (Math.abs(drivetrain.pointToTranslation2d(this.target.getTranslation(), velx.getAsDouble(), vely.getAsDouble())) < ROTATION_DEADBAND && shooter.isAtSetpoint()) {
      stopper.home();
      // feeder.run(0.9);
      feeder.feed();
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
    // trajcalc.clearVirTarget();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
