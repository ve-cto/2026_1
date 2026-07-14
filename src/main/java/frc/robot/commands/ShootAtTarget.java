// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Hood;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.shooter.HoodedShooter;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.util.TrajectoryCalculator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootAtTarget extends Command {
  Pose2d target;
  Shooter shooter;
  HoodedShooter hood;
  CommandSwerveDrivetrain drivetrain;
  TrajectoryCalculator trajcalc;
  Led led;
  /** Creates a new ShootAtTarget. */
  public ShootAtTarget(Pose2d target, Shooter shooter, HoodedShooter hood, CommandSwerveDrivetrain drivetrain, TrajectoryCalculator trajcalc, Led led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.target = target;
    this.shooter = shooter;
    this.hood = hood;
    this.drivetrain = drivetrain;
    this.trajcalc = trajcalc;
    this.led = led;
    addRequirements(shooter, hood, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] data = trajcalc.getHoodShooterAngle(target);
    shooter.runRPM(() -> data[0]);
    hood.gotoAngleCommand(() -> data[1]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
