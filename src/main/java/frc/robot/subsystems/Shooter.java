// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final WPI_VictorSPX m_shooter;

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooter = new WPI_VictorSPX(Constants.Hardware.kShooterId);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(double speed) {
    m_shooter.set(speed);
  }

  public void stop() {
    m_shooter.stopMotor();
  }
}
