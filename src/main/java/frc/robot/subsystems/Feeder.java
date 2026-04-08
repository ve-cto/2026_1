// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private final WPI_VictorSPX m_feeder;
  
  /** Creates a new Feeder. */
  public Feeder() {
    m_feeder = new WPI_VictorSPX(Constants.Hardware.kLoaderId);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(double speed) {
    m_feeder.set(speed);
  }

  public void stop() {
    m_feeder.stopMotor();
  }

  public void coast() {
    m_feeder.set(0.0);
  }

  public Command feedCommand(double speed) {
    return startEnd(() -> this.run(speed), () -> this.coast());
  }

  public Command feedCommand() {
    return startEnd(() -> this.run(Constants.Loader.kLoadSpeed), () -> this.coast());
  }
}