// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NetworkTablesIO extends SubsystemBase {
  Pose2d drivetrainPose = new Pose2d();

  double[] networkPose = new double[] {0.0, 0.0, 0.0};
  private boolean isRedAlliance = true;
  private boolean isInOwnAllianceZone = true;
  private boolean isInRedAllianceZone = false;
  private boolean isInBlueAllianceZone = false;
  private boolean isInCenterField = false;

  private final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
  private final NetworkTable fmsTable = ntInst.getTable("FMSInfo");
  private final NetworkTable drivetrainTable = ntInst.getTable("Pose");
  private final DoubleArraySubscriber poseSubscriber = drivetrainTable.getDoubleArrayTopic("robotPose").subscribe(new double[] {0.0, 0.0, 0.0});
  private final BooleanSubscriber allianceSubscriber = fmsTable.getBooleanTopic("IsRedAlliance").subscribe(isRedAlliance);

  private final DecimalFormat oneDP = new DecimalFormat("#.#");
  private final DecimalFormat twoDP = new DecimalFormat("#.##");

  private final Rectangle2d blueAllianceZoneRect = new Rectangle2d(new Pose2d(new Translation2d(2, 4), new Rotation2d()), 4, 8);
  private final Rectangle2d redAllianceZoneRect = new Rectangle2d(new Pose2d(new Translation2d(14.5, 4), new Rotation2d()), 4, 8);
  private final Rectangle2d centerFieldZoneRect = new Rectangle2d(new Pose2d(new Translation2d(8.25, 4), new Rotation2d()), 8.5, 8);
  /** Creates a new NetworkTablesIO. */
  public NetworkTablesIO() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.networkPose = poseSubscriber.get();
    Translation2d translation = new Translation2d(networkPose[0], networkPose[1]);
    Rotation2d rotation = new Rotation2d(networkPose[2] * (Math.PI / 180));
    this.drivetrainPose = new Pose2d(translation, rotation);
    this.isRedAlliance = allianceSubscriber.get();
    
    // If the match time is -1, IE, it doesn't exist, replace with 0.
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime() == -1 ? 0 : Double.valueOf(oneDP.format(DriverStation.getMatchTime())));

    if (redAllianceZoneRect.contains(getNetworkPose().getTranslation())) {
      this.isInRedAllianceZone = true;
    } else {
      this.isInRedAllianceZone = false;
    }
    
    if (blueAllianceZoneRect.contains(getNetworkPose().getTranslation())) {
      this.isInBlueAllianceZone = true;
    } else {
      this.isInBlueAllianceZone = false;
    }

    if (getAlliance() && this.isInRedAllianceZone || !getAlliance() && this.isInBlueAllianceZone) {
      this.isInOwnAllianceZone = true;
    } else {
      this.isInOwnAllianceZone = false;
    }

    if (centerFieldZoneRect.contains(getNetworkPose().getTranslation())) {
      this.isInCenterField = true;
    } else {
      this.isInCenterField = false;
    }
  
    SmartDashboard.putBoolean("isInBlueAllianceZone", isInBlueAllianceZone);
    SmartDashboard.putBoolean("isInRedAllianceZone", isInRedAllianceZone);
    SmartDashboard.putBoolean("isInOwnAllianceZone", this.isInOwnAllianceZone);
    SmartDashboard.putBoolean("isInCenterField", this.isInCenterField);
  }

  public Pose2d getNetworkPose() {
    return this.drivetrainPose;
  }

  public boolean isInOwnAllianceZone() {
    return this.isInOwnAllianceZone;
  }

  public boolean isInRedAllianceZone() {
    return this.isInRedAllianceZone;
  }

  public boolean isInBlueAllianceZone() {
    return this.isInBlueAllianceZone;
  }

  public boolean isInCenterFiel() {
    return this.isInCenterField;
  }

  public double[] getNetworkPoseArray() {
    return this.networkPose;
  }

  public boolean getAlliance() {
    return this.isRedAlliance;
  }

  public double getMatchTime() {
    return 0.0;
  }
}
