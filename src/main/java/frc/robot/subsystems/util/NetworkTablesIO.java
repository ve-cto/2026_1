// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.util;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.text.DecimalFormat;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
// import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class NetworkTablesIO extends SubsystemBase {
  Pose2d drivetrainPose = new Pose2d();

  double[] networkPose = new double[] {0.0, 0.0, 0.0};
  private boolean isRedAlliance = true;
  private boolean isInOwnAllianceZone = true;
  private boolean isInRedAllianceZone = false;
  private boolean isInBlueAllianceZone = false;
  private boolean isInCenterField = false;

  private double speedLimit = 1;

  private final Pose2d blueHubPose = new Pose2d(4.65, 4, new Rotation2d());
  private final Pose2d redHubPose = new Pose2d(12, 4, new Rotation2d());

  private final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
  private final NetworkTable fmsTable = ntInst.getTable("FMSInfo");
  private final NetworkTable drivetrainTable = ntInst.getTable("Pose");
  // private final NetworkTable smartDashboardTable = ntInst.getTable("SmartDashboard");
  private final DoubleArraySubscriber poseSubscriber = drivetrainTable.getDoubleArrayTopic("robotPose").subscribe(new double[] {0.0, 0.0, 0.0});
  private final BooleanSubscriber allianceSubscriber = fmsTable.getBooleanTopic("IsRedAlliance").subscribe(isRedAlliance);
  private final DoubleArrayPublisher virPublisher = drivetrainTable.getDoubleArrayTopic("virPose").publish();

  private final DecimalFormat oneDP = new DecimalFormat("#.#");
  // private final DecimalFormat twoDP = new DecimalFormat("#.##");

  private final Rectangle2d blueAllianceZoneRect = new Rectangle2d(new Pose2d(new Translation2d(2, 4), new Rotation2d()), 4, 8);
  private final Rectangle2d redAllianceZoneRect = new Rectangle2d(new Pose2d(new Translation2d(14.5, 4), new Rotation2d()), 4, 8);
  private final Rectangle2d centerFieldZoneRect = new Rectangle2d(new Pose2d(new Translation2d(8.25, 4), new Rotation2d()), 8.5, 8);

  // private final Alert AlertDebugModeEnabled = new Alert("Debug Mode is enabled, some functions may be inoperable or inaccessible during this time until it is disabled. Ask the lead programmer or technician for assistance in using Debug Mode.", AlertType.kWarning);
  private final SendableChooser<Boolean> debugChooser = new SendableChooser<>();

  private CommandXboxController primaryJoystick;
  private CommandXboxController secondaryJoystick;
  private boolean secondaryJoystickEnabled = false;
  private final Alert alertPrimaryJoystickUnplugged = new Alert("Primary controller is disconnected from the DriverStation.", AlertType.kError);
  private final Alert alertSecondaryJoystickUnplugged = new Alert("Secondary controller is disconnected from the DriverStation.", AlertType.kWarning);

  /** Creates a new NetworkTablesIO. */
  public NetworkTablesIO(CommandXboxController primaryJoystick, CommandXboxController secondaryJoystick) {
    this.primaryJoystick = primaryJoystick;
    this.secondaryJoystick = secondaryJoystick;
    SmartDashboard.putNumber("Swerve Speed Limit", this.speedLimit);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.networkPose = poseSubscriber.get();
    Translation2d translation = new Translation2d(networkPose[0], networkPose[1]);
    Rotation2d rotation = new Rotation2d(networkPose[2] * (Math.PI / 180));
    this.drivetrainPose = new Pose2d(translation, rotation);
    this.isRedAlliance = allianceSubscriber.get();
    
    // If the match time is -1, IE, it doesn't exist or the robot is not in a match, replace with 0.
    SmartDashboard.putNumber("Match/Match Time", DriverStation.getMatchTime() == -1 ? 0 : Double.valueOf(oneDP.format(DriverStation.getMatchTime())));
    SmartDashboard.putNumber("Match/Shift Remaining Time", DriverStation.getMatchTime() == -1 ? 0 : Double.valueOf(oneDP.format(this.getShiftRemainingTime())));

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

    // 
    alertPrimaryJoystickUnplugged.set(primaryJoystick.getHID().getAxisCount() == 6 || !primaryJoystick.isConnected() ? false : true);
    alertSecondaryJoystickUnplugged.set(secondaryJoystick.getHID().getAxisCount() == 6 || !secondaryJoystick.isConnected() ? false : true);

    SmartDashboard.putBoolean("Vision/isInBlueAllianceZone", isInBlueAllianceZone);
    SmartDashboard.putBoolean("Vision/isInRedAllianceZone", isInRedAllianceZone);
    SmartDashboard.putBoolean("Vision/isInOwnAllianceZone", this.isInOwnAllianceZone);
    SmartDashboard.putBoolean("Vision/isInCenterField", this.isInCenterField);
    SmartDashboard.putBoolean("Match/isHubActive", this.isHubActive());
    SmartDashboard.putBoolean("Match/Secondary Overrides Enabled", this.secondaryJoystickEnabled);
    SmartDashboard.putNumber("Match/Robot Battery Voltage", this.getRobotBatteryVoltage());
    SmartDashboard.putBoolean("Match/Is Browning Out", this.isRobotBrowningOut().getAsBoolean());
  }

  public boolean getTestModeEnabled() {
    return DriverStation.isTestEnabled();
  }

  public Command updateSpeedLimit() {
    return runOnce(() -> {
    this.speedLimit = SmartDashboard.getNumber("Swerve Speed Limit", this.speedLimit)
    ;}
    );
  }

  // public boolean getDebugModeEnabled() {
  //   return (debugChooser.getSelected() && getTestModeEnabled());
  // }

  public BooleanSupplier getDebugModeEnabled() {
    return () -> (debugChooser.getSelected() && getTestModeEnabled());
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

  public void putVirPose(double[] pose) {
    virPublisher.set(pose);
  }

  public Pose2d getOwnHubPose() {
    if (!getAlliance()) {
      // blue alliance
      return blueHubPose;
    } else {
      return redHubPose;
    }
  }

  public double getSpeedLimiter() {
    return this.speedLimit;
  }

  public double getMatchTime() {
    return DriverStation.getMatchTime();
  }

  public Constants.DS.GameState getMatchState() {
    if (!DriverStation.isDSAttached()) {
      return Constants.DS.GameState.NONE;
    }
    if (DriverStation.isAutonomousEnabled()) {
      return Constants.DS.GameState.AUTONOMOUS;
    }
    double matchTime = getMatchTime();
    if (matchTime == -1) {
      return Constants.DS.GameState.NONE;
    }
    if (matchTime > 130) {
      return Constants.DS.GameState.TRANSITION;
    } else if (matchTime > 105) {
      return Constants.DS.GameState.SHIFT1;
    } else if (matchTime > 80) {
      return Constants.DS.GameState.SHIFT2;
    } else if (matchTime > 55) {
      return Constants.DS.GameState.SHIFT3;
    } else if (matchTime > 30) {
      return Constants.DS.GameState.SHIFT4;
    } else {
      return Constants.DS.GameState.ENDGAME;
    }
  }

  public double getRobotBatteryVoltage() {
    return RobotController.getBatteryVoltage();
  }

  public Trigger isRobotBrowningOut() {
    return new Trigger(() -> RobotController.isBrownedOut());
  }

  public double getShiftRemainingTime() {
    Constants.DS.GameState state = getMatchState();
    SmartDashboard.putString("Match/State", state.toString());
    switch (state) {
      case AUTONOMOUS:
        return getMatchTime();
      case TRANSITION:
        return (getMatchTime() - 130);
      case SHIFT1:
        return (getMatchTime() - 105);
      case SHIFT2:
        return (getMatchTime() - 80);
      case SHIFT3:
        return (getMatchTime() - 55);
      case SHIFT4:
        return (getMatchTime() - 30);
      case ENDGAME:
        return getMatchTime();
      default:
        break;
    }
    return 0.0;
  }
  
  public boolean isHubActive() {
    // Copypasted from the example code @ https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return false;
    }

    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        return true;
      }
    }

    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  public Trigger radioReady() {
    return new Trigger(() -> {
        try {
            NetworkInterface eth0 = NetworkInterface.getByName("eth0");
            return eth0 != null && eth0.isUp();
        } catch (SocketException e) {
            return false;
        }
      }
    );
  };
  
  public Trigger hubActive() {
    return new Trigger(() -> isHubActive());
  }

  public Trigger DSAttached() {
    return new Trigger(() -> DriverStation.isDSAttached());
  }

  public void setSecondaryJoystickEnabled(boolean t) {
    this.secondaryJoystickEnabled = t;
  }

  public Command setSecondaryJoystickEnabled(BooleanSupplier t) {
    return this.runOnce(() -> this.setSecondaryJoystickEnabled(t.getAsBoolean()));
  }

  public Trigger secondaryJoystickEnabled() {
    return new Trigger(() -> this.secondaryJoystickEnabled);
  }
}

