package frc.robot.subsystems.util;

import java.util.ArrayList;
import java.util.Collection;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.NetworkTablesIO;


public class TrajectoryCalculator extends SubsystemBase {
    private final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    private NetworkTablesIO networkTablesIO;
    private final NetworkTable ntTrajCalcTable = ntInst.getTable("TrajectoryCalculator");
    
    private final Pose2d blueHubPose = new Pose2d(4.65, 4, new Rotation2d());
    private final Pose2d redHubPose = new Pose2d(12, 4, new Rotation2d());

    private final Pose2d blueFuelHigh = new Pose2d(new Translation2d(1, 6), new Rotation2d());
    private final Pose2d blueFuelLow = new Pose2d(new Translation2d(1, 2), new Rotation2d());
    private final Pose2d redFuelHigh = new Pose2d(new Translation2d(15, 6), new Rotation2d());
    private final Pose2d redFuelLow = new Pose2d(new Translation2d(15, 2), new Rotation2d());

    private Collection<Pose2d> redPoses = new ArrayList<>(); 
    private Collection<Pose2d> bluePoses = new ArrayList<>(); 

    private double hubHeightMeters = 1.8288;
    private double hubHeightInches = 72;

    private SwerveDriveState driveState = new SwerveDriveState();

    private boolean isRedAlliance = true;

    private double shooterSpeedPairs[][] = {
        {0.9, 2700},
        {1.9, 2800},
        {2.9, 3300},
        {3.9, 3690},
        {5.5, 3995},
        {6.5, 4200}
    };

    private double hoodAnglePairsDegrees[][] = {
        {0.9, 70},
        {1.9, 65},
        {2.9, 62},
        {3.9, 45},
        {5.5, 47},
        {6.5, 45},
    };

    private double ballAirtimePairs[][] = { // seconds
        {0.9, 0.89},
        {1.9, 1},
        {2.9, 1.2},
        {3.9, 1.3},
        {5.5, 1.4},
        {6.5, 1.5},
    };

    private InterpolatingDoubleTreeMap shooterSpeedTreeMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap hoodAngleTreeMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap ballAirtimeTreeMap = new InterpolatingDoubleTreeMap();

    public TrajectoryCalculator(NetworkTablesIO m_networkTablesIO) {
        this.networkTablesIO = m_networkTablesIO;
        updateAlliance();
        for (int i = 0; i < shooterSpeedPairs.length; i++) {
            shooterSpeedTreeMap.put(shooterSpeedPairs[i][0], shooterSpeedPairs[i][1]);
        }

        for (int i = 0; i < hoodAnglePairsDegrees.length; i++) {
            hoodAngleTreeMap.put(hoodAnglePairsDegrees[i][0], hoodAnglePairsDegrees[i][1]);
        }

        for (int i = 0; i < ballAirtimePairs.length; i++) {
            ballAirtimeTreeMap.put(ballAirtimePairs[i][0], ballAirtimePairs[i][1]);
        }

        this.redPoses.add(redFuelHigh);
        this.redPoses.add(redFuelLow);
        this.bluePoses.add(blueFuelHigh);
        this.bluePoses.add(blueFuelLow);
    }

    @Override
    public void periodic() {}

    public Supplier<Pose2d> getClosestAllianceFuel(Supplier<SwerveDriveState> state) {
        return () -> {
            Pose2d nearest = this.isRedAlliance ? state.get().Pose.nearest(redPoses) : state.get().Pose.nearest(bluePoses);
            return nearest;
        };
    }
    
    public Translation2d getVirTarget(SwerveDriveState state, Pose2d target) {
        Pose2d pose = state.Pose;
        // double tarX = target.getX() - pose.getX();
        // double tarY = target.getY() - pose.getY();
        // Translation2d tarPos = new Translation2d(tarX, tarY);
        double realDistanceLength = target.getTranslation().minus(pose.getTranslation()).getNorm();
        double robotRotation = pose.getRotation().getRadians();
        double velXField = Math.cos(robotRotation) * state.Speeds.vxMetersPerSecond - Math.sin(robotRotation) * state.Speeds.vyMetersPerSecond;

        double velYField = Math.sin(robotRotation) * state.Speeds.vxMetersPerSecond + Math.cos(robotRotation) * state.Speeds.vyMetersPerSecond;
        velYField = -velYField;
        velXField = -velXField;
        // Translation2d robotVel = new Translation2d(this.robotVelX, this.robotVelY).times(1);
        Translation2d robotVel = new Translation2d(velXField, velYField);
        double airtime = ballAirtimeTreeMap.get(realDistanceLength);
        // double dist = tarPos.getNorm();
        // double idealSpeed = shooterSpeedTreeMap.get(dist); // This ideal speed needs to be the horizontal component of the balls travel. It gets hard because it obviously changes based on hood angle.
        // Translation2d tarVect = target.getTranslation().div(dist).times(idealSpeed);
        

        // Translation2d shotVec = tarVect.minus(robotVel);

        // double shotDist = shotVec.getNorm();
        // double shotAngle = shotVec.getAngle().getRadians();
        
        Translation2d targetCompensationOffset = robotVel.times(airtime);
        Translation2d compensatedTargetLocation = target.getTranslation().plus(targetCompensationOffset);
        return (realDistanceLength > 1) ? compensatedTargetLocation : target.getTranslation();
    }
   
    // public double[] getDistanceAngle(SwerveDriveState state, Pose2d target) {
    //     Translation2d compensatedTarget = getVirTarget(state, target);
    //     Pose2d pose = networkTablesIO.getNetworkPose();
    //     double compensatedDistanceLength = compensatedTarget.minus(pose.getTranslation()).getNorm();
        
    //     double robotAngleToTargetRads = Math.atan2(compensatedTarget.getY() - pose.getY(), compensatedTarget.getX() - pose.getX());
    //     // SmartDashboard.putNumberArray("Pose/VirtualTarget", new double[] {compensatedTarget.getX(), compensatedTarget.getY(), 0.0});
    //     networkTablesIO.putVirPose(new double[] {compensatedTarget.getX() - pose.getX(), compensatedTarget.getY() - pose.getY(), 0.0});
    //     return new double[] {compensatedDistanceLength, robotAngleToTargetRads};
    // }

    // public double[] getHoodShooterAngle(SwerveDriveState state, Pose2d target) {
    //     double[] distang = getDistanceAngle(state, target);
    //     double shooterspeed = shooterSpeedTreeMap.get(distang[0]);
    //     double hoodangl = hoodAngleTreeMap.get(distang[0]);
    //     return new double[] {shooterspeed, hoodangl, distang[1]};
    // }

    public Object[] getHoodShooter(SwerveDriveState state, Pose2d target) {
        Translation2d compensatedTargetLocation = getVirTarget(state, target);
        double compensatedDistanceLength = compensatedTargetLocation.minus(state.Pose.getTranslation()).getNorm();
        networkTablesIO.putVirPose(new double[] {compensatedTargetLocation.getX(), compensatedTargetLocation.getY(), 0.0});
        return new Object[] {hoodAngleTreeMap.get(compensatedDistanceLength), shooterSpeedTreeMap.get(compensatedDistanceLength), compensatedTargetLocation};
    }

    public void clearVirTarget() {
        networkTablesIO.putVirPose(new double[] {0, 0, 0.0});
    }

    public void updateAlliance() {
        this.isRedAlliance = networkTablesIO.getAlliance();
    }
    public Command updateAllianceCommand() {
        return runOnce(() -> this.updateAlliance());
    }

    public DoubleSupplier getRequiredShooterSpeedHub() {
        return () -> shooterSpeedTreeMap.get(getHubDistance());
    }

    public DoubleSupplier getRequiredHoodAngleHub() {
        return () -> hoodAngleTreeMap.get(getHubDistance());
    }

    public double getRequiredShooterSpeedPose(Pose2d pose) {
        return shooterSpeedTreeMap.get(this.getRobotDistanceToPose(pose));
    }

    public double getRequiredShooterSpeed(double distance) {
        return shooterSpeedTreeMap.get(distance);
    }

    public double getRequiredHoodAngle(double distance) {
        return hoodAngleTreeMap.get(distance);
    }

    public double getRequiredHoodAnglePose(Pose2d pose) {
        return hoodAngleTreeMap.get(this.getRobotDistanceToPose(pose));
    }

    public double getRobotDistanceToPose(Pose2d pose) {
        return pose.getTranslation().getDistance(networkTablesIO.getNetworkPose().getTranslation());
    }

    // public double getRobotAngleToPoseDegrees(Pose2d pose) {
    //     return pose.getTranslation().getAngle().relativeTo(networkTablesIO.getNetworkPose().getTranslation().getAngle()).getDegrees();
    // }

    // public double getRobotAngleToPoseRadians(Pose2d pose) {
    //     return pose.getTranslation().getAngle().relativeTo(networkTablesIO.getNetworkPose().getTranslation().getAngle()).getRadians();
    // }

    public double getHubDistance() {
        return networkTablesIO.getNetworkPose().getTranslation().getDistance(isRedAlliance ? redHubPose.getTranslation() : blueHubPose.getTranslation());
    }
}
