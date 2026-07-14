package frc.robot.subsystems.util;

import java.util.function.DoubleSupplier;

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
import frc.robot.subsystems.NetworkTablesIO;


public class TrajectoryCalculator extends SubsystemBase {
    private final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    private NetworkTablesIO networkTablesIO;
    private final NetworkTable ntTrajCalcTable = ntInst.getTable("TrajectoryCalculator");
    
    private final Pose2d blueHubPose = new Pose2d(4.65, 4, new Rotation2d());
    private final Pose2d redHubPose = new Pose2d(12, 4, new Rotation2d());
    private double hubHeightMeters = 1.8288;
    private double hubHeightInches = 72;
    private double robotVelX = 0;
    private double robotVelY = 0;

    private SwerveDriveState driveState = new SwerveDriveState();

    private boolean isRedAlliance = true;

    private double shooterSpeedPairs[][] = {
        {1.5, 3200},
        {2.5, 3500},
        {3.5, 3620},
        {4.5, 3690},
        {5.5, 3995},
        {6.5, 4200}
    };

    private double hoodAnglePairsDegrees[][] = {
        {1, 67},
        {2, 60},
        {3.5, 55},
        {4.5, 459},
        {5.5, 47},
        {6.5, 45},
    };

    private double ballAirtimePairs[][] = {
        {1, 0},
        {2, 0},
        {3.5, 0},
        {4.5, 0},
        {5.5, 0},
        {6.5, 0},
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
    }

    @Override
    public void periodic() {
        
    }

    // public Command updateState(SwerveDriveState state) {
    //     return new InstantCommand(() -> this.driveState = state, this);
    // }

    public void updateVel(double velx, double velY) {

    }
    public Command updateVelocities(DoubleSupplier velX, DoubleSupplier velY) {
        return run(() -> this.updateVel(velX.getAsDouble(), velY.getAsDouble()));
    }

    public DoubleSupplier getRequiredRobotAngleSOTM(SwerveDriveState state) {
        return () -> this.getRequiredSpeedsSOTM(state)[1];
    }

    public DoubleSupplier getRequiredHoodAngleSOTM(SwerveDriveState state) {
        return () -> this.getRequiredHoodAngle(this.getRequiredSpeedsSOTM(state)[2]);
    }

    public DoubleSupplier getRequiredShooterSpeedSOTM(SwerveDriveState state) {
        return () -> this.getRequiredShooterSpeed(this.getRequiredSpeedsSOTM(state)[0]);
    }

    // // https://blog.eeshwark.com/robotblog/shooting-on-the-fly
    // // https://blog.eeshwark.com/robotblog/shooting-on-the-fly-pt2
    // Note to me: i started looking at this and it is literally just vector math. All those questions where it's like a boat can go x speed, but there's a current going x speed in alpha direction. It's pretty much just that. It's HARD though. like, really hard.
    public double[] getRequiredSpeedsSOTM(SwerveDriveState state) {
        Translation2d robotTranslation2d = networkTablesIO.getNetworkPose().getTranslation();
        double robotHeadingRadians = networkTablesIO.getNetworkPose().getRotation().getRadians();
        Translation2d hubTranslation2d = isRedAlliance ? redHubPose.getTranslation() : blueHubPose.getTranslation();
        
        double robotVelX = state.Speeds.vxMetersPerSecond;
        double robotVelY = state.Speeds.vyMetersPerSecond;
        // convert velocity into robot centric
        double velXField = (Math.cos(robotHeadingRadians) * robotVelX) - (Math.sin(robotHeadingRadians) * robotVelY);
        double velYField = (Math.sin(robotHeadingRadians) * robotVelX) + robotVelY * Math.cos(robotHeadingRadians);
        ntTrajCalcTable.putValue("velXField", NetworkTableValue.makeDouble(velXField));
        ntTrajCalcTable.putValue("velYField", NetworkTableValue.makeDouble(velYField));


        double targetXRelative = hubTranslation2d.getX() - robotTranslation2d.getX();
        double targetYRelative = hubTranslation2d.getY() - robotTranslation2d.getY();
        ntTrajCalcTable.putValue("targetXRelative", NetworkTableValue.makeDouble(targetXRelative));
        ntTrajCalcTable.putValue("targetYRelative", NetworkTableValue.makeDouble(targetYRelative));
        Translation2d targetPositionRelative = new Translation2d(targetXRelative, targetYRelative);
        
        Translation2d shotVectorRelative = new Translation2d(targetXRelative - velXField, targetYRelative - velYField);

        if (!this.isRedAlliance) { // if we're on blue we need to invert else the robot is backwards for some reason
            shotVectorRelative = shotVectorRelative.times(-1);
        }

        double distanceToShot = shotVectorRelative.getNorm();
        ntTrajCalcTable.putValue("distanceToShot", NetworkTableValue.makeDouble(distanceToShot));

        double angleToShotRadians = Math.atan2(shotVectorRelative.getY(), shotVectorRelative.getX());
        ntTrajCalcTable.putValue("angleToShotRadians", NetworkTableValue.makeDouble(angleToShotRadians));
        angleToShotRadians += Math.PI;
        
        double quadrant = 0;
        if (targetXRelative <= 0 && targetYRelative <= 0) {
            // goal is down left, robot up right
            quadrant = 1;
        } else if (targetXRelative <= 0 && targetYRelative >= 0) {
            // goal is up left, robot down right
            quadrant = 4;
        } else if (targetXRelative >= 0 && targetYRelative <= 0) {
            // goal is down right, robot up left
            quadrant = 2;
        } else if (targetXRelative >= 0 && targetYRelative >= 0) {
            // goal is up right, robot down left
            quadrant = 3;
        }

        ntTrajCalcTable.putValue("quadrant", NetworkTableValue.makeDouble(quadrant));

        // catch broken
        // if (quadrant == 0) {
        //     quadrant = 1;
        // }

        // old manual quadrant assignment logic, preserved for achive.
        // if (quadrant == 1) {
        //     // angleToShotRadians = -angleToShotRadians; 
        // }
        // if (quadrant == 2) {
        //     // angleToShotRadians = angleToShotRadians; 
        // }
        // if (quadrant == 3) {
        //     // angleToShotRadians = angleToShotRadians;
        // }
        // if (quadrant == 4) {
        //     // angleToShotRadians = Math.PI/2 - angleToShotRadians; 
        // }
        // ntTrajCalcTable.putValue("angleToShotRadians 2", NetworkTableValue.makeDouble(angleToShotRadians));
        
        // double angleToShotDegrees = angleToShotRadians * (180/Math.PI);

        // if (angleToShotRadians >= Math.PI) {
        //     // put the angle into -180 to 180 space
        //     angleToShotRadians = angleToShotRadians - 2*Math.PI;
        // }
        ntTrajCalcTable.putValue("angleToShotRadians", NetworkTableValue.makeDouble(angleToShotRadians));

        return new double[] {distanceToShot, angleToShotRadians, distanceToShot};
        

        // if (quadrant ==)

        // since we are working in robot relative (that's why we subtracted the robot position earlier), the robot pose becomes the origin
        // double distance = targetPositionRelative.getNorm();

        // // split the speeds into components. this really needs to be as instantaneous velocity OUT of the shooter though.
        // double horizontalSpeedIdeal = getRequiredShooterSpeed(new Pose2d(targetPositionRelative, new Rotation2d())) * Math.cos(getRequiredHoodAngle(new Pose2d(targetPositionRelative, new Rotation2d())));
        // double verticalSpeedIdeal = getRequiredShooterSpeed(new Pose2d(targetPositionRelative, new Rotation2d())) * Math.sin(getRequiredHoodAngle(new Pose2d(targetPositionRelative, new Rotation2d())));

        // Translation2d targetVector = targetPositionRelative.div(distance).times(horizontalSpeedIdeal);

        // Translation2d robotVelocity = new Translation2d(robotVelX, robotVelY);

        // Translation2d shotVector = targetVector.minus(robotVelocity);

        // double requiredRobotAngle = shotVector.getAngle().getDegrees();
        // double requiredSpeed = shotVector.getNorm();

        // return null;
    }
    
    public double[] getRequiredSpeedsSOTM(SwerveDriveState state, Pose2d targetPose) {
        Translation2d robotTranslation2d = networkTablesIO.getNetworkPose().getTranslation();
        double robotHeadingRadians = networkTablesIO.getNetworkPose().getRotation().getRadians();
        Translation2d targetTranslation2d = targetPose.getTranslation();
        
        double robotVelX = state.Speeds.vxMetersPerSecond;
        double robotVelY = state.Speeds.vyMetersPerSecond;
        // convert velocity into robot centric
        double velXField = (Math.cos(robotHeadingRadians) * robotVelX) - (Math.sin(robotHeadingRadians) * robotVelY);
        double velYField = (Math.sin(robotHeadingRadians) * robotVelX) + robotVelY * Math.cos(robotHeadingRadians);
        ntTrajCalcTable.putValue("velXField", NetworkTableValue.makeDouble(velXField));
        ntTrajCalcTable.putValue("velYField", NetworkTableValue.makeDouble(velYField));


        double targetXRelative = targetTranslation2d.getX() - robotTranslation2d.getX();
        double targetYRelative = targetTranslation2d.getY() - robotTranslation2d.getY();
        ntTrajCalcTable.putValue("targetXRelative", NetworkTableValue.makeDouble(targetXRelative));
        ntTrajCalcTable.putValue("targetYRelative", NetworkTableValue.makeDouble(targetYRelative));
        Translation2d targetPositionRelative = new Translation2d(targetXRelative, targetYRelative);
        
        Translation2d shotVectorRelative = new Translation2d(targetXRelative - velXField, targetYRelative - velYField);

        if (!this.isRedAlliance) { // if we're on blue we need to invert else the robot is backwards for some reason
            shotVectorRelative = shotVectorRelative.times(-1);
        }

        double distanceToShot = shotVectorRelative.getNorm();
        ntTrajCalcTable.putValue("distanceToShot", NetworkTableValue.makeDouble(distanceToShot));

        double angleToShotRadians = Math.atan2(shotVectorRelative.getY(), shotVectorRelative.getX());
        ntTrajCalcTable.putValue("angleToShotRadians", NetworkTableValue.makeDouble(angleToShotRadians));
        angleToShotRadians += Math.PI;
        
        double quadrant = 0;
        if (targetXRelative <= 0 && targetYRelative <= 0) {
            // goal is down left, robot up right
            quadrant = 1;
        } else if (targetXRelative <= 0 && targetYRelative >= 0) {
            // goal is up left, robot down right
            quadrant = 4;
        } else if (targetXRelative >= 0 && targetYRelative <= 0) {
            // goal is down right, robot up left
            quadrant = 2;
        } else if (targetXRelative >= 0 && targetYRelative >= 0) {
            // goal is up right, robot down left
            quadrant = 3;
        }

        ntTrajCalcTable.putValue("quadrant", NetworkTableValue.makeDouble(quadrant));

        // catch broken
        // if (quadrant == 0) {
        //     quadrant = 1;
        // }

        // old manual quadrant assignment logic, preserved for achive.
        // if (quadrant == 1) {
        //     // angleToShotRadians = -angleToShotRadians; 
        // }
        // if (quadrant == 2) {
        //     // angleToShotRadians = angleToShotRadians; 
        // }
        // if (quadrant == 3) {
        //     // angleToShotRadians = angleToShotRadians;
        // }
        // if (quadrant == 4) {
        //     // angleToShotRadians = Math.PI/2 - angleToShotRadians; 
        // }
        // ntTrajCalcTable.putValue("angleToShotRadians 2", NetworkTableValue.makeDouble(angleToShotRadians));
        
        // double angleToShotDegrees = angleToShotRadians * (180/Math.PI);

        // if (angleToShotRadians >= Math.PI) {
        //     // put the angle into -180 to 180 space
        //     angleToShotRadians = angleToShotRadians - 2*Math.PI;
        // }
        ntTrajCalcTable.putValue("angleToShotRadians", NetworkTableValue.makeDouble(angleToShotRadians));

        return new double[] {distanceToShot, angleToShotRadians, distanceToShot};
        

        // if (quadrant ==)

        // since we are working in robot relative (that's why we subtracted the robot position earlier), the robot pose becomes the origin
        // double distance = targetPositionRelative.getNorm();

        // // split the speeds into components. this really needs to be as instantaneous velocity OUT of the shooter though.
        // double horizontalSpeedIdeal = getRequiredShooterSpeed(new Pose2d(targetPositionRelative, new Rotation2d())) * Math.cos(getRequiredHoodAngle(new Pose2d(targetPositionRelative, new Rotation2d())));
        // double verticalSpeedIdeal = getRequiredShooterSpeed(new Pose2d(targetPositionRelative, new Rotation2d())) * Math.sin(getRequiredHoodAngle(new Pose2d(targetPositionRelative, new Rotation2d())));

        // Translation2d targetVector = targetPositionRelative.div(distance).times(horizontalSpeedIdeal);

        // Translation2d robotVelocity = new Translation2d(robotVelX, robotVelY);

        // Translation2d shotVector = targetVector.minus(robotVelocity);

        // double requiredRobotAngle = shotVector.getAngle().getDegrees();
        // double requiredSpeed = shotVector.getNorm();

        // return null;
    }
    

    // public DoubleSupplier SOTMgetRequiredRobotRotationHub(DoubleSupplier robotVelX, DoubleSupplier robotVelY, DoubleSupplier heading) {
    //     return () -> {
    //     Translation2d robotTranslation2d = networkTablesIO.getNetworkPose().getTranslation();
    //     Translation2d hubTranslation2d = isRedAlliance ? redHubPose.getTranslation() : blueHubPose.getTranslation();
        
    //     double targetX = hubTranslation2d.getX() - robotTranslation2d.getX();
    //     SmartDashboard.putNumber("shotTargetX", targetX);
    //     double targetY = hubTranslation2d.getY() - robotTranslation2d.getY();
    //     Translation2d targetPosition = new Translation2d(targetX, targetY);
        
    //     // since we are working in robot relative (that's why we subtracted the robot position earlier), the robot pose becomes the origin
    //     double distance = targetPosition.getNorm();
    //     SmartDashboard.putNumber("shotDistance", distance);
    //     // split the speeds into components. this really needs to be as instantaneous velocity OUT of the shooter though. presume shooting at a 45 degree angle 
    //     // divide it by 875 to get the approx exit velocity (this needs to be tuned) TODO:
    //     double horizontalSpeedIdeal = (getRequiredShooterSpeed(new Pose2d(targetPosition, new Rotation2d())) / 875) * Math.cos(Math.PI/3);
    //     Translation2d targetVector = targetPosition.div(distance).times(horizontalSpeedIdeal);

    //     Translation2d robotVelocity = new Translation2d(robotVelX.getAsDouble(), robotVelY.getAsDouble());
    //     // Rotation2d robotHeading = new Rotation2d(heading.getAsDouble());
    //     // Translation2d robotVelRobotFrame =
    //     //     new Translation2d(robotVelX.getAsDouble(), robotVelY.getAsDouble())
    //     //     .rotateBy(robotHeading.unaryMinus());

    //     Translation2d shotVector = targetVector.minus(robotVelocity);
        
    //     // Translation2d robotVelocity = new Translation2d(robotVelX.getAsDouble(), robotVelY.getAsDouble());
    //     // Translation2d shotVector = targetVector.minus(robotVelocity);

    //     // Convert to robot-relative frame
    //     Rotation2d robotHeading = new Rotation2d(heading.getAsDouble());
    //     Translation2d shotVectorRobotFrame = shotVector.rotateBy(robotHeading.unaryMinus());

    //     SmartDashboard.putNumber("shotVecAngleRobotFrame", shotVectorRobotFrame.getAngle().getDegrees());

    //     SmartDashboard.putNumber("shotVecAngle", shotVector.getAngle().getDegrees());

    //     SmartDashboard.putNumber("shotVecX", shotVector.getX());
    //     SmartDashboard.putNumber("shotVecY", shotVector.getY());

    //     SmartDashboard.putNumber("mag_targetVector", targetVector.getNorm());
    //     SmartDashboard.putNumber("mag_robotVelocity", robotVelocity.getNorm());
    //     SmartDashboard.putNumber("mag_shotVector", shotVector.getNorm());

    //     return shotVectorRobotFrame.getAngle().getDegrees();

    //     };
    // }

    public Translation2d getVirTarget(SwerveDriveState state, Pose2d target) {
        Pose2d pose = networkTablesIO.getNetworkPose();
        double tarX = target.getX() - pose.getX();
        double tarY = target.getY() - pose.getY();
        Translation2d tarPos = new Translation2d(tarX, tarY);
    
        double dist = tarPos.getNorm();
        double idealSpeed = shooterSpeedTreeMap.get(dist); // This ideal speed needs to be the horizontal component of the balls travel. It gets hard because it obviously changes based on hood angle.
        Translation2d tarVect = target.getTranslation().div(dist).times(idealSpeed);
    
        Translation2d robotVel = new Translation2d(this.robotVelX, this.robotVelY);

        Translation2d shotVec = tarVect.minus(robotVel);

        double shotDist = shotVec.getNorm();
        double shotAngle = shotVec.getAngle().getRadians();
        
        Translation2d targetCompensationOffset = robotVel.times(ballAirtimeTreeMap.get(dist));
        Translation2d compensatedTargetLocation = target.getTranslation().plus(targetCompensationOffset);
        return (dist > 1) ? compensatedTargetLocation : target.getTranslation();
    }
   
    public double[] getDistanceAngle(Pose2d target) {
        Translation2d compensatedTarget = getVirTarget(null, target);
        Pose2d pose = networkTablesIO.getNetworkPose();
        double compensatedDistanceLength = compensatedTarget.minus(pose.getTranslation()).getNorm();
        
        double robotAngleToTargetRads = Math.atan2(compensatedTarget.getY() - pose.getY(), compensatedTarget.getX() - pose.getX());

        return new double[] {compensatedDistanceLength, robotAngleToTargetRads};
    }

    public double[] getHoodShooterAngle(Pose2d target) {
        double[] distang = getDistanceAngle(target);
        double shooterspeed = shooterSpeedTreeMap.get(distang[0]);
        double hoodangl = hoodAngleTreeMap.get(distang[0]);
        return new double[] {shooterspeed, hoodangl, distang[1]};
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
