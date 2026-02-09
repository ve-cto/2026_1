package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    public PhotonPoseEstimator poseEstimator;
    public Optional<EstimatedRobotPose> poseEstimate = Optional.empty();

    // final StructSubscriber<Pose2d> sub;

    NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    NetworkTable table = ntInst.getTable("Pose");
    DoubleArraySubscriber poseSubscriber = table.getDoubleArrayTopic("robotPose").subscribe(new double[] {0.0, 0.0, 0.0});

    public final PhotonCamera cameraAlpha = new PhotonCamera(Constants.Vision.kCameraNameAlpha);
    public final PhotonCamera cameraBeta = new PhotonCamera(Constants.Vision.kCameraNameBeta);

    public final Integer[] hubTagsRed = {8, 9, 10, 11};
    public final Integer[] hubTagsBlue = {24, 25, 26, 27};
    
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d kRobotToCamAlpha = new Transform3d(new Translation3d(0.0, 0.0, 0.5), new Rotation3d(0, 0, 0));
    public static final Transform3d kRobotToCamBeta = new Transform3d(new Translation3d(-2.0, 0.0, 3.0), new Rotation3d(0, 0.84, 0)); // -1.5708 radians = 90 degrees

    private final CommandSwerveDrivetrain m_drivetrain;

    private final VisionSystemSim visionSim = new VisionSystemSim("sim");
    private final TargetModel targetModel = TargetModel.kAprilTag36h11;

    private List<PhotonPipelineResult> resultsAlpha;
    
    // Executor to run expensive vision estimation off the main thread so commands/periodics stay fast
    private final ExecutorService visionExecutor = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "vision-thread");
        t.setDaemon(true);
        return t;
    });

    // Pending estimate produced by the background thread; applied quickly on the main thread in periodic()
    private volatile Pose2d pendingPose2d = null;
    private volatile double pendingTimestamp = 0.0;
    private volatile boolean pendingEstimateAvailable = false;

    double[] networkPose = {0.0, 0.0, 0.0};
    double[] diff = networkPose;
    Pose2d drivetrainPose = new Pose2d();

    public Vision(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        poseEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCamAlpha);
                
        // only set up sim if we're in sim, cuz like why waste resources
        if (RobotBase.isSimulation()) {
            visionSim.addAprilTags(kTagLayout);
            SimCameraProperties simCameraAlphaProp = new SimCameraProperties();
            SimCameraProperties simCameraBetaProp = new SimCameraProperties();
            
            PhotonCameraSim cameraAlphaSim = new PhotonCameraSim(cameraAlpha, simCameraAlphaProp);
            PhotonCameraSim cameraBetaSim = new PhotonCameraSim(cameraBeta, simCameraBetaProp);
            cameraAlphaSim.enableDrawWireframe(true);
            cameraBetaSim.enableDrawWireframe(true);

            visionSim.addCamera(cameraAlphaSim, kRobotToCamAlpha);
            visionSim.addCamera(cameraBetaSim, kRobotToCamBeta);

            visionSim.getDebugField();
        }
    }

    @Override
    public void periodic() {
        // Apply any background vision estimate to the drivetrain quickly. Keep this path very cheap.
        if (pendingEstimateAvailable) {
            Pose2d pose = pendingPose2d;
            double ts = pendingTimestamp;
            if (pose != null) {
                m_drivetrain.addVisionMeasurement(pose, ts);
            }
            pendingEstimateAvailable = false;
        }
    }

    @Override
    public void simulationPeriodic() {
        // Pose2d drivetrainPose = m_drivetrain.getPose();
        this.networkPose = poseSubscriber.get();
        if (networkPose[0] != diff[0] || networkPose[1] != diff[1] || networkPose[2] != diff[2]) {
            Translation2d translation = new Translation2d(networkPose[0], networkPose[1]);
            Rotation2d rotation = new Rotation2d(networkPose[2] * (Math.PI/180));

            this.drivetrainPose = new Pose2d(translation, rotation);
            visionSim.update(this.drivetrainPose);
        }
        this.diff = this.networkPose;
    }   

    public void addVisionMeasurement() {
        // only run if we aren't simulating
        if (!RobotBase.isSimulation()) {
            // Run the heavy PhotonPoseEstimator work off the main thread and publish a small, ready-to-apply
            // result (Pose2d + timestamp). The main thread will apply it during its periodic() quickly.
            final Pose2d reference = m_drivetrain.getPose();
            poseEstimator.setReferencePose(reference);

            visionExecutor.submit(() -> {
                var result = cameraAlpha.getLatestResult();
                if (result == null || !result.hasTargets()) {
                    return;
                }

                Optional<EstimatedRobotPose> estimateOpt = poseEstimator.estimateCoprocMultiTagPose(result);
                if (estimateOpt.isEmpty()) {
                    estimateOpt = poseEstimator.estimateLowestAmbiguityPose(result);
                }

                if (estimateOpt.isPresent()) {
                    EstimatedRobotPose est = estimateOpt.get();
                    // store small, simple values for the main thread to apply quickly
                    pendingPose2d = est.estimatedPose.toPose2d();
                    pendingTimestamp = result.getTimestampSeconds();
                    pendingEstimateAvailable = true;
                    poseEstimate = estimateOpt;
                }
            });
        }
    }

    // /*
    // * Return true if specifieed target is currently visible
    // */    
    public boolean getTargetVisible(int targetID) {
        var result = this.cameraAlpha.getLatestResult();
        if (result == null || !result.hasTargets()) {
            return false;
        }
        for (var target : result.getTargets()) {
            if (target.getFiducialId() == targetID) {
                return true;
            }
        }
        return false;
    }

    public Pose3d getTargetPoseRelative(int targetID) {  
        var result = this.cameraAlpha.getLatestResult();
        if (result == null || !result.hasTargets()) {
            return null;
        }
        for (var target : result.getTargets()) {
            if (target.getFiducialId() == targetID) {
                // tag found
                var targetBest = target.getBestCameraToTarget();
                return new Pose3d(targetBest.getTranslation(), targetBest.getRotation());
            }
        }
        // if the target isn't visible, return null.
        return null;
    }

    public double[] getTargetAngles(int targetID) {
        Pose3d targetPose = getTargetPoseRelative(targetID);
        if (targetPose == null) {
            return null;
        }
        Rotation3d targetRotation3d = targetPose.getRotation();

        double pitch = targetRotation3d.getY();
        double yaw = targetRotation3d.getZ();
        double roll = targetRotation3d.getX();
        return new double[] {roll, pitch, yaw};
    }
}
