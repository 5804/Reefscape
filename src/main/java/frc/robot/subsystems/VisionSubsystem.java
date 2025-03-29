package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Vector;

import edu.wpi.first.math.geometry.Transform3d;

import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionSubsystem extends SubsystemBase{

    private final CommandSwerveDrivetrain drivetrain;
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator poseEstimator;
    private Optional<EstimatedRobotPose>  estimatedRobotPose;
    private boolean safetyOverride = false;
    private String cameraName;
    private Vector<N3> visionMeasurementStdDevs;
   
    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    
    public VisionSubsystem(CommandSwerveDrivetrain  drivetrain, String cameraName, Transform3d robotToCam, Vector<N3> stdDev){
        this.drivetrain = drivetrain;
        this.cameraName = cameraName;
        this.visionMeasurementStdDevs = stdDev;

        photonCamera = new PhotonCamera(cameraName);

        // Construct PhotonPoseEstimator
        poseEstimator = new PhotonPoseEstimator(Constants.PhotonVisionConstants.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void setSafetyOverride(boolean safetyOverride){
        this.safetyOverride = safetyOverride;
    }

    public void periodic(){
        estimatedRobotPose = getEstimatedGlobalPose();
        if (estimatedRobotPose.isPresent()){
            // send this new vision position to drivetrain to adjust odometry if we are within 1 M of out last position
            // validate the position before using it.
            if ((drivetrain.getState().Pose.getTranslation().getDistance(estimatedRobotPose.get().estimatedPose.toPose2d().getTranslation()) <= 0.5) || (DriverStation.isDisabled()) || safetyOverride) {
                drivetrain.addVisionMeasurement(estimatedRobotPose.get().estimatedPose.toPose2d(), estimatedRobotPose.get().timestampSeconds, visionMeasurementStdDevs);
            }
            // dont display if locked out
            SmartDashboard.putString(cameraName + " Pose 2d", estimatedRobotPose.get().estimatedPose.toPose2d().toString());
        }
        SmartDashboard.putBoolean(cameraName + " Safety Override", safetyOverride);
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : photonCamera.getAllUnreadResults()) {
            visionEst = poseEstimator.update(change);
        }
        return visionEst;
    }
}