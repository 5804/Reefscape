package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Vector;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionSubsystem extends SubsystemBase{

    private final CommandSwerveDrivetrain drivetrain;
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator poseEstimator;
    private Optional<EstimatedRobotPose>  estimatedRobotPose;
    private boolean safetyOverride = false;
    private String cameraName;
    private Vector<N3> visionMeasurementStdDevs;
    private PhotonTrackedTarget cameraClosestTarget;
   
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

    public void captureClosestTargets(List<PhotonPipelineResult> frameResults) {
      PhotonTrackedTarget closestTarget = null;
      double closestDistance = Double.MAX_VALUE;
      int frameIndex = frameResults.size() - 1;

      if(!frameResults.isEmpty() && frameResults.get(frameIndex).hasTargets()) {
        PhotonPipelineResult result = frameResults.get(frameIndex);
        List<PhotonTrackedTarget> targets = result.getTargets();

        for(int targetIndex = 0; targetIndex < targets.size(); targetIndex++) {
          double distanceToTag = targets.get(targetIndex).getBestCameraToTarget().getTranslation().getDistance(new Translation3d(0, 0, 0));
          if(distanceToTag < closestDistance) {
            closestTarget = targets.get(targetIndex);
            closestDistance = distanceToTag;
          }
        }
        cameraClosestTarget = closestTarget;
      } else {
        cameraClosestTarget = null;
      }
    }

    public double closestTargetYaw(int cameraIndex){
        return cameraClosestTarget != null ? cameraClosestTarget.getBestCameraToTarget().getRotation().getZ() : 0;
    }

    public double closestTargetXMeters(int cameraIndex){
        return cameraClosestTarget != null ? cameraClosestTarget.getBestCameraToTarget().getMeasureX().in(Meters) : 0;
    }

    public double closestTargetYMeters(int cameraIndex){
        return cameraClosestTarget != null ? cameraClosestTarget.getBestCameraToTarget().getMeasureY().in(Meters) : 0;
    }

    public int closestTargetID(int cameraIndex){
        return cameraClosestTarget != null ? cameraClosestTarget.getFiducialId() : 0;
    }

    public Command alignLeft(){
        return moveToReefLeft(Constants.PhotonVisionConstants.rightCameraID);
    }

    public Command alignRight(){
        return moveToReefRight(Constants.PhotonVisionConstants.leftCameraID);
    }

    public Command moveToReefLeft(int cameraIndex) {
        return drivetrain.applyRequest(() -> 
            RobotContainer.driveRobotCentric
                .withVelocityX(this.cameraClosestTarget == null ? 0 : (closestTargetXMeters(cameraIndex) - Constants.PhotonVisionConstants.reefLeftOffsetMagnitudeX) * Constants.PhotonVisionConstants.visionOrthogonalSpeedScale)
                .withVelocityY(this.cameraClosestTarget == null ? 0 : (closestTargetYMeters(cameraIndex) - Constants.PhotonVisionConstants.reefLeftOffsetMagnitudeY) * Constants.PhotonVisionConstants.visionOrthogonalSpeedScale)
                .withRotationalRate(this.cameraClosestTarget == null ? 0 : ((Math.PI - (Math.abs(closestTargetYaw(cameraIndex)))) * Math.signum(closestTargetYaw(cameraIndex))) * Constants.inversion * Constants.PhotonVisionConstants.visionRotationalSpeedScale)
            );
    }

    public Command moveToReefRight(int cameraIndex) {
        return drivetrain.applyRequest(() -> 
            RobotContainer.driveRobotCentric
                .withVelocityX(this.cameraClosestTarget == null ? 0 : (closestTargetXMeters(cameraIndex) - Constants.PhotonVisionConstants.reefRightOffsetMagnitudeX) * Constants.PhotonVisionConstants.visionOrthogonalSpeedScale)
                .withVelocityY(this.cameraClosestTarget == null ? 0 : (closestTargetYMeters(cameraIndex) + Constants.PhotonVisionConstants.reefRightOffsetMagnitudeY) * Constants.PhotonVisionConstants.visionOrthogonalSpeedScale)
                .withRotationalRate(this.cameraClosestTarget == null ? 0 : ((Math.PI - (Math.abs(closestTargetYaw(cameraIndex)))) * Math.signum(closestTargetYaw(cameraIndex))) * Constants.inversion * Constants.PhotonVisionConstants.visionRotationalSpeedScale)
            );
    }

    /**
     * GaCo's Stuff
     * @param safetyOverride
     */
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
        List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
        // do our processing, save our data to our variables then we get both
        
        if (!results.isEmpty())
            captureClosestTargets(results);

        for (PhotonPipelineResult change : results) {
            visionEst = poseEstimator.update(change);
        }
        return visionEst;
    }
}