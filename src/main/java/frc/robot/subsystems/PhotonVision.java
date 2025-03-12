// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.PhotonCameraWithTransform;
import frc.robot.Constants;

public class PhotonVision extends SubsystemBase {
  
  public PhotonPoseEstimator photonPoseEstimator;
  public PhotonCamera frontCamera = new PhotonCamera("Front");
  public PhotonCamera[] cameras = {frontCamera};
  public Transform3d[] cameraTransforms = {new Transform3d(0, 0.172339, 0.13024104, new Rotation3d(0, 0.4799655, 0))};
  public static PhotonTrackedTarget[] cameraTargets = {null};

  public PhotonVision() {}

  public void captureBestTargets(){
    for(int cameraIndex = 0; cameraIndex < cameras.length; cameraIndex++){
      PhotonCamera currentCamera = cameras[cameraIndex];
      List<PhotonPipelineResult> frameResults = currentCamera.getAllUnreadResults();
      int frameIndex = frameResults.size() - 1;
      cameraTargets[cameraIndex] = (!frameResults.isEmpty() && frameResults.get(frameIndex).hasTargets()) ? frameResults.get(frameIndex).getBestTarget() : null;
    }
  }

  public double bestTargetYaw(int cameraIndex){
    return cameraTargets[cameraIndex].getYaw();
  }

  public double bestTargetXMeters(int cameraIndex){
    return cameraTargets[cameraIndex].getBestCameraToTarget().getMeasureX().in(Meters);
  }

  public double bestTargetYMeters(int cameraIndex){
    return cameraTargets[cameraIndex].getBestCameraToTarget().getMeasureY().in(Meters);
  }

  public int bestTargetID(int cameraIndex){
    return cameraTargets[cameraIndex].getFiducialId();
  }

  public void dumpSingleTagCameraData(PhotonCamera[] cameras, Transform3d[] cameraTransforms) {
    for(int cameraIndex = 0; cameraIndex < cameras.length; cameraIndex++){
      PhotonCamera currentCamera = cameras[cameraIndex];
      Transform3d currentCameraTransform = cameraTransforms[cameraIndex];
      List<PhotonPipelineResult> frameResults = currentCamera.getAllUnreadResults();

      if (!frameResults.isEmpty()) {
        PhotonPipelineResult result = frameResults.get(frameResults.size() - 1);

        if (result.hasTargets()) {          
          PhotonTrackedTarget bestTarget = result.getBestTarget();
          Pose3d estimatedRobotPose = null;
          String timestamp = "" + java.time.LocalDateTime.now().getHour() + ":" + java.time.LocalDateTime.now().getMinute() + ":" + java.time.LocalDateTime.now().getSecond();

          SmartDashboard.putNumber(currentCamera.getName() + ".bt.id", bestTarget.getFiducialId());
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.x", trunc(bestTarget.getBestCameraToTarget().getMeasureX().in(Meters)));
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.y", trunc(bestTarget.getBestCameraToTarget().getMeasureY().in(Meters)));
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.z", trunc(bestTarget.getBestCameraToTarget().getMeasureZ().in(Meters)));
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.rot.x", trunc(bestTarget.getBestCameraToTarget().getRotation().getMeasureX().in(Degrees)));
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.rot.y", trunc(bestTarget.getBestCameraToTarget().getRotation().getMeasureY().in(Degrees)));
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.rot.z", trunc(bestTarget.getBestCameraToTarget().getRotation().getMeasureZ().in(Degrees)));
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.ambiguity", trunc(bestTarget.getPoseAmbiguity()));

          estimatedRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getBestCameraToTarget(), Constants.PhotonVisionConstants.aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).get(), currentCameraTransform);
          
          SmartDashboard.putNumber(currentCamera.getName() + ".fr.rte.x", trunc(estimatedRobotPose.getX()));
          SmartDashboard.putNumber(currentCamera.getName() + ".fr.rte.y", trunc(estimatedRobotPose.getY()));
          SmartDashboard.putNumber(currentCamera.getName() + ".fr.rte.z", trunc(estimatedRobotPose.getZ()));
          SmartDashboard.putNumber(currentCamera.getName() + ".fr.rte.rot.x", trunc(Units.radiansToDegrees(estimatedRobotPose.getRotation().getX())));
          SmartDashboard.putNumber(currentCamera.getName() + ".fr.rte.rot.y", trunc(Units.radiansToDegrees(estimatedRobotPose.getRotation().getY())));
          SmartDashboard.putNumber(currentCamera.getName() + ".fr.rte.rot.z", trunc(Units.radiansToDegrees(estimatedRobotPose.getRotation().getZ())));
          SmartDashboard.putString(currentCamera.getName() + ".fr.rte.timestamp", timestamp);

          SmartDashboard.putNumber(currentCamera.getName() + ".bt.fr.tt.x", Constants.PhotonVisionConstants.aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).get().getX());
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.fr.tt.y", Constants.PhotonVisionConstants.aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).get().getY());
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.fr.tt.z", Constants.PhotonVisionConstants.aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).get().getZ());
        } 
      }
    }
  }

  public void debugMultiTagData(PhotonCameraWithTransform[] photonCamerasWithTransforms) {
    for(int index = 0; index < photonCamerasWithTransforms.length; index++){

      PhotonCamera currentCamera = photonCamerasWithTransforms[index].getPhotonCamera();
      Transform3d currentCameraTransform = photonCamerasWithTransforms[index].getCameraTransform();
      List<PhotonPipelineResult> frameResults = currentCamera.getAllUnreadResults();

      photonPoseEstimator = new PhotonPoseEstimator(Constants.PhotonVisionConstants.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, currentCameraTransform); // Moving this inside of the nex if could be more efficient for memory usage

      if (!frameResults.isEmpty()) {
        PhotonPipelineResult result = frameResults.get(frameResults.size() - 1);

        if (result.getMultiTagResult().isPresent()) {
          Pose3d estimatedRobotPose = photonPoseEstimator.update(result).get().estimatedPose;
          String timestamp = "" + java.time.LocalDateTime.now().getHour() + ":" + java.time.LocalDateTime.now().getMinute() + ":" + java.time.LocalDateTime.now().getSecond();

          SmartDashboard.putNumber(currentCamera.getName() + ".frmt.rte.x", estimatedRobotPose.getX());
          SmartDashboard.putNumber(currentCamera.getName() + ".frmt.rte.y", estimatedRobotPose.getY());
          SmartDashboard.putNumber(currentCamera.getName() + ".frmt.rte.z", estimatedRobotPose.getZ());
          SmartDashboard.putNumber(currentCamera.getName() + ".frmt.rte.rot.x", Units.radiansToDegrees(estimatedRobotPose.getRotation().getX()));
          SmartDashboard.putNumber(currentCamera.getName() + ".frmt.rte.rot.y", Units.radiansToDegrees(estimatedRobotPose.getRotation().getY()));
          SmartDashboard.putNumber(currentCamera.getName() + ".frmt.rte.rot.z", Units.radiansToDegrees(estimatedRobotPose.getRotation().getZ()));
          SmartDashboard.putString(currentCamera.getName() + ".frmt.rte.timestamp", timestamp);
        } 
      }
    }
  }

  public double trunc(double x){
    return Math.floor(x * 1000) / 1000;
  }
}
