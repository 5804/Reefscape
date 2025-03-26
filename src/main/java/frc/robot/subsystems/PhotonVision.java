// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.PhotonCameraWithTransform;
import frc.robot.Constants;

public class PhotonVision extends SubsystemBase {
  
  public PhotonPoseEstimator debugPhotonPoseEstimator;
  public PhotonTrackedTarget noTarget = new PhotonTrackedTarget(0, 0, 0, 0, 0, 0, 0, new Transform3d(), new Transform3d(), 0, new ArrayList<TargetCorner>(), new ArrayList<TargetCorner>());
  public PhotonCamera backCamera = new PhotonCamera("Back");   //CameraID 11, cameraIndex 0
  public PhotonCamera leftCamera = new PhotonCamera("Left");   //CameraID 12, cameraIndex 1
  public PhotonCamera rightCamera = new PhotonCamera("Right"); //CameraID 13, cameraIndex 2
  public PhotonCamera[] cameras = {backCamera, leftCamera, rightCamera};
  public Transform3d[] cameraTransforms = {
    new Transform3d(-0.02033524, 0.14771624, 0.9484741, new Rotation3d(0, 0.558505, 0)),
    new Transform3d(0.2794, 0.254, 0.1905, new Rotation3d(0, 0.436332, 0)),
    new Transform3d(0.2794, -0.254, 0.1905, new Rotation3d(0, 0.436332, 0))
  };
  public PhotonTrackedTarget[] cameraBestTargets;
  public PhotonTrackedTarget[] cameraClosestTargets;
  public PhotonPoseEstimator[] cameraPoseEstimator;
  public Pose3d[] estimatedPoses;
  public boolean[] cameraIsLive = new boolean[4];

  public PhotonVision() {
    cameraBestTargets = new PhotonTrackedTarget[cameras.length];
    cameraClosestTargets = new PhotonTrackedTarget[cameras.length];
    estimatedPoses = new Pose3d[cameras.length];
    cameraPoseEstimator = new PhotonPoseEstimator[cameras.length];

    for(int cameraIndex = 0; cameraIndex < cameras.length; cameraIndex++){
      cameraBestTargets[cameraIndex] = new PhotonTrackedTarget(0.0, 0.0, 0.0, 0.0, 0, 0, (float) 0.0, new Transform3d(0,0,0, new Rotation3d(0,0,0)), new Transform3d(0,0,0, new Rotation3d(0,0,0)), 0.0, new ArrayList<TargetCorner>(), new ArrayList<TargetCorner>());
      cameraBestTargets[cameraIndex].bestCameraToTarget = new Transform3d(0,0,0, new Rotation3d(0,0,0));
      cameraClosestTargets[cameraIndex] = new PhotonTrackedTarget(0.0, 0.0, 0.0, 0.0, 0, 0, (float) 0.0, new Transform3d(0,0,0, new Rotation3d(0,0,0)), new Transform3d(0,0,0, new Rotation3d(0,0,0)), 0.0, new ArrayList<TargetCorner>(), new ArrayList<TargetCorner>());
      cameraClosestTargets[cameraIndex].bestCameraToTarget = new Transform3d(0,0,0, new Rotation3d(0,0,0));
      estimatedPoses[cameraIndex] = new Pose3d();
      cameraPoseEstimator[cameraIndex] = new PhotonPoseEstimator(Constants.PhotonVisionConstants.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraTransforms[cameraIndex]);
    }
  }

  public void captureBestTargets() {
    for(int cameraIndex = 0; cameraIndex < cameras.length; cameraIndex++){
      PhotonCamera currentCamera = cameras[cameraIndex];
      List<PhotonPipelineResult> frameResults = currentCamera.getAllUnreadResults();
      PhotonPipelineResult selectedFrame = new PhotonPipelineResult();

      if (!frameResults.isEmpty()){
        for(int i=frameResults.size()-1; i>=0; i--){
          if (!frameResults.isEmpty() && frameResults.get(i).hasTargets()){
            selectedFrame = frameResults.get(i);
            break;
          }
        }

        cameraBestTargets[cameraIndex] = selectedFrame.getBestTarget();
      }
    }
  }

  public void captureClosestTargets() {
    for(int cameraIndex = 0; cameraIndex < cameras.length; cameraIndex++) {
      PhotonCamera currentCamera = cameras[cameraIndex];
      PhotonTrackedTarget closestTarget = new PhotonTrackedTarget();
      List<PhotonPipelineResult> frameResults = currentCamera.getAllUnreadResults();
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

        cameraClosestTargets[cameraIndex] = closestTarget;
      }
    }
  }

  public void capturePoseEstimations(){
    for(int cameraIndex = 0; cameraIndex < cameras.length; cameraIndex++){
      PhotonCamera currentCamera = cameras[cameraIndex];
      List<PhotonPipelineResult> frameResults = currentCamera.getAllUnreadResults();
      for (PhotonPipelineResult frame : frameResults){
        if (frame.getMultiTagResult().isPresent()){
          estimatedPoses[cameraIndex] = cameraPoseEstimator[cameraIndex].update(frame).get().estimatedPose;
        }
      }
    }
  }

  public Pose3d[] getEstimatedPoses() {
    return estimatedPoses;
  }

  public double bestTargetYaw(int cameraIndex){
    return cameraBestTargets[cameraIndex].getBestCameraToTarget().getRotation().getZ();
  }

  public double bestTargetXMeters(int cameraIndex){
    return cameraBestTargets[cameraIndex].getBestCameraToTarget().getMeasureX().in(Meters);
  }

  public double bestTargetYMeters(int cameraIndex){
    return cameraBestTargets[cameraIndex].getBestCameraToTarget().getMeasureY().in(Meters);
  }

  public int bestTargetID(int cameraIndex){
    return cameraBestTargets[cameraIndex].getFiducialId();
  }

  public double closestTargetYaw(int cameraIndex){
    return cameraClosestTargets[cameraIndex].getBestCameraToTarget().getRotation().getZ();
  }

  public double closestTargetXMeters(int cameraIndex){
    return cameraClosestTargets[cameraIndex].getBestCameraToTarget().getMeasureX().in(Meters);
  }

  public double closestTargetYMeters(int cameraIndex){
    return cameraClosestTargets[cameraIndex].getBestCameraToTarget().getMeasureY().in(Meters);
  }

  public int closestTargetID(int cameraIndex){
    return cameraClosestTargets[cameraIndex].getFiducialId();
  }

  public void debugSingleTagCameraData(PhotonCamera[] cameras, Transform3d[] cameraTransforms) {
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
        else {
          // frontTargetYaw = 0;
          // frontTargetRangeX = 0;
          // frontTargetRangeY = 0;
          // SmartDashboard.putNumber("Yaw", frontTargetYaw);
          // SmartDashboard.putNumber("Range X", frontTargetYaw);
          // SmartDashboard.putNumber("Range Y", frontTargetYaw);
        }
      }
    }
  }

  public void debugMultiTagData(PhotonCameraWithTransform[] photonCamerasWithTransforms) {
    for(int index = 0; index < photonCamerasWithTransforms.length; index++){

      PhotonCamera currentCamera = photonCamerasWithTransforms[index].getPhotonCamera();
      Transform3d currentCameraTransform = photonCamerasWithTransforms[index].getCameraTransform();
      List<PhotonPipelineResult> frameResults = currentCamera.getAllUnreadResults();

      debugPhotonPoseEstimator = new PhotonPoseEstimator(Constants.PhotonVisionConstants.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, currentCameraTransform); // Moving this inside of the nex if could be more efficient for memory usage

      if (!frameResults.isEmpty()) {
        PhotonPipelineResult result = frameResults.get(frameResults.size() - 1);

        if (result.getMultiTagResult().isPresent()) {
          Pose3d estimatedRobotPose = debugPhotonPoseEstimator.update(result).get().estimatedPose;
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

  @Override
  public void periodic() {
    captureBestTargets();
    //captureClosestTargets();
    //capturePoseEstimations();
  }
}
