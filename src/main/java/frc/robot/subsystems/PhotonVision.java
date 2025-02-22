// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.util.List;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;

// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.PhotonCameraWithTransform;
// import frc.robot.Constants;

// public class PhotonVision extends SubsystemBase {
//   PhotonPoseEstimator photonPoseEstimator;
  
//   /** Creates a new PhotonVision. */
//   public PhotonVision() {
    
//   }

//   public void dumpMultiTagData(PhotonCameraWithTransform[] photonCamerasWithTransforms) {
//     for(int index = 0; index < photonCamerasWithTransforms.length; index++){
//       /** Storing objects in temporary variables */
//       PhotonCamera currentCamera = photonCamerasWithTransforms[index].getPhotonCamera();
//       Transform3d currentCameraTransform = photonCamerasWithTransforms[index].getCameraTransform();

//       List<PhotonPipelineResult> frameResults = currentCamera.getAllUnreadResults();

//       photonPoseEstimator = new PhotonPoseEstimator(Constants.PhotonVisionConstants.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, currentCameraTransform); // Moving this inside of the nex if could be more efficient for memory usage

//       if (!frameResults.isEmpty()) {
//         PhotonPipelineResult result = frameResults.get(frameResults.size() - 1);

//         if (result.getMultiTagResult().isPresent()) {
//           Pose3d estimatedRobotPose = photonPoseEstimator.update(result).get().estimatedPose;
//           String timestamp = "" + java.time.LocalDateTime.now().getHour() + ":" + java.time.LocalDateTime.now().getMinute() + ":" + java.time.LocalDateTime.now().getSecond();

//           // Dump information to SmartDashboard. RTE = Robot Transform Estimation. RTE.Rot.X = Robot Transform Estimation's Rotation X. 
//           SmartDashboard.putNumber(currentCamera.getName() + ".frmt.rte.x", estimatedRobotPose.getX());
//           SmartDashboard.putNumber(currentCamera.getName() + ".frmt.rte.y", estimatedRobotPose.getY());
//           SmartDashboard.putNumber(currentCamera.getName() + ".frmt.rte.z", estimatedRobotPose.getZ());
//           SmartDashboard.putNumber(currentCamera.getName() + ".frmt.rte.rot.x", Units.radiansToDegrees(estimatedRobotPose.getRotation().getX()));
//           SmartDashboard.putNumber(currentCamera.getName() + ".frmt.rte.rot.y", Units.radiansToDegrees(estimatedRobotPose.getRotation().getY()));
//           SmartDashboard.putNumber(currentCamera.getName() + ".frmt.rte.rot.z", Units.radiansToDegrees(estimatedRobotPose.getRotation().getZ()));
//           SmartDashboard.putString(currentCamera.getName() + ".frmt.rte.timestamp", timestamp);
//         } 
//       }
//     }
//   }

//   @Override
//   public void periodic() {
//     dumpMultiTagData(Constants.PhotonVisionConstants.photonCamerasWithTransforms);
//   }
// }
