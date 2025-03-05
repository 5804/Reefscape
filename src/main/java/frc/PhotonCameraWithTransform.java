// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;

public class PhotonCameraWithTransform {
    public PhotonCamera photonCamera;
    public Transform3d photonCameraTransform;

    public PhotonCameraWithTransform(PhotonCamera photonCamera, Transform3d photonCameraTransform) {
        this.photonCamera = photonCamera;
        this.photonCameraTransform = photonCameraTransform;
    }

    public PhotonCamera getPhotonCamera() {
        return photonCamera;
    }

    public Transform3d getCameraTransform() {
        return photonCameraTransform;
    }
}
