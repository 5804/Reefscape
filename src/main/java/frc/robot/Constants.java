package frc.robot;



import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.PhotonCameraWithTransform;

public final class Constants {
    public final class PhotonVisionConstants {
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark); // SWITCH AT WORLDS, IF WE MAKE IT

        public static final PhotonCameraWithTransform photonCamerasWithTransforms[] = {
            new PhotonCameraWithTransform(new PhotonCamera("front"), new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0))),
            new PhotonCameraWithTransform(new PhotonCamera("left"), new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0))),
            new PhotonCameraWithTransform(new PhotonCamera("back"), new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0))),
            new PhotonCameraWithTransform(new PhotonCamera("right"), new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)))
        };
    } 

    public final class ElevatorConstants {
        /** IDs */
        public static final int leftElevatorMotorID = 52;
        public static final int rightElevatorMotorID = 51;

        /**
         * Elevator positions based on the leftElevatorMotor's encoder for the elevator
         * to be set to while placing coral
         * Should be used with setElevatorPosition()
         */
        // NEED TO SET ALL OF THESE STILL
        public static final double l4ElevatorPosition = 0;
        public static final double l3ElevatorPosition = 0;
        public static final double l2ElevatorPosition = 0;
        public static final double l1ElevatorPosition = 0;
        public static final double groundElevatorPosition = 0;
        public static final double handoffElevatorPosition = 0;

        /**
         * Elevator positions based on the correct heights in inches for the elevator to
         * be set to while placing coral
         * Should be used with setElevatorHeight(), which automatically converts inches
         * to encoder values
         */
        // NEED TO SET ALL OF THESE STILL
        public static final double l4ElevatorHeightInches = 0;
        public static final double l3ElevatorHeightInches = 0;
        public static final double l2ElevatorHeightInches = 0;
        public static final double l1ElevatorHeightInches = 0;
        public static final double groundElevatorHeightInches = 0;
        public static final double handoffElevatorHeightInches = 0;

        // Used to shrink software limits to slightly smaller than the full elevator.
        public static final double softwareLimitSafetyThreshold = 5;
    }

    public final class ArmConstants {
        /** IDs */
        public static final int elbowMotorID = 55;
        public static final int wristMotorID = 57;
        public static final int clawMotorID = 58;
        public static final int wristEncoderID = 59;
        public static final int elbowEncoderID = 60;

        /**
         * Elbow positions based on the elbow absolute encoder for the elbow to be set
         * to while placing coral
         * Should be used with //PUT CORRECT METHOD HERE
         */
        // NEED TO SET ALL OF THESE STILL
        public static final double l4ElbowPosition = 0;
        public static final double l3ElbowPosition = 0;
        public static final double l2ElbowPosition = 0;
        public static final double l1ElbowPosition = 0;
        public static final double groundElbowPosition = 0;
        public static final double handoffElbowPosition = 0;

        /**
         * Wrist positions based on the wrist absolute encoder for the wrist to be set
         * to while placing coral
         * Should be used with //PUT CORRECT METHOD HERE
         */
        public static final double verticalWristPosition = 0;
        public static final double horizontalWristPosition = 0.248;

        public static final double clawMotorIntakeSpeed = 1;
        public static final double clawMotorDropSpeed = -1;
    }

    // NEED TO SET ALL OF THESE STILL
    public final class ClimberConstants {
        public static final double downClimberPosition = 0;
        public static final double climbClimberPosition = 0;
        public static final double stowClimberPosition = 0;
    }
}