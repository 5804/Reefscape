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
        /** PID slot 0 gains */
        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final double kP = 20;
        public static final double kI = 0;
        public static final double kD = 0.1;
        public static final double cruiseVelocity = 80;
        public static final double acceleration = 160;
        public static final double jerk = 1600;
        // public static final double forwardSofLimitThreshold =  ; // NEED TO TEST
        /** IDs */
        public static final int leftElevatorMotorID = 52;
        public static final int rightElevatorMotorID = 51;

        /**
         * Elevator positions based on the leftElevatorMotor's encoder for the elevator
         * to be set to while placing coral
         * Should be used with setElevatorPosition()
         */
        // NEED TO SET ALL OF THESE STILL
        public static final double zeroElevatorPosition = 0.26709;
        public static final double l4ElevatorPosition = 0;
        public static final double l3ElevatorPosition = 0;
        public static final double l2ElevatorPosition = 0;
        public static final double l1ElevatorPosition = 0;
        public static final double groundElevatorReadyPosition = -4.95;
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
        public final class ShoulderConstants {
            public static final double kS = 0.50;
            public static final double kV = 0.12;
            public static final double kA = 0.01;
            public static final double kP = 30.0;
            public static final double kI = 0;
            public static final double kD = 0.1;
            public static final double cruiseVelocity = 80;
            public static final double acceleration = 160;
            public static final double jerk = 1600;
        }

        public final class WristConstants {
            public static final double kS = 0.50;
            public static final double kV = 0.12;
            public static final double kA = 0.01;
            public static final double kP = 40.8;
            public static final double kI = 0;
            public static final double kD = 0.1;
            public static final double cruiseVelocity = 80;
            public static final double acceleration = 160;
            public static final double jerk = 1600;
        }

        public final class ClawConstants {
            public static final double kS = 0.25;
            public static final double kV = 0.12;
            public static final double kA = 0.01;
            public static final double kP = 30.0;
            public static final double kI = 0;
            public static final double kD = 0.1;
            public static final double cruiseVelocity = 80;
            public static final double acceleration = 160;
            public static final double jerk = 1600;
        }
        
        /** IDs */
        public static final int shoulderMotorID = 55;
        public static final int wristMotorID = 57;
        public static final int clawMotorID = 58;
        public static final int wristEncoderID = 59;
        public static final int shoulderEncoderID = 60;

        /**
         * Shoulder positions based on the shoulder absolute encoder for the shoulder to be set
         * to while placing coral
         * Should be used with //PUT CORRECT METHOD HERE
         */
        // NEED TO SET ALL OF THESE STILL
        public static final double shoulderMinSafeValue = -0.256592;
        public static final double l4ShoulderPosition = 0;
        public static final double l3ShoulderPosition = 0;
        public static final double l2ShoulderPosition = 0;
        public static final double l1ShoulderPosition = 0;
        public static final double groundShoulderPosition = 0.062012;
        public static final double handoffShoulderPosition = 0;

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