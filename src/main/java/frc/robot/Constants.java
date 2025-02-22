package frc.robot;



import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.PhotonCameraWithTransform;

public final class Constants {
    public final class PhotonVision {
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark); // SWITCH AT WORLDS, IF WE MAKE IT

        public static final PhotonCameraWithTransform photonCamerasWithTransforms[] = {
            new PhotonCameraWithTransform(new PhotonCamera("front"), new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0))),
            new PhotonCameraWithTransform(new PhotonCamera("left"), new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0))),
            new PhotonCameraWithTransform(new PhotonCamera("back"), new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0))),
            new PhotonCameraWithTransform(new PhotonCamera("right"), new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)))
        };
    } 

    public final class Elevator {
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
         * to be set to while placing coral.
         * Should be used with setElevatorPosition()
         */
        // NEED TO SET ALL OF THESE STILL
        public static final double zeroPosition = 0.26709;
        public static final double l4Position = 0;
        public static final double l3Position = 0;
        public static final double l2Position = 0;
        public static final double l1Position = 0;
        public static final double groundReadyPosition = -4.95;
        public static final double groundPickupPosition = -3; // NEED TO SET!!!
        public static final double hopperIntakePosition = 0;

        // Used to shrink software limits to slightly smaller than the full elevator.
        public static final double softwareLimitSafetyThreshold = 5;
    }

    public final class Arm {
        public final class Shoulder {
            public static final int motorID = 55;
            public static final int encoderID = 60;

            public static final double kS = 0.50;
            public static final double kV = 0.12;
            public static final double kA = 0.01;
            public static final double kP = 30.0;
            public static final double kI = 0;
            public static final double kD = 0.1;

            public static final double cruiseVelocity = 80;
            public static final double acceleration = 160;
            public static final double jerk = 1600;

            /**
             * Shoulder positions based on the shoulder absolute encoder for the shoulder to be set
             * to while placing coral.
             * Should be used with setShoulderPosition
             */
            // NEED TO SET ALL OF THESE STILL
            public static final double minSafeValue = -0.256592;
            public static final double l4Position = 0;
            public static final double l3Position = 0;
            public static final double l2Position = 0;
            public static final double l1Position = 0;
            public static final double groundPosition = 0.062012;
            public static final double hopperIntakePosition = 0;
        }

        public final class Wrist {
            public static final int motorID = 57;
            public static final int encoderID = 59;

            public static final double kS = 0.50;
            public static final double kV = 0.12;
            public static final double kA = 0.01;
            public static final double kP = 40.8;
            public static final double kI = 0;
            public static final double kD = 0.1;
            public static final double cruiseVelocity = 80;
            public static final double acceleration = 160;
            public static final double jerk = 1600;

            /**
             * Wrist positions based on the wrist absolute encoder for the wrist to be set
             * to while placing coral.
             */
            public static final double verticalPosition = 0;
            public static final double horizontalPosition = 0.248;
        }

        public final class Claw {
            public static final int motorID = 58;

            public static final double kS = 0.25;
            public static final double kV = 0.12;
            public static final double kA = 0.01;
            public static final double kP = 30.0;
            public static final double kI = 0;
            public static final double kD = 0.1;
            public static final double cruiseVelocity = 80;
            public static final double acceleration = 160;
            public static final double jerk = 1600;

            public static final double motorIntakeSpeed = 1;
            public static final double motorEjectSpeed = -1;
        }
    }

    // NEED TO SET ALL OF THESE STILL
    public final class Climber {
        public static final int leftMotorID = 0;
        public static final int rightMotorID = 0;

        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final double kP = 4.8;
        public static final double kI = 0;
        public static final double kD = 0.1;
        public static final double cruiseVelocity = 80;
        public static final double acceleration = 160;
        public static final double jerk = 1600;

        public static final double downClimberPosition = 0;
        public static final double climbClimberPosition = 0;
        public static final double stowClimberPosition = 0;
    }
}