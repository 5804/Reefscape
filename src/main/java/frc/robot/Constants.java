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

    public final class LimelightConstants {
        public static final double offsetLeft = 0;
        public static final double offsetRight = 0;
    }

    public final class ElevatorConstants {
        /** PID slot 0 gains */
        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final double kP = 5; // 5
        public static final double kI = 0;
        public static final double kD = 0.1;
        public static final double cruiseVelocity = 75;
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
        public static final double zeroPosition = 0.15832;
        public static final double minSafePosition = -38;
        public static final double l4Position = -30.29;
        public static final double l3Position = -15.6;
        public static final double l2Position = -5.9;
        public static final double l1Position = 0;
        public static final double groundReadyPosition = -4.95;
        public static final double groundPickupPosition = -1.652; // -2.14826
        public static final double hopperIntakePosition = zeroPosition;

        // Used to shrink software limits to slightly smaller than the full elevator.
        public static final double softwareLimitSafetyThreshold = 5;
    }

    public final class ArmConstants {
        public final class ShoulderConstants {
            public static final int motorID = 55;
            public static final int encoderID = 60;

            public static final double kS = 0.50;
            public static final double kV = 0.12;
            public static final double kA = 0.01;
            public static final double kP = 40.0; // 40
            public static final double kI = 0;
            public static final double kD = 0.1;

            public static final double cruiseVelocity = 400 * 100; // 320 // 400
            public static final double acceleration = 400 * 100;
            public static final double jerk = 1600;

            /**
             * Shoulder positions based on the shoulder absolute encoder for the shoulder to be set
             * to while placing coral.
             * Should be used with setShoulderPosition
             */
            // NEED TO SET ALL OF THESE STILL
            public static final double minSafeValue = 0.13; // 0.12
            public static final double l4Position = 0.16; // 0.13
            public static final double l3Position = 0.14;
            public static final double l2Position = 0.14;
            public static final double l1Position =  0.13;
            public static final double groundPosition = 0.495117; // 0.062012
            public static final double groundPostpickupPosition = 0.25;
            public static final double hopperIntakePosition = .0712; // 0.091553;
        }

        public final class WristConstants {
            public static final int motorID = 57;
            public static final int encoderID = 59;

            public static final double kS = 0.50;
            public static final double kV = 0.12;
            public static final double kA = 0.01;
            public static final double kP = 60;
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
            public static final double horizontalPosition = 0.25;
        }

        public final class ClawConstants {
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

            public static final double tofHasCoralUpperBound = 40; // The maximum distance (mm) away from the time of flight sensor that the coral needs to be to intake
            public static final double tofHasCoralLowerBound = 5; // The minimum distance ^
        }
    }

    // NEED TO SET ALL OF THESE STILL
    public final class ClimberConstants {
        public static final int leftMotorID = 53;
        public static final int rightMotorID = 54;

        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final double kP = 15;
        public static final double kI = 0;
        public static final double kD = 0.1;
        public static final double cruiseVelocity = 100;
        public static final double acceleration = 200;
        public static final double jerk = 1600;

        public static final double downClimberPosition = 253;
        public static final double climbClimberPosition = 1; // 5
        public static final double stowClimberPosition = 0.1;
    }
}