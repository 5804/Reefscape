package frc.robot;



import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.PhotonCameraWithTransform;

public final class Constants {
    public static final double inversion = -1;

    public final class AutonomousConstants {
        public static final double systemTestDelay = 3;
    }

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
        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final double kP = 5;
        public static final double kI = 0;
        public static final double kD = 0.1;
        public static final double cruiseVelocity = 75;
        public static final double acceleration = 160;
        public static final double jerk = 1600;

        public static final int leftElevatorMotorID  = 52;
        public static final int rightElevatorMotorID = 51;

        public static final double zeroPosition         = 0.15832;
        public static final double minSafePosition      = -38;
        public static final double l4Position           = -31.73;
        public static final double l3Position           = -15.6;
        public static final double l3TestPosition       = -15.355;
        public static final double l4TestPosition       = -35.183; //32.214
        public static final double l3SCORETestPosition  = -7.368;
        public static final double l4SCORETestPosition  = -23.892;
        public static final double l2Position           = -5.9;
        public static final double l1Position           = -5.11; // Measured but needs to be tested more
        public static final double groundReadyPosition  = -4.95;
        public static final double groundPickupPosition = -1.652; // -2.14826
        public static final double hopperIntakePosition = zeroPosition;
        
        public static final double softwareLimitSafetyThreshold = 5;
        public static final double forwardSoftLimitThreshold = 0;
        public static final double reverseSoftLimitThreshold = -38;

        public static final double manualTravelSpeedVoltage = 1;
        public static final double supplyCurrentLimit = 70;
        public static final double statorCurrentLimit = 120;
    }

    public final class ArmConstants {
        public final class ShoulderConstants {
            public static final int motorID = 55;
            public static final int encoderID = 60;
            public static final double kS = 0.3;
            public static final double kV = 32.57;
            public static final double kA = 0.01;
            public static final double kP = 10;
            public static final double kI = 0;
            public static final double kD = 0.1;
            public static final double kG = 0.00;

            public static final double cruiseVelocity = 0.75;
            public static final double acceleration   = .75;
            public static final double jerk           = 2;

            public static final double minSafeValue             = 0.147; 
            public static final double l4Position               = 0.16; 
            public static final double l3Position               = 0.1443;
            public static final double l3TestPosition           = 0.184;
            public static final double l4TestPosition           = 0.232;
            public static final double l2Position               = 0.14;
            public static final double l1Position               =  0.316;
            public static final double groundPosition           = 0.4922; 
            public static final double groundPostpickupPosition = 0.25;
            public static final double hopperIntakePosition     = .0712;

            public static final double manualTravelSpeedVoltage = 5;
            public static final double supplyCurrentLimit = 60;
            public static final double statorCurrentLimit = 120;
        }

        public final class WristConstants {
            public static final int motorID   = 57;
            public static final int encoderID = 59;

            public static final double kS = 0.50;
            public static final double kV = 0.12;
            public static final double kA = 0.01;
            public static final double kP = 60;
            public static final double kI = 0;
            public static final double kD = 0.1;

            public static final double cruiseVelocity = 80;
            public static final double acceleration   = 160;
            public static final double jerk           = 1600;

            public static final double verticalPosition   = 0;
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
            public static final double acceleration   = 160;
            public static final double jerk           = 1600;

            public static final double motorIntakeSpeed = .8;
            public static final double motorEjectSpeed  = -1;

            public static final double tofHasCoralUpperBound = 40; // The maximum distance (mm) away from the time of flight sensor that the coral needs to be to intake
            public static final double tofHasCoralLowerBound = 5;  // The minimum distance ^
        }
    }

    public final class ClimberConstants {
        public static final int leftMotorID  = 53;
        public static final int rightMotorID = 54;
        public static final int leftRachetPWMChannel  = 1;
        public static final int rightRachetPWMChannel = 2;

        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final double kP = 15;
        public static final double kI = 0;
        public static final double kD = 0.1;

        public static final double cruiseVelocity = 100;
        public static final double acceleration   = 200;
        public static final double jerk           = 1600;

        public static final double downClimberPosition  = 253;
        public static final double climbClimberPosition = 1;
        public static final double stowClimberPosition  = 0.1;
    }
}