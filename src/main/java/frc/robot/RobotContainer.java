// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.ButtonBoard;
import frc.CoralSystem;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ArmConstants.ShoulderConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.PhotonVision;

public class RobotContainer {
    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);               // kSpeedAt12Volts desired top speed
    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double speedMultiplier = 1;

    public void increaseSpeedMultiplier(double value) {
        speedMultiplier += value;
    }

    /** Setting up bindings for necessary control of the swerve drive platform. */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.005).withRotationalDeadband(maxAngularRate * 0.005) // Add a 20% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);                       // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(maxSpeed * 0.005).withRotationalDeadband(maxAngularRate * 0.005) // Add a 20% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);                       // Use open-loop control for drive motors


    private final Telemetry logger = new Telemetry(maxSpeed);

    /** Controllers */
    public final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController assistantController = new CommandXboxController(1);
    public final ButtonBoard buttonBoard = new ButtonBoard(11, 2);

    /** Subsytem initializations. */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public Arm arm = new Arm();
    public Elevator elevator = new Elevator();
    public Climber climber = new Climber(() -> { return driveController.getLeftX(); });
    public PhotonVision photonVision = new PhotonVision();
    public Limelight limelight = new Limelight();

    // coralSystem is used to set the elevator and arm to states
    private final CoralSystem coralSystem = new CoralSystem(elevator, arm, climber);

    /** Shuffleboard configurations. */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private ShuffleboardTab tab1 = Shuffleboard.getTab("Tab1");

    public RobotContainer() {
        configureBindings();

        autoChooser.addOption("systemsTest", systemsTest());
        // autoChooser.addOption("oneMeter", oneMeterAuto());
        // autoChooser.addOption("twoMeter", twoMeterAuto());
        // autoChooser.addOption("ninetyDegrees", ninetyDegreesAuto());
        // autoChooser.addOption("plus", plusAuto());
        autoChooser.addOption("Use This Auto (One Meter Forward)", oneMeter());



        SmartDashboard.putData("Auto choices", autoChooser);
        tab1.add("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        /** driveController bindings */
        /**
         * Note that X is defined as forward according to WPILib convention,
         * and Y is defined as to the left according to WPILib convention. 
         * As a default command the drive train will call this continually.
         */
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-1 * Math.pow(MathUtil.applyDeadband(driveController.getLeftY(), 0.1), 3) * maxSpeed * speedMultiplier)            // Drive forward with negative Y (forward)
                     .withVelocityY(-1 * Math.pow(MathUtil.applyDeadband(driveController.getLeftX(), 0.1), 3) * maxSpeed * speedMultiplier)             // Drive left with negative X (left)
                     .withRotationalRate(-1 * Math.pow(MathUtil.applyDeadband(driveController.getRightX(), 0.1), 3) * maxAngularRate * speedMultiplier) // Drive counterclockwise with negative X (left)
            )
        );

        // driveController.leftBumper().whileTrue(); // Strafe
        driveController.leftTrigger().onTrue(coralSystem.setCoralSystemGroundReady()); // Ground Intake
        driveController.leftTrigger().onFalse(coralSystem.setCoralSystemGroundPickup()); // Ground Intake

        //driveController.rightBumper().onTrue(climber.setClimberDown(0.01)); // Lower Climber
        //driveController.rightTrigger().onTrue(climber.setClimberClimb(0.01)); // Raise Climber

        // Need to add ratchet. 
        driveController.y().onTrue(arm.setClawIntakeWithTimeOfFlight());
        driveController.y().onFalse(arm.setClawStop());
        // Toggle Arm Pos (Up and down)
        driveController.b().onTrue(arm.setClawIntake());
        driveController.b().onFalse(arm.setClawStop());

        // driveController.x().onTrue(coralSystem.setCoralSystemHopperIntake()); // Hopper Intake (Uneeded)
        driveController.a().onTrue(
            coralSystem.stowAll()
        ); // Stow

        driveController.povUp().onTrue(new InstantCommand(() -> { speedMultiplier = 1; }));
        driveController.povDown().onTrue(new InstantCommand(() -> { speedMultiplier = 0.25; }));

        // Reset the field-centric heading on left bumper press
        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driveController.start().onTrue(new InstantCommand(() -> { cancelAllActiveCommands(); }));

        /** buttonBoard bindings DON'T DELETE */
        buttonBoard.getButton(4).whileTrue(coralSystem.setCoralSystemHerdAlgaePosition());
        buttonBoard.getButton(4).onFalse(arm.setClawStop());


        // buttonBoard.getButton(8).onTrue(
        //     coralSystem.setCoralSystemL1()
        //                .andThen(() -> speedMultiplier = .75)
        // ); 
        
        buttonBoard.getButton(9).onTrue(
            coralSystem.setCoralSystemL2()
        );
        
        // buttonBoard.getButton(10).onTrue(
        //     coralSystem.setCoralSystemL3()
        // ); 

        buttonBoard.getButton(5).onTrue(
            coralSystem.setCoralSystemL3()
        ); 
        
        buttonBoard.getButton(7).onTrue(
            coralSystem.setCoralSystemL4()
        ); 

        // buttonBoard.getButton(2).onTrue(climber.deactivateRatchets());

        buttonBoard.getButton(3).onTrue(coralSystem.stowAll());
        
        buttonBoard.getButton(1).onTrue(arm.setClawEject());
        buttonBoard.getButton(1).onFalse(arm.setClawStop());

        buttonBoard.getButton(11).onTrue(arm.setWristHorizontal());
        buttonBoard.getButton(6).onTrue(arm.setWristVertical());

        // buttonBoard.getButton(11).whileTrue(limelightAimAtTarget());
        // buttonBoard.getButton(5).whileTrue(limelightMoveToTargetLeft());



        // REFACTOR INTO BUTTONBOARD CLASS
        Trigger buttonBoardRawAxis0Positive = new Trigger(() -> { return buttonBoard.getButtonBoard().getRawAxis(0) > 0.7; });
        Trigger buttonBoardRawAxis0Negative = new Trigger(() -> { return buttonBoard.getButtonBoard().getRawAxis(0) < -0.7; });
        Trigger buttonBoardRawAxis1Positive = new Trigger(() -> { return buttonBoard.getButtonBoard().getRawAxis(1) > 0.7; });
        Trigger buttonBoardRawAxis1Negative = new Trigger(() -> { return buttonBoard.getButtonBoard().getRawAxis(1) < -0.7; });

        // THese were janky and not ready yet. Do not enable.
        buttonBoardRawAxis0Positive.whileTrue(arm.armUp()); // SHOULDER UP
        buttonBoardRawAxis0Negative.whileTrue(arm.armDown()); // SHOULDER DOWN
        buttonBoardRawAxis1Positive.whileTrue(elevator.moveElevatorDown()); // ELEVATOR UP
        buttonBoardRawAxis1Negative.whileTrue(elevator.moveElevatorUp()); // ELEVATOR DOWN

        /** Assistant controller bindings COMMENTED OUT ARE UNIMPLEMENTED, DON'T DELETE */
        assistantController.rightTrigger(0.5).onTrue(coralSystem.stowAll());
        
        assistantController.a().onTrue(coralSystem.setCoralSystemL2());
        assistantController.b().onTrue(coralSystem.setCoralSystemL3());
        assistantController.y().onTrue(coralSystem.setCoralSystemL4());

        assistantController.povRight().whileTrue(arm.armDown());
        assistantController.povLeft().whileTrue(arm.armUp());

        assistantController.leftTrigger(0.5).onTrue(arm.setClawEject());
        assistantController.leftTrigger(0.5).onFalse(arm.setClawStop());

        assistantController.povUp().whileTrue(elevator.moveElevatorUp());
        assistantController.povDown().whileTrue(elevator.moveElevatorDown());

        assistantController.rightBumper().whileTrue(arm.setWristVertical());
        assistantController.leftBumper().whileTrue(arm.setWristHorizontal());

        assistantController.back().whileTrue(coralSystem.setCoralSystemHerdAlgaePosition());
        assistantController.back().onFalse(arm.setClawStop());




        // Logs telemetry every time the swerve drive updates.
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // Sets the autonomous command based off of the sendable chooser 
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /** Declare autonomous commands here. */
    public Command systemsTest() {
        return 
              /** Arm */
              arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.groundPostpickupPosition, 0.01)
                 .andThen(arm.setWristHorizontal())
                 .andThen(arm.setWristVertical())
                 .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, Constants.ArmConstants.ShoulderConstants.minSafeValue))
                 
                 /** Elevator */
                 // NEED TO ADD L1 ONCE WE IMPLEMENT IT
                 .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l2Position, 0.1))
                 .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l3Position, 0.1))
                 .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l4Position, 0.1))

                 /** Combined Arm, Elevator, Wrist */
                 // NEED TO ADD L1 ONCE WE IMPLEMENT IT
                 .andThen(coralSystem.setCoralSystemL2())
                 .andThen(coralSystem.setCoralSystemL3())
                 .andThen(coralSystem.setCoralSystemL4())
                 
                 /** Climber */
                 // NEED TO ADD ACTUAL VALUES AND THRESHOLDS
                 //  .andThen(climber.setClimberDown())
                 //  .until(() -> { return climber.getClimberPosition() < Constants.ClimberConstants.downClimberPosition && climber.getClimberPosition() > Constants.ClimberConstants.downClimberPosition; })
                 //  .andThen(climber.setClimberStow())
                 //  .until(() -> { return climber.getClimberPosition() < Constants.ClimberConstants.stowClimberPosition && climber.getClimberPosition() > Constants.ClimberConstants.stowClimberPosition; })
                 /** Drivetrain */
                 .andThen(
                    drivetrain.applyRequest(() ->
                    drive.withVelocityX(-1) // Drive forward with negative Y (forward)
                         .withVelocityY(0) // Drive left with negative X (left)
                         .withRotationalRate(0) // Drive counterclockwise with negative X (left)
                    ))
                 .withTimeout(5)
                 .andThen(
                    drivetrain.applyRequest(() ->
                    drive.withVelocityX(0) // Drive forward with negative Y (forward)
                         .withVelocityY(-1) // Drive left with negative X (left)
                         .withRotationalRate(0) // Drive counterclockwise with negative X (left)
                    ))
                 .withTimeout(5);

    }

    public Command aimAtTarget() {
        double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        return drivetrain.applyRequest(() -> 
                drive.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(Math.toRadians(PhotonVision.frontTargetYaw) * -1 * MaxAngularRate)
        );
    }

    public Command moveToTarget() {
        double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        return drivetrain.applyRequest(() ->
            driveRobotCentric.withVelocityX(-1 * MaxSpeed * PhotonVision.frontTargetRangeX)
                .withVelocityY(-1 * MaxSpeed * PhotonVision.frontTargetRangeY)
                .withRotationalRate(0)
        );
    }

    // Move to subsystems eventually
    public Command limelightAimAtTarget() {
        double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        return drivetrain.applyRequest(() -> 
                drive.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate((Math.PI - Math.toRadians(limelight.x/360)) * -1 * MaxAngularRate)
        );  
    }

    public Command limelightMoveToTargetLeft() {
        double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        return drivetrain.applyRequest(() -> 
                driveRobotCentric.withVelocityX(0)
                    .withVelocityY(-1 * MaxSpeed * (limelight.y/360 - 15.5575))
                    .withRotationalRate(0)
        );  
    }

    public Command limelightMoveToTargetRight() {
        double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        return drivetrain.applyRequest(() -> 
                driveRobotCentric.withVelocityX(0)
                    .withVelocityY(-1 * MaxSpeed * (limelight.y/360 + 15.5575))
                    .withRotationalRate(0)
        );  
    }
 
    public Command oneMeterAuto() {
        return new PathPlannerAuto("OneMeterAuto");
    }
    public Command twoMeterAuto() {
        return new PathPlannerAuto("TwoMeterAuto");
    }
    public Command ninetyDegreesAuto() {
        return new PathPlannerAuto("NinetyDegreesAuto");
    }
    public Command plusAuto() {
        return new PathPlannerAuto("PlusAuto");
    }

    public Command oneMeter() {
        return new PathPlannerAuto("OneMeter");
    }

    public void cancelAllActiveCommands() {
        CommandScheduler.getInstance().cancelAll();
    }
}
