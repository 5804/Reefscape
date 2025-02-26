// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
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
import frc.ButtonBoard;
import frc.CoralSystem;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ArmConstants.ShoulderConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
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


    private final Telemetry logger = new Telemetry(maxSpeed);

    /** Controllers */
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController assistantController = new CommandXboxController(1);
    // private final ButtonBoard buttonBoard = new ButtonBoard(12, 1);

    /** Subsytem initializations. */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public Arm arm = new Arm();
    public Elevator elevator = new Elevator();
    public Climber climber = new Climber(() -> { return driveController.getLeftX(); });
    // public PhotonVision photonVision = new PhotonVision();

    // coralSystem is used to set the elevator and arm to states
    private final CoralSystem coralSystem = new CoralSystem(elevator, arm);

    /** Shuffleboard configurations. */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private ShuffleboardTab tab1 = Shuffleboard.getTab("Tab1");

    public RobotContainer() {
        configureBindings();

        autoChooser.addOption("systemsTest", systemsTest());
        autoChooser.addOption("oneMeter", oneMeterAuto());
        autoChooser.addOption("twoMeter", twoMeterAuto());
        autoChooser.addOption("ninetyDegrees", ninetyDegreesAuto());
        autoChooser.addOption("plus", plusAuto());

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

        driveController.leftBumper().onTrue(arm.setClawIntakeWithTimeOfFlight());

        driveController.rightBumper().onTrue(arm.setClawIntake());
        driveController.rightBumper().onFalse(arm.setClawStop());

        // Need to add ratchet.
        // driveController.a().onTrue(climber.setClimberDown());
        // driveController.y().onTrue(climber.setClimberClimb());


        driveController.povUp().onTrue(new InstantCommand(() -> { if (speedMultiplier <= 0.9) { speedMultiplier += 0.1; } }));
        driveController.povDown().onTrue(new InstantCommand(() -> { if (speedMultiplier > 0.1) { speedMultiplier -= 0.1; } }));

        // Reset the field-centric heading on left bumper press
        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driveController.start().onTrue(new InstantCommand(() -> { cancelAllActiveCommands(); }));

        /** buttonBoard bindings DON'T DELETE */
        // buttonBoard.getButton(1).onTrue(coralSystem.setCoralSystemL1()); 
        // buttonBoard.getButton(2).onTrue(coralSystem.setCoralSystemL2());
        // buttonBoard.getButton(3).onTrue(coralSystem.setCoralSystemL3());
        // buttonBoard.getButton(4).onTrue(coralSystem.setCoralSystemL4());
        // buttonBoard.getButton(5).onTrue(coralSystem.setCoralSystemHopperIntake());
        // buttonBoard.getButton(6).onTrue(coralSystem.setCoralSystemGroundReady());
        // buttonBoard.getButton(7).onTrue(arm.setWristVertical());
        // buttonBoard.getButton(8).onTrue(arm.setWristHorizontal());
        // buttonBoard.getButton(9).onTrue(climber.deactivateRatchets());
        // buttonBoard.getButton(10).onTrue(); // Unused
        // buttonBoard.getButton(11).onTrue(new ParallelCommandGroup(coralSystem.setCoralSystemHopperIntake(), climber.setClimberStow()));
        // buttonBoard.getButton(12).onTrue(arm.setClawEjectWithTimeOfFlight());

        /** Assistant controller bindings COMMENTED OUT ARE UNIMPLEMENTED, DON'T DELETE */
        // assistantController.a().onTrue(coralSystem.setCoralSystemL1()); 
        assistantController.b().onTrue(coralSystem.setCoralSystemL2());
        assistantController.x().onTrue(coralSystem.setCoralSystemL3());
        assistantController.y().onTrue(coralSystem.setCoralSystemL4());
        // assistantController.povUp().onTrue(coralSystem.setCoralSystemHopperIntake());1234567

        // assistantController.povDown().onTrue(coralSystem.setCoralSystemGroundReady());
        // assistantController.povUp().onTrue(coralSystem.setCoralSystemGroundPickup());
        
        assistantController.povRight().onTrue(arm.setWristVertical());
        assistantController.povLeft().onTrue(arm.setWristHorizontal());
        // assistantController.start().onTrue(climber.deactivateRatchets());
        // assistantController.getButton().onTrue(); // Unused
        assistantController.rightTrigger(0.5).onTrue(new ParallelCommandGroup(coralSystem.setCoralSystemHopperIntake()/*, climber.setClimberStow()*/));
        assistantController.leftTrigger(0.5).onTrue(arm.setClawEjectWithTimeOfFlight());
        // assistantController.leftTrigger(0.5).onTrue(arm.setClawIntake());
        // assistantController.leftTrigger(0.5).onFalse(arm.setClawStop());

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
              arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.groundPostpickupPosition)
                 .until(() -> { return arm.getShoulderPosition() < Constants.ArmConstants.ShoulderConstants.groundPostpickupPosition + 0.01 && arm.getShoulderPosition() > Constants.ArmConstants.ShoulderConstants.groundPostpickupPosition - 0.01; })
                 .andThen(arm.setWristHorizontal())
                 .until(() -> { return arm.getWristPosition() > Constants.ArmConstants.WristConstants.horizontalPosition - 0.1; })
                 .andThen(arm.setWristVertical())
                 .until(() -> { return arm.getWristPosition() < Constants.ArmConstants.WristConstants.verticalPosition + 0.1; })
                 .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue))
                 .until(() -> { return arm.getShoulderPosition() < Constants.ArmConstants.ShoulderConstants.minSafeValue + 0.01 && arm.getShoulderPosition() > Constants.ArmConstants.ShoulderConstants.minSafeValue - 0.01; })
                 
                 /** Elevator */
                 // NEED TO ADD L1 ONCE WE IMPLEMENT IT
                 .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l2Position))
                 .until(() -> { return elevator.getElevatorPosition() < Constants.ElevatorConstants.l2Position + 0.1 && elevator.getElevatorPosition() > Constants.ElevatorConstants.l2Position - 0.1; })
                 .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l3Position))
                 .until(() -> { return elevator.getElevatorPosition() < Constants.ElevatorConstants.l3Position + 0.1 && elevator.getElevatorPosition() > Constants.ElevatorConstants.l3Position - 0.1; })
                 .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l4Position))
                 .until(() -> { return elevator.getElevatorPosition() < Constants.ElevatorConstants.l4Position + 0.1 && elevator.getElevatorPosition() > Constants.ElevatorConstants.l4Position - 0.1; })

                 /** Combined Arm, Elevator, Wrist */

                 .andThen(coralSystem.setCoralSystemL2())
                 .until(null)
                 .andThen(coralSystem.setCoralSystemL3())
                 .until(null)
                 .andThen(coralSystem.setCoralSystemL4())
                 .until(null)
                 
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

    public void cancelAllActiveCommands() {
        CommandScheduler.getInstance().cancelAll();
    }
}
