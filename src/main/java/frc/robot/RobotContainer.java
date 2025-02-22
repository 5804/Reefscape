// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.ButtonBoard;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PhotonVision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /** Setting up bindings for necessary control of the swerve drive platform. */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.005).withRotationalDeadband(MaxAngularRate * 0.005) // Add a 20% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final ButtonBoard buttonBoard = new ButtonBoard(12, 1);
    

    /** Subsytem initializations. */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public Arm arm = new Arm();
    public Elevator elevator = new Elevator();
    public PhotonVision photonVision = new PhotonVision();

    /** Shuffleboard configurations. */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private ShuffleboardTab tab1 = Shuffleboard.getTab("Tab1");

    public RobotContainer() {
        configureBindings();

        autoChooser.addOption("oneMeter", oneMeterAuto());
        autoChooser.addOption("twoMeter", twoMeterAuto());
        autoChooser.addOption("ninetyDegrees", ninetyDegreesAuto());
        autoChooser.addOption("plus", plusAuto());

        SmartDashboard.putData("Auto choices", autoChooser);
        tab1.add("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        /**
         * Note that X is defined as forward according to WPILib convention,
         * and Y is defined as to the left according to WPILib convention.
         */
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-1 * Math.pow(driveController.getLeftY(), 3) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-1 * Math.pow(driveController.getLeftX(), 3) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-1 * Math.pow(driveController.getRightX(), 3) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Reset the field-centric heading on left bumper press
        driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
   
        // Logs telemetry every time the swerve drive updates.
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // Sets the autonomous command based off of the sendable chooser 
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /** Declare autonomous commands here. */
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
}
