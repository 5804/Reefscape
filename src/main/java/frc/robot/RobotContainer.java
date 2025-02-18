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

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

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

    private final CommandXboxController joystick = new CommandXboxController(0);

    /** Subsytem initializations. */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public Arm arm = new Arm();
    public Elevator elevator = new Elevator();

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
                drive.withVelocityX(-1 * Math.pow(joystick.getLeftY(), 3) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-1 * Math.pow(joystick.getLeftX(), 3) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-1 * Math.pow(joystick.getRightX(), 3) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        /** CTRE Swerve built in controls, will probably be deleted at some point. */
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Elevator testing
        // joystick.a().whileTrue(new InstantCommand(() -> { elevator.setElevatorPosition(5); }));
        // joystick.a().whileFalse(new InstantCommand(() -> { elevator.voltageDebug(0); }));

        // Reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
     
        // Wrist position testing
        joystick.a().whileTrue(new InstantCommand(() -> { arm.setClawIntake(); }));
        joystick.a().whileFalse(new InstantCommand(() -> { arm.setClawStop(); }));
   
        // Logs telemetry every time the swerve drive updates.
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // public Command groundIntakeCommand() {
    //     return elevator.setElevatorGround()
    //                    .until(() -> { return elevator.getElevatorPosition() <= Constants.ElevatorConstants.groundElevatorPosition + 0.01; })
    //                    .andThen(arm.setElbowPosition(Constants.ArmConstants.groundElbowPosition))
    //                    .until(() -> { return arm.getElbowPosition() <= 0;})
    //                    .andThen(arm.wristIntakePositionCommand())
    //                    .until(() -> { return arm.getWristPosition() <= 0;}); // Need to change from 0!!!!! (╯°□°)╯︵ ┻━┻
    // }

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
