// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.ButtonBoard;
import frc.CoralSystem;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Climber;

public class RobotContainer {
    private static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);               // kSpeedAt12Volts desired top speed
    private static final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double speedMultiplier = 1;

    public static final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.005).withRotationalDeadband(maxAngularRate * 0.005) // Add a 20% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);                       // Use open-loop control for drive motors

    public static final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(maxSpeed * 0.005).withRotationalDeadband(maxAngularRate * 0.005) // Add a 20% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);                       // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(maxSpeed);

    public final CommandXboxController driveController = new CommandXboxController(0);
    public final ButtonBoard buttonBoard = new ButtonBoard(12, 1);
    public final ButtonBoard buttonBoard2 = new ButtonBoard(12, 2);
    public final Joystick joystick = new Joystick(3);
    public int triggerHeld = 0;
    

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public Arm arm = new Arm();
    public Claw claw = new Claw();
    public Wrist wrist = new Wrist();
    public Elevator elevator = new Elevator();
    public Climber climber = new Climber(() -> { return -1 * joystick.getRawAxis(1) * triggerHeld; });

    public final CoralSystem coralSystem = new CoralSystem(elevator, arm, climber, claw, wrist);
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private ShuffleboardTab tab1 = Shuffleboard.getTab("Tab1");

    public Transform3d leftCameraTransforms = new Transform3d(0.26035, 0.20657312, 0.19354292, new Rotation3d(0, 0.436332, 0));
    static final Vector<N3> leftCamStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5));
    public Transform3d rightCameraTransforms = new Transform3d(0.26035, -0.20657312, 0.19354292, new Rotation3d(0, 0.436332, 0));
    static final Vector<N3> rightCamStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5));
    public Transform3d backCameraTransforms = new Transform3d(-0.02033524, 0.14771624, 0.9484741, new Rotation3d(0, 0.558505, 0));
    static final Vector<N3> backCamStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5));
       
    public final VisionSubsystem LeftVisionSubsystem = new VisionSubsystem(drivetrain, "Left", leftCameraTransforms, leftCamStdDevs);
    public final VisionSubsystem rightVisionSubsystem = new VisionSubsystem(drivetrain, "Right", rightCameraTransforms, rightCamStdDevs);
    public final VisionSubsystem backVisionSubsystem = new VisionSubsystem(drivetrain, "Back", backCameraTransforms, backCamStdDevs);

    public RobotContainer() {
        configureBindings();

        NamedCommands.registerCommand("drop", autoLOneDrop());
        NamedCommands.registerCommand("LFourDrop", autoLFourDrop());
        NamedCommands.registerCommand("LFourUp", coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l4Position, Constants.ElevatorConstants.l4Position));
        NamedCommands.registerCommand("LFourScore", coralSystem.scoreL4Auto());
        NamedCommands.registerCommand("Stow", coralSystem.setCoralSystemStow());
        NamedCommands.registerCommand("CensorIntake", claw.setClawIntakeWithTimeOfFlight().andThen(coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l4Position, Constants.ElevatorConstants.l4Position)));
        NamedCommands.registerCommand("StopIntake", claw.setClawStop());
        NamedCommands.registerCommand("Ejectintake", claw.setClawEject());
        NamedCommands.registerCommand("IntakeTOF", claw.setClawIntakeWithTimeOfFlight());

        NamedCommands.registerCommand("CoralAlignRight", LeftVisionSubsystem.alignRight().withTimeout(1.0));
        NamedCommands.registerCommand("CoralAlignLeft", rightVisionSubsystem.alignLeft().withTimeout(1.0));
        NamedCommands.registerCommand("PlayerStationAlign", backVisionSubsystem.alignBack().withTimeout(1.0));

        autoChooser.setDefaultOption("Default Auto", oneMeter());
        autoChooser.addOption("1 Coral Center", oneCoralAuto());
        autoChooser.addOption("1 Coral Center with Algae", oneCoralWithAlgaeAuto());
        autoChooser.addOption("2 Coral Left - Error driven", leftAuto());
        autoChooser.addOption("2 Coral Right - Error driven", rightAuto());
        autoChooser.addOption("3 Coral Left", threeCoralLeft());
        autoChooser.addOption("3 Coral Right", threeCoralRight());

        SmartDashboard.putData("Auto choices", autoChooser);
        tab1.add("Auto Chooser", autoChooser);
    }

    private void configureBindings() {

        /**
         * Note that X is defined as forward according to WPILib convention,
         * and Y is defined as to the left according to WPILib convention. 
         * As a default command the drive train will call this continually.
         */
        // drivetrain.setDefaultCommand(
        //     drivetrain.applyRequest(() ->
        //         driveFieldCentric.withVelocityX(-1 * Math.pow(Math.abs(MathUtil.applyDeadband(driveController.getLeftY(), 0.1)), 3/2) * Math.signum(driveController.getLeftY()) * maxSpeed * speedMultiplier) // Drive forward with negative Y (forward)
        //              .withVelocityY(-1 * Math.pow(Math.abs(MathUtil.applyDeadband(driveController.getLeftX(), 0.1)), 3/2) * Math.signum(driveController.getLeftX()) * maxSpeed * speedMultiplier)             // Drive left with negative X (left)
        //              .withRotationalRate(-1 * Math.pow(Math.abs(MathUtil.applyDeadband(driveController.getRightX(), 0.05)), 3/2) * Math.signum(driveController.getRightX()) * maxAngularRate * speedMultiplier) // Drive counterclockwise with negative X (left)
        //     )
        // );
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                driveFieldCentric.withVelocityX(-1 * Math.pow(MathUtil.applyDeadband(driveController.getLeftY(), 0.1), 3) * maxSpeed * speedMultiplier) // Drive forward with negative Y (forward)
                     .withVelocityY(-1 * Math.pow(MathUtil.applyDeadband(driveController.getLeftX(), 0.1), 3) * maxSpeed * speedMultiplier)             // Drive left with negative X (left)
                     .withRotationalRate(-1 * Math.pow(MathUtil.applyDeadband(driveController.getRightX(), 0.05), 3) * maxAngularRate * speedMultiplier) // Drive counterclockwise with negative X (left)
            )
        );

        climber.setDefaultCommand(climber.setClimberSpeed());

        // USB Controller
        driveController.leftBumper().onTrue(coralSystem.setCoralSystemGroundReady());   
        driveController.leftBumper().onFalse(coralSystem.setCoralSystemGroundPickup());
        driveController.leftTrigger().whileTrue(rightVisionSubsystem.alignLeft());
        driveController.rightTrigger().whileTrue(LeftVisionSubsystem.alignRight());

        driveController.y().onTrue(claw.setClawIntakeWithTimeOfFlight());
        driveController.y().onFalse(claw.setClawStop());
        driveController.b().onTrue(coralSystem.setCoralSystemScoreL4());
        driveController.x().onTrue(claw.setClawIntake());
        driveController.x().onFalse(claw.setClawStop());
        // driveController.a().onTrue(coralSystem.setCoralSystemStow());
        driveController.a().onTrue(coralSystem.setStowPositions());

        driveController.povUp().onTrue(new InstantCommand(() -> { speedMultiplier = 1; }));
        driveController.povDown().onTrue(new InstantCommand(() -> { speedMultiplier = 0.25; }));
        
        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // driveController.rightBumper().whileTrue(moveToStation(Constants.PhotonVisionConstants.backCameraID));

        // USB Button Board
        // buttonBoard.getButton(4).onTrue(coralSystem.setBargeScore());
        // buttonBoard.getButton(3).onTrue(coralSystem.setAlgaeTop());
        // buttonBoard.getButton(2).onTrue(coralSystem.setAlgaeBottom()); 
        // buttonBoard.getButton(1).onTrue(coralSystem.setAlgaeProcessor());
        
        // buttonBoard.getButton(5).onTrue(coralSystem.setCoralSystemL1()); 
        // buttonBoard.getButton(6).onTrue(coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l2Position, Constants.ElevatorConstants.l2Position));
        // buttonBoard.getButton(10).onTrue(coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l3Position, Constants.ElevatorConstants.l3Position)); 
        // buttonBoard.getButton(7).onTrue(coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l4Position, Constants.ElevatorConstants.l4Position)); 
        
        // PARALLEL COMMANDS

        buttonBoard.getButton(4).onTrue(coralSystem.setSystemPositions(Constants.ArmConstants.ShoulderConstants.bargePlacePosition, Constants.ElevatorConstants.bargePlacePosition));
        buttonBoard.getButton(3).onTrue(coralSystem.setSystemPositions(.192, -17.129));
        buttonBoard.getButton(2).onTrue(coralSystem.setSystemPositions(.191, Constants.ElevatorConstants.l2Position)); 
        buttonBoard.getButton(1).onTrue(coralSystem.setSystemPositions(.23, Constants.ElevatorConstants.zeroPosition));
        
        buttonBoard.getButton(5).onTrue(coralSystem.setTrough()); 
        buttonBoard.getButton(6).onTrue(coralSystem.setSystemPositions(Constants.ArmConstants.ShoulderConstants.l2Position, Constants.ElevatorConstants.l2Position));
        buttonBoard.getButton(10).onTrue(coralSystem.setSystemPositions(Constants.ArmConstants.ShoulderConstants.l3Position, Constants.ElevatorConstants.l3Position)); 
        buttonBoard.getButton(7).onTrue(coralSystem.setSystemPositions(Constants.ArmConstants.ShoulderConstants.l4Position, Constants.ElevatorConstants.l4Position));

        buttonBoard.getButton(8).whileTrue(wrist.moveWristHorizontal());
        buttonBoard.getButton(9).whileTrue(wrist.moveWristVertical());

        // USB Button Board 2
        buttonBoard2.getButton(1).onTrue(claw.setClawEject());
        buttonBoard2.getButton(1).onFalse(claw.setClawStop());
        buttonBoard2.getButton(2).onTrue(new InstantCommand(() -> {CommandScheduler.getInstance().cancelAll();}));
        
        Trigger buttonBoardRawAxis0Positive = new Trigger(() -> { return buttonBoard2.getButtonBoard().getRawAxis(0) > 0.7; });
        Trigger buttonBoardRawAxis0Negative = new Trigger(() -> { return buttonBoard2.getButtonBoard().getRawAxis(0) < -0.7; });
        Trigger buttonBoardRawAxis1Positive = new Trigger(() -> { return buttonBoard2.getButtonBoard().getRawAxis(1) > 0.7; });
        Trigger buttonBoardRawAxis1Negative = new Trigger(() -> { return buttonBoard2.getButtonBoard().getRawAxis(1) < -0.7; });
        Trigger joystickTrigger = new Trigger(() -> { return joystick.getTrigger(); });
        Trigger joystickButton2Trigger = new Trigger(() -> { return joystick.getRawButton(2); });

        buttonBoardRawAxis0Positive.whileTrue(arm.manuallyRotateShoulderUp());
        buttonBoardRawAxis0Negative.whileTrue(arm.manuallyRotateShoulderDown());
        buttonBoardRawAxis1Positive.whileTrue(elevator.moveElevatorDown());
        buttonBoardRawAxis1Negative.whileTrue(elevator.moveElevatorUp());

        joystickTrigger.whileTrue(new InstantCommand(() -> { this.triggerHeld=1; }));
        joystickTrigger.whileFalse(new InstantCommand(() -> { this.triggerHeld=0; }));
        joystickButton2Trigger.onTrue(elevator.zeroElevatorPosition());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command systemsTest() {
        return 
              /** Arm */
              arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.groundPostpickupPosition, Constants.ArmConstants.ShoulderConstants.tolerance)
                 .andThen(() -> { try {Thread.sleep(2000);} catch (InterruptedException e) {e.printStackTrace();}})
                 .andThen(wrist.setWristHorizontal())
                 .andThen(() -> { try {Thread.sleep(2000);} catch (InterruptedException e) {e.printStackTrace();}})
                 .andThen(wrist.setWristVertical())
                 .andThen(() -> { try {Thread.sleep(2000);} catch (InterruptedException e) {e.printStackTrace();}})
                 .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, Constants.ArmConstants.ShoulderConstants.tolerance))
                 .andThen(() -> { try {Thread.sleep(2000);} catch (InterruptedException e) {e.printStackTrace();}})

                 /** Coral System (Combined Subsystems) */
                 .andThen(coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l1Position, Constants.ElevatorConstants.l1Position))
                 .andThen(() -> { try {Thread.sleep(2000);} catch (InterruptedException e) {e.printStackTrace();}})
                 .andThen(coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l2Position, Constants.ElevatorConstants.l2Position))
                 .andThen(() -> { try {Thread.sleep(2000);} catch (InterruptedException e) {e.printStackTrace();}})
                 .andThen(coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l3Position, Constants.ElevatorConstants.l3Position))
                 .andThen(() -> { try {Thread.sleep(2000);} catch (InterruptedException e) {e.printStackTrace();}})
                 .andThen(coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l4Position, Constants.ElevatorConstants.l4Position))
                 .andThen(() -> { try {Thread.sleep(2000);} catch (InterruptedException e) {e.printStackTrace();}})
                 .andThen(coralSystem.setCoralSystemStow())
                 .andThen(() -> { try {Thread.sleep(2000);} catch (InterruptedException e) {e.printStackTrace();}});
    }

    public Command autoLOneDrop() {
        return coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l1Position, Constants.ElevatorConstants.l1Position)
               .andThen(claw.setClawEject());
    }

    public Command autoLFourDrop() {
        return coralSystem.combinedL4();
    }

    public Command autoLFourDropStop() {
        return coralSystem.combinedL4NoStow();
   }
 
    public Command onePieceAuto() {
        return new PathPlannerAuto("OnePieceAuto");
    }

    public Command oneMeter() {
        return new PathPlannerAuto("OneMeter");
    }

    public Command LFourAuto() {
        return new PathPlannerAuto("LFourAuto");
    }

    public Command leftAuto() {
        return new PathPlannerAuto("leftAuto");
    }

    public Command stopIfCoralHeld() {
        return claw.setClawStopInf().until(() -> {return !claw.sensorSeesCoral();});
    }

    public Command oneCoralAuto() {
        // return new PathPlannerAuto("OneCoralAuto");
        return new SequentialCommandGroup(
        new WaitCommand(5),
        LeftVisionSubsystem.alignRight().withTimeout(2.0), 
        autoLFourDrop());
        }

    public Command oneCoralWithAlgaeAuto() {
        return 
        new SequentialCommandGroup(
        new WaitCommand(5),
        LeftVisionSubsystem.alignRight().withTimeout(2.0), 
        autoLFourDropStop(),
        stopIfCoralHeld(),
        coralSystem.setAlgaeBottom(),
        LeftVisionSubsystem.alignAlgae().withTimeout(2.0),
        claw.setClawIntakeWithTimeOfFlight(),
        claw.setClawIntake().withTimeout(1),
        claw.setClawStop(),
        drivetrain.applyRequest(() -> driveRobotCentric
        .withVelocityX(0)
        .withVelocityY(-2)
        .withRotationalRate(0)).withTimeout(0.5)
        );

        // new SequentialCommandGroup(
        //     stopIfCoralHeld(),
        //     claw.setClawIntake().withTimeout(3));

    }

    public Command rightAuto() {
        return new PathPlannerAuto("RightAuto");
    }
    
    public Command threeCoralLeft() {
        return new PathPlannerAuto("threeCoralLeft");
    }
    public Command threeCoralRight() {
        return new PathPlannerAuto("threeCoralRight");
    }
    
}
