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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Climber;

public class RobotContainer {
    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);               // kSpeedAt12Volts desired top speed
    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double speedMultiplier = 1;

    public void increaseSpeedMultiplier(double value) {
        speedMultiplier += value;
    }

    private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.005).withRotationalDeadband(maxAngularRate * 0.005) // Add a 20% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);                       // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(maxSpeed * 0.005).withRotationalDeadband(maxAngularRate * 0.005) // Add a 20% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);                       // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(maxSpeed);

    public final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController assistantController = new CommandXboxController(2);
    public final ButtonBoard buttonBoard = new ButtonBoard(11, 1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public Arm arm = new Arm();
    public Claw claw = new Claw();
    public Wrist wrist = new Wrist();
    public Elevator elevator = new Elevator();
    public Climber climber = new Climber(() -> { return assistantController.getLeftY(); }); //Left Joystick
    public PhotonVision photonVision = new PhotonVision();

    private final CoralSystem coralSystem = new CoralSystem(elevator, arm, climber, claw, wrist);
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private ShuffleboardTab tab1 = Shuffleboard.getTab("Tab1");

    public RobotContainer() {
        configureBindings();

        NamedCommands.registerCommand("drop", autoLOneDrop());
        NamedCommands.registerCommand("LFourDrop", autoLFourDrop());

        NamedCommands.registerCommand("CoralAlignLeft", alignLeft().withTimeout(1.0));
        NamedCommands.registerCommand("CoralAlignRight", alignRight().withTimeout(1.0));

        NamedCommands.registerCommand("PlayerStationAlign", moveToStation(Constants.PhotonVisionConstants.backCameraID).withTimeout(2.0));
        NamedCommands.registerCommand("CensorIntake", claw.setClawIntakeWithTimeOfFlight());

        autoChooser.setDefaultOption("Default Auto", onePieceAuto());

        autoChooser.addOption("One Meter", oneMeter());
        autoChooser.addOption("One Piece", onePieceAuto());
        autoChooser.addOption("One Piece L4", LFourAuto());
        autoChooser.addOption("Left Auto", leftAuto());

        autoChooser.addOption("humantoreef", humantoreef());


        SmartDashboard.putData("Auto choices", autoChooser);
        tab1.add("Auto Chooser", autoChooser);
    }

    private void configureBindings() {

        /**
         * Note that X is defined as forward according to WPILib convention,
         * and Y is defined as to the left according to WPILib convention. 
         * As a default command the drive train will call this continually.
         */
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                driveFieldCentric.withVelocityX(-1 * Math.pow(MathUtil.applyDeadband(driveController.getLeftY(), 0.1), 3) * maxSpeed * speedMultiplier) // Drive forward with negative Y (forward)
                     .withVelocityY(-1 * Math.pow(MathUtil.applyDeadband(driveController.getLeftX(), 0.1), 3) * maxSpeed * speedMultiplier)             // Drive left with negative X (left)
                     .withRotationalRate(-1 * Math.pow(MathUtil.applyDeadband(driveController.getRightX(), 0.1), 3) * maxAngularRate * speedMultiplier) // Drive counterclockwise with negative X (left)
            )
        );

        climber.setDefaultCommand(climber.setClimberSpeed());

        driveController.leftTrigger().onTrue(coralSystem.setCoralSystemGroundReady());   
        driveController.leftTrigger().onFalse(coralSystem.setCoralSystemGroundPickup());
        driveController.leftBumper().whileTrue(alignLeft());
        driveController.rightBumper().whileTrue(alignRight());
        driveController.y().onTrue(claw.setClawIntakeWithTimeOfFlight());
        driveController.y().onFalse(claw.setClawStop());
        driveController.b().onTrue(coralSystem.setCoralSystemScoreL4());
        driveController.x().onTrue(claw.setClawIntake());
        driveController.x().onFalse(claw.setClawStop());
        driveController.a().onTrue(coralSystem.setCoralSystemStow());
        driveController.povUp().onTrue(new InstantCommand(() -> { speedMultiplier = 1; }));
        driveController.povDown().onTrue(new InstantCommand(() -> { speedMultiplier = 0.25; }));
        driveController.povLeft().onTrue(coralSystem.setAlgaeScore());
        driveController.povRight().onTrue(coralSystem.setAlgaeBottom());
        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        driveController.rightTrigger().whileTrue(moveToStation(Constants.PhotonVisionConstants.backCameraID));

        buttonBoard.getButton(4).onTrue(coralSystem.setAlgaeTop());
        buttonBoard.getButton(8).onTrue(coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l1Position, Constants.ElevatorConstants.l1Position)); 
        buttonBoard.getButton(9).onTrue(coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l2Position, Constants.ElevatorConstants.l2Position));
        buttonBoard.getButton(5).onTrue(coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l3Position, Constants.ElevatorConstants.l3Position)); 
        buttonBoard.getButton(7).onTrue(coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l4Position, Constants.ElevatorConstants.l4Position)); 
        buttonBoard.getButton(3).onTrue(coralSystem.setCoralSystemStow());
        buttonBoard.getButton(1).onTrue(claw.setClawEject());
        buttonBoard.getButton(1).onFalse(claw.setClawStop());
        buttonBoard.getButton(11).onTrue(wrist.setWristHorizontal());
        buttonBoard.getButton(6).onTrue(wrist.setWristVertical());

        Trigger buttonBoardRawAxis0Positive = new Trigger(() -> { return buttonBoard.getButtonBoard().getRawAxis(0) > 0.7; });
        Trigger buttonBoardRawAxis0Negative = new Trigger(() -> { return buttonBoard.getButtonBoard().getRawAxis(0) < -0.7; });
        Trigger buttonBoardRawAxis1Positive = new Trigger(() -> { return buttonBoard.getButtonBoard().getRawAxis(1) > 0.7; });
        Trigger buttonBoardRawAxis1Negative = new Trigger(() -> { return buttonBoard.getButtonBoard().getRawAxis(1) < -0.7; });

        buttonBoardRawAxis0Positive.whileTrue(arm.manuallyRotateShoulderUp());
        buttonBoardRawAxis0Negative.whileTrue(arm.manuallyRotateShoulderDown());
        buttonBoardRawAxis1Positive.whileTrue(elevator.moveElevatorDown());
        buttonBoardRawAxis1Negative.whileTrue(elevator.moveElevatorUp());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command systemsTest() {
        return 
              /** Arm */
              arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.groundPostpickupPosition, 0.02)
                 .andThen(() -> { try {Thread.sleep(2000);} catch (InterruptedException e) {e.printStackTrace();}})
                 .andThen(wrist.setWristHorizontal())
                 .andThen(() -> { try {Thread.sleep(2000);} catch (InterruptedException e) {e.printStackTrace();}})
                 .andThen(wrist.setWristVertical())
                 .andThen(() -> { try {Thread.sleep(2000);} catch (InterruptedException e) {e.printStackTrace();}})
                 .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.02))
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
                 .andThen(coralSystem.setCoralSystemStow());

                 /** Climber */
                 // .andThen()

                 /** DriveTrain */
                //  .andThen(drivetrain.applyRequest(() -> driveRobotCentric.withVelocityX(-1).withVelocityY(0).withRotationalRate(0)))
                //  .andThen(drivetrain.applyRequest(() -> driveRobotCentric.withVelocityX(0).withVelocityY(-1).withRotationalRate(0)));

    }

    public Command moveToReefLeft(int cameraIndex) {
        return drivetrain.applyRequest(() -> 
                driveRobotCentric
                    .withVelocityX((photonVision.bestTargetXMeters(cameraIndex) - 0.36) * 0.625 * 4)         // Needs Offset
                    .withVelocityY((photonVision.bestTargetYMeters(cameraIndex) - 0.07) * 0.625 * 4)       // Needs Offset
                    .withRotationalRate(((Math.PI - (Math.abs(photonVision.bestTargetYaw(cameraIndex)))) * Math.signum(photonVision.bestTargetYaw(cameraIndex))) * Constants.inversion * 0.75 )
        );
    }

    
    public Command moveToReefRight(int cameraIndex) {
        return drivetrain.applyRequest(() -> 
                driveRobotCentric
                    .withVelocityX((photonVision.bestTargetXMeters(cameraIndex) - 0.35) * 0.625 * 4)       // Needs Offset
                    .withVelocityY((photonVision.bestTargetYMeters(cameraIndex) + 0.10) * 0.625 * 4)       // Needs Offset
                    .withRotationalRate(((Math.PI - (Math.abs(photonVision.bestTargetYaw(cameraIndex)))) * Math.signum(photonVision.bestTargetYaw(cameraIndex))) * Constants.inversion * 0.75 )
        );
    }


    
     public Command moveToStation(int cameraIndex) {
        return drivetrain.applyRequest(() -> 
                driveRobotCentric
                    .withVelocityX((photonVision.bestTargetXMeters(cameraIndex) - 0.70) * 1.5 * -1.5) // Needs Offset
                    .withVelocityY((photonVision.bestTargetYMeters(cameraIndex) - 0.04) * 1.5 * -1.5)       // Needs Offset
                    .withRotationalRate((Math.toRadians(163.5) - Math.abs((photonVision.bestTargetYaw(cameraIndex)))) * Math.signum((Math.toRadians(163.5) - Math.abs((photonVision.bestTargetYaw(cameraIndex))))) * Constants.inversion * 0.75 )
                    // .withRotationalRate((Math.abs((Math.toRadians(163.5) - (Math.abs(photonVision.bestTargetYaw(cameraIndex))))) * Math.signum(photonVision.bestTargetYaw(cameraIndex))) * Constants.inversion * 1 )
        );
    }
     
    public Command alignLeft(){
        return moveToReefLeft(Constants.PhotonVisionConstants.rightCameraID); // Use rightCamera
    }

    public Command alignRight(){
        return moveToReefRight(Constants.PhotonVisionConstants.leftCameraID); // Use leftCamera
    }

    public Command autoLOneDrop() {
        return coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l1Position, Constants.ElevatorConstants.l1Position)
               .andThen(claw.setClawEject());
    }

    public Command autoLFourDrop() {
        return coralSystem.combinedL4();
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
    public Command humantoreef() {
        return new PathPlannerAuto("humantoreef");
    }
}
