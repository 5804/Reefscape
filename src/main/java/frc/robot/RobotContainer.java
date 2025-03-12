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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.ButtonBoard;
import frc.CoralSystem;
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

    /** Controllers */
    public final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController assistantController = new CommandXboxController(2);
    public final ButtonBoard buttonBoard = new ButtonBoard(11, 1);

    /** Subsytem initializations. */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public Arm arm = new Arm();
    public Claw claw = new Claw();
    public Wrist wrist = new Wrist();
    public Elevator elevator = new Elevator();
    public Climber climber = new Climber(() -> { return assistantController.getLeftY(); });
    public PhotonVision photonVision = new PhotonVision();

    // coralSystem is used to set the elevator and arm to states
    private final CoralSystem coralSystem = new CoralSystem(elevator, arm, climber, claw, wrist);

    /** Shuffleboard configurations. */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private ShuffleboardTab tab1 = Shuffleboard.getTab("Tab1");

    public RobotContainer() {
        configureBindings();

        NamedCommands.registerCommand("drop", autoLOneDrop());
        NamedCommands.registerCommand("LFourDrop", autoLFourDrop());
        NamedCommands.registerCommand("leftAlign", moveToTargetLeft());
        NamedCommands.registerCommand("rightAlign", moveToTargetRight());

        autoChooser.setDefaultOption("Default Auto", onePieceAuto());

        autoChooser.addOption("systemsTest", oneMeter());
        autoChooser.addOption("One Meter", oneMeter());
        autoChooser.addOption("One Piece Auto", onePieceAuto());
        autoChooser.addOption("One Piece L4 Auto", LFourAuto());
        autoChooser.addOption("Left Auto", leftAuto());
        autoChooser.addOption("Drive One Meter", oneMeterTest());
        autoChooser.addOption("turn 90 degrees", ninetyDegreeTest());

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

        // REFACTOR INTO BUTTONBOARD CLASS
        Trigger buttonBoardRawAxis0Positive = new Trigger(() -> { return buttonBoard.getButtonBoard().getRawAxis(0) > 0.7; });
        Trigger buttonBoardRawAxis0Negative = new Trigger(() -> { return buttonBoard.getButtonBoard().getRawAxis(0) < -0.7; });
        Trigger buttonBoardRawAxis1Positive = new Trigger(() -> { return buttonBoard.getButtonBoard().getRawAxis(1) > 0.7; });
        Trigger buttonBoardRawAxis1Negative = new Trigger(() -> { return buttonBoard.getButtonBoard().getRawAxis(1) < -0.7; });

        buttonBoardRawAxis0Positive.whileTrue(arm.manuallyRotateShoulderUp());
        buttonBoardRawAxis0Negative.whileTrue(arm.manuallyRotateShoulderDown());
        buttonBoardRawAxis1Positive.whileTrue(elevator.moveElevatorDown());
        buttonBoardRawAxis1Negative.whileTrue(elevator.moveElevatorUp());

        /** Assistant controller bindings COMMENTED OUT ARE UNIMPLEMENTED, DON'T DELETE */
        // // assistantController.rightTrigger(0.5).onTrue(coralSystem.stowAll());
        
        // assistantController.a().onTrue(coralSystem.setCoralSystemL2());
        // assistantController.b().onTrue(coralSystem.setCoralSystemL3());
        // assistantController.y().onTrue(coralSystem.setCoralSystemL4());

        // assistantController.povRight().whileTrue(arm.armDown());
        // assistantController.povLeft().whileTrue(arm.armUp());

        // // assistantController.leftTrigger(0.5).onTrue(claw.setClawEject());
        // // assistantController.leftTrigger(0.5).onFalse(claw.setClawStop());

        // assistantController.povUp().whileTrue(elevator.moveElevatorUp());
        // assistantController.povDown().whileTrue(elevator.moveElevatorDown());

        // assistantController.rightBumper().whileTrue(wrist.setWristVertical());
        // assistantController.leftBumper().whileTrue(wrist.setWristHorizontal());
 
        // assistantController.back().whileTrue(coralSystem.setCoralSystemHerdAlgaePosition());
        // assistantController.back().onFalse(claw.setClawStop());

        // Logs telemetry every time the swerve drive updates.
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command systemsTest() {
        return 
              /** Arm */
              arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.groundPostpickupPosition, 0.01)
                 .andThen(wrist.setWristHorizontal())
                 .andThen(wrist.setWristVertical())
                 .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.01))
                 
                 /** Elevator */
                 .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l2Position, 0.1))
                 .withTimeout(Constants.AutonomousConstants.systemTestDelay)
                 .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l3Position, 0.1))
                 .withTimeout(Constants.AutonomousConstants.systemTestDelay)
                 .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l4Position, 0.1))
                 .withTimeout(Constants.AutonomousConstants.systemTestDelay)

                 /** Coral System (Combined Subsystems) */
                 .andThen(coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l1Position, Constants.ElevatorConstants.l1Position))
                 .withTimeout(Constants.AutonomousConstants.systemTestDelay)
                 .andThen(coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l2Position, Constants.ElevatorConstants.l2Position))
                 .withTimeout(Constants.AutonomousConstants.systemTestDelay)
                 .andThen(coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l3Position, Constants.ElevatorConstants.l3Position))
                 .withTimeout(Constants.AutonomousConstants.systemTestDelay)
                 .andThen(coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l4Position, Constants.ElevatorConstants.l4Position))
                 .withTimeout(Constants.AutonomousConstants.systemTestDelay)
                 .andThen(coralSystem.setCoralSystemStow())
                 .withTimeout(Constants.AutonomousConstants.systemTestDelay)

                 /** Climber */
                 // .andThen()

                 /** DriveTrain */
                 .andThen(drivetrain.applyRequest(() -> driveRobotCentric.withVelocityX(-1).withVelocityY(0).withRotationalRate(0)))
                 .withTimeout(Constants.AutonomousConstants.systemTestDelay)
                 .andThen(drivetrain.applyRequest(() -> driveRobotCentric.withVelocityX(0).withVelocityY(-1).withRotationalRate(0)))
                 .withTimeout(Constants.AutonomousConstants.systemTestDelay);

    }

    public int getDesiredAngleForAprilId(int id){
        int desiredAngle = 0;
        
        desiredAngle = (id == 17 || id == 6)  ? -120 : desiredAngle;
        desiredAngle = (id == 18 || id == 7)  ? 180 : desiredAngle;
        desiredAngle = (id == 19 || id == 8)  ? 120 : desiredAngle;
        desiredAngle = (id == 20 || id == 9)  ? -60 : desiredAngle;
        desiredAngle = (id == 21 || id == 10) ? 0 : desiredAngle;
        desiredAngle = (id == 22 || id == 11) ? 60 : desiredAngle;

        return desiredAngle;
    }


    public Command moveToTargetRight() {
        return drivetrain.applyRequest(() -> 
                driveRobotCentric.withVelocityX((photonVision.bestTargetXMeters(0) - 0.6))
                    .withVelocityY((photonVision.bestTargetYMeters(0) - 0.19) * 4)
                    .withRotationalRate(0)
        );
    }

    public Command moveToTargetLeft() {
        return drivetrain.applyRequest(() -> 
                driveRobotCentric.withVelocityX((photonVision.bestTargetXMeters(0) - 0.6))
                    .withVelocityY((photonVision.bestTargetYMeters(0) + 0.19) * 4)
                    .withRotationalRate(0)
        );
    }

    public Command rotateParallelToReefTarget() {
        return drivetrain.applyRequest(() ->
            driveRobotCentric.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(Math.toRadians(getDesiredAngleForAprilId(photonVision.bestTargetID(0))) * maxAngularRate)
        );
    }

    public Command alignLeft(){
        return moveToTargetLeft().andThen(rotateParallelToReefTarget());
    }

    public Command alignRight(){
        return moveToTargetRight().andThen(rotateParallelToReefTarget());
    }

    public Command autoLOneDrop() {
        return coralSystem.setCoralSystemLevel(Constants.ArmConstants.ShoulderConstants.l1Position, Constants.ElevatorConstants.l1Position)
               .andThen(claw.setClawEject());
    }

    public Command autoLFourDrop() {
        return coralSystem.combinedL4();
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

    public Command onePieceAuto() {
        return new PathPlannerAuto("OnePieceAuto");
    }

    public Command oneMeter() {
        return new PathPlannerAuto("OneMeter");
    }

    public Command oneMeterTest() {
        return new PathPlannerAuto("MeterAuto");
    }

    public Command ninetyDegreeTest() {
        return new PathPlannerAuto("DegreeAuto");
    }

    public Command LFourAuto() {
        return new PathPlannerAuto("LFourAuto");
    }

    public Command leftAuto() {
        return new PathPlannerAuto("leftAuto");
    }

    public void cancelAllActiveCommands() {
        CommandScheduler.getInstance().cancelAll();
    }
}
