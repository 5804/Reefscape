// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

/** Add your docs here. */
/** TODO: Can we move the until checks into the commands we are using them with? They seem commonly repeated. */
public class CoralSystem extends SubsystemBase {
    Elevator elevator;
    Arm arm;
    Claw claw;
    Wrist wrist;
    Climber climber;

    public CoralSystem(Elevator elevator, Arm arm, Climber climber, Claw claw, Wrist wrist) {
        this.elevator = elevator;
        this.arm = arm;
        this.climber = climber;
        this.claw = claw;
        this.wrist = wrist;
    }

    public Command setCoralSystemHopperIntake() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, Constants.ArmConstants.ShoulderConstants.tolerance)
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.hopperIntakePosition, Constants.ElevatorConstants.tolerance))
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.hopperIntakePosition, Constants.ArmConstants.ShoulderConstants.tolerance));
    }

    // public Command setCoralSystemL4() {
    //     return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, Constants.ArmConstants.ShoulderConstants.tolerance)
    //               .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l4TestPosition, Constants.ElevatorConstants.tolerance))
    //               .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.l4TestPosition, Constants.ArmConstants.ShoulderConstants.tolerance));
    // }

    public Command setCoralSystemLevel(double shoulderPosition, double elevatorPosition){
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, Constants.ArmConstants.ShoulderConstants.tolerance)
                  .andThen(elevator.setElevatorPosition(elevatorPosition, Constants.ElevatorConstants.tolerance))
                  .andThen(arm.setShoulderPosition(shoulderPosition, Constants.ArmConstants.ShoulderConstants.tolerance));
    }

    public Command setCoralSystemScoreL4() {
        return elevator.setElevatorPosition(Constants.ElevatorConstants.l4SCORETestPosition, Constants.ElevatorConstants.tolerance)
                       .unless(() -> (elevator.getElevatorPosition() > -22))
                       .andThen(setCoralSystemStow());
    }
    public Command scoreL4Auto() {
        return elevator.setElevatorPosition(Constants.ElevatorConstants.l4SCORETestPosition, Constants.ElevatorConstants.tolerance)
                       .unless(() -> (elevator.getElevatorPosition() > -22));
    }

    // public Command setCoralSystemL3() {
    //     return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, Constants.ArmConstants.ShoulderConstants.tolerance)
    //               .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l3Position, Constants.ElevatorConstants.tolerance))
    //               .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.l3Position, Constants.ArmConstants.ShoulderConstants.tolerance));
    // }

    // public Command setCoralSystemL2() {
    //     return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, Constants.ArmConstants.ShoulderConstants.tolerance)
    //               .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l2Position, Constants.ElevatorConstants.tolerance))
    //               .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.l2Position, Constants.ArmConstants.ShoulderConstants.tolerance));
    // }

    public Command setCoralSystemL1() {
          return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, Constants.ArmConstants.ShoulderConstants.tolerance)
                    .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l1Position, Constants.ElevatorConstants.tolerance))
                    .andThen(wrist.setWristHorizontal()) 
                    .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.l1Position, Constants.ArmConstants.ShoulderConstants.tolerance));
    }

    public Command setCoralSystemGroundReady() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, Constants.ArmConstants.ShoulderConstants.tolerance)
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.groundReadyPosition, Constants.ElevatorConstants.tolerance))
                  .andThen(wrist.setWristHorizontal())
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.groundPosition, Constants.ArmConstants.ShoulderConstants.tolerance));
    }

    public Command setCoralSystemGroundPickup() {
        return claw.setClawIntake()
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.groundPickupPosition, Constants.ElevatorConstants.tolerance))
                  .until(() -> { return arm.timeOfFlight.getRange() < 40 && arm.timeOfFlight.getRange() > 5; })
                  .andThen(claw.setClawStop())
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.groundPostpickupPosition, Constants.ArmConstants.ShoulderConstants.tolerance)) //groundPostpickupPosition
                  .andThen(wrist.setWristVertical())
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.hopperIntakePosition, Constants.ArmConstants.ShoulderConstants.tolerance)) //groundPostpickupPosition
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.hopperIntakePosition, Constants.ElevatorConstants.tolerance));
    }

    public Command setCoralSystemHerdAlgaePosition() {
        return arm.setShoulderPosition(0.3, Constants.ArmConstants.ShoulderConstants.tolerance)
            .andThen(wrist.setWristHorizontal())
            .andThen(claw.setClawEject())
            .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.hopperIntakePosition, Constants.ElevatorConstants.tolerance));
    }
    
    public Command setCoralSystemStow() {
        return climber.setClimberStow(0.01)
                      .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, Constants.ArmConstants.ShoulderConstants.tolerance))
                      .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.hopperIntakePosition, Constants.ElevatorConstants.tolerance))
                      .andThen(wrist.setWristVertical())
                      .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.hopperIntakePosition, Constants.ArmConstants.ShoulderConstants.tolerance));
    }

    public Command combinedL4() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, Constants.ArmConstants.ShoulderConstants.tolerance)
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l4TestPosition, Constants.ElevatorConstants.tolerance))
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.l4Position, Constants.ArmConstants.ShoulderConstants.tolerance))
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l4SCORETestPosition, Constants.ElevatorConstants.tolerance))
                        //    .unless(() -> (elevator.getElevatorPosition() > -22))
                           .andThen(setCoralSystemStow());
    }
    public Command setAlgaeTop() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, Constants.ArmConstants.ShoulderConstants.tolerance)
                  .andThen(elevator.setElevatorPosition(-19.23, Constants.ElevatorConstants.tolerance))
                  .andThen(arm.setShoulderPosition(.192, Constants.ArmConstants.ShoulderConstants.tolerance));
    }
    public Command setAlgaeBottom() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, Constants.ArmConstants.ShoulderConstants.tolerance)
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l2Position, Constants.ElevatorConstants.tolerance))
                  .andThen(arm.setShoulderPosition(.191, Constants.ArmConstants.ShoulderConstants.tolerance));
    }
    public Command setAlgaeProcessor() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, Constants.ArmConstants.ShoulderConstants.tolerance)
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.zeroPosition, Constants.ElevatorConstants.tolerance))
                  .andThen(arm.setShoulderPosition(.23, Constants.ArmConstants.ShoulderConstants.tolerance));
    }

    public Command setBargeScore() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, Constants.ArmConstants.ShoulderConstants.tolerance)
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.bargePlacePosition, Constants.ElevatorConstants.tolerance))
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.bargePlacePosition, Constants.ArmConstants.ShoulderConstants.tolerance));
    }
}
