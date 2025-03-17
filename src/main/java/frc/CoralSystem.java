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
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.01)
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.hopperIntakePosition, 0.1))
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.hopperIntakePosition, 0.01));
    }

    // public Command setCoralSystemL4() {
    //     return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.002)
    //               .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l4TestPosition, 0.1))
    //               .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.l4TestPosition, 0.002));
    // }

    public Command setCoralSystemLevel(double shoulderPosition, double elevatorPosition){
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.002)
                  .andThen(elevator.setElevatorPosition(elevatorPosition, 0.1))
                  .andThen(arm.setShoulderPosition(shoulderPosition, 0.002));
    }

    public Command setCoralSystemScoreL4() {
        return elevator.setElevatorPosition(Constants.ElevatorConstants.l4SCORETestPosition, 0.1)
                       .unless(() -> (elevator.getElevatorPosition() > -22))
                       .andThen(setCoralSystemStow());
    }
    public Command scoreL4Auto() {
        return elevator.setElevatorPosition(Constants.ElevatorConstants.l4SCORETestPosition, 0.1)
                       .unless(() -> (elevator.getElevatorPosition() > -22));
    }

    // public Command setCoralSystemL3() {
    //     return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.01)
    //               .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l3Position, 0.1))
    //               .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.l3Position, 0.01));
    // }

    // public Command setCoralSystemL2() {
    //     return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.01)
    //               .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l2Position, 0.1))
    //               .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.l2Position, 0.01));
    // }

    public Command setCoralSystemL1() {
          return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.01)
                    .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l1Position, 0.1))
                    .andThen(wrist.setWristHorizontal()) 
                    .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.l1Position, 0.01));
    }

    public Command setCoralSystemGroundReady() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.01)
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.groundReadyPosition, 0.1))
                  .andThen(wrist.setWristHorizontal())
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.groundPosition, 0.01));
    }

    public Command setCoralSystemGroundPickup() {
        return claw.setClawIntake()
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.groundPickupPosition, 0.1))
                  .until(() -> { return arm.timeOfFlight.getRange() < 40 && arm.timeOfFlight.getRange() > 5; })
                  .andThen(claw.setClawStop())
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.groundPostpickupPosition, 0.01)) //groundPostpickupPosition
                  .andThen(wrist.setWristVertical())
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.hopperIntakePosition, 0.01)) //groundPostpickupPosition
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.hopperIntakePosition, 0.1));
    }

    public Command setCoralSystemHerdAlgaePosition() {
        return arm.setShoulderPosition(0.3, 0.01)
            .andThen(wrist.setWristHorizontal())
            .andThen(claw.setClawEject())
            .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.hopperIntakePosition, 0.01));
    }
    
    public Command setCoralSystemStow() {
        return climber.setClimberStow(0.01)
                      .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.03))
                      .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.hopperIntakePosition, 0.1))
                      .andThen(wrist.setWristVertical())
                      .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.hopperIntakePosition, 0.01));
    }

    public Command combinedL4() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.02)
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l4TestPosition, 0.1))
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.l4Position, 0.02))
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l4SCORETestPosition, 0.1))
                        //    .unless(() -> (elevator.getElevatorPosition() > -22))
                           .andThen(setCoralSystemStow());
    }
    public Command setAlgaeTop() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.01)
                  .andThen(elevator.setElevatorPosition(-19.23, 0.1))
                  .andThen(arm.setShoulderPosition(.192, 0.01));
    }
    public Command setAlgaeBottom() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.01)
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l2Position, 0.1))
                  .andThen(arm.setShoulderPosition(.191, 0.01));
    }
    public Command setAlgaeProcessor() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.01)
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.zeroPosition, 0.1))
                  .andThen(arm.setShoulderPosition(.23, 0.01));
    }

    public Command setBargeScore() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.01)
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.bargePlacePosition, 0.1))
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.bargePlacePosition, 0.01));
    }
}
