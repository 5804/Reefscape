// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

/** Add your docs here. */
public class CoralSystem extends SubsystemBase {
    Elevator elevator;
    Arm arm;

    public CoralSystem(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
    }

    public Command setCoralSystemGroundReady() {
        return arm.setShoulderPosition(Constants.ArmConstants.shoulderMinSafeValue)
                  .until(() -> { return arm.getShoulderPosition() < Constants.ArmConstants.shoulderMinSafeValue + 0.01; })
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.groundElevatorReadyPosition))
                  .until(() -> { return elevator.getElevatorPosition() < Constants.ElevatorConstants.groundElevatorReadyPosition + 0.01; })
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.groundShoulderPosition));
    }

    // public Command setCoralSystemPickup() {
    //     return arm.setClawIntake()
    //               .andThen(elevator.setElevatorPosition(0));
    // }
}
