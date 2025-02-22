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

    public Command setCoralSystemL4Ready() {
        return null;
    }

    public Command setCoralSystemL4Eject() {
        return null;
    }

    public Command setCoralSystemL3Ready() {
        return null;
    }

    public Command setCoralSystemL3Eject() {
        return null;
    }

    public Command setCoralSystemL2Ready() {
        return null;
    }

    public Command setCoralSystemL2Eject() {
        return null;
    }

    public Command setCoralSystemL1Ready() {
        return null;
    }

    public Command setCoralSystemL1Eject() {
        return null;
    }

    public Command setCoralSystemGroundReady() {
        return arm.setShoulderPosition(Constants.Arm.Shoulder.minSafeValue)
                  .until(() -> { return arm.getShoulderPosition() < Constants.Arm.Shoulder.minSafeValue + 0.01; })
                  .andThen(elevator.setElevatorPosition(Constants.Elevator.groundReadyPosition))
                //   .until(() -> { return elevator.getElevatorPosition() < Constants.Elevator.groundReadyPosition + 0.01; })
                  .andThen(arm.setShoulderPosition(Constants.Arm.Shoulder.groundPosition));
    }

    public Command setCoralSystemGroundPickup() {
        return arm.setClawIntake()
                  .andThen(elevator.setElevatorPosition(Constants.Elevator.groundPickupPosition));
    }

    public Command setCoralSystemHopperIntake() {
        return null;
    }
}
