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

    public Command setCoralSystemStow() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue) // NEED TO SEE IF THIS COMMAND STICKS AFTER THE UNTIL
                  .until(() -> { return arm.getShoulderPosition() < Constants.ArmConstants.ShoulderConstants.minSafeValue + 0.01; })
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.stowPosition));
    }

    public Command setCoralSystemL4() {
        return null;
    }

    public Command setCoralSystemL3() {
        return null;
    }

    public Command setCoralSystemL2() {
        return null;
    }

    public Command setCoralSystemL1() {
        return null;
    }

    public Command setCoralSystemGroundReady() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue)
                  .until(() -> { return arm.getShoulderPosition() < Constants.ArmConstants.ShoulderConstants.minSafeValue + 0.01; })
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.groundReadyPosition))
                //   .until(() -> { return elevator.getElevatorPosition() < Constants.ElevatorConstants.groundReadyPosition + 0.01; })
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.groundPosition));
    }

    public Command setCoralSystemGroundPickup() {
        return arm.setClawIntake()
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.groundPickupPosition));
    }

    public Command setCoralSystemHopperIntake() {
        return null;
    }
}
