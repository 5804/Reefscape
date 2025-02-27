// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

/** Add your docs here. */
/** TODO: Can we move the until checks into the commands we are using them with? They seem commonly repeated. */
public class CoralSystem extends SubsystemBase {
    Elevator elevator;
    Arm arm;

    public CoralSystem(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
    }

    public Command setCoralSystemHopperIntake() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.01)
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.hopperIntakePosition, 0.1))
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.hopperIntakePosition, 0.01));
    }

    public Command setCoralSystemL4() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.01)
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l4Position, 0.1))
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.l4Position, 0.01));
    }

    public Command setCoralSystemL3() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.01)
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l3Position, 0.1))
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.l3Position, 0.01));
    }

    public Command setCoralSystemL2() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.01)
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.l2Position, 0.1))
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.l2Position, 0.01));
    }

    public Command setCoralSystemL1() {
        return null;
    }

    public Command setCoralSystemGroundReady() {
        return arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.minSafeValue, 0.01)
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.groundReadyPosition, 0.1))
                  .andThen(arm.setWristHorizontal())
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.groundPosition, 0.01));
    }

    public Command setCoralSystemGroundPickup() {
        return arm.setClawIntake()
                  .andThen(elevator.setElevatorPosition(Constants.ElevatorConstants.groundPickupPosition, 0.1))
                  .until(() -> { return arm.timeOfFlight.getRange() < 40 && arm.timeOfFlight.getRange() > 5; })
                  .andThen(arm.setClawStop())
                  .andThen(arm.setShoulderPosition(Constants.ArmConstants.ShoulderConstants.groundPostpickupPosition, 0.01))
                  .andThen(arm.setWristVertical());
    }

    // public Command herdAlgaePosition() {
    //     return 
    // }
}
