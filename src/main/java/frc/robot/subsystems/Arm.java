// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

public TalonFX wristMotor = new TalonFX(57);
public TalonFX elbowMotor = new TalonFX(55); 
public TalonFX intakeMotor = new TalonFX(58);
public CANcoder wristEncoder = new CANcoder(59);
public CANcoder elbowEncoder = new CANcoder(60);
// public TimeOfFlight timeOfFlight = new TimeOfFlight(5);


  /** Creates a new ArmSubsystem. */
  public Arm() {}

  public void wristIntakePosition() {
    wristMotor.setPosition(0.252);
    // horizontal coral (need to zero gyro and convert to degrees)
  }
  public void wristDropPosition() { // change name to include handoff
    wristMotor.setPosition(0);
    // vertical coral
  }

  public Command wristIntakePositionCommand() {
    return run(() -> { wristIntakePosition(); });
  }

  public Command wristDropPositionCommand() {
    return run(() -> { wristDropPosition(); });
  }

  public double getWristPosition() {
    return wristEncoder.getPosition().getValueAsDouble();
  }

  public Command setElbowPosition(double position) {
    return run(() -> { elbowMotor.setPosition(position); });
  }

  public double getElbowPosition() {
    return elbowEncoder.getPosition().getValueAsDouble();
  }

  // public boolean hasCoral() {
  //   return (timeOfFlight.getRange() <= 30.00);
  // }

  public void intakeCoral() {
    intakeMotor.set(0.1);
    // Measure direction (could be the wrong way)
  }

  public void stopIntakeMotor() {
    intakeMotor.set(0);
  }

  // public Command intakeCoralCommand() {
  //   return run(() -> { intakeCoral(); })
  //         .until(() -> { return hasCoral(); })
  //         .finallyDo(() -> { stopIntakeMotor(); });
  // }

  public void dropCoral() {
    intakeMotor.set(-0.1);
  }

  public Command dropCoralCommand() {
    return run(() -> { dropCoral(); })
          .finallyDo(() -> { stopIntakeMotor(); });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
