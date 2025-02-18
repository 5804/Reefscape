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
public TalonFX clawMotor = new TalonFX(58);
public CANcoder wristEncoder = new CANcoder(59);
public CANcoder elbowEncoder = new CANcoder(60);
// public TimeOfFlight timeOfFlight = new TimeOfFlight(5);


  /** Creates a new ArmSubsystem. */
  public Arm() {
    
  }

  /** Commands to manipulate the wrist */
  public Command setWristHorizontal() {
    return run(() -> { wristMotor.setPosition(0.252); });
  }

  public Command setWristVertical() {
    return run(() -> { wristMotor.setPosition(0); });
  }

  public double getWristPosition() {
    return wristEncoder.getPosition().getValueAsDouble();
  }

  /** Commands to manipulate the elbow */
  public Command setElbowPosition(double position) {
    return run(() -> { elbowMotor.setPosition(position); });
  }

  public double getElbowPosition() {
    return elbowEncoder.getPosition().getValueAsDouble();
  }

  /** Commands to manipulate the claw */
  public Command setClawIntake() {
    return run(() -> { clawMotor.set(-1); });
  }

  public Command setClawStop() {
    return run(() -> { clawMotor.set(0); });
  }

  public Command setClawDrop() {
    return run(() -> { clawMotor.set(1); });
  }

  // NEED TO FINISH THESE
  // public Command intakeCoralCommand() {
  //   return run(() -> { intakeCoral(); })
  //         .until(() -> { return hasCoral(); })
  //         .finallyDo(() -> { stopIntakeMotor(); });
  // }

// public boolean hasCoral() {
  //   return (timeOfFlight.getRange() <= 30.00);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
