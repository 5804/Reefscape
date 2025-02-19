// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Type;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Declare variables and assign values */
  public TalonFX leftElevatorMotor = new TalonFX(52);
  public TalonFX rightElevatorMotor = new TalonFX(51);

  /** Motor config objects */
  public TalonFXConfiguration elevatorTalonFXConfigs = new TalonFXConfiguration();
  public Slot0Configs elevatorSlot0FXConfigs = elevatorTalonFXConfigs.Slot0;
  public MotionMagicConfigs elevatorMotionMagicFXConfigs = elevatorTalonFXConfigs.MotionMagic;
  public MotorOutputConfigs elevatorMotorOutputFXConfigs = elevatorTalonFXConfigs.MotorOutput;
  public SoftwareLimitSwitchConfigs elevatorSoftwareLimitSwitchFXConfigs = elevatorTalonFXConfigs.SoftwareLimitSwitch;

  public Elevator() {
    /** Configure objects here */
    /**
     * Sets rightElevator motor to follow all commands applied to leftElevatorMotor
     * and says if rightElevatorMotor should be inverted relative to
     * leftElevatorMotor, so all commands should be run on leftElevatorMotor.
     */
    rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), false));

    /** Set slot 0 gains */
    elevatorSlot0FXConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    elevatorSlot0FXConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    elevatorSlot0FXConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    elevatorSlot0FXConfigs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    elevatorSlot0FXConfigs.kI = 0; // no output for integrated error
    elevatorSlot0FXConfigs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    /** Set Motion Magic settings */
    elevatorMotionMagicFXConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    elevatorMotionMagicFXConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    elevatorMotionMagicFXConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    /** Set elevaotr motor output configs */
    elevatorMotorOutputFXConfigs.Inverted = InvertedValue.Clockwise_Positive;
    elevatorMotorOutputFXConfigs.NeutralMode = NeutralModeValue.Brake;

    /** Set software limit switches */
    elevatorSoftwareLimitSwitchFXConfigs.ForwardSoftLimitEnable = true;
    elevatorSoftwareLimitSwitchFXConfigs.ForwardSoftLimitThreshold = 37.942383 - Constants.ElevatorConstants.softwareLimitSafetyThreshold;
    // softwareLimitSwitch.ReverseSoftLimitEnable = true; // NEED TO TEST
    // softwareLimitSwitch.ReverseSoftLimitThreshold = 0; // NEED TO TEST

    /** Applies motor configs */
    leftElevatorMotor.getConfigurator().apply(elevatorTalonFXConfigs);
    rightElevatorMotor.getConfigurator().apply(elevatorTalonFXConfigs);
  }

  /**
   * Sets elevator positions based on the leftElevatorMotor's encoder position.
   * Should be used with Constants.ElevatorConstants.xElevatorPosition.
   */
  public Command setElevatorPosition(double position) { // Max pos: 37.942383
    MotionMagicVoltage request = new MotionMagicVoltage(position);
    return run(() -> { leftElevatorMotor.setControl(request.withPosition(position)); });
  }

  /**
   * Sets elevator positions in inches (still based on the left motor's encoder).
   * Should be used with Constants.ElevatorConstants.xElevatorHeightInches.
   */
  // DO NOT USE YET, NEED TO ADD CONVERSION FACTOR
  public Command setElevatorHeightInches(double height) {
    double position = height /* ADD CONVERSION HERE */;
    MotionMagicVoltage request = new MotionMagicVoltage(position);
    return run(() -> { leftElevatorMotor.setControl(request.withPosition(position)); });
  }

  /** 
   * Gets the position of the elevator based off of leftElevatorMotor, 
   * the leader to rightElevatorMotor. 
   */
  public double getElevatorPosition() {
    return leftElevatorMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
