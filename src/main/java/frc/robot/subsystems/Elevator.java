// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  public TalonFX leftElevatorMotor  = new TalonFX(Constants.ElevatorConstants.leftElevatorMotorID);
  public TalonFX rightElevatorMotor = new TalonFX(Constants.ElevatorConstants.rightElevatorMotorID);

  public TalonFXConfiguration elevatorTalonFXConfigs = new TalonFXConfiguration();
  public Slot0Configs elevatorSlot0FXConfigs = elevatorTalonFXConfigs.Slot0;
  public MotionMagicConfigs elevatorMotionMagicFXConfigs = elevatorTalonFXConfigs.MotionMagic;
  public MotorOutputConfigs elevatorMotorOutputFXConfigs = elevatorTalonFXConfigs.MotorOutput;
  public SoftwareLimitSwitchConfigs elevatorSoftwareLimitSwitchFXConfigs = elevatorTalonFXConfigs.SoftwareLimitSwitch;
  public CurrentLimitsConfigs elevatorMotorCurrentLimitsConfigs = elevatorTalonFXConfigs.CurrentLimits;

  public Elevator() {
    rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), false));

    leftElevatorMotor.setPosition(0);
    rightElevatorMotor.setPosition(0);
    
    elevatorSlot0FXConfigs.kS = Constants.ElevatorConstants.kS; // Add 0.25 V output to overcome static friction
    elevatorSlot0FXConfigs.kV = Constants.ElevatorConstants.kV; // A velocity target of 1 rps results in 0.12 V output
    elevatorSlot0FXConfigs.kA = Constants.ElevatorConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
    elevatorSlot0FXConfigs.kP = Constants.ElevatorConstants.kP; // A position error of 2.5 rotations results in 12 V output
    elevatorSlot0FXConfigs.kI = Constants.ElevatorConstants.kI; // no output for integrated error
    elevatorSlot0FXConfigs.kD = Constants.ElevatorConstants.kD; // A velocity error of 1 rps results in 0.1 V output

    elevatorSoftwareLimitSwitchFXConfigs.ForwardSoftLimitThreshold = Constants.ElevatorConstants.forwardSoftLimitThreshold;
    elevatorSoftwareLimitSwitchFXConfigs.ReverseSoftLimitThreshold = Constants.ElevatorConstants.reverseSoftLimitThreshold;
    elevatorSoftwareLimitSwitchFXConfigs.ForwardSoftLimitEnable = true;
    elevatorSoftwareLimitSwitchFXConfigs.ReverseSoftLimitEnable = true;

    elevatorMotionMagicFXConfigs.MotionMagicCruiseVelocity = Constants.ElevatorConstants.cruiseVelocity;
    elevatorMotionMagicFXConfigs.MotionMagicAcceleration   = Constants.ElevatorConstants.acceleration;
    elevatorMotionMagicFXConfigs.MotionMagicJerk           = Constants.ElevatorConstants.jerk; 

    elevatorMotorOutputFXConfigs.Inverted    = InvertedValue.Clockwise_Positive;
    elevatorMotorOutputFXConfigs.NeutralMode = NeutralModeValue.Brake;

    elevatorMotorCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
    elevatorMotorCurrentLimitsConfigs.SupplyCurrentLimit       = Constants.ElevatorConstants.supplyCurrentLimit; 
    elevatorMotorCurrentLimitsConfigs.StatorCurrentLimit       = Constants.ElevatorConstants.statorCurrentLimit;

    leftElevatorMotor.getConfigurator().apply(elevatorTalonFXConfigs);
    rightElevatorMotor.getConfigurator().apply(elevatorTalonFXConfigs);
  }

  public Command setElevatorPosition(double position, double tolerance) {
    MotionMagicVoltage request = new MotionMagicVoltage(0);
    return run(() -> { leftElevatorMotor.setControl(request.withPosition(position)); })
          .until(() -> { return Math.abs(getElevatorPosition() - position) < tolerance; });
  }

  public Command moveElevatorUp() {
    MotionMagicVoltage request = new MotionMagicVoltage(0);
    return run(() -> { leftElevatorMotor.setVoltage(Constants.inversion * Constants.ElevatorConstants.manualTravelSpeedVoltage);})
           .finallyDo(() -> {leftElevatorMotor.setControl(request.withPosition(leftElevatorMotor.getPosition().getValueAsDouble()));});
  }

  public Command moveElevatorDown() {
    MotionMagicVoltage request = new MotionMagicVoltage(0);
    return run(() -> {leftElevatorMotor.setVoltage(Constants.ElevatorConstants.manualTravelSpeedVoltage);})
           .finallyDo(() -> {leftElevatorMotor.setControl(request.withPosition(leftElevatorMotor.getPosition().getValueAsDouble()));});
  }

  public double getElevatorPosition() {
    return leftElevatorMotor.getPosition().getValueAsDouble();
  }

  public Command zeroElevatorPosition() {
    return runOnce(() -> { leftElevatorMotor.setPosition(0); });
  }
}
