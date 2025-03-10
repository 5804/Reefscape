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
  /** Declare variables and assign values */
  public TalonFX leftElevatorMotor = new TalonFX(Constants.ElevatorConstants.leftElevatorMotorID);
  public TalonFX rightElevatorMotor = new TalonFX(Constants.ElevatorConstants.rightElevatorMotorID);

  /** Motor config objects */
  public TalonFXConfiguration elevatorTalonFXConfigs = new TalonFXConfiguration();
  public Slot0Configs elevatorSlot0FXConfigs = elevatorTalonFXConfigs.Slot0;
  public MotionMagicConfigs elevatorMotionMagicFXConfigs = elevatorTalonFXConfigs.MotionMagic;
  public MotorOutputConfigs elevatorMotorOutputFXConfigs = elevatorTalonFXConfigs.MotorOutput;
  public SoftwareLimitSwitchConfigs elevatorSoftwareLimitSwitchFXConfigs = elevatorTalonFXConfigs.SoftwareLimitSwitch;
  public CurrentLimitsConfigs elevatorMotorCurrentLimitsConfigs = elevatorTalonFXConfigs.CurrentLimits;


  double elevatorPos = 0;

  public Elevator() {
    /** Configure objects here */
    /**
     * Sets rightElevator motor to follow all commands applied to leftElevatorMotor
     * and says if rightElevatorMotor should be inverted relative to
     * leftElevatorMotor, so all commands should be run on leftElevatorMotor.
     */
    rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), false));

    leftElevatorMotor.setPosition(0);
    rightElevatorMotor.setPosition(0);
    
    /** Set slot 0 gains */
    elevatorSlot0FXConfigs.kS = Constants.ElevatorConstants.kS; // Add 0.25 V output to overcome static friction
    elevatorSlot0FXConfigs.kV = Constants.ElevatorConstants.kV; // A velocity target of 1 rps results in 0.12 V output
    elevatorSlot0FXConfigs.kA = Constants.ElevatorConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
    elevatorSlot0FXConfigs.kP = Constants.ElevatorConstants.kP; // A position error of 2.5 rotations results in 12 V output
    elevatorSlot0FXConfigs.kI = Constants.ElevatorConstants.kI; // no output for integrated error
    elevatorSlot0FXConfigs.kD = Constants.ElevatorConstants.kD; // A velocity error of 1 rps results in 0.1 V output

    /** Elevator Soft Limits */
    elevatorSoftwareLimitSwitchFXConfigs.ForwardSoftLimitThreshold = 0;
    elevatorSoftwareLimitSwitchFXConfigs.ReverseSoftLimitThreshold = -38;

    /** Set Motion Magic settings */
    elevatorMotionMagicFXConfigs.MotionMagicCruiseVelocity = Constants.ElevatorConstants.cruiseVelocity; // Target cruise velocity of 80 rps
    elevatorMotionMagicFXConfigs.MotionMagicAcceleration = Constants.ElevatorConstants.acceleration; // Target acceleration of 160 rps/s (0.5 seconds)
    elevatorMotionMagicFXConfigs.MotionMagicJerk = Constants.ElevatorConstants.jerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

    /** Set elevator motor output configs */
    elevatorMotorOutputFXConfigs.Inverted = InvertedValue.Clockwise_Positive;
    elevatorMotorOutputFXConfigs.NeutralMode = NeutralModeValue.Brake;

    /** Set software limit switches */
    // elevatorSoftwareLimitSwitchFXConfigs.ForwardSoftLimitEnable = true; // NEED TO TEST
    // elevatorSoftwareLimitSwitchFXConfigs.ForwardSoftLimitThreshold = ; // NEED TO TEST
    // elevatorSoftwareLimitSwitchFXConfigs.ReverseSoftLimitEnable = true; // NEED TO TEST
    // elevatorSoftwareLimitSwitchFXConfigs.ReverseSoftLimitThreshold = -37.65; // NEED TO TEST // -31

    /** Set elevator current limit configs */
    elevatorMotorCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
    elevatorMotorCurrentLimitsConfigs.SupplyCurrentLimit = 70; // 70
    elevatorMotorCurrentLimitsConfigs.StatorCurrentLimit = 120;

    /** Applies motor configs */
    leftElevatorMotor.getConfigurator().apply(elevatorTalonFXConfigs);
    rightElevatorMotor.getConfigurator().apply(elevatorTalonFXConfigs);
  }

  /**
   * Sets elevator positions based on the leftElevatorMotor's encoder position.
   * Should be used with Constants.ElevatorConstants.xElevatorPosition.
   */
  public Command setElevatorPosition(double position, double tolerance) {
    MotionMagicVoltage request = new MotionMagicVoltage(0);
    elevatorPos = position;
    return run(() -> { leftElevatorMotor.setControl(request.withPosition(position)); })
          .until(() -> { return Math.abs(getElevatorPosition() - position) < tolerance; });
  }

  public Command moveElevatorUp() {
    MotionMagicVoltage request = new MotionMagicVoltage(0);
    return run(() -> { 
      leftElevatorMotor.setVoltage(-1);
    })
    .finallyDo(() -> { 
      leftElevatorMotor.setControl(request.withPosition(leftElevatorMotor.getPosition().getValueAsDouble()));
    });
    // .finallyDo(
    //   () -> {
    //     leftElevatorMotor.setControl(request.withPosition(leftElevatorMotor.getPosition().getValueAsDouble())); }
    //   );
  }

  public Command moveElevatorDown() {
    MotionMagicVoltage request = new MotionMagicVoltage(0);

    return run(() -> { 
      leftElevatorMotor.setVoltage(1);
    })
    .finallyDo(() -> { 
      leftElevatorMotor.setControl(request.withPosition(leftElevatorMotor.getPosition().getValueAsDouble()));
    });
  }

  public double getCurrentPos() { 
    return elevatorPos; 
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
