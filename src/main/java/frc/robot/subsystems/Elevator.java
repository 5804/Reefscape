// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Type;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Declare variables and assign values */
  public TalonFX leftElevatorMotor = new TalonFX(0); // NEED TO ID
  public TalonFX rightElevatorMotor = new TalonFX(0); // NEED TO ID

  public TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration(); // PROBABLY DOESN'T, BUT MIGHT NEED TO BE
                                                                           // ASSIGNED IN CONSTRUCTOR

  // Motor config objects
  public Slot0Configs slot0Configs = talonFXConfigs.Slot0;
  public MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

  /** Creates a new Elevator. */
  public Elevator() {
    /** Configure objects here */
    // Applies motor configs
    leftElevatorMotor.getConfigurator().apply(talonFXConfigs);
    rightElevatorMotor.getConfigurator().apply(talonFXConfigs);
    leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);

    /**
     * Sets rightElevator motor to follow all commands applied to leftElevatorMotor
     * and says if rightElevatorMotor should be inverted relative to
     * leftElevatorMotor, so all commands should be run on leftElevatorMotor.
     */
    rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), false));

    // Set slot 0 gains
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // Set Motion Magic settings
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

  }

  /**
   * Sets elevator positions based on the leftElevatorMotor's encoder position.
   * Should be used with Constants.ElevatorConstants.xElevatorPosition.
   */
  public void setElevatorPosition(double position) {
    MotionMagicVoltage request = new MotionMagicVoltage(position);
    leftElevatorMotor.setControl(request.withPosition(position));

  }

  /**
   * Sets elevator positions in inches (still based on the left motor's encoder).
   * Should be used with Constants.ElevatorConstants.xElevatorHeightInches.
   */
  // DO NOT USE YET, NEED TO ADD CONVERSION FACTOR
  public void setElevatorHeightInches(double height) {
    double position = height /* ADD CONVERSION HERE */;
    MotionMagicVoltage request = new MotionMagicVoltage(position);
    leftElevatorMotor.setControl(request.withPosition(position));
  }

  public void getElevatorPosition() {
    leftElevatorMotor.getPosition();
  }

  public void zeroElevatorPosition() {
    leftElevatorMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
