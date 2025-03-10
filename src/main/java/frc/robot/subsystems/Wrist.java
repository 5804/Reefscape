// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  public TalonFXS wristMotor = new TalonFXS(Constants.ArmConstants.WristConstants.motorID);
  public CANcoder wristEncoder = new CANcoder(Constants.ArmConstants.WristConstants.encoderID);

  /** Wrist motor config objects */
  public TalonFXSConfiguration wristTalonFXSConfigs = new TalonFXSConfiguration();
  public Slot0Configs wristSlot0FXSConfigs = wristTalonFXSConfigs.Slot0;
  public CommutationConfigs wristCommutationFXSConfigs = wristTalonFXSConfigs.Commutation;
  public MotionMagicConfigs wristMotionMagicFXSConfigs = wristTalonFXSConfigs.MotionMagic;
  public MotorOutputConfigs wristMotorOutputFXSConfigs = wristTalonFXSConfigs.MotorOutput;
  public ExternalFeedbackConfigs wristMotorFeedbackFXSConfigs = wristTalonFXSConfigs.ExternalFeedback;
  // public SoftwareLimitSwitchConfigs wristSoftwareLimitSwitchFXSConfigs = wristTalonFXSConfigs.SoftwareLimitSwitch; // Maybe implement

  /** Creates a new WristSubsystem. */
  public Wrist() {

    // Set wrist motor arrangement
    wristCommutationFXSConfigs.MotorArrangement = MotorArrangementValue.NEO550_JST;

    /** Set wrist slot 0 gains */
    wristSlot0FXSConfigs.kS = Constants.ArmConstants.WristConstants.kS; // Add 0.25 V output to overcome static friction
    wristSlot0FXSConfigs.kV = Constants.ArmConstants.WristConstants.kV; // A velocity target of 1 rps results in 0.12 V output
    wristSlot0FXSConfigs.kA = Constants.ArmConstants.WristConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
    wristSlot0FXSConfigs.kP = Constants.ArmConstants.WristConstants.kP; // A position error of 2.5 rotations results in 12 V output
    wristSlot0FXSConfigs.kI = Constants.ArmConstants.WristConstants.kI; // no output for integrated error
    wristSlot0FXSConfigs.kD = Constants.ArmConstants.WristConstants.kD; // A velocity error of 1 rps results in 0.1 V output

    /** Set wrist Motion Magic settings */
    wristMotionMagicFXSConfigs.MotionMagicCruiseVelocity = Constants.ArmConstants.WristConstants.cruiseVelocity; // Target cruise velocity of 80 rps
    wristMotionMagicFXSConfigs.MotionMagicAcceleration = Constants.ArmConstants.WristConstants.acceleration; // Target acceleration of 160 rps/s (0.5 seconds)
    wristMotionMagicFXSConfigs.MotionMagicJerk = Constants.ArmConstants.WristConstants.jerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

    /** Set wrist motor output configs */
    wristMotorOutputFXSConfigs.Inverted = InvertedValue.Clockwise_Positive;
    wristMotorOutputFXSConfigs.NeutralMode = NeutralModeValue.Brake;

    /** Set wrist fused encoder configs */
    wristMotorFeedbackFXSConfigs.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
    wristMotorFeedbackFXSConfigs.FeedbackRemoteSensorID = Constants.ArmConstants.WristConstants.encoderID;

    /** Applies motor configs */
    wristMotor.getConfigurator().apply(wristTalonFXSConfigs);
  }

  /** Commands to manipulate the wrist */
  public Command setWristHorizontal() {
    MotionMagicVoltage request = new MotionMagicVoltage(0);
    return run(() -> { wristMotor.setControl(request.withPosition(Constants.ArmConstants.WristConstants.horizontalPosition));})
          .until(() -> { return getWristPosition() > Constants.ArmConstants.WristConstants.horizontalPosition - 0.1; });
  }

  public Command setWristVertical() {
    MotionMagicVoltage request = new MotionMagicVoltage(0);
    return run(() -> { wristMotor.setControl(request.withPosition(Constants.ArmConstants.WristConstants.verticalPosition));})
          .until(() -> { return getWristPosition() < Constants.ArmConstants.WristConstants.verticalPosition + 0.1; });
  }
    //.0812
  public double getWristPosition() {
    return wristEncoder.getPosition().getValueAsDouble();
  }
}
