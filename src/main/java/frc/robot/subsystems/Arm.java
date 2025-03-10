// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  public TalonFX shoulderMotor = new TalonFX(Constants.ArmConstants.ShoulderConstants.motorID);
  public CANcoder shoulderEncoder = new CANcoder(Constants.ArmConstants.ShoulderConstants.encoderID);
  public TimeOfFlight timeOfFlight = new TimeOfFlight(1);

  /** Motor config objects */
  /** Shoulder motor config objects */
  public TalonFXConfiguration shoulderTalonFXConfigs = new TalonFXConfiguration();
  public Slot0Configs shoulderSlot0FXConfigs = shoulderTalonFXConfigs.Slot0;
  public MotionMagicConfigs shoulderMotionMagicFXConfigs = shoulderTalonFXConfigs.MotionMagic;
  public MotorOutputConfigs shoulderMotorOutputFXConfigs = shoulderTalonFXConfigs.MotorOutput;
  public FeedbackConfigs shoulderMotorFeedbackFXConfigs = shoulderTalonFXConfigs.Feedback;
  // public SoftwareLimitSwitchConfigs shoulderSoftwareLimitSwitchFXConfigs = shoulderTalonFXConfigs.SoftwareLimitSwitch; // Maybe implement
  // public VoltageConfigs shoulderMotorVoltageFXConfigs = shoulderTalonFXConfigs.Voltage;
  public CurrentLimitsConfigs shoulderMotorCurrentLimitsConfigs = shoulderTalonFXConfigs.CurrentLimits;

  double armPos = 0;

  /** Creates a new ArmSubsystem. */
  public Arm() {

    // Sets time of flight sensor ranging mode to short
    timeOfFlight.setRangingMode(RangingMode.Long, 24);

    /** Set shoulder slot 0 gains */
    shoulderSlot0FXConfigs.kS = Constants.ArmConstants.ShoulderConstants.kS; // Add 0.25 V output to overcome static friction
    shoulderSlot0FXConfigs.kV = Constants.ArmConstants.ShoulderConstants.kV; // A velocity target of 1 rps results in 0.12 V output
    shoulderSlot0FXConfigs.kA = Constants.ArmConstants.ShoulderConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
    shoulderSlot0FXConfigs.kP = Constants.ArmConstants.ShoulderConstants.kP; // A position error of 2.5 rotations results in 12 V output // 4.8
    shoulderSlot0FXConfigs.kI = Constants.ArmConstants.ShoulderConstants.kI; // no output for integrated error
    shoulderSlot0FXConfigs.kD = Constants.ArmConstants.ShoulderConstants.kD; // A velocity error of 1 rps results in 0.1 V output
    shoulderSlot0FXConfigs.kG = Constants.ArmConstants.ShoulderConstants.kG;
    shoulderSlot0FXConfigs.GravityType = GravityTypeValue.Arm_Cosine;    
    /** Set Motion Magic settings */
    /** Set shoulder Motion Magic settings */
    shoulderMotionMagicFXConfigs.MotionMagicCruiseVelocity = Constants.ArmConstants.ShoulderConstants.cruiseVelocity; // Target cruise velocity of 80 rps
    shoulderMotionMagicFXConfigs.MotionMagicAcceleration = Constants.ArmConstants.ShoulderConstants.acceleration; // Target acceleration of 160 rps/s (0.5 seconds)
    shoulderMotionMagicFXConfigs.MotionMagicJerk = Constants.ArmConstants.ShoulderConstants.jerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

    /** Set shoulder motor output configs */
    shoulderMotorOutputFXConfigs.Inverted = InvertedValue.Clockwise_Positive;
    shoulderMotorOutputFXConfigs.NeutralMode = NeutralModeValue.Brake;

    /** Set shoulder fused encoder configs */
    shoulderMotorFeedbackFXConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    shoulderMotorFeedbackFXConfigs.FeedbackRemoteSensorID = Constants.ArmConstants.ShoulderConstants.encoderID;

    /** Set shoulder current limit configs */
    shoulderMotorCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
    shoulderMotorCurrentLimitsConfigs.SupplyCurrentLimit = 60; // 70
    shoulderMotorCurrentLimitsConfigs.StatorCurrentLimit = 120;

    /** Applies motor configs */
    shoulderMotor.getConfigurator().apply(shoulderTalonFXConfigs);
  }

  /** Commands to manipulate the shoulder */
  public Command setShoulderPosition(double position, double tolerance) {
    MotionMagicVoltage request = new MotionMagicVoltage(0); // position
    armPos = position;
    return run(() -> { shoulderMotor.setControl(request.withPosition(position)); })
          .until(() -> { return Math.abs(getShoulderPosition() - position) < tolerance; });
  }

  public double getShoulderPosition() {
    return shoulderEncoder.getPosition().getValueAsDouble();
  }

  public double getArmPos() { return armPos; }

  public Command armDown() {
    MotionMagicVoltage request = new MotionMagicVoltage(0);
    return run(() -> { 
      shoulderMotor.setVoltage(5);
    })
    .finallyDo(() -> { 
      shoulderMotor.setControl(request.withPosition(shoulderEncoder.getPosition().getValueAsDouble()));
    });
  }

  public Command armUp() {
    MotionMagicVoltage request = new MotionMagicVoltage(0);
    return run(() -> { 
      shoulderMotor.setVoltage(-5);
    })
    .finallyDo(() -> { 
      shoulderMotor.setControl(request.withPosition(shoulderEncoder.getPosition().getValueAsDouble()));
    });
  }
}
