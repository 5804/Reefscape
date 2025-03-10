// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  public TalonFX shoulderMotor = new TalonFX(Constants.ArmConstants.ShoulderConstants.motorID); 
  public TalonFXS wristMotor = new TalonFXS(Constants.ArmConstants.WristConstants.motorID);
  public TalonFXS clawMotor = new TalonFXS(Constants.ArmConstants.ClawConstants.motorID);
  public CANcoder wristEncoder = new CANcoder(Constants.ArmConstants.WristConstants.encoderID);
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

  /** Wrist motor config objects */
  public TalonFXSConfiguration wristTalonFXSConfigs = new TalonFXSConfiguration();
  public Slot0Configs wristSlot0FXSConfigs = wristTalonFXSConfigs.Slot0;
  public CommutationConfigs wristCommutationFXSConfigs = wristTalonFXSConfigs.Commutation;
  public MotionMagicConfigs wristMotionMagicFXSConfigs = wristTalonFXSConfigs.MotionMagic;
  public MotorOutputConfigs wristMotorOutputFXSConfigs = wristTalonFXSConfigs.MotorOutput;
  public ExternalFeedbackConfigs wristMotorFeedbackFXSConfigs = wristTalonFXSConfigs.ExternalFeedback;
  // public SoftwareLimitSwitchConfigs wristSoftwareLimitSwitchFXSConfigs = wristTalonFXSConfigs.SoftwareLimitSwitch; // Maybe implement
  

  /** Claw motor config objects */
  public TalonFXSConfiguration clawTalonFXSConfigs = new TalonFXSConfiguration();
  public Slot0Configs clawSlot0FXSConfigs = clawTalonFXSConfigs.Slot0;
  public CommutationConfigs clawCommutationFXSConfigs = clawTalonFXSConfigs.Commutation;
  public MotionMagicConfigs clawMotionMagicFXSConfigs = clawTalonFXSConfigs.MotionMagic;
  public MotorOutputConfigs clawMotorOutputFXSConfigs = clawTalonFXSConfigs.MotorOutput;

  double armPos = 0;

  /** Creates a new ArmSubsystem. */
  public Arm() {

    // Sets time of flight sensor ranging mode to short
    timeOfFlight.setRangingMode(RangingMode.Long, 30);

    /** Set motor arrangements (only for TalonFXS) */
    // Set wrist motor arrangement
    wristCommutationFXSConfigs.MotorArrangement = MotorArrangementValue.NEO550_JST;

    // Set claw motor arrangement
    clawCommutationFXSConfigs.MotorArrangement = MotorArrangementValue.NEO550_JST;

    /** Set slot 0 gains */
    /** Set shoulder slot 0 gains */
    shoulderSlot0FXConfigs.kS = Constants.ArmConstants.ShoulderConstants.kS; // Add 0.25 V output to overcome static friction
    shoulderSlot0FXConfigs.kV = Constants.ArmConstants.ShoulderConstants.kV; // A velocity target of 1 rps results in 0.12 V output
    shoulderSlot0FXConfigs.kA = Constants.ArmConstants.ShoulderConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
    shoulderSlot0FXConfigs.kP = Constants.ArmConstants.ShoulderConstants.kP; // A position error of 2.5 rotations results in 12 V output // 4.8
    shoulderSlot0FXConfigs.kI = Constants.ArmConstants.ShoulderConstants.kI; // no output for integrated error
    shoulderSlot0FXConfigs.kD = Constants.ArmConstants.ShoulderConstants.kD; // A velocity error of 1 rps results in 0.1 V output
    shoulderSlot0FXConfigs.kG = Constants.ArmConstants.ShoulderConstants.kG;
    shoulderSlot0FXConfigs.GravityType = GravityTypeValue.Arm_Cosine;    

    /** Set wrist slot 0 gains */
    wristSlot0FXSConfigs.kS = Constants.ArmConstants.WristConstants.kS; // Add 0.25 V output to overcome static friction
    wristSlot0FXSConfigs.kV = Constants.ArmConstants.WristConstants.kV; // A velocity target of 1 rps results in 0.12 V output
    wristSlot0FXSConfigs.kA = Constants.ArmConstants.WristConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
    wristSlot0FXSConfigs.kP = Constants.ArmConstants.WristConstants.kP; // A position error of 2.5 rotations results in 12 V output
    wristSlot0FXSConfigs.kI = Constants.ArmConstants.WristConstants.kI; // no output for integrated error
    wristSlot0FXSConfigs.kD = Constants.ArmConstants.WristConstants.kD; // A velocity error of 1 rps results in 0.1 V output

    /** Set claw slot 0 gains */
    clawSlot0FXSConfigs.kS = Constants.ArmConstants.ClawConstants.kS; // Add 0.25 V output to overcome static friction
    clawSlot0FXSConfigs.kV = Constants.ArmConstants.ClawConstants.kV; // A velocity target of 1 rps results in 0.12 V output
    clawSlot0FXSConfigs.kA = Constants.ArmConstants.ClawConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
    clawSlot0FXSConfigs.kP = Constants.ArmConstants.ClawConstants.kP; // A position error of 2.5 rotations results in 12 V output
    clawSlot0FXSConfigs.kI = Constants.ArmConstants.ClawConstants.kI; // no output for integrated error
    clawSlot0FXSConfigs.kD = Constants.ArmConstants.ClawConstants.kD; // A velocity error of 1 rps results in 0.1 V output

    /** Set Motion Magic settings */
    /** Set shoulder Motion Magic settings */
    shoulderMotionMagicFXConfigs.MotionMagicCruiseVelocity = Constants.ArmConstants.ShoulderConstants.cruiseVelocity; // Target cruise velocity of 80 rps
    shoulderMotionMagicFXConfigs.MotionMagicAcceleration = Constants.ArmConstants.ShoulderConstants.acceleration; // Target acceleration of 160 rps/s (0.5 seconds)
    shoulderMotionMagicFXConfigs.MotionMagicJerk = Constants.ArmConstants.ShoulderConstants.jerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

    /** Set wrist Motion Magic settings */
    wristMotionMagicFXSConfigs.MotionMagicCruiseVelocity = Constants.ArmConstants.WristConstants.cruiseVelocity; // Target cruise velocity of 80 rps
    wristMotionMagicFXSConfigs.MotionMagicAcceleration = Constants.ArmConstants.WristConstants.acceleration; // Target acceleration of 160 rps/s (0.5 seconds)
    wristMotionMagicFXSConfigs.MotionMagicJerk = Constants.ArmConstants.WristConstants.jerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

    /** Set claw Motion Magic settings */
    clawMotionMagicFXSConfigs.MotionMagicCruiseVelocity = Constants.ArmConstants.ClawConstants.cruiseVelocity; // Target cruise velocity of 80 rps
    clawMotionMagicFXSConfigs.MotionMagicAcceleration = Constants.ArmConstants.ClawConstants.acceleration; // Target acceleration of 160 rps/s (0.5 seconds)
    clawMotionMagicFXSConfigs.MotionMagicJerk = Constants.ArmConstants.ClawConstants.jerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

    /** Set shoulder motor output configs */
    shoulderMotorOutputFXConfigs.Inverted = InvertedValue.Clockwise_Positive;
    shoulderMotorOutputFXConfigs.NeutralMode = NeutralModeValue.Brake;

    /** Set wrist motor output configs */
    wristMotorOutputFXSConfigs.Inverted = InvertedValue.Clockwise_Positive;
    wristMotorOutputFXSConfigs.NeutralMode = NeutralModeValue.Brake;

    /** Set claw motor output configs */
    clawMotorOutputFXSConfigs.Inverted = InvertedValue.Clockwise_Positive;
    clawMotorOutputFXSConfigs.NeutralMode = NeutralModeValue.Brake;

    /** Set shoulder fused encoder configs */
    shoulderMotorFeedbackFXConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    shoulderMotorFeedbackFXConfigs.FeedbackRemoteSensorID = Constants.ArmConstants.ShoulderConstants.encoderID;

    /** Set wrist fused encoder configs */
    wristMotorFeedbackFXSConfigs.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
    wristMotorFeedbackFXSConfigs.FeedbackRemoteSensorID = Constants.ArmConstants.WristConstants.encoderID;

    /** Set shoulder current limit configs */
    shoulderMotorCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
    shoulderMotorCurrentLimitsConfigs.SupplyCurrentLimit = 60; // 70
    shoulderMotorCurrentLimitsConfigs.StatorCurrentLimit = 120;

    /** Applies motor configs */
    shoulderMotor.getConfigurator().apply(shoulderTalonFXConfigs);
    wristMotor.getConfigurator().apply(wristTalonFXSConfigs);
    clawMotor.getConfigurator().apply(clawTalonFXSConfigs);
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

  /** Commands to manipulate the claw */
  public Command setClawIntake() {
    return run(() -> { clawMotor.set(Constants.ArmConstants.ClawConstants.motorIntakeSpeed); })
          .until(() -> { return getClawVelocity() > 150; });
  }

  public Command setClawIntakeForTOF() {
    return run(() -> { clawMotor.set(Constants.ArmConstants.ClawConstants.motorIntakeSpeed); });
  }

  // Stops the claw from spinning.
  public Command setClawStop() {
    return run(() -> { clawMotor.set(0); })
          .until(() -> { return getClawVelocity() < 40; });
  }

  // Spins the claw backwards, immediately pops the coral out.
  public Command setClawEject() {
    return run(() -> { clawMotor.set(Constants.ArmConstants.ClawConstants.motorEjectSpeed); })
          .until(() -> { return getClawVelocity() > -150; });
  }

  // Intakes the coral, automatically stopping intaking when the coral is in the correct position.
  public Command setClawIntakeWithTimeOfFlight() {
    return setClawIntakeForTOF()
          .until(() -> { return timeOfFlight.getRange() < Constants.ArmConstants.ClawConstants.tofHasCoralUpperBound && timeOfFlight.getRange() > Constants.ArmConstants.ClawConstants.tofHasCoralLowerBound; })
          // .finallyDo(() -> { clawMotor.set(0); });
          .andThen(setClawStop());          
  }

  // Ejects the coral, automatically stopping ejecting when the coral is in the correct position.
  public Command setClawEjectWithTimeOfFlight() {
    return setClawEject()
          .until(() -> { return timeOfFlight.getRange() > Constants.ArmConstants.ClawConstants.tofHasCoralUpperBound; })
          .finallyDo(() -> { clawMotor.set(0); });           
  }

  public double getClawVelocity() {
    return clawMotor.getVelocity().getValueAsDouble();
  }

  public double getArmPos() { return armPos; }

  public Command armDown() {
    MotionMagicVoltage request = new MotionMagicVoltage(0);
    return run(() -> { 
      shoulderMotor.setVoltage(3);
    })
    .finallyDo(() -> { 
      shoulderMotor.setControl(request.withPosition(shoulderEncoder.getPosition().getValueAsDouble()));
    });
  }

  public Command armUp() {
    MotionMagicVoltage request = new MotionMagicVoltage(0);
    return run(() -> { 
      shoulderMotor.setVoltage(-3);
    })
    .finallyDo(() -> { 
      shoulderMotor.setControl(request.withPosition(shoulderEncoder.getPosition().getValueAsDouble()));
    });
  }
}
