// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  public TalonFX elbowMotor = new TalonFX(Constants.ArmConstants.elbowMotorID); 
  public TalonFXS wristMotor = new TalonFXS(Constants.ArmConstants.wristMotorID);
  public TalonFXS clawMotor = new TalonFXS(Constants.ArmConstants.clawMotorID);
  public CANcoder wristEncoder = new CANcoder(Constants.ArmConstants.wristEncoderID);
  public CANcoder elbowEncoder = new CANcoder(Constants.ArmConstants.elbowEncoderID);
  // public TimeOfFlight timeOfFlight = new TimeOfFlight(5);

  /** Motor config objects */
  /** Elbow motor config objects */
  public TalonFXConfiguration elbowTalonFXConfigs = new TalonFXConfiguration();
  public Slot0Configs elbowSlot0FXConfigs = elbowTalonFXConfigs.Slot0;
  public MotionMagicConfigs elbowMotionMagicFXConfigs = elbowTalonFXConfigs.MotionMagic;
  public MotorOutputConfigs elbowMotorOutputFXConfigs = elbowTalonFXConfigs.MotorOutput;
  public FeedbackConfigs elbowMotorFeedbackFXConfigs = elbowTalonFXConfigs.Feedback;
  // public SoftwareLimitSwitchConfigs elbowSoftwareLimitSwitchFXConfigs = elbowTalonFXConfigs.SoftwareLimitSwitch; // Maybe implement

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

  /** Creates a new ArmSubsystem. */
  public Arm() {
    /** Set motor arrangements (only for TalonFXS) */
    // Set wrist motor arrangement
    wristCommutationFXSConfigs.MotorArrangement = MotorArrangementValue.NEO550_JST;

    // Set claw motor arrangement
    clawCommutationFXSConfigs.MotorArrangement = MotorArrangementValue.NEO550_JST;

    /** Set slot 0 gains */
    /** Set elbow slot 0 gains */
    elbowSlot0FXConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    elbowSlot0FXConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    elbowSlot0FXConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    elbowSlot0FXConfigs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    elbowSlot0FXConfigs.kI = 0; // no output for integrated error
    elbowSlot0FXConfigs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    /** Set wrist slot 0 gains */
    wristSlot0FXSConfigs.kS = 0.50; // Add 0.25 V output to overcome static friction
    wristSlot0FXSConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    wristSlot0FXSConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    wristSlot0FXSConfigs.kP = 40.8; // A position error of 2.5 rotations results in 12 V output
    wristSlot0FXSConfigs.kI = 0; // no output for integrated error
    wristSlot0FXSConfigs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    /** Set claw slot 0 gains */
    clawSlot0FXSConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    clawSlot0FXSConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    clawSlot0FXSConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    clawSlot0FXSConfigs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    clawSlot0FXSConfigs.kI = 0; // no output for integrated error
    clawSlot0FXSConfigs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    /** Set Motion Magic settings */
    /** Set elbow Motion Magic settings */
    elbowMotionMagicFXConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    elbowMotionMagicFXConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    elbowMotionMagicFXConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    /** Set wrist Motion Magic settings */
    wristMotionMagicFXSConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    wristMotionMagicFXSConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    wristMotionMagicFXSConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    /** Set claw Motion Magic settings */
    clawMotionMagicFXSConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    clawMotionMagicFXSConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    clawMotionMagicFXSConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    /** Set elbow motor output configs */
    elbowMotorOutputFXConfigs.Inverted = InvertedValue.Clockwise_Positive;
    elbowMotorOutputFXConfigs.NeutralMode = NeutralModeValue.Brake;

    /** Set wrist motor output configs */
    wristMotorOutputFXSConfigs.Inverted = InvertedValue.Clockwise_Positive;
    wristMotorOutputFXSConfigs.NeutralMode = NeutralModeValue.Brake;

    /** Set claw motor output configs */
    clawMotorOutputFXSConfigs.Inverted = InvertedValue.Clockwise_Positive;
    clawMotorOutputFXSConfigs.NeutralMode = NeutralModeValue.Brake;

    /** Set elbow fused encoder configs */
    elbowMotorFeedbackFXConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    elbowMotorFeedbackFXConfigs.FeedbackRemoteSensorID = Constants.ArmConstants.elbowEncoderID;

    /** Set wrist fused encoder configs */
    wristMotorFeedbackFXSConfigs.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
    wristMotorFeedbackFXSConfigs.FeedbackRemoteSensorID = Constants.ArmConstants.wristEncoderID;

    /** Applies motor configs */
    elbowMotor.getConfigurator().apply(elbowTalonFXConfigs);
    wristMotor.getConfigurator().apply(wristTalonFXSConfigs);
    clawMotor.getConfigurator().apply(clawTalonFXSConfigs);
  }

  /** Commands to manipulate the wrist */
  public Command setWristHorizontal() {
    MotionMagicVoltage request = new MotionMagicVoltage(0); // Constants.ArmConstants.horizontalWristPosition
    return run(() -> { wristMotor.setControl(request.withPosition(Constants.ArmConstants.horizontalWristPosition)); });  }

  public Command setWristVertical() {
    MotionMagicVoltage request = new MotionMagicVoltage(0);  // Constants.ArmConstants.verticalWristPosition
    return run(() -> { wristMotor.setControl(request.withPosition(Constants.ArmConstants.verticalWristPosition)); });
  }

  public double getWristPosition() {
    return wristEncoder.getPosition().getValueAsDouble();
  }

  /** Commands to manipulate the elbow */
  public Command setElbowPosition(double position) {
    MotionMagicVoltage request = new MotionMagicVoltage(0); // position
    return run(() -> { elbowMotor.setControl(request.withPosition(position)); });
  }

  public double getElbowPosition() {
    return elbowEncoder.getPosition().getValueAsDouble();
  }

  /** Commands to manipulate the claw */
  public Command setClawIntake() {
    return run(() -> { clawMotor.set(Constants.ArmConstants.clawMotorIntakeSpeed); });
  }

  public Command setClawStop() {
    return run(() -> { clawMotor.set(0); });
  }

  public Command setClawDrop() {
    return run(() -> { clawMotor.set(Constants.ArmConstants.clawMotorDropSpeed); });
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
