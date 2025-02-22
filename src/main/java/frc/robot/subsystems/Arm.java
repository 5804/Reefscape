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
  public TalonFX shoulderMotor = new TalonFX(Constants.Arm.Shoulder.motorID); 
  public TalonFXS wristMotor = new TalonFXS(Constants.Arm.Wrist.motorID);
  public TalonFXS clawMotor = new TalonFXS(Constants.Arm.Claw.motorID);
  public CANcoder wristEncoder = new CANcoder(Constants.Arm.Wrist.encoderID);
  public CANcoder shoulderEncoder = new CANcoder(Constants.Arm.Shoulder.encoderID);
  // public TimeOfFlight timeOfFlight = new TimeOfFlight(5);

  /** Motor config objects */
  /** Shoulder motor config objects */
  public TalonFXConfiguration shoulderTalonFXConfigs = new TalonFXConfiguration();
  public Slot0Configs shoulderSlot0FXConfigs = shoulderTalonFXConfigs.Slot0;
  public MotionMagicConfigs shoulderMotionMagicFXConfigs = shoulderTalonFXConfigs.MotionMagic;
  public MotorOutputConfigs shoulderMotorOutputFXConfigs = shoulderTalonFXConfigs.MotorOutput;
  public FeedbackConfigs shoulderMotorFeedbackFXConfigs = shoulderTalonFXConfigs.Feedback;
  // public SoftwareLimitSwitchConfigs shoulderSoftwareLimitSwitchFXConfigs = shoulderTalonFXConfigs.SoftwareLimitSwitch; // Maybe implement

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
    /** Set shoulder slot 0 gains */
    shoulderSlot0FXConfigs.kS = Constants.Arm.Shoulder.kS; // Add 0.25 V output to overcome static friction
    shoulderSlot0FXConfigs.kV = Constants.Arm.Shoulder.kV; // A velocity target of 1 rps results in 0.12 V output
    shoulderSlot0FXConfigs.kA = Constants.Arm.Shoulder.kA; // An acceleration of 1 rps/s requires 0.01 V output
    shoulderSlot0FXConfigs.kP = Constants.Arm.Shoulder.kP; // A position error of 2.5 rotations results in 12 V output // 4.8
    shoulderSlot0FXConfigs.kI = Constants.Arm.Shoulder.kI; // no output for integrated error
    shoulderSlot0FXConfigs.kD = Constants.Arm.Shoulder.kD; // A velocity error of 1 rps results in 0.1 V output

    /** Set wrist slot 0 gains */
    wristSlot0FXSConfigs.kS = Constants.Arm.Wrist.kS; // Add 0.25 V output to overcome static friction
    wristSlot0FXSConfigs.kV = Constants.Arm.Wrist.kV; // A velocity target of 1 rps results in 0.12 V output
    wristSlot0FXSConfigs.kA = Constants.Arm.Wrist.kA; // An acceleration of 1 rps/s requires 0.01 V output
    wristSlot0FXSConfigs.kP = Constants.Arm.Wrist.kP; // A position error of 2.5 rotations results in 12 V output
    wristSlot0FXSConfigs.kI = Constants.Arm.Wrist.kI; // no output for integrated error
    wristSlot0FXSConfigs.kD = Constants.Arm.Wrist.kD; // A velocity error of 1 rps results in 0.1 V output

    /** Set claw slot 0 gains */
    clawSlot0FXSConfigs.kS = Constants.Arm.Claw.kS; // Add 0.25 V output to overcome static friction
    clawSlot0FXSConfigs.kV = Constants.Arm.Claw.kV; // A velocity target of 1 rps results in 0.12 V output
    clawSlot0FXSConfigs.kA = Constants.Arm.Claw.kA; // An acceleration of 1 rps/s requires 0.01 V output
    clawSlot0FXSConfigs.kP = Constants.Arm.Claw.kP; // A position error of 2.5 rotations results in 12 V output
    clawSlot0FXSConfigs.kI = Constants.Arm.Claw.kI; // no output for integrated error
    clawSlot0FXSConfigs.kD = Constants.Arm.Claw.kD; // A velocity error of 1 rps results in 0.1 V output

    /** Set Motion Magic settings */
    /** Set shoulder Motion Magic settings */
    shoulderMotionMagicFXConfigs.MotionMagicCruiseVelocity = Constants.Arm.Shoulder.cruiseVelocity; // Target cruise velocity of 80 rps
    shoulderMotionMagicFXConfigs.MotionMagicAcceleration = Constants.Arm.Shoulder.acceleration; // Target acceleration of 160 rps/s (0.5 seconds)
    shoulderMotionMagicFXConfigs.MotionMagicJerk = Constants.Arm.Shoulder.jerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

    /** Set wrist Motion Magic settings */
    wristMotionMagicFXSConfigs.MotionMagicCruiseVelocity = Constants.Arm.Wrist.cruiseVelocity; // Target cruise velocity of 80 rps
    wristMotionMagicFXSConfigs.MotionMagicAcceleration = Constants.Arm.Wrist.acceleration; // Target acceleration of 160 rps/s (0.5 seconds)
    wristMotionMagicFXSConfigs.MotionMagicJerk = Constants.Arm.Wrist.jerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

    /** Set claw Motion Magic settings */
    clawMotionMagicFXSConfigs.MotionMagicCruiseVelocity = Constants.Arm.Claw.cruiseVelocity; // Target cruise velocity of 80 rps
    clawMotionMagicFXSConfigs.MotionMagicAcceleration = Constants.Arm.Claw.acceleration; // Target acceleration of 160 rps/s (0.5 seconds)
    clawMotionMagicFXSConfigs.MotionMagicJerk = Constants.Arm.Claw.jerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

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
    shoulderMotorFeedbackFXConfigs.FeedbackRemoteSensorID = Constants.Arm.Shoulder.encoderID;

    /** Set wrist fused encoder configs */
    wristMotorFeedbackFXSConfigs.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
    wristMotorFeedbackFXSConfigs.FeedbackRemoteSensorID = Constants.Arm.Wrist.encoderID;

    /** Applies motor configs */
    shoulderMotor.getConfigurator().apply(shoulderTalonFXConfigs);
    wristMotor.getConfigurator().apply(wristTalonFXSConfigs);
    clawMotor.getConfigurator().apply(clawTalonFXSConfigs);
  }

  /** Commands to manipulate the wrist */
  public Command setWristHorizontal() {
    MotionMagicVoltage request = new MotionMagicVoltage(0);
    return run(() -> { wristMotor.setControl(request.withPosition(Constants.Arm.Wrist.horizontalPosition)); });  }

  public Command setWristVertical() {
    MotionMagicVoltage request = new MotionMagicVoltage(0);
    return run(() -> { wristMotor.setControl(request.withPosition(Constants.Arm.Wrist.verticalPosition)); });
  }

  public double getWristPosition() {
    return wristEncoder.getPosition().getValueAsDouble();
  }

  /** Commands to manipulate the shoulder */
  public Command setShoulderPosition(double position) {
    MotionMagicVoltage request = new MotionMagicVoltage(0); // position
    return run(() -> { shoulderMotor.setControl(request.withPosition(position)); });
  }

  public double getShoulderPosition() {
    return shoulderEncoder.getPosition().getValueAsDouble();
  }

  /** Commands to manipulate the claw */
  public Command setClawIntake() {
    return run(() -> { clawMotor.set(Constants.Arm.Claw.motorIntakeSpeed); });
  }

  public Command setClawStop() {
    return run(() -> { clawMotor.set(0); });
  }

  public Command setClawEject() {
    return run(() -> { clawMotor.set(Constants.Arm.Claw.motorEjectSpeed); });
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
