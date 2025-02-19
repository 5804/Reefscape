// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  public TalonFX wristMotor = new TalonFX(57);
  public TalonFX elbowMotor = new TalonFX(55); 
  public TalonFX clawMotor = new TalonFX(58);
  public CANcoder wristEncoder = new CANcoder(59);
  public CANcoder elbowEncoder = new CANcoder(60);

  // Motor config objects
  public TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
  public TalonFXConfiguration wristConfigs = new TalonFXConfiguration();
  public TalonFXConfiguration elbowConfigs = new TalonFXConfiguration();
  public Slot0Configs slot0Configs = elbowConfigs.Slot0;
  public MotionMagicConfigs motionMagicConfigs = elbowConfigs.MotionMagic;
  public SoftwareLimitSwitchConfigs softwareLimitSwitch = talonFXConfigs.SoftwareLimitSwitch;
  // public MotorArrangementValue motorArrangementValue = MotorArrangementValue.NEO550_JST;
  // public MotorOutputConfigs motorOutputConfigs = talonFXConfigs.MotorOutput;

  public MotorOutputConfigs motorOutputConfigs = talonFXConfigs.MotorOutput;

  // public TimeOfFlight timeOfFlight = new TimeOfFlight(5);

  /** Creates a new ArmSubsystem. */
  public Arm() {
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

    // motorOutputConfigs.withMotorArrangementValue();
    wristConfigs.Feedback.FeedbackRemoteSensorID = wristEncoder.getDeviceID();
    wristConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    elbowConfigs.Feedback.FeedbackRemoteSensorID = elbowEncoder.getDeviceID();
    elbowConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    
    // var slot0Configs = talonFXConfigs.Slot0;
    // slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    // slot0Configs.kV = 7; // 7 
    // slot0Configs.kP = 80; // 50 // 70 
    // slot0Configs.kI = 0; // 0 
    // slot0Configs.kD = 2; // 3.375 // 2 
    // slot0Configs.kS = 0; // 0 
    // Applies motor configs
    wristMotor.getConfigurator().apply(wristConfigs);
    elbowMotor.getConfigurator().apply(elbowConfigs);
    clawMotor.getConfigurator().apply(talonFXConfigs);
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
    elbowMotor.setNeutralMode(NeutralModeValue.Brake);
    clawMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /** Commands to manipulate the wrist */
  public Command setWristHorizontal() {
    MotionMagicVoltage request = new MotionMagicVoltage(Constants.ArmConstants.horizontalWristPosition);
    return run(() -> { wristMotor.setControl(request.withPosition(Constants.ArmConstants.horizontalWristPosition)); });  }

  public Command setWristVertical() {
    MotionMagicVoltage request = new MotionMagicVoltage(Constants.ArmConstants.verticalWristPosition);
    return run(() -> { wristMotor.setControl(request.withPosition(Constants.ArmConstants.verticalWristPosition)); });
  }

  public double getWristPosition() {
    return wristEncoder.getPosition().getValueAsDouble();
  }

  /** Commands to manipulate the elbow */
  public Command setElbowPosition(double position) { // HELP doesn't set position??
    MotionMagicVoltage request = new MotionMagicVoltage(position);
    return run(() -> { elbowMotor.setControl(request.withPosition(position)); });
  }

  public Command setElbowStop() {
    return run(() -> { elbowMotor.set(0); });
  }

  public Command setElbowBrakeOn() {
    return run(() -> { elbowMotor.setNeutralMode(NeutralModeValue.Brake); });
  }

  public Command setElbowBrakeOff() {
    return run(() -> { elbowMotor.setNeutralMode(NeutralModeValue.Coast); });
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
