// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  public TalonFXS clawMotor = new TalonFXS(Constants.ArmConstants.ClawConstants.motorID);
  public TimeOfFlight timeOfFlight = new TimeOfFlight(1);

  public TalonFXSConfiguration clawTalonFXSConfigs = new TalonFXSConfiguration();
  public Slot0Configs clawSlot0FXSConfigs = clawTalonFXSConfigs.Slot0;
  public CommutationConfigs clawCommutationFXSConfigs = clawTalonFXSConfigs.Commutation;
  public MotionMagicConfigs clawMotionMagicFXSConfigs = clawTalonFXSConfigs.MotionMagic;
  public MotorOutputConfigs clawMotorOutputFXSConfigs = clawTalonFXSConfigs.MotorOutput;

  public Claw() {
    timeOfFlight.setRangingMode(RangingMode.Long, 30);
    clawCommutationFXSConfigs.MotorArrangement = MotorArrangementValue.NEO550_JST;

    clawSlot0FXSConfigs.kS = Constants.ArmConstants.ClawConstants.kS; // Add 0.25 V output to overcome static friction
    clawSlot0FXSConfigs.kV = Constants.ArmConstants.ClawConstants.kV; // A velocity target of 1 rps results in 0.12 V output
    clawSlot0FXSConfigs.kA = Constants.ArmConstants.ClawConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
    clawSlot0FXSConfigs.kP = Constants.ArmConstants.ClawConstants.kP; // A position error of 2.5 rotations results in 12 V output
    clawSlot0FXSConfigs.kI = Constants.ArmConstants.ClawConstants.kI; // no output for integrated error
    clawSlot0FXSConfigs.kD = Constants.ArmConstants.ClawConstants.kD; // A velocity error of 1 rps results in 0.1 V output

    clawMotionMagicFXSConfigs.MotionMagicCruiseVelocity = Constants.ArmConstants.ClawConstants.cruiseVelocity; // Target cruise velocity of 80 rps
    clawMotionMagicFXSConfigs.MotionMagicAcceleration   = Constants.ArmConstants.ClawConstants.acceleration; // Target acceleration of 160 rps/s (0.5 seconds)
    clawMotionMagicFXSConfigs.MotionMagicJerk           = Constants.ArmConstants.ClawConstants.jerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

    clawMotorOutputFXSConfigs.Inverted    = InvertedValue.Clockwise_Positive;
    clawMotorOutputFXSConfigs.NeutralMode = NeutralModeValue.Brake;

    clawMotor.getConfigurator().apply(clawTalonFXSConfigs);
  }

  public Command setClawIntake() {
    return run(() -> { clawMotor.set(Constants.ArmConstants.ClawConstants.motorIntakeSpeed); })
          .until(() -> { return getClawVelocity() > 150; });
  }

  public Command setClawIntakeForTOF() {
    return run(() -> { clawMotor.set(Constants.ArmConstants.ClawConstants.motorIntakeSpeed); });
  }

  public Command setClawStop() {
    return run(() -> { clawMotor.set(0); })
          .until(() -> { return getClawVelocity() < 40; });
  }

  public Command setClawEject() {
    return run(() -> { clawMotor.set(Constants.ArmConstants.ClawConstants.motorEjectSpeed); })
          .until(() -> { return getClawVelocity() > -150; });
  }

  // Intakes the coral, automatically stopping intaking when the coral is in the correct position.
  public Command setClawIntakeWithTimeOfFlight() {
    return setClawIntakeForTOF()
          .until(() -> { return timeOfFlight.getRange() < Constants.ArmConstants.ClawConstants.tofHasCoralUpperBound && timeOfFlight.getRange() > Constants.ArmConstants.ClawConstants.tofHasCoralLowerBound; })
          .andThen(setClawStop());          
  }

  // Ejects the coral, automatically stopping ejecting when the coral is in the correct position.
  public Command setClawEjectWithTimeOfFlight() {
    return setClawEject()
          .until(() -> { return timeOfFlight.getRange() > Constants.ArmConstants.ClawConstants.tofHasCoralUpperBound; })
          .andThen(setClawStop());          
  }

  public double getClawVelocity() {
    return clawMotor.getVelocity().getValueAsDouble();
  }
}
