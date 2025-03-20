package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private DoubleSupplier climberSup;

    public TalonFX climberMotor  = new TalonFX(Constants.ClimberConstants.climbMotorID);

    public TalonFXConfiguration climberTalonFXConfigs = new TalonFXConfiguration();
    public Slot0Configs climberSlot0Configs = climberTalonFXConfigs.Slot0;
    public MotionMagicConfigs climberMotionMagicConfigs = climberTalonFXConfigs.MotionMagic;
    public MotorOutputConfigs climberMotorOutputFXConfigs = climberTalonFXConfigs.MotorOutput;

    public Climber(DoubleSupplier climberSup) {
        climberMotor.setPosition(0);
        
        climberSlot0Configs.kS = Constants.ClimberConstants.kS; // Add 0.25 V output to overcome static friction
        climberSlot0Configs.kV = Constants.ClimberConstants.kV; // A velocity target of 1 rps results in 0.12 V output
        climberSlot0Configs.kA = Constants.ClimberConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
        climberSlot0Configs.kP = Constants.ClimberConstants.kP; // A position error of 2.5 rotations results in 12 V output
        climberSlot0Configs.kI = Constants.ClimberConstants.kI; // no output for integrated error
        climberSlot0Configs.kD = Constants.ClimberConstants.kD; // A velocity error of 1 rps results in 0.1 V output

        climberMotionMagicConfigs.MotionMagicCruiseVelocity = Constants.ClimberConstants.cruiseVelocity;
        climberMotionMagicConfigs.MotionMagicAcceleration = Constants.ClimberConstants.acceleration;
        climberMotionMagicConfigs.MotionMagicJerk = Constants.ClimberConstants.jerk;

        climberMotorOutputFXConfigs.NeutralMode = NeutralModeValue.Brake;

        climberMotor.getConfigurator().apply(climberTalonFXConfigs);
        
        this.climberSup = climberSup;
    }

    public Command setClimberPosition(double position, double tolerance) {
        MotionMagicVoltage request = new MotionMagicVoltage(0);
        return run(() -> { climberMotor.setControl(request.withPosition(position)); })
              .until(() -> { return Math.abs(getClimberPosition() - position) < tolerance; });
      }

    public Command setClimberSpeed() {
        return run(() -> { climberMotor.set(climberSup.getAsDouble()); });
    }

    public double getClimberPosition() {
        return climberMotor.getPosition().getValueAsDouble();
    }
}