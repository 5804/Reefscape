package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private DoubleSupplier climberSup;

    public TalonFX leftClimberMotor  = new TalonFX(Constants.ClimberConstants.leftMotorID);
    public TalonFX rightClimberMotor = new TalonFX(Constants.ClimberConstants.rightMotorID);

    public PWM leftRatchet  = new PWM(Constants.ClimberConstants.leftRachetPWMChannel);
    public PWM rightRatchet = new PWM(Constants.ClimberConstants.rightRachetPWMChannel);

    public TalonFXConfiguration climberTalonFXConfigs = new TalonFXConfiguration();
    public Slot0Configs climberSlot0Configs = climberTalonFXConfigs.Slot0;
    public MotionMagicConfigs climberMotionMagicConfigs = climberTalonFXConfigs.MotionMagic;
    public MotorOutputConfigs climberMotorOutputFXConfigs = climberTalonFXConfigs.MotorOutput;

    public Climber(DoubleSupplier climberSup) {
        leftClimberMotor.setPosition(0);
        rightClimberMotor.setPosition(0);
        leftClimberMotor.setNeutralMode(NeutralModeValue.Brake);
        rightClimberMotor.setNeutralMode(NeutralModeValue.Brake);

        climberSlot0Configs.kS = Constants.ClimberConstants.kS; // Add 0.25 V output to overcome static friction
        climberSlot0Configs.kV = Constants.ClimberConstants.kV; // A velocity target of 1 rps results in 0.12 V output
        climberSlot0Configs.kA = Constants.ClimberConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
        climberSlot0Configs.kP = Constants.ClimberConstants.kP; // A position error of 2.5 rotations results in 12 V output
        climberSlot0Configs.kI = Constants.ClimberConstants.kI; // no output for integrated error
        climberSlot0Configs.kD = Constants.ClimberConstants.kD; // A velocity error of 1 rps results in 0.1 V output

        climberMotionMagicConfigs.MotionMagicCruiseVelocity = Constants.ClimberConstants.cruiseVelocity;
        climberMotionMagicConfigs.MotionMagicAcceleration = Constants.ClimberConstants.acceleration;
        climberMotionMagicConfigs.MotionMagicJerk = Constants.ClimberConstants.jerk;

        leftClimberMotor.getConfigurator().apply(climberTalonFXConfigs);
        this.climberSup = climberSup;
    }

    public void setClimberPosition(double position) {
        MotionMagicVoltage request = new MotionMagicVoltage(0);
        leftClimberMotor.setControl(request.withPosition(position));
    }

    public Command setClimberDown(double tolerance) {
        return run(() -> { setClimberPosition(Constants.ClimberConstants.downClimberPosition); })
                .until(() -> { return Math.abs(getClimberPosition() - Constants.ClimberConstants.downClimberPosition) < tolerance; });
    }

    public Command setClimberClimb(double tolerance) {
        return run(() -> { setClimberPosition(Constants.ClimberConstants.climbClimberPosition); })
                .until(() -> { return Math.abs(getClimberPosition() - Constants.ClimberConstants.stowClimberPosition) < tolerance; });
    }

    public Command setClimberStow(double tolerance) {
        return run(() -> { setClimberPosition(Constants.ClimberConstants.stowClimberPosition); })
                .until(() -> { return Math.abs(getClimberPosition() - Constants.ClimberConstants.stowClimberPosition) < tolerance; });
    }

    public Command setClimberSpeed() {
        return run(() -> { leftClimberMotor.set(climberSup.getAsDouble()); });
    }

    public double getClimberPosition() {
        return leftClimberMotor.getPosition().getValueAsDouble();
    }

    public Command activateRatchets() {
        return new ParallelCommandGroup(run(() -> { leftRatchet.setPosition(1); }), run(() -> { rightRatchet.setPosition(1); }));
    }

    public Command deactivateRatchets() {
        return new ParallelCommandGroup(run(() -> { leftRatchet.setPosition(0); }), run(() -> { rightRatchet.setPosition(0); }));
    }
}