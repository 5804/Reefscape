package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private DoubleSupplier climberSup;

    /** Declare variables and assign values */
    public TalonFX leftClimberMotor = new TalonFX(Constants.ClimberConstants.leftMotorID); // NEED TO ID
    public TalonFX rightClimberMotor = new TalonFX(Constants.ClimberConstants.rightMotorID); // NEED TO ID
    public PWM leftRatchet = new PWM(1); // NEED TO ID
    public PWM rightRatchet = new PWM(2); // NEED TO ID


    /** Motor config objects */
    // public TalonFXConfiguration elevatorTalonFXConfigs = new TalonFXConfiguration();
    // public Slot0Configs elevatorSlot0FXConfigs = elevatorTalonFXConfigs.Slot0;
    // public MotionMagicConfigs elevatorMotionMagicFXConfigs = elevatorTalonFXConfigs.MotionMagic;
    // public MotorOutputConfigs elevatorMotorOutputFXConfigs = elevatorTalonFXConfigs.MotorOutput;

    public TalonFXConfiguration climberTalonFXConfigs = new TalonFXConfiguration();
    public Slot0Configs climberSlot0Configs = climberTalonFXConfigs.Slot0;
    public MotionMagicConfigs climberMotionMagicConfigs = climberTalonFXConfigs.MotionMagic;
    public MotorOutputConfigs climberMotorOutputFXConfigs = climberTalonFXConfigs.MotorOutput;

    public Climber(DoubleSupplier climberSup) {
        /** Configure objects here */
        /* Application of motor configs */
        leftClimberMotor.setPosition(0);
        rightClimberMotor.setPosition(0);
        leftClimberMotor.setNeutralMode(NeutralModeValue.Brake);
        rightClimberMotor.setNeutralMode(NeutralModeValue.Brake);

        /**
         * Sets rightClimber motor to follow all commands applied to leftClimberMotor
         * and says if rightClimberMotor should be inverted relative to
         * leftClimberMotor, so all commands should be run on leftClimberMotor.
         */
        rightClimberMotor.setControl(new Follower(leftClimberMotor.getDeviceID(), true));

        // Set slot 0 gains
        climberSlot0Configs.kS = Constants.ClimberConstants.kS; // Add 0.25 V output to overcome static friction
        climberSlot0Configs.kV = Constants.ClimberConstants.kV; // A velocity target of 1 rps results in 0.12 V output
        climberSlot0Configs.kA = Constants.ClimberConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
        climberSlot0Configs.kP = Constants.ClimberConstants.kP; // A position error of 2.5 rotations results in 12 V output
        climberSlot0Configs.kI = Constants.ClimberConstants.kI; // no output for integrated error
        climberSlot0Configs.kD = Constants.ClimberConstants.kD; // A velocity error of 1 rps results in 0.1 V output

        // Set Motion Magic settings
        climberMotionMagicConfigs.MotionMagicCruiseVelocity = Constants.ClimberConstants.cruiseVelocity; // Target cruise velocity of 80 rps
        climberMotionMagicConfigs.MotionMagicAcceleration = Constants.ClimberConstants.acceleration; // Target acceleration of 160 rps/s (0.5 seconds)
        climberMotionMagicConfigs.MotionMagicJerk = Constants.ClimberConstants.jerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

        leftClimberMotor.getConfigurator().apply(climberTalonFXConfigs);
        rightClimberMotor.getConfigurator().apply(climberTalonFXConfigs);

        this.climberSup = climberSup;
    }

    public void setClimberPosition(double position) {
        MotionMagicVoltage request = new MotionMagicVoltage(0);
        leftClimberMotor.setControl(request.withPosition(position));
    }

    /** Methods to set the climber to preset positions. */
    public Command setClimberDown(double tolerance) {
        return run(() -> { setClimberPosition(Constants.ClimberConstants.downClimberPosition); })
                .until(() -> { return Math.abs(getClimberPosition() - Constants.ClimberConstants.stowClimberPosition) < tolerance; });
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
        return run(() -> { leftClimberMotor.set(climberSup.getAsDouble() / 5); });
    }

    public double getClimberPosition() {
        return leftClimberMotor.getPosition().getValueAsDouble();
    }

    // public Command activateRatchets() {
    //     return new ParallelCommandGroup(run(() -> { leftRatchet.setPosition(1); }), run(() -> { rightRatchet.setPosition(1); }));
    // }

    // public Command deactivateRatchets() {
    //     return new ParallelCommandGroup(run(() -> { leftRatchet.setPosition(0); }), run(() -> { rightRatchet.setPosition(0); }));
    // }
}