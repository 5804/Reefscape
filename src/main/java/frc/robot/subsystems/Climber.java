package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
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

    public TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    /** Motor config objects */
    public Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    public MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

    public Climber(DoubleSupplier climberSup) {
        /** Configure objects here */
        /* Application of motor configs */
        leftClimberMotor.getConfigurator().apply(talonFXConfigs);
        rightClimberMotor.getConfigurator().apply(talonFXConfigs);
        leftClimberMotor.setNeutralMode(NeutralModeValue.Brake);
        rightClimberMotor.setNeutralMode(NeutralModeValue.Brake);

        /**
         * Sets rightClimber motor to follow all commands applied to leftClimberMotor
         * and says if rightClimberMotor should be inverted relative to
         * leftClimberMotor, so all commands should be run on leftClimberMotor.
         */
        rightClimberMotor.setControl(new Follower(leftClimberMotor.getDeviceID(), true));

        // Set slot 0 gains
        slot0Configs.kS = Constants.ClimberConstants.kS; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = Constants.ClimberConstants.kV; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = Constants.ClimberConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = Constants.ClimberConstants.kP; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = Constants.ClimberConstants.kI; // no output for integrated error
        slot0Configs.kD = Constants.ClimberConstants.kD; // A velocity error of 1 rps results in 0.1 V output

        // Set Motion Magic settings
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.ClimberConstants.cruiseVelocity; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = Constants.ClimberConstants.acceleration; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = Constants.ClimberConstants.jerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

        this.climberSup = climberSup;
    }

    public void setClimberPosition(double position) {
        MotionMagicVoltage request = new MotionMagicVoltage(position);
        leftClimberMotor.setControl(request.withPosition(position));
    }

    /** Methods to set the climber to preset positions. */
    public Command setClimberDown() {
        return run(() -> { setClimberPosition(Constants.ClimberConstants.downClimberPosition); });
    }

    public Command setClimberClimb() {
        return run(() -> { setClimberPosition(Constants.ClimberConstants.climbClimberPosition); });
    }

    public Command setClimberStow() {
        return run(() -> { setClimberPosition(Constants.ClimberConstants.stowClimberPosition); });
    }

    public Command setClimberSpeed() {
        return run(() -> { leftClimberMotor.set(climberSup.getAsDouble() / 5); });
    }

    public Command activateRatchets() {
        return new ParallelCommandGroup(run(() -> { leftRatchet.setPosition(1); }), run(() -> { rightRatchet.setPosition(1); }));
    }

    public Command deactivateRatchets() {
        return new ParallelCommandGroup(run(() -> { leftRatchet.setPosition(0); }), run(() -> { rightRatchet.setPosition(0); }));
    }
}