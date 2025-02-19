package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    /** Declare variables and assign values */
    public TalonFX leftClimberMotor = new TalonFX(0); // NEED TO ID
    public TalonFX rightClimberMotor = new TalonFX(0); // NEED TO ID

    public TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration(); // PROBABLY DOESN'T, BUT MIGHT NEED TO BE
                                                                             // ASSIGNED IN CONSTRUCTOR

    /** Motor config objects */
    public Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    public MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

    public Climber() {
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
    }

    public void setClimberPosition(double position) {
        MotionMagicVoltage request = new MotionMagicVoltage(position);
        leftClimberMotor.setControl(request.withPosition(position));
    }

    /**
     * Methods to set the climber to preset positions.
     */
    public void setClimberDown() {
        setClimberPosition(Constants.ClimberConstants.downClimberPosition);
    }

    public void setClimberClimb() {
        setClimberPosition(Constants.ClimberConstants.climbClimberPosition);
    }

    public void setClimberStow() {
        setClimberPosition(Constants.ClimberConstants.stowClimberPosition);
    }
}