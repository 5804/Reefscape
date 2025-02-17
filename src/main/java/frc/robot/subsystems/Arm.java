// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

public TalonFX wristMotor = new TalonFX(0);
public TalonFX elbowMotor = new TalonFX(1); 
public TalonFX intakeMotor = new TalonFX(2);
public CANcoder wristEncoder = new CANcoder(3);
public CANcoder elbowEncoder = new CANcoder(4);
public TimeOfFlight timeOfFlight = new TimeOfFlight(5);


  /** Creates a new ArmSubsystem. */
  public Arm() {}

  public void wristIntakePosition() {
    wristMotor.setPosition(0);
    // horizontal coral (need to zero gyro and convert to degrees)
  }
  public void wristDropPosition() {
    wristMotor.setPosition(0);
    // vertical coral
  }

  public Command wristIntakePositionCommand() {
    return run(() -> { wristIntakePosition(); });
  }

  public Command wristDropPositionCommand() {
    return run(() -> { wristDropPosition(); });
  }


  public void elbowPosition(int position){
    elbowMotor.setPosition(position);  
  }


  public boolean hasCoral() {
    return (timeOfFlight.getRange() <= 30.00);
  }

  public void intakeCoral() {
    intakeMotor.set(1);
    // Measure direction (could be the wrong way)
  }

  public void stopIntakeMotor() {
    intakeMotor.set(0);
  }

  public Command intakeCoralCommand() {
    return run(() -> { intakeCoral(); })
          .until(() -> { return hasCoral(); })
          .finallyDo(() -> { stopIntakeMotor(); });
  }

  public void dropCoral() {
    intakeMotor.set(-1);
  }

  public Command dropCoralCommand() {
    return run(() -> { dropCoral(); })
          .finallyDo(() -> { stopIntakeMotor(); });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
