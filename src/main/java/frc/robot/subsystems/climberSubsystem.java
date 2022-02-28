// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climberSubsystem extends SubsystemBase {
  private final VictorSPX climberRight = new VictorSPX(9); 
  private final VictorSPX climberLeft = new VictorSPX(16); 

  /** Creates a new climberSubsystem. */
  public climberSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climberUp () {
    this.climberRight.set(ControlMode.PercentOutput, -1);
    this.climberLeft.set(ControlMode.PercentOutput, -1);
  }

  public void climberDown () {
    this.climberRight.set(ControlMode.PercentOutput, 1);
    this.climberLeft.set(ControlMode.PercentOutput, 1);
  }

  // public void auxArm() {
  //   this.aux.
  // }

  public void stop() {
    this.climberRight.set(ControlMode.PercentOutput, 0);
    this.climberLeft.set(ControlMode.PercentOutput, 0);
  }
}
