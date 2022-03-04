// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AuxArmSubsystem extends SubsystemBase {
  private final VictorSPX auxArm = new VictorSPX(CLIMBER_ID_AUX);
  /** Creates a new climberSubsystem. */
  public AuxArmSubsystem() {
    this.auxArm.setStatusFramePeriod(1, 50);
    this.auxArm.setStatusFramePeriod(2, 50);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void auxArmMove(Double upSpeed, Double downSpeed) {
    if(upSpeed < -.2) {
      this.auxArm.set(ControlMode.PercentOutput, -upSpeed);
    } else if (downSpeed < -.2) {
      this.auxArm.set(ControlMode.PercentOutput, downSpeed);
    } else {
      this.auxArm.set(ControlMode.PercentOutput, 0);
    }
  }

  public void AuxArmUp() {
    this.auxArm.set(ControlMode.PercentOutput, AUX_ARM_SPEED);
  }

  public void AuxArmDown() {
    this.auxArm.set(ControlMode.PercentOutput, -AUX_ARM_SPEED);
  }

  public void stopAuxArm() {
    this.auxArm.set(ControlMode.PercentOutput, 0);
  }
}
