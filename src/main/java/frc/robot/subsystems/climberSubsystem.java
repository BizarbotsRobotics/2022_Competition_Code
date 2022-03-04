// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private final VictorSPX climberRight = new VictorSPX(CLIMBER_ID_RIGHT); 
  private final VictorSPX climberLeft = new VictorSPX(CLIMBER_ID_LEFT); 
  /** Creates a new climberSubsystem. */
  public ClimberSubsystem() {}

  @Override
  public void periodic() {

    // Lower frame rate on motor controllers
    this.climberRight.setStatusFramePeriod(1,50);
    this.climberLeft.setStatusFramePeriod(1,50);
    this.climberRight.setStatusFramePeriod(2,50);
    this.climberLeft.setStatusFramePeriod(2,50);
    
  }

  public void climberUp () {
    this.climberRight.set(ControlMode.PercentOutput, -CLIMBER_SPEED);
    this.climberLeft.set(ControlMode.PercentOutput, -CLIMBER_SPEED);
  }

  public void climberDown () {
    this.climberRight.set(ControlMode.PercentOutput, CLIMBER_SPEED);
    this.climberLeft.set(ControlMode.PercentOutput, CLIMBER_SPEED);
  }

  public void climberMove(Double upSpeed, Double downSpeed) {
    if(upSpeed < -.2) {
      this.climberRight.set(ControlMode.PercentOutput, -upSpeed);
      this.climberLeft.set(ControlMode.PercentOutput, -upSpeed);
    } else if (downSpeed < -.2) {
      this.climberRight.set(ControlMode.PercentOutput, downSpeed);
      this.climberLeft.set(ControlMode.PercentOutput, downSpeed);
    } else {
      this.climberRight.set(ControlMode.PercentOutput, 0);
      this.climberLeft.set(ControlMode.PercentOutput, 0);
    }
  }

  public void stop() {
    this.climberRight.set(ControlMode.PercentOutput, 0);
    this.climberLeft.set(ControlMode.PercentOutput, 0);
  }
}
