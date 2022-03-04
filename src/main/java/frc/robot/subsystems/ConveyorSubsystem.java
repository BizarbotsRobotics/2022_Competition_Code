// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class ConveyorSubsystem extends SubsystemBase {

  private final CANSparkMax topConveyor = new CANSparkMax(TOP_CONVEYOR_ID, MotorType.kBrushless);
  /** Creates a new ConveyorSubsytem. */
  public ConveyorSubsystem() {}

  @Override
  public void periodic() {
    this.topConveyor.setControlFramePeriodMs(100);
    // This method will be called once per scheduler run
  }

  public void runConveyorIn(){
    this.topConveyor.set(TOP_CONVEYOR_SPEED);
  }

  public void runConveyor(Double speed) {
    if(Math.abs(speed) > .1) {
      this.topConveyor.set(speed);
    } else {
      this.topConveyor.set(0);
    }
  }
  public void stop(){
    this.topConveyor.set(0);
  }
}
