// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
  
  private final CANSparkMax sushi = new CANSparkMax(SUSHI_ID,MotorType.kBrushless);

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  //private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  //private final Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Port.kMXP);

  /** Creates a new FeederSubsytem. */
  public FeederSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void sushiIn(){
    this.sushi.set(SUSHI_SPEED);
  }

  public void sushiStop(){
    this.sushi.set(0);
  }
}
