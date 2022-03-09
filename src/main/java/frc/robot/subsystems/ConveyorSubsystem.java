// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.Constants.*;

import java.util.HashMap;

public class ConveyorSubsystem extends SubsystemBase {
  DriverStation.Alliance alliance = DriverStation.getAlliance();
  private final CANSparkMax topConveyor = new CANSparkMax(TOP_CONVEYOR_ID, MotorType.kBrushless);
  private HashMap<Integer,DriverStation.Alliance> balls = new HashMap<Integer,DriverStation.Alliance>();

  private final I2C.Port i2cPort = I2C.Port.kMXP;
  private final I2C.Port onboard = I2C.Port.kOnboard;

  // FIX ME UPDATE WITH PORT
  private ColorSensorV3 ballTwoSensor = new ColorSensorV3(i2cPort);
  private ColorSensorV3 frontColorSensor = new ColorSensorV3(onboard);

  private SparkMaxPIDController conveyorPidController;
  /** Creates a new ConveyorSubsytem. */
  public ConveyorSubsystem() {
    balls.put(1, null);
    balls.put(2, null);
    this.topConveyor.setControlFramePeriodMs(100);

    conveyorPidController = this.topConveyor.getPIDController();
    conveyorPidController.setP(CONVEYOR_kP);
    conveyorPidController.setI(CONVEYOR_kI);
    conveyorPidController.setD(CONVEYOR_kD);
    conveyorPidController.setIZone(CONVEYOR_kIz);
    conveyorPidController.setFF(CONVEYOR_kFF);
    conveyorPidController.setOutputRange(CONVEYOR_kMinOutput, CONVEYOR_kMaxOutput);

    this.alliance = DriverStation.getAlliance();
  }

  @Override
  public void periodic() {
    this.checkBallOneStatus();
    this.checkBallTwoStatus();
    this.alliance = DriverStation.getAlliance();
  }

  public void checkBallOneStatus() {
    DriverStation.Alliance color = null;
    if(getBallLoadedFront()) {
      if(this.frontColorSensor.getRawColor().red > CONVEYOR_COLOR_THRESHOLD) {
        color = DriverStation.Alliance.Red;
      } else if(this.frontColorSensor.getRawColor().blue > CONVEYOR_COLOR_THRESHOLD) {
        color = DriverStation.Alliance.Blue;
      }
      this.balls.put(1, color);
    } else {
      this.balls.put(1,null);
    }
  }

  public void checkBallTwoStatus() {
    DriverStation.Alliance color = null;
    if(getBallLoadedBack()) {
      if(this.ballTwoSensor.getRawColor().red > CONVEYOR_COLOR_THRESHOLD) {
        color = DriverStation.Alliance.Red;
      } else if(this.ballTwoSensor.getRawColor().blue > CONVEYOR_COLOR_THRESHOLD) {
        color = DriverStation.Alliance.Blue;
      }
      this.balls.put(2, color);
    } else {
      this.balls.put(2,null);
    }
  }


  public boolean getBallLoadedFront() {
    return this.frontColorSensor.getProximity() >= CONVEYOR_PROXIMITY_THRESHOLD;
  }

  public boolean getBallLoadedBack() {
    return this.ballTwoSensor.getProximity() >= CONVEYOR_PROXIMITY_THRESHOLD;
  }
  
  public void runConveyorIn(){
    this.topConveyor.set(TOP_CONVEYOR_SPEED);
  }

  public void runConveyorOut(){
    this.topConveyor.set(-TOP_CONVEYOR_SPEED);
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

  public DriverStation.Alliance getBallOneColor() {
    return this.balls.get(1);
  }

  public DriverStation.Alliance getBallTwoColor() {
    return this.balls.get(2);
  }

  public boolean isBallOneCorrectColor() {
      return this.getBallOneColor() == this.alliance || this.getBallOneColor() == null;
  }

  public boolean isBallTwoCorrectColor() {
       return this.getBallTwoColor() == this.alliance || this.getBallTwoColor() == null;
  }

  public void moveDistance(int distance) {
    this.conveyorPidController.setReference(distance, CANSparkMax.ControlType.kSmartMotion);
  }
}
