package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeFeeder extends SubsystemBase {

  // Define solenoid used for moving intake up and down
  private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,0,1);
  private final VictorSPX intakeWheels = new VictorSPX(INTAKE_WHEEL_ID);
  private final CANSparkMax sushi = new CANSparkMax(SUSHI_ID,MotorType.kBrushless);
  private final CANSparkMax topConveyor = new CANSparkMax(TOP_CONVEYOR_ID, MotorType.kBrushless);

  private DriverStation.Alliance alliance = DriverStation.getAlliance();

  private final I2C.Port i2cPort = I2C.Port.kMXP;
  private final I2C.Port onboard = I2C.Port.kOnboard;
  
  private ColorSensorV3 ballTwoSensor = new ColorSensorV3(i2cPort);
  private ColorSensorV3 frontColorSensor = new ColorSensorV3(onboard);

  private static Timer conveyorTimer;

  private BallStatus currentStatus;

  public IntakeFeeder() {
     this.lowerIntake();
     this.currentStatus = BallStatus.IDLE;
     this.intakeWheels.setStatusFramePeriod(1,50);
     this.alliance = DriverStation.getAlliance();
     this.intakeWheels.setStatusFramePeriod(2,50);
     conveyorTimer = new Timer();
     conveyorTimer.reset();
  }

  @Override
  public void periodic() {
    //this.colorAutomation();
  }


  /** Puts down the intake. */
  public void lowerIntake() {
    this.intakeSolenoid.set(kForward);
  }

  /** Raises intake */
  public void raiseIntake() {
    this.intakeSolenoid.set(kReverse);
  }

  public void toggleIntake() {
    this.intakeSolenoid.toggle();
  }
  // Runs the intake wheels so they intake Cargo
  public void intakeCargo() {
    this.intakeWheels.set(ControlMode.PercentOutput, INTAKE_WHEEL_SPEED);
  }

  public void outtakeCargo() {
    this.intakeWheels.set(ControlMode.PercentOutput, -OUTTAKE_WHEEL_SPEED);
  }

  public void runIntake(double speed) {
    this.intakeWheels.set(ControlMode.PercentOutput, speed);
  }

  public void stopIntake() {
    this.intakeWheels.set(ControlMode.PercentOutput, 0);
  }

  public void sushiIn(){
    this.sushi.set(SUSHI_SPEED);
  }

  public void sushiStop(){
    this.sushi.set(0);
  }

  public void runSushi(Double speed) {
    if(Math.abs(speed) > .1) {
      this.sushi.set(speed);
    } else {
      this.sushi.set(0);
    }
  }

  public void sushiOut() {
    this.sushi.set(-SUSHI_SPEED);
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
  public void conveyorStop(){
    this.topConveyor.set(0);
  }

  public boolean getBallLoadedFront() {
    return this.frontColorSensor.getProximity() >= CONVEYOR_PROXIMITY_THRESHOLD;
  }

  public boolean getBallLoadedBack() {
    return this.ballTwoSensor.getProximity() >= CONVEYOR_PROXIMITY_THRESHOLD;
  }

  public DriverStation.Alliance getBallOneColor() {
    DriverStation.Alliance color = null;
    if(getBallLoadedFront()) {
      if(this.frontColorSensor.getRawColor().red > CONVEYOR_COLOR_THRESHOLD) {
        color = DriverStation.Alliance.Red;
      } else if(this.frontColorSensor.getRawColor().blue > CONVEYOR_COLOR_THRESHOLD) {
        color = DriverStation.Alliance.Blue;
      }
      return color;
    } else {
      return null;
    }
  }

  public DriverStation.Alliance getBallTwoColor() {
    DriverStation.Alliance color = null;
    if(getBallLoadedBack()) {
      if(this.ballTwoSensor.getRawColor().red > CONVEYOR_COLOR_THRESHOLD) {
        color = DriverStation.Alliance.Red;
      } else if(this.ballTwoSensor.getRawColor().blue > CONVEYOR_COLOR_THRESHOLD) {
        color = DriverStation.Alliance.Blue;
      }
      return color;
    } else {
      return null;
    }
  }

  public boolean isBallOneCorrectColor() {
    return this.getBallOneColor() == this.alliance; //|| this.getBallOneColor() == null;
  }

  public boolean isBallTwoCorrectColor() {
      return this.getBallTwoColor() == this.alliance; //|| this.getBallTwoColor() == null;
  }

  private enum BallStatus {
    IDLE,
    FIRST_BALL_CORRECT_NO_BACK_BALL,
    NO_FIRST_BALL_BACK_BALL_CORRECT,
    NO_FIRST_BALL_BACK_BALL_INCORRECT,
    FIRST_BALL_CORRECT_BACK_BALL_CORRECT,
    FIRST_BALL_CORRECT_BACK_BALL_INCORRECT,
    FIRST_BALL_INCORRECT_BACK_BALL_CORRECT,
    FIRST_BALL_INCORRECT_BACK_BALL_INCORRECT,
    FIRST_BALL_INCORRECT_NO_BACK_BALL

  }

  public void colorAutomation() {
    switch(this.currentStatus) {
      case IDLE:
        
        if(this.getBallOneColor() != null && this.isBallOneCorrectColor() && this.getBallLoadedFront() && !this.getBallLoadedBack()) { 
          currentStatus = BallStatus.FIRST_BALL_CORRECT_NO_BACK_BALL;
          conveyorTimer.reset();
          conveyorTimer.start();
        }
        else if(this.getBallOneColor() != null && !this.isBallOneCorrectColor() && this.getBallLoadedFront() && !this.getBallLoadedBack()) { 
          currentStatus = BallStatus.FIRST_BALL_INCORRECT_NO_BACK_BALL;
          conveyorTimer.reset();
          conveyorTimer.start();
        }
        // else if(this.isBallOneCorrectColor() && this.getBallLoadedFront() && this.getBallLoadedBack() && !this.isBallTwoCorrectColor()) {
        //   currentStatus = BallStatus.FIRST_BALL_CORRECT_BACK_BALL_INCORRECT;
        //   conveyorTimer.reset();
        //   conveyorTimer.start();
        // }
        // else if(!this.getBallLoadedFront() && this.getBallLoadedBack() && !this.isBallTwoCorrectColor()) {
        //   currentStatus = BallStatus.FIRST_BALL_CORRECT_BACK_BALL_INCORRECT;
        //   conveyorTimer.reset();
        //   conveyorTimer.start();
        // }
        else if(this.getBallOneColor() != null && this.getBallTwoColor() != null && this.getBallLoadedFront() && this.getBallLoadedBack() && !this.isBallOneCorrectColor() && this.isBallTwoCorrectColor()) {
          currentStatus = BallStatus.FIRST_BALL_INCORRECT_BACK_BALL_CORRECT;
          conveyorTimer.reset();
          conveyorTimer.start();
        }
        break;
      case FIRST_BALL_CORRECT_NO_BACK_BALL:
        this.topConveyor.set(TOP_CONVEYOR_SPEED);
        this.sushi.set(0.0);
        if(conveyorTimer.get() > .5) {
          currentStatus = BallStatus.IDLE;
          this.topConveyor.set(0.0);
          this.sushi.set(0.0);
        }
        break;
      case FIRST_BALL_INCORRECT_NO_BACK_BALL:
        this.topConveyor.set(TOP_CONVEYOR_SPEED);
        this.sushiIn();
        if(conveyorTimer.get() > 1) {
          currentStatus = BallStatus.IDLE;
          this.topConveyor.set(0.0);
          this.sushi.set(0.0);
        }
        break;
      case  FIRST_BALL_CORRECT_BACK_BALL_INCORRECT:
        this.topConveyor.set(TOP_CONVEYOR_SPEED);
        this.sushi.set(SUSHI_SPEED);
        if(conveyorTimer.get() > .5) {
          currentStatus = BallStatus.IDLE;
          this.topConveyor.set(0.0);
          this.sushi.set(0.0);
        }
        break;
      case NO_FIRST_BALL_BACK_BALL_INCORRECT:
        this.topConveyor.set(TOP_CONVEYOR_SPEED);
        this.sushi.set(SUSHI_SPEED);
        if(conveyorTimer.get() > .5) {
          currentStatus = BallStatus.IDLE;
          this.topConveyor.set(0.0);
          this.sushi.set(0.0);
        }
        break;
      case FIRST_BALL_INCORRECT_BACK_BALL_CORRECT:
        this.topConveyor.set(-TOP_CONVEYOR_SPEED);
        this.outtakeCargo();
        if(conveyorTimer.get() > .5) {
          currentStatus = BallStatus.IDLE;
          this.topConveyor.set(0.0);
          this.sushi.set(0.0);
          this.stopIntake();
        }
        break;
      default:
        this.currentStatus = BallStatus.IDLE;
        break;
    }
  }
}