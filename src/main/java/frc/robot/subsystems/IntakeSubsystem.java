package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class IntakeSubsystem extends SubsystemBase {

  // Define solenoid used for moving intake up and down
  private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,0,1);
  private final VictorSPX intakeWheels = new VictorSPX(INTAKE_WHEEL_ID);

  public IntakeSubsystem() {
     this.lowerIntake();
     this.intakeWheels.setStatusFramePeriod(1,50);
     this.intakeWheels.setStatusFramePeriod(2,50);
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

  public void stop() {
    this.intakeWheels.set(ControlMode.PercentOutput, 0);
  }
}