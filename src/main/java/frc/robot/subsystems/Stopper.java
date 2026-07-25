package frc.robot.subsystems;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

import com.ctre.phoenix.motorcontrol.VictorSPXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.util.CANUtil;
public class Stopper extends SubsystemBase {
  WPI_VictorSPX m_motor;
  private final CANUtil kCANUtil = CANUtil.getInstance();
  
  private final Encoder s_Encoder = new Encoder(Constants.Hardware.kStopperEncoderAChannel, Constants.Hardware.kStopperEncoderBChannel);
  private final DigitalInput s_switch = new DigitalInput(Constants.Hardware.kStopperSwitchChannel);
  private double mechanismPosition = 0;
  private double mechanismPositionDebounced = 0;
  private PIDController mechanismPIDController = new PIDController(0.005, 0.001, 0.0);
  private Queue<Double> mechanismPositionHistory = new ArrayBlockingQueue<>(3);
  private boolean switchTriggered = false;
  private double extendedPosition = -700;
  private boolean hasHomed = false;
  
  public Stopper() {
    m_motor = new WPI_VictorSPX(Constants.Hardware.kStopperId);
    kCANUtil.registerDevice("Stopper", Constants.Hardware.kStopperId, Constants.Hardware.DeviceType.VictorSPX, m_motor);
    s_Encoder.setSamplesToAverage(10);
  }

  @Override
  public void periodic() {
    this.mechanismPosition = s_Encoder.getDistance();
    SmartDashboard.putNumber("stopperDist", this.mechanismPosition);
    this.switchTriggered = s_switch.get();
  }

  public void home() {
    if (!switchTriggered) {
      this.run(-0.4);
    } else {
      this.stop();
      this.reset();
    }
  }

  public void run(double speed) {
    m_motor.set(speed);
  }

  public void extend() {
    this.run(-mechanismPIDController.calculate(this.mechanismPosition, extendedPosition));
  }

  public void extend(double pos) {
    double t = mechanismPIDController.calculate(-pos);
    t = Math.abs(t) < 0.3 ? Math.copySign(0.3, t) : t;
    this.run(t);
  }

  public void retract() {
    this.home();
  }

  public void reset() {
    this.hasHomed = true;
    s_Encoder.reset();
    mechanismPIDController.reset();
  }

  public void coast() {
    m_motor.set(0.0);
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public void homeExtend() {
    if (!this.hasHomed) {
      this.home();
    } else {
      this.extend();
    }
  }

  public Command stopCommand() {
    return runEnd(() -> this.stop(), ()-> this.coast());
  }
  public Command extendCommand() {
    return runEnd(() -> this.extend(), ()-> this.coast());
  }
  public Command homeCommand() {
    return runEnd(() -> this.home(), ()-> this.coast());
  }
  public Command homeExtendCommand() {
    return runEnd(() -> this.homeExtend(), () -> this.coast());
  }
}
