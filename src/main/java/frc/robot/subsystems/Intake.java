package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.pathplanner.lib.config.RobotConfig;

import frc.robot.Constants;
import frc.robot.subsystems.util.CANUtil;

public class Intake extends SubsystemBase {
    private final WPI_VictorSPX m_intake;
    private final CANUtil kCANUtil = CANUtil.getInstance();
    SlewRateLimiter filter = new SlewRateLimiter(100, -8, 0);

    private double output = 0.0;

    /** Instantiate */
    public Intake() {
        m_intake = new WPI_VictorSPX(Constants.Hardware.kIntakeId);
        
        kCANUtil.registerDevice("m_intake", Constants.Hardware.kIntakeId, Constants.Hardware.DeviceType.VictorSPX, m_intake);
    }

    public void periodic() {
        SmartDashboard.putNumber("Intake/Intake Output", this.output);
    }

    /* 
    * run the intake motor at the provided speed 
    */
    public void run(double speed) {
        this.output = speed;
        m_intake.set(speed);
    }

    public Command runVoltageCommand(Voltage voltage) {
        return this.runEnd(() -> this.runVoltage(voltage), () -> this.stopIntake());
    }

    public void runVoltage(Voltage voltage) {
        this.output = voltage.magnitude();
        m_intake.setVoltage(voltage);
    }

    public Command stopIntakeCommand() {
        return this.run(() -> this.stopIntake());
    }
    /*
    
    * stop the intake motor with braking force
    */
    public void stopIntake() {
        this.output = 0;
        this.filter.calculate(0);
        this.filter.reset(0);
        m_intake.stopMotor();
    }

    /*
     * stop the intake motor without braking force
     */
    public void coastIntake() {
        this.filter.calculate(0);
        this.filter.reset(0);
        m_intake.set(0.0);
    }
    /* 
     * run the intake forward with the speed given in constants
     */
    public Command runIntakeCommand() {
        return this.runEnd(() -> this.run(Constants.Intake.kIntakeForwardSpeed), () -> this.stopIntake());
    }

    public Command runIntakeVoltageCommand(Voltage voltage) {
        return this.runEnd(() -> this.runVoltage(voltage), () -> this.stopIntake());
    }

    /*
     * run the intake backward with the speed given in constants
     */
    public Command reverseIntakeCommand() {
        return this.runEnd(() -> this.run(Constants.Intake.kIntakeReverseSpeed), () -> this.stopIntake());
    }

    /*
     * run the intake with a given speed
     */
    public Command runIntakeCommand(double speed) {
        return this.runEnd(() -> this.run(speed), () -> this.stopIntake());
    }

    public Command coastCommand() {
        return this.run(() -> this.run(0.0));
    }

    public void simulationInit() {}

    // these motors are already automatically simulated - you don't need to put anything in here buddy :]
    // the encoder can't be simulated though unfortunately
    // if a kraken gets put on the intake, then you will need to put stuff here though, use the shooter subsystem for reference
    public void simulationPeriodic() {}
}
