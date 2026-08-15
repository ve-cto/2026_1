package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Constants;
import frc.robot.subsystems.util.CANUtil;

public class IntakeNeo extends SubsystemBase {
    private final SparkMax m_intake;
    private final CANUtil kCANUtil = CANUtil.getInstance();


    /** Instantiate */
    public IntakeNeo() {
        m_intake = new SparkMax(Constants.Hardware.kIntakeId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        kCANUtil.registerDevice("m_intakeSM", Constants.Hardware.kIntakeId, Constants.Hardware.DeviceType.SparkMax, m_intake);
    }

    public void periodic() {
        SmartDashboard.putNumber("Intake/Intake Speed", this.m_intake.get());
    }

    /* 
    * run the intake motor at the provided speed 
    */
    // public void run(double speed) {
    //     this.output = speed;
    //     m_intake.set(speed);
    // }

    // public void runRPM(double rpm) {
    //     this.m_intake.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
        
    // }

    public void run(double speed) {
        this.m_intake.set(speed);
    }

    /*
    * stop the intake motor with braking force
    */
    public void stop() {
        m_intake.stopMotor();
    }

    /*
     * stop the intake motor without braking force
     */
    public void coast() {
        m_intake.set(0.0);
    }
    /* 
     * run the intake forward with the speed given in constants
     */
    public Command runIntakeCommand() {
        return this.runEnd(() -> this.run(Constants.Intake.kIntakeForwardSpeed), () -> this.stop());
    }

    /*
     * run the intake backward with the speed given in constants
     */
    public Command reverseIntakeCommand() {
        return this.runEnd(() -> this.run(Constants.Intake.kIntakeReverseSpeed), () -> this.stop());
    }

    /*
     * run the intake with a given speed
     */
    public Command runIntakeCommand(double speed) {
        return this.runEnd(() -> this.run(speed), () -> this.stop());
    }

    public Command coastCommand() {
        return this.run(() -> this.coast());
    }

    public void simulationInit() {}

    // these motors are already automatically simulated - you don't need to put anything in here buddy :]
    // the encoder can't be simulated though unfortunately
    // if a kraken gets put on the intake, then you will need to put stuff here though, use the shooter subsystem for reference
    public void simulationPeriodic() {}
}
