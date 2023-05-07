package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.General;

public final class Claw extends SubsystemBase {

    private final CANSparkMax m_clawMotor = new CANSparkMax(9, MotorType.kBrushless);
    private final RelativeEncoder m_clawEncoder = m_clawMotor.getEncoder(); // Relative encoder

    private final PIDController m_PIDController;
    private ClawState m_state;
    private double m_moveStartTime;

    public enum ClawState  {
        OPEN, CLOSE, HOLD
    }

    public Claw() {
        m_clawMotor.restoreFactoryDefaults(); 
        m_clawMotor.setCANTimeout(General.kCANConfigTimeout);
        m_clawMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, Constants.General.kTalonCANStatusSlowRateInMs);
        m_clawMotor.enableVoltageCompensation(Constants.General.kNominalBatteryVoltage);
        m_clawMotor.setSmartCurrentLimit(1); //REPLACE WITH CONSTANTS FOR VOLTAGE
        m_clawMotor.setIdleMode(IdleMode.kBrake);
        m_clawMotor.setInverted(false);

        m_clawEncoder.setPosition(0);

        m_PIDController = new PIDController(ClawConstants.kP, ClawConstants.kI, ClawConstants.kD);
        SmartDashboard.putData("clawPID", m_PIDController);
        m_PIDController.setSetpoint(getClawEncoderPosition());
        m_PIDController.setTolerance(ClawConstants.kTolerance);
        m_PIDController.disableContinuousInput();

        this.operate(ClawState.HOLD, false);
    }

    private double getClawEncoderPosition() {
        return m_clawEncoder.getPosition();
    }

    public boolean isFinished() {
        return this.m_state == ClawState.HOLD;
    }

    public void operate(ClawState state, boolean timeBased) {

        System.out.println("Operating claw");
        m_state = state;
        switch (state) {
            default:
            case HOLD:
                this.doHold();
                break;

            case CLOSE:
                this.doMove(1, timeBased); 
                break;

            case OPEN:
                this.doMove(-1, timeBased); 
                break;
        }
    }

    private void doMove(int multiplier, boolean timeBased) {
        // Run the claw for a set period of time.
        // Once we're sure the claw is OPEN or CLOSED, set the setPoint to the current location so 
        // the PID loop will maintain that position.
        double speed = ClawConstants.kClawMotorMaxSpeed * multiplier;
        m_clawMotor.set(speed);
        if (timeBased) {
            m_moveStartTime = Timer.getFPGATimestamp();
        } else {
            m_moveStartTime = -1;
        }
    }

    private void doHold() {
        // force PID loop value because it keeps changing to 1.0
        m_PIDController.setP(ClawConstants.kP);
        double position = getClawEncoderPosition();
        m_PIDController.setSetpoint(position);
        position *= 1.1;
        m_moveStartTime = -1;
    }

    @Override
    public void periodic() {
        
        if (this.m_state == ClawState.HOLD) {
            double calculatedSpeed = m_PIDController.calculate(getClawEncoderPosition());
            calculatedSpeed = MathUtil.clamp(
                calculatedSpeed, -ClawConstants.kClawMotorMaxSpeed, ClawConstants.kClawMotorMaxSpeed);    
            m_clawMotor.set(calculatedSpeed);
            SmartDashboard.putNumber("claw PID output", calculatedSpeed);
            return;
        }

         if (this.m_moveStartTime > 0 && Timer.getFPGATimestamp() - this.m_moveStartTime > Constants.ClawConstants.kClawRunTime) {
            m_state = ClawState.HOLD;
            this.doHold();
        }

        SmartDashboard.putNumber("claw PID setpoint", m_PIDController.getSetpoint());
        SmartDashboard.putNumber("claw PID kP", m_PIDController.getP());
        SmartDashboard.putNumber("claw PID tolerance", m_PIDController.getPositionTolerance());
    }
}
