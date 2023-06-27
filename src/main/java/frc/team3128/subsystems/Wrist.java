package frc.team3128.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;

import static frc.team3128.Constants.WristConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Wrist extends PIDSubsystem {

    private static Wrist instance;
    private NAR_CANSparkMax m_rotate;
    private DutyCycleEncoder m_encoder;

	private Wrist() {
        super(new PIDController(kP, kI, kD));
    }

    public static synchronized Wrist getInstance() {
        if (instance == null) {
            instance = new Wrist();
        }
        return instance;
    }

    public void configMotor() {
        m_rotate = new NAR_CANSparkMax(WRIST_MOTOR_ID);
        m_rotate.setSmartCurrentLimit(WRIST_CURRENT_LIMIT);
        m_rotate.setInverted(false);
        m_rotate.enableVoltageCompensation(12.0);
        m_rotate.setIdleMode(IdleMode.kBrake);
    }

    private void configEncoders() {
        m_encoder = new DutyCycleEncoder(ENC_DIO_ID);
    }
    
	@Override
	protected void useOutput(double output, double setpoint) {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'useOutput'");
	}

	@Override
	protected double getMeasurement() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'getMeasurement'");
	}
    
}
