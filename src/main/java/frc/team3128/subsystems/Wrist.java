package frc.team3128.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;

import static frc.team3128.Constants.WristConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Wrist extends NAR_PIDSubsystem {

    private static Wrist instance;
    private NAR_CANSparkMax m_rotate;
    private DutyCycleEncoder m_encoder;

    public enum WristPosition {
        //TODO: GET ANGLES

        TOP_CONE(112, true),  
        TOP_CUBE(105, false),
        MID_CONE(105, true),
        MID_CUBE(90, false), 
        LOW_FLOOR(45, false), 

        NEUTRAL(5, null), 

        HP_SHELF_CONE(115, null), 
        HP_SHELF_CUBE(108, null), 
        GROUND_PICKUP(37, null),
        GROUND_PICKUP_CONE(37, null), 
        GROUND_PICKUP_CUBE(37.5, null), 
        CONE_POLE(-40, null);

        public final double wristAngle;
        public final Boolean isCone;

        private WristPosition(double wristAngle, Boolean isCone) {
            this.wristAngle = wristAngle;
            this.isCone = isCone;
        }
    }

	private Wrist() {
        super(new PIDController(kP, kI, kD), kS, kV, kG);
        
        setkG_Function(()-> Math.cos(Units.degreesToRadians(getMeasurement())));
        getController().enableContinuousInput(-180, 180);
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
        m_rotate.setIdleMode(IdleMode.kBrake);
    }

    private void configEncoders() {
        m_encoder = new DutyCycleEncoder(ENC_DIO_ID);
    }

    private void setPower(double power) {
        m_rotate.set(power);
    }

    private void stopWrist() {
        setPower(0);
    }

	@Override
	protected void useOutput(double output, double setpoint) {
        m_rotate.set(MathUtil.clamp(output, -1,1));
	}

	@Override
	protected double getMeasurement() {
        return MathUtil.inputModulus(m_encoder.get() * 360 - ANGLE_OFFSET, -180, 180);
	}
    
}
