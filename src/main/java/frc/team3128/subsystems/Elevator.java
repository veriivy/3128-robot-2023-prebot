package frc.team3128.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import static frc.team3128.Constants.ElevatorConstants.*;

import com.ctre.phoenix.motorcontrol.InvertType;

public class Elevator extends NAR_PIDSubsystem{

    public NAR_CANSparkMax m_elv1, m_elv2;

    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null){
            instance = new Elevator();  
        }
        return instance;
    }

    public Elevator() {
        super(new PIDController(kP, kI, kD), kS, kV, kG);
        setConstraints(MIN_DIST, MAX_DIST);
        configMotors();
    }

    private void configMotors() {
        m_elv1 = new NAR_CANSparkMax(0);
        m_elv2 = new NAR_CANSparkMax(1);

        m_elv2.setInverted(true);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        m_elv1.set(MathUtil.clamp(output, -1, 1));
        m_elv2.set(MathUtil.clamp(output, -1, 1));
        
    }

    @Override
    protected double getMeasurement() {
        return m_elv1.getSelectedSensorPosition() * GEAR_RATIO * SPOOL_CIRCUMFERENCE;
    }

    public void resetEncoder() {
        m_elv1.setSelectedSensorPosition(0);
    }
}
