package frc.team3128.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import static frc.team3128.Constants.ElevatorConstants.*;

public class Elevator extends NAR_PIDSubsystem {

    private NAR_CANSparkMax m_elv1, m_elv2;

    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null){
            instance = new Elevator();  
        }
        return instance;
    }

    public enum States {
        LOW(0),
        MID(0),
        HIGH(0),
        GROUND(0),
        SHELF(0),
        DROP(0);

        public double height;
        private States(double height) {
            this.height = height;
        }
    }

    public Elevator() {
        super(new PIDController(kP, kI, kD), kS, kV, kG);
        setConstraints(MIN_DIST, MAX_DIST);
        configMotors();
        initShuffleboard(kS, kV, kG);
    }

    private void configMotors() {
        m_elv1 = new NAR_CANSparkMax(ELV1_ID);
        m_elv2 = new NAR_CANSparkMax(ELV2_ID);

        m_elv1.setInverted(false);
        m_elv2.setInverted(true);

        m_elv1.setSmartCurrentLimit(CURRENT_LIMIT); 
        m_elv2.setSmartCurrentLimit(CURRENT_LIMIT);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        set(MathUtil.clamp(output, -1, 1));
    }

    public void set(double power) {
        m_elv1.set(power);
        m_elv2.set(power);
    }

    public void stop() {
        set(0);
    }

    public void resetEncoder() {
        m_elv1.setSelectedSensorPosition(0);
    }

    @Override
    public double getMeasurement() {
        return m_elv1.getSelectedSensorPosition() * GEAR_RATIO * SPOOL_CIRCUMFERENCE;
    }


}
