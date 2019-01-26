package frc.robot.utilities;

import frc.robot.subsystems.Time;

public class Acceleration {

    private double m_start = 0.0;
    private double m_stop = 0.0;
    private double m_time = 1.0;
    private double m_baseTime = 0.0;
    private double m_elapsedTime;

    private double m_calc;

    public Acceleration() {
        super();

    }

    public void set(double start, double stop, double time) {
        m_start = start;
        m_stop = stop;
        m_time = time;
        m_baseTime = Time.get();
    }

    public double calculate() {
        m_elapsedTime = Time.get() - m_baseTime;
        if (m_elapsedTime > m_time) {
            m_elapsedTime = m_time;
        }
        m_calc = m_start+(m_elapsedTime/m_time)*(m_stop-m_start);

        return m_calc;
    }
}
