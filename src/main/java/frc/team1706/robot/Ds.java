package frc.team1706.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class Ds {

	static DriverStation ds = DriverStation.getInstance();
	
	public static double getBatteryVoltage() {
		return ds.getBatteryVoltage();
	}
}
