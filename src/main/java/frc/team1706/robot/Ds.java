package frc.team1706.robot;

import edu.wpi.first.wpilibj.DriverStation;

class Ds {

	private static DriverStation ds = DriverStation.getInstance();

	static double getBatteryVoltage() {
		return ds.getBatteryVoltage();
	}
}
