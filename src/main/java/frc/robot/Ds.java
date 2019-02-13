package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class Ds {

	private static DriverStation ds = DriverStation.getInstance();

	public static double getBatteryVoltage() {
		return ds.getBatteryVoltage();
	}
}
