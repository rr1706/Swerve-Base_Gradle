package frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

public class Time {

	private static Timer systemTimer = new Timer();

	public static void start() {
		systemTimer.start();
	}

	public static double get() {
		return systemTimer.get();
	}

	public static double getMatch() {
		return systemTimer.getMatchTime();
	}

	public static void reset() {
		systemTimer.reset();

	}
}
