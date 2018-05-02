package frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class PowerPanel {
	private static PowerDistributionPanel pdp = new PowerDistributionPanel();

	// Everything return amps
	public static double zero() {
		return pdp.getCurrent(0);
	}

	public static double one() {
		return pdp.getCurrent(1);
	}

	public static double two() {
		return pdp.getCurrent(2);
	}

	public static double three() {
		return pdp.getCurrent(3);
	}

	public static double four() {
		return pdp.getCurrent(4);
	}

	public static double five() {
		return pdp.getCurrent(5);
	}

	public static double six() {
		return pdp.getCurrent(6);
	}

	public static double seven() {
		return pdp.getCurrent(7);
	}

	public static double eight() {
		return pdp.getCurrent(8);
	}

	public static double nine() {
		return pdp.getCurrent(9);
	}

	public static double ten() {
		return pdp.getCurrent(10);
	}

	public static double eleven() {
		return pdp.getCurrent(11);
	}

	public static double twelve() {
		return pdp.getCurrent(12);
	}

	public static double thirteen() {
		return pdp.getCurrent(13);
	}

	public static double fourteen() {
		return pdp.getCurrent(14);
	}

	public static double fifteen() {
		return pdp.getCurrent(15);
	}
}
