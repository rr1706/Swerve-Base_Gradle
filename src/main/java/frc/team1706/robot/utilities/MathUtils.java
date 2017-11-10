package frc.team1706.robot.utilities;

/**
 * Various math functions
 *
 * @author team1706
 */
public class MathUtils {

	/**
	 * Converts from field-oriented driving commands to robot-oriented
	 * commands to allow the driver to drive field oriented
	 */
	public static Vector convertOrientation(double currentHeading, double fwd, double str) {
		double x;
		double y;

		x = (str * Math.cos(currentHeading)) - (fwd * Math.sin(currentHeading)); // STR
		y = (fwd * Math.cos(currentHeading)) + (str * Math.sin(currentHeading)); // FWD

		return new Vector(x, y);
	}

	public static double reverseWheelDirection(double direction) {
		if (direction > Math.PI) {
			direction -= Math.PI;
		} else {
			direction += Math.PI;
		}

		return direction;
	}

	public static double getDelta(double num1, double num2) {
		return num1 - num2;

	}

	public static double reverseErrorDirection(double error) {
		if (error < 0.0) {
			error += Math.PI;
		} else {
			error -= Math.PI;
		}
		return error;
	}

	public static double convertRange(double oldMin, double oldMax, double newMin, double newMax, double oldValue) {
		double oldRange = (oldMax - oldMin);
		double newRange = (newMax - newMin);
		double newValue = (((oldValue - oldMin) * newRange) / oldRange) + newMin;

		return newValue;
	}

	public static double calculateError(double direction, double sensor) {
		double error = direction - sensor;
		if (Math.abs(error) > 5.236) { // 300 Degrees
			if (error < 0.0) {
				error += (Math.PI * 2);
			} else {
				error -= (Math.PI * 2);
			}
		}
		return error;
	}

	/**
	 * Converts a number from radians to degrees
	 */
	public static double radToDeg(double x) {
		return x * 180.0 / Math.PI;
	}

	/**
	 * Converts a number from degrees to radians
	 */
	public static double degToRad(double x) {
		return x * Math.PI / 180;
	}

	public static double meterToInch(double x) {
		return x * 39.3701;
	}

	public static double inchToMeter(double x) {
		return x / 39.3701;
	}

	public static double pythagorean(double a, double b) {
		double c = Math.sqrt((Math.pow(a, 2) + Math.pow(b, 2)));

		return c;
	}

	public static double resolveDeg(double deg) {
		while (deg > 360) {
			deg -= 360;
		}

		while (deg < 0) {
			deg += 360;
		}

		return deg;
	}

	/**
	 * Unwraps angles to be between 0 and 2pi
	 */
	public static double resolveAngle(double rad) {
		while (rad >= Math.PI * 2) {
			rad -= Math.PI * 2;
		}

		while (rad < 0) {
			rad += Math.PI * 2;
		}
		return rad;
	}

	public static double resolveXrotAngle(double rad) {
		while (rad >= Math.PI) {
			rad -= Math.PI * 2;
		}

		while (rad < -Math.PI) {
			rad += Math.PI * 2;
		}
		return rad;
	}

	public static boolean lessThan(double currAngle, double searchLim) {
		if (currAngle > 180) {
			currAngle -= 360;
		}

		if (searchLim > 180) {
			searchLim -= 360;
		}

		if (currAngle < searchLim) {
			return true;
		} else {
			return false;
		}

	}
}
