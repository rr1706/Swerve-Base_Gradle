package frc.robot.utilities;

/**
 * Various math functions
 */
public class MathUtils {

	/**
	 * Converts from field-oriented driving commands to robot-oriented
	 * commands to allow the driver to drive field oriented
	 * @param currentHeading Where the robot is currently facing
	 * @param fwd Field-Oriented Forward command
	 * @param str Feild-Oriented Strafe command
	 *
	 * @return The new STR and FWD commands for the robot
	 */
	public static Vector convertOrientation(double currentHeading, double fwd, double str) {
		double x;
		double y;

		x = (str * Math.cos(currentHeading)) - (fwd * Math.sin(currentHeading)); // STR
		y = (fwd * Math.cos(currentHeading)) + (str * Math.sin(currentHeading)); // FWD

		return new Vector(x, y);
	}

	/**
	 * Reverses the direction of a wheel
	 * @param direction Direction of wheel in radians
	 * @return new direction
	 */
	public static double reverseWheelDirection(double direction) {
		if (direction > 180.0) {
			direction -= 180.0;
		} else {
			direction += 180.0;
		}
		return direction;
	}

	/**
	 * Returns the delta of two numbers
	 * @param num1 First number
	 * @param num2 Second number
	 * @return Delta
	 */
	public static double getDelta(double num1, double num2) {
		return num1 - num2;
	}

	/**
	 * Reverses the error of a wheel
	 * @param error Error of wheel in radians
	 * @return reversed error
	 */
	public static double reverseErrorDirection(double error) {
		if (error < 0.0) {
			error += 180.0;
		} else {
			error -= 180.0;
		}
		return error;
	}

	/**
	 * Return the equivilant value between two different ranges
	 * @param oldMin Minimum of previous range
	 * @param oldMax Maximum of previous range
	 * @param newMin Minimum of new range
	 * @param newMax Maximum of new range
	 * @param oldValue Value in old range
	 * @return The value in the new range
	 */
	public static double convertRange(double oldMin, double oldMax, double newMin, double newMax, double oldValue) {
		double oldRange = (oldMax - oldMin);
		double newRange = (newMax - newMin);
		double newValue = (((oldValue - oldMin) * newRange) / oldRange) + newMin;

		return newValue;
	}

	/**
	 * Calculates the error between a sensor and desired direction
	 * @param setpoint Direction in degrees
	 * @param input Sensor in degrees
	 * @return Error
	 */
	public static double calculateContinuousError(double setpoint, double input, double maximumInput, double minimumInput) {
		// Calculate the error signal
		double error = setpoint - input;

		if (Math.abs(error) > (maximumInput - minimumInput) / 2) {
			if (error > 0) {
				error = error - maximumInput + minimumInput;
			} else {
				error = error + maximumInput - minimumInput;
			}
		}

		return error;
	}

	public static double getAngleError(double input, double setpoint) {
		double error = setpoint - input;
		if (Math.abs(error) > 180.0) {
			if (error > 0) {
				error -= 360.0;
			} else {
				error += 360.0;
			}
		}
		return error;
	}

	/**
	 * Converts a number from radians to degrees
	 * @param x radians
	 * @return Degrees
	 */
	public static double radToDeg(double x) {
		return x * 180.0 / Math.PI;
	}

	/**
	 * Converts a number from degrees to radians
	 * @param x degrees
	 * @return Radians
	 */
	public static double degToRad(double x) {
		return x * Math.PI / 180;
	}

	/**
	 * Converts a number from meters to inches
	 * @param x meters
	 * @return Inches
	 */
	public static double meterToInch(double x) {
		return x * 39.3701;
	}

	/**
	 * Converts a number from inches to meters
	 * @param x inches
	 * @return Meters
	 */
	public static double inchToMeter(double x) {
		return x / 39.3701;
	}

	/**
	 * Return the length of the long side of a right triangle
	 * @param a length of one side of triangle
	 * @param b length of second side of triangle
	 * @return Length of the long side of a right triangle
	 */
	public static double pythagorean(double a, double b) {
		double c = Math.sqrt((Math.pow(a, 2) + Math.pow(b, 2)));

		return c;
	}

	/**
	 * Unwraps a degree to be between 0 and 360
	 * @param deg degrees
	 * @return degree from 0 to 360
	 */
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
	 * Unwraps a radian to be between 0 and 2pi
	 * @param rad radians
	 * @return radian from 0 to 2pi
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

	/**
	 * Adjusts {@code ticks} so that it is between 0 and {@code ticksPerRev}.
	 *
	 * @param ticks - the count to adjust in native ticks
	 * @param ticksPerRev - number of native ticks per revolution
	 * @return the adjusted value
	 */
	public static double resolveAngleNative(double ticks, double ticksPerRev) {
		while (ticks >= ticksPerRev) {
			ticks -= ticksPerRev;
		}
		while (ticks < 0.0) {
			ticks += ticksPerRev;
		}
		return ticks;
	}
}
