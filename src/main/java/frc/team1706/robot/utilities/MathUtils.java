package frc.team1706.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1706.robot.subsystems.SwerveDrivetrain;

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

	//Returns distance gone in last frame
	public static double getRobotDistance() {
		double[] xyDistFR = SwerveDrivetrain.swerveModules.get(SwerveDrivetrain.WheelType.FRONT_LEFT).getXYDist();
		double[] xyDistFL = SwerveDrivetrain.swerveModules.get(SwerveDrivetrain.WheelType.FRONT_RIGHT).getXYDist();
		double[] xyDistBL = SwerveDrivetrain.swerveModules.get(SwerveDrivetrain.WheelType.BACK_RIGHT).getXYDist();
		double[] xyDistBR = SwerveDrivetrain.swerveModules.get(SwerveDrivetrain.WheelType.BACK_LEFT).getXYDist();

		SwerveDrivetrain.swerveModules.get(SwerveDrivetrain.WheelType.FRONT_RIGHT).resetDelta();
		SwerveDrivetrain.swerveModules.get(SwerveDrivetrain.WheelType.FRONT_LEFT).resetDelta();
		SwerveDrivetrain.swerveModules.get(SwerveDrivetrain.WheelType.BACK_LEFT).resetDelta();
		SwerveDrivetrain.swerveModules.get(SwerveDrivetrain.WheelType.BACK_RIGHT).resetDelta();

		SmartDashboard.putNumber("XoverT", (xyDistFR[0] + xyDistFL[0] + xyDistBL[0] + xyDistBR[0]));
		SmartDashboard.putNumber("YoverT", (xyDistFR[1] + xyDistFL[1] + xyDistBL[1] + xyDistBR[1]));

		double xAvg = (xyDistFR[0] + xyDistFL[0] + xyDistBL[0] + xyDistBR[0])/4.0;
		double yAvg = (xyDistFR[1] + xyDistFL[1] + xyDistBL[1] + xyDistBR[1])/4.0;

		return pythagorean(xAvg, yAvg);
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
