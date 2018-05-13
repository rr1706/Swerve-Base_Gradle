package frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1706.robot.utilities.MathUtils;
import frc.team1706.robot.utilities.Vector;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.Properties;

public class SwerveDrivetrain {
	private static String[] FRPorts;
	private static String[] FLPorts;
	private static String[] BLPorts;
	private static String[] BRPorts;
	private static double encoders = 4.0;

	public enum WheelType {
		FRONT_RIGHT, FRONT_LEFT, BACK_LEFT, BACK_RIGHT
	}

	public static Map<WheelType, SwerveModule> swerveModules;

	public static void loadPorts() {
		Properties application = new Properties();
		File offsets = new File("/home/lvuser/SWERVE_OFFSET.txt");
		try {
			FileInputStream in = new FileInputStream(offsets);
			application.load(in);
		} catch (IOException e) {
			e.printStackTrace();
		}

		FRPorts = application.get("front_right_ports").toString().split(",");
		FLPorts = application.get("front_left_ports").toString().split(",");
		BRPorts = application.get("back_right_ports").toString().split(",");
		BLPorts = application.get("back_left_ports").toString().split(",");

	}

	/**
	 * Initializes the swerve drive train
	 */
	public SwerveDrivetrain() {
		swerveModules = new HashMap<>();
		swerveModules.put(WheelType.FRONT_RIGHT, new SwerveModule(Integer.parseInt(FRPorts[0]), Integer.parseInt(FRPorts[1]), Integer.parseInt(FRPorts[2]), Integer.parseInt(FRPorts[3])/* 0, 0, 6, 7 */));
		swerveModules.put(WheelType.BACK_RIGHT, new SwerveModule(Integer.parseInt(BRPorts[0]), Integer.parseInt(BRPorts[1]), Integer.parseInt(BRPorts[2]), Integer.parseInt(BRPorts[3])/* 1, 1, 2, 3 */));
		swerveModules.put(WheelType.BACK_LEFT, new SwerveModule(Integer.parseInt(BLPorts[0]), Integer.parseInt(BLPorts[1]), Integer.parseInt(BLPorts[2]), Integer.parseInt(BLPorts[3])/* 2, 2, 4, 5 */));
		swerveModules.put(WheelType.FRONT_LEFT, new SwerveModule(Integer.parseInt(FLPorts[0]), Integer.parseInt(FLPorts[1]), Integer.parseInt(FLPorts[2]), Integer.parseInt(FLPorts[3])/* 3, 3, 0, 1 */));

	}

	/**
	 * Get the translation motor command
	 */
	public double getWheelTMCommand(WheelType wheel) {
		return swerveModules.get(wheel).getSpeedCommand();
	}

	/**
	 * Get the rotation motor command
	 */
	public double getWheelRMCommand(WheelType wheel) {
		return swerveModules.get(wheel).getAngleCommand();
	}

	/**
	 * Calculate a wheel's velocity and heading commands for the given FWD, STR, and RCW command
	 */
	public void drive(Vector translation, double rotation) {
		double max = 1.0;
		double radius;

		for (WheelType type : swerveModules.keySet()) {
			SwerveModule wheel = swerveModules.get(type);

			radius = MathUtils.inchToMeter(MathUtils.pythagorean(wheel.getPosition().getX(), wheel.getPosition().getY()));

			// swerveN
			double Wxi = translation.getX() + rotation * MathUtils.inchToMeter(wheel.getPosition().getY()) / radius;
			double Wyi = translation.getY() - rotation * MathUtils.inchToMeter(wheel.getPosition().getX()) / radius;
			double speed = Math.sqrt(Math.pow(Wxi, 2) + Math.pow(Wyi, 2));
			double angle = Math.atan2(Wxi, Wyi);
			wheel.setSpeedCommand(speed);
			wheel.setAngleCommand(MathUtils.convertRange(0, Math.PI * 2, 0, 1024, MathUtils.resolveAngle(angle + wheel.getOffset())));

			// find the maximum speed command for normalizing below
			if (speed > max) {
				max = speed;
			}
		}

//		encoders = 0.0;
		for (WheelType type : swerveModules.keySet()) {
			SwerveModule wheel = swerveModules.get(type);

			double speed = wheel.getSpeedCommand() / max; // normalized to maximum of 1
			wheel.setSpeedCommand(speed);
			wheel.drive();

//			if (wheel.getEncoderAlive()) {
//				encoders++;
//			}
		}
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

		double xAvg = (xyDistFR[0] + xyDistFL[0] + xyDistBL[0] + xyDistBR[0])/encoders;
		double yAvg = (xyDistFR[1] + xyDistFL[1] + xyDistBL[1] + xyDistBR[1])/encoders;

		return MathUtils.pythagorean(xAvg, yAvg);
	}
}
