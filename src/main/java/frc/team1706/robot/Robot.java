package frc.team1706.robot;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;
import java.util.Properties;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.team1706.robot.subsystems.*;
import frc.team1706.robot.subsystems.SwerveDrivetrain.WheelType;
import frc.team1706.robot.utilities.*;
import frc.team1706.robot.RRLogger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the manifest file in the
 * resource directory.
 */
public class Robot extends TimedRobot {

	private SendableChooser<Integer> autoChooser;

	private Compressor compressor;

	public static XboxController xbox1 = new XboxController(0);
//	public static XboxController xbox2 = new XboxController(1);
//	private static XboxController endbox = new XboxController(2);

	private int autonomousChoice;
//	private JetsonServer jet;
	private Thread t;
	private SwerveDrivetrain driveTrain;
	private IMU imu;
	private RRLogger log;

	private boolean robotBackwards;

	private double robotOffset;

	private int disabled = 0;

	private double[][] commands;
	private int arrayIndex = -1;
	private int autoMove = 0;
	private int translateType;
	private double autonomousAngle;
	private double tSpeed;
	private double rSpeed;
	private double previousDistance = 0.0;
	private double currentDistance = 0.0;
	private boolean override;
	private boolean driveDone;
	private boolean turnDone;
	private boolean timeDone;
	private boolean collisionDone;
	private boolean moonDone;
	private double timeBase;
	private boolean timeCheck;
	private double smoothArc;
	private double smoothRotate;
	private boolean smoothRotateStarted = false;
	private boolean smoothRotateChangeCheck = false;
	private boolean stepStarted = false;
	private double smoothTranslate;
	private double smoothTranslateNum;
	private double smoothAccelerate;
	private double smoothAccelerateNum;
	private double initialAngle;
	private double initialError;
	private final double minSpeed = 0.2;

	private int dx = -1;

	private double FWD;
	private double STR;
	private double RCW;

	private double wheelRamp = 0;
	private double rampRate = 0;
	private double currentRampTime = 0;
	private double prevRampTime = 0;

	private double keepAngle;

	private boolean autonomous;

	private boolean fieldOriented = true; // start on field orientation
	private boolean previousOrientedButton = false;
	private boolean currentOrientedButton = false;

	private PIDController SwerveCompensate;

	private double lead;

	private double robotRotation;

	private double imuOffset = 0;

	private Properties application = new Properties();
	private File offsets = new File("/home/lvuser/SWERVE_OFFSET.txt");

	private boolean rumble = false;
	private int rumbleTime = 0;

	private void keepAngle() {
		// LABEL keepAngle

		SwerveCompensate.enable();

		double leadNum = SmartDashboard.getNumber("leadNum", 0);
		lead = RCW * leadNum;

		SmartDashboard.putNumber("DPAD", xbox1.DPad());

//		System.out.println(Math.abs(xbox1.LStickX()));

		// This will update the angle to keep the robot's orientation
		if (Math.abs(xbox1.RStickX()) > 0.05 || // If right stick is pressed
				(Math.abs(FWD) < 0.05 && Math.abs(STR) < 0.05) && // If left stick is not pressed
						(xbox1.DPad() == -1) && // If dpad is not pressed
						(!autonomous)) { // If teleop

			SwerveCompensate.setPID(0.015, 0.0, 0.0);
			keepAngle = imu.getAngle();

		} else {

			SwerveCompensate.setPID(0.02, 0.0, 0.0);
			SwerveCompensate.setTolerance(12);

			SwerveCompensate.setInput(imu.getAngle());
			SwerveCompensate.setSetpoint(keepAngle);

			if (!SwerveCompensate.onTarget()) {
				SwerveCompensate.setPID(0.005, SmartDashboard.getNumber("CompensateI", 0.0), SmartDashboard.getNumber("CompensateD", 0.0));
			}

			robotRotation = SwerveCompensate.performPID();

			RCW = robotRotation;

			SmartDashboard.putNumber("Robot Rotation", robotRotation);
		}
	}

	/**
	 * Each potentiometer is positioned slightly differently so its initial value is different than the others,
	 * so even when the wheels are pointing straight there are differences.
	 * Proper values may be found and must be calculated for each wheel.
	 */
	private void loadOffsets() {
		// LABEL load offsets

		// Set the offset of each wheapel from a file on the roborio
		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setOffset(Double.parseDouble(application.getProperty("front_right_offset", "0")));
		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setOffset(Double.parseDouble(application.getProperty("front_left_offset", "0")));
		SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setOffset(Double.parseDouble(application.getProperty("back_left_offset", "0")));
		SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setOffset(Double.parseDouble(application.getProperty("back_right_offset", "0")));

		SmartDashboard.putNumber("FR offset: ", Double.parseDouble(application.getProperty("front_right_offset", "0")));
		SmartDashboard.putNumber("FL offset: ", Double.parseDouble(application.getProperty("front_left_offset", "0")));
		SmartDashboard.putNumber("BL offset: ", Double.parseDouble(application.getProperty("back_left_offset", "0")));
		SmartDashboard.putNumber("BR offset: ", Double.parseDouble(application.getProperty("back_right_offset", "0")));

		// Set the position of each wheel from a file on the roborio
		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setPosition(Vector.load(application.getProperty("front_right_pos", "0.0,0.0")));
		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setPosition(Vector.load(application.getProperty("front_left_pos", "0.0,0.0")));
		SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setPosition(Vector.load(application.getProperty("back_left_pos", "0.0,0.0")));
		SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setPosition(Vector.load(application.getProperty("back_right_pos", "0.0,0.0")));

		robotBackwards = Boolean.parseBoolean(application.getProperty("robot_backwards", "false"));
	}

	/**
	 * This function is run when the robot is first started up and should be used for any initialization code.
	 */
	public void robotInit() {
		// LABEL robot init
		// Load the wheel offset file from the roborio
		try {
			FileInputStream in = new FileInputStream(offsets);
			application.load(in);
		} catch (IOException e) {
			e.printStackTrace();
		}

		// Connect to jetson
//		try {
//			jet = new JetsonServer((short) 5800, (short) 5801);
//			t = new Thread(jet);
//			t.start();
//			jet.setDisabled();
//		} catch (IOException e) {
//			throw new RuntimeException(e);
//		}

		compressor = new Compressor(0);

		xbox1.setDeadband(0.01);

		Time.start();

		SwerveDrivetrain.loadPorts();
		driveTrain = new SwerveDrivetrain();
		loadOffsets();

		SmartDashboard.putNumber("CompensateP", 0.02);
		SmartDashboard.putNumber("CompensateI", 0.0);
		SmartDashboard.putNumber("CompensateD", 0.0);

		SmartDashboard.putNumber("Autonomous Delay", 0);

		autoChooser = new SendableChooser<>();
		autoChooser.addDefault("Middle", 1);
		autoChooser.addObject("Left", 2);
		autoChooser.addObject("Right", 3);
		autoChooser.addObject("Forward", 4);
		SmartDashboard.putData("Autonomous Mode Chooser", autoChooser);

		log = new RRLogger();

		imu = new IMU();
		imu.IMUInit();

		keepAngle = imu.getAngle();

		SwerveCompensate = new PIDController(0.015, 0.00, 0.00);
		SwerveCompensate.setContinuous(true);
		SwerveCompensate.setOutputRange(-1.0, 1.0);
		SwerveCompensate.setInputRange(0.0, 360.0);
		SwerveCompensate.setTolerance(1.0);

		SwerveCompensate.enable();
	}

	public void autonomousInit() {
		// LABEL autonomous init
//		jet.setAuto(); // this line is important because it does clock synchronization

		timeCheck = true;
		imu.reset(0);

		autonomousChoice = autoChooser.getSelected();

		String choice;

		if (autonomousChoice == 1) {
			choice = "/home/lvuser/Mid.csv";
		} else if (autonomousChoice == 2) {
			choice = "/home/lvuser/Left.csv";
		} else if (autonomousChoice == 3) {
			choice = "/home/lvuser/Right.csv";
		} else {
			choice = "/home/lvuser/Forward.csv";
		}

		choice = "/home/lvuser/Test.csv";

		SmartDashboard.putString("Autonomous File", choice);

		imu.reset(0);
		arrayIndex = 0;
		initialAngle = imu.getAngle();
		turnDone = false;
		driveDone = false;

		log.start();

		// Fill the array of commands from a csv file on the roborio
		try {
			List<String> lines = Files.readAllLines(Paths.get(choice));
			commands = new double[lines.size()][];
			int l = 0;
			for (String line : lines) {
				String[] parts = line.split(";");
				double[] linel = new double[parts.length];
				int i = 0;
				for (String part : parts) {
					linel[i++] = Double.parseDouble(part);
//					System.out.println(Double.parseDouble(part));
				}
				commands[l++] = linel;
			}
		} catch (IOException e) {
			e.printStackTrace();
		} catch (NumberFormatException e) {
			System.err.println("Error in configuration!");
		}
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		// LABEL autonomous periodic

		SmartDashboard.putNumber("IMU Angle", imu.getAngle());

		if (timeCheck) {
			timeBase = Time.get();
			timeCheck = false;
		}

		SmartDashboard.putNumber("Elapsed Time", Time.get());

		switch (autoMove) {

			// Pause the robot for x seconds at the start of auto
			case 0:

				driveTrain.drive(new Vector(0, 0), 0);
				if (Time.get() > timeBase + SmartDashboard.getNumber("Autonomous Delay", 0)) {
					autoMove = 1;
				}
				previousDistance = currentDistance;

				break;


			case 1:

//				SmartDashboard.putNumber("Array Index", arrayIndex);
//				SmartDashboard.putNumber("Auto Distance Gone", Math.abs(currentDistance - previousDistance));
//				SmartDashboard.putNumber("Auto Distance Command", commands[arrayIndex][4]);
//				SmartDashboard.putNumber("FWD", FWD);
//				SmartDashboard.putNumber("STR", STR);
//				SmartDashboard.putNumber("RCW", RCW);

				/*
				 * 0 = translate speed, 1 = rotate speed, 2 = direction to translate, 3 = direction to face,
				 * 4 = distance(in), 5 = How to accelerate(0 = no modification, 1 = transition, 2 = accelerate, 3 = decelerate)
				 * 6 = smoothArcStartAngle, 7 = smoothArcEndAngle,
				 * 8 = time out(seconds), 9 = imu offset
				 *
				 */

//				if (stepStarted) {
//					commands[arrayIndex][2] = MathUtils.resolveDeg(commands[arrayIndex][2] - commands[arrayIndex][12]);
//					commands[arrayIndex][3] = MathUtils.resolveDeg(commands[arrayIndex][3] - commands[arrayIndex][12]);
//					stepStarted = true;
//				}

				tSpeed = commands[arrayIndex][0];
				rSpeed = commands[arrayIndex][1];

				if (commands[arrayIndex][2] != -1) {
					FWD = Math.cos(Math.toRadians(commands[arrayIndex][2]));
					STR = Math.sin(Math.toRadians(commands[arrayIndex][2]));
				} else {
					FWD = 0;
					STR = 0;
				}

				if (commands[arrayIndex][6] <= 360.0 && commands[arrayIndex][6] >= 0.0) {
					smoothArc = Math.toRadians(MathUtils.convertRange(0.0, commands[arrayIndex][4], commands[arrayIndex][6], commands[arrayIndex][7], Math.abs(currentDistance - previousDistance)));
					FWD = Math.cos(smoothArc);
					STR = Math.sin(smoothArc);
				}

				keepAngle = commands[arrayIndex][3];
//				if (commands[arrayIndex][3] <= 360.0 && commands[arrayIndex][3] >= 0.0 && !turnDone) {
//				System.out.println(Math.abs(Math.abs(MathUtils.getAngleError(imu.getAngle(), commands[arrayIndex][3]))));
					if (Math.abs(MathUtils.getAngleError(imu.getAngle(), commands[arrayIndex][3])) < 5.0) {
						initialAngle = imu.getAngle();
						turnDone = true;
					} else {
						double direction;
						direction = MathUtils.getAngleError(initialAngle, commands[arrayIndex][3]);
						if (Math.abs(direction) > 180.0) {
							direction *= -1.0;
						}
						RCW = Math.signum(direction);
						turnDone = false;
					}
//				}

//				System.out.println(MathUtils.resolveDeg(commands[arrayIndex][8]-initialAngle));
//				System.out.println(RCW);

				if (commands[arrayIndex][5] == 1) {
					smoothAccelerateNum = (MathUtils.convertRange(previousDistance, previousDistance + commands[arrayIndex][4], commands[arrayIndex][0], commands[arrayIndex+1][0], SmartDashboard.getNumber("Distance", 0)));
					smoothAccelerate = smoothAccelerateNum;
					FWD *= smoothAccelerate;
					STR *= smoothAccelerate;
				} else if (commands[arrayIndex][5] == 2) {
					smoothAccelerateNum = (MathUtils.convertRange(previousDistance, previousDistance + commands[arrayIndex][4], minSpeed, commands[arrayIndex][0], SmartDashboard.getNumber("Distance", 0)));
					smoothAccelerate = smoothAccelerateNum;
					FWD *= smoothAccelerate;
					STR *= smoothAccelerate;
				} else if (commands[arrayIndex][5] == 3) {
					smoothAccelerateNum = (MathUtils.convertRange(previousDistance, previousDistance + commands[arrayIndex][4], commands[arrayIndex][0], minSpeed, SmartDashboard.getNumber("Distance", 0)));
					 smoothAccelerate = smoothAccelerateNum;
					FWD *= smoothAccelerate;
					STR *= smoothAccelerate;
				} else {
					FWD *= tSpeed;
					STR *= tSpeed;
				}

				Vector driveCommands;
				driveCommands = MathUtils.convertOrientation(Math.toRadians(imu.getAngle()), FWD, STR);
				FWD = driveCommands.getY();
				STR = driveCommands.getX();
				RCW *= rSpeed;

				if ((Math.abs(SmartDashboard.getNumber("Distance", 0) - previousDistance) >= commands[arrayIndex][4]) || commands[arrayIndex][4] == 0) {
					driveDone = true;
					STR = 0;
					FWD = 0;
				}

				SmartDashboard.putNumber("Auto Distance Gone", Math.abs(currentDistance - previousDistance));
				SmartDashboard.putNumber("Auto Distance Command", commands[arrayIndex][4]);

				SwerveCompensate.setTolerance(1);
//				if ((SwerveCompensate.onTarget() || commands[arrayIndex][3] == -1) && !(commands[arrayIndex][8] <= 360.0 && commands[arrayIndex][8] >= -360.0)) {
//					turnDone = true;
//					RCW = 0;
//					System.out.println("AAA");
//				}

				if (Time.get() > timeBase + commands[arrayIndex][8] && commands[arrayIndex][8] > 0) {
					override = true;
				} else if (commands[arrayIndex][8] == 0) {
					timeDone = true;
				}

				imuOffset = commands[arrayIndex][9];

				if (turnDone) {
					keepAngle();
				}

//				System.out.println(FWD);

				if (robotBackwards) {
					driveTrain.drive(new Vector(-STR, -FWD), -RCW);
				} else {
					driveTrain.drive(new Vector(STR, FWD), RCW);
				}

//				System.out.println("d: " + driveDone + " | t: " + turnDone);

				if (override) {
					driveDone = true;
					turnDone = true;
				}

				if (driveDone) {
					arrayIndex++;
					driveDone = false;
					initialAngle = imu.getAngle();
					previousDistance = SmartDashboard.getNumber("Distance", 0);//currentDistance;
					turnDone = false;
					timeDone = false;
					override = false;
					timeBase = Time.get();
					stepStarted = false;
				}
				break;
		}
	}

	public void teleopInit() {
//		jet.startTeleop();

		log.start();

		imu.setOffset(imuOffset);

		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setID(1);
		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setID(2);
		SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setID(4);
		SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setID(3);

	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		// LABEL teleop periodic
		autonomous = false;

		SmartDashboard.putNumber("IMU Angle", imu.getAngle());

		// calibration from smartdashboard
		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setOffset(SmartDashboard.getNumber("FR offset: ", 0));
		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setOffset(SmartDashboard.getNumber("FL offset: ", 0));
		SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setOffset(SmartDashboard.getNumber("BL offset: ", 0));
		SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setOffset(SmartDashboard.getNumber("BR offset: ", 0));

		SmartDashboard.putNumber("DistanceFR", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getDistance());
		SmartDashboard.putNumber("DistanceFL", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getDistance());
		SmartDashboard.putNumber("DistanceBL", SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getDistance());
		SmartDashboard.putNumber("DistanceBR", SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getDistance());

		if (xbox1.Back()) {
			imu.reset(0); // robot should be perpendicular to field when pressed.
		} else if (xbox1.Y()) {
			imu.reset(180);
		} else if (xbox1.X()) {
			imu.reset(270);
		} else if (xbox1.B()) {
			imu.reset(90);
		}

		// forward command (-1.0 to 1.0)
		FWD = -xbox1.LStickY() / 10.5 * Ds.getBatteryVoltage();

		// strafe command (-1.0 to 1.0)
		STR = xbox1.LStickX() / 10.5 * Ds.getBatteryVoltage();

		// Increase the time it takes for the robot to accelerate
		currentRampTime = Time.get();
		if (FWD != 0.0 || STR != 0.0 || RCW != 0.0) {
			if (wheelRamp < 1.0) {
				rampRate = currentRampTime - prevRampTime;

				// rampRate is x if this is set to a different equation
				wheelRamp = rampRate;
			} else {
				wheelRamp = 1.0;
			}

			FWD *= wheelRamp;
			STR *= wheelRamp;
		} else {
			prevRampTime = currentRampTime;
		}

		if (imu.collisionDetected()) {
			xbox1.rumbleRight(1.0);
			xbox1.rumbleLeft(1.0);
		} else {
			xbox1.stopRumble();
		}

//		if (rumble) {
//			xbox2.rumbleRight(0.5);
//			xbox2.rumbleLeft(0.5);
//			rumbleTime++;
//			if (rumbleTime > 20) {
//				rumble = false;
//			}
//		} else {
//			xbox2.stopRumble();
//		}

		SmartDashboard.putNumber("FWD", FWD);
		SmartDashboard.putNumber("STR", STR);
		SmartDashboard.putNumber("RCW", RCW);
		SmartDashboard.putNumber("IMU Angle", imu.getAngle());

		double headingDeg = imu.getAngle();
		double headingRad = Math.toRadians(headingDeg);

		currentOrientedButton = xbox1.A();
		if (currentOrientedButton && !previousOrientedButton) {
			fieldOriented = !fieldOriented;

		}
		previousOrientedButton = currentOrientedButton;

		if (fieldOriented) {
			Vector commands;
			commands = MathUtils.convertOrientation(headingRad, FWD, STR);
			FWD = commands.getY();
			STR = commands.getX();
		} else {
			if (!robotBackwards) {
				FWD *= -1;
				STR *= -1;
			}
		}

		SmartDashboard.putBoolean("Field Oriented", fieldOriented);

		if (xbox1.LStickButton()) {
			FWD = 0.0;
			STR = 0.0;
			RCW = 0.0;
		}

		if (xbox1.RStickButton()) {
			FWD = 0.0;
			STR = 0.0;
			RCW = 0.0;
		}

		SmartDashboard.putNumber("FWD", FWD);
		SmartDashboard.putNumber("STR", STR);
		SmartDashboard.putNumber("RCW", RCW);

		// rotate clockwise command (-1.0 to 1.0)
		// Limited to half speed because of wheel direction calculation issues when rotating quickly
		// Let robot rotate at full speed if it is not translating
		if (FWD + STR == 0.0) {
			RCW = xbox1.RStickX();
		} else {
			RCW = xbox1.RStickX() * 0.5;
		}

		keepAngle();

		if (robotBackwards) {
			driveTrain.drive(new Vector(-STR, -FWD), -RCW); // x = str, y = fwd, rotation = rcw
		} else {
			driveTrain.drive(new Vector(STR, FWD), RCW); // x = str, y = fwd, rotation = rcw
		}
	}

	public void robotPeriodic() {
		currentDistance += SwerveDrivetrain.getRobotDistance();
		SmartDashboard.putNumber("Distance", currentDistance);
	}

	public void disabledInit() {
//		jet.setDisabled();
		autoMove = 0;

		// When robot is turned on, disabledInit is called once
		if (disabled < 1) {
			System.out.println("Hello, I am Otto");
			disabled++;
		} else {
			System.out.println("Saving log file(s)");
			log.writeFromQueue();
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		// LABEL test
		double speed = (xbox1.RStickX() * 0.3);

		if (xbox1.DPad() != -1) {
			dx = xbox1.DPad();
		}

		System.out.println("FL Angle: " + MathUtils.resolveDeg(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getAngle()));
		System.out.println("BL Angle: " + MathUtils.resolveDeg(SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getAngle()));
		System.out.println("BR Angle: " + MathUtils.resolveDeg(SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getAngle()));
		System.out.println("FR Angle: " + MathUtils.resolveDeg(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getAngle()));

		// Move a single motor from the drivetrain depending on Dpad and right stick
		if (dx == 0) {
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(speed);

			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
		}

		if (dx == 45) {
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(speed);

			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
		}

		if (dx == 90) {
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(speed);

			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
		}

		if (dx == 135) {
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(speed);

			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
		}

		if (dx == 180) {
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(speed);

			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
		}

		if (dx == 225) {
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(speed);

			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
		}

		if (dx == 270) {
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(speed);

			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
		}

		if (dx == 315) {
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(speed);

			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
		}
	}
}
