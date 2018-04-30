package frc.team1706.robot;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;
import java.util.Properties;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.team1706.robot.subsystems.IMU;
import frc.team1706.robot.subsystems.JetsonServer;
//import frc.team1706.robot.subsystems.PowerPanel;
import frc.team1706.robot.subsystems.SwerveDrivetrain;
import frc.team1706.robot.subsystems.SwerveDrivetrain.WheelType;
import frc.team1706.robot.subsystems.Time;
import frc.team1706.robot.subsystems.XboxController;
import frc.team1706.robot.utilities.MathUtils;
import frc.team1706.robot.utilities.PIDController;
import frc.team1706.robot.utilities.Vector;
import frc.team1706.robot.RRLogger;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the manifest file in the
 * resource directory.
 */
public class Robot extends IterativeRobot {

	private SendableChooser<Integer> autoChooser;

//	private Compressor compressor;

	private XboxController xbox1 = new XboxController(0);

	private int autonomousChoice;
	private JetsonServer jet;
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
	private double time = 0;
	private double autonomousAngle;
	private double tSpeed;
	private double rSpeed;
	private double PIDSpeed;
	private double initialPitch;
	private double previousDistanceFR;
	private double currentDistanceFR;
	private double previousDistanceFL;
	private double currentDistanceFL;
	private double previousDistanceBL;
	private double currentDistanceBL;
	private double previousDistanceBR;
	private double currentDistanceBR;
	private double currentTime = 0;
	private boolean FRDone;
	private boolean FLDone;
	private boolean BLDone;
	private boolean BRDone;
	private boolean turnDone;
	private boolean timeDone;
	private boolean collisionDone;
	private boolean moonDone;
	private double offsetDeg;
	private double prevOffset = 0;
	private double timeBase;
	private boolean timeCheck;

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

	private double prevVoltTime = 0;
	private double currentVoltTime = 0;

	private boolean fieldOriented = true; // start on field orientation
	private boolean previousOrientedButton = false;
	private boolean currentOrientedButton = false;

	private boolean slow = false;
	private boolean previousSlowButton = false;
	private boolean currentSlowButton = false;

	private double command = 0;

	private PIDController SwerveCompensate;
	private PIDController AutoTranslate;

	private double lead;

	private double robotRotation;

	private double imuOffset = 0;

	private Properties application = new Properties();
	private File offsets = new File("/home/lvuser/SWERVE_OFFSET.txt");

	AnalogInput test = new AnalogInput(0);

	private void keepAngle() {
		// LABEL keepAngle

		SwerveCompensate.enable();

		if (xbox1.DPad() != -1) {
			keepAngle = xbox1.DPad();
		}

		double leadNum = SmartDashboard.getNumber("leadNum", 0);
		lead = RCW * leadNum;

		SmartDashboard.putNumber("DPAD", xbox1.DPad());

		// This will update the angle to keep the robot's orientation
		if (Math.abs(RCW) > 0.132 || // If right stick is pressed
				(Math.abs(FWD) < 0.01 && Math.abs(STR) < 0.01) && // If left stick is not pressed
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

		// Set the offset of each wheel from a file on the roborio
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

	private void autonomousAngle(double angle) {
		// LABEL autonomous angle

		SwerveCompensate.setInput(imu.getAngle());
		SwerveCompensate.setSetpoint(angle);

		// FIXME try having move to next step immediately, like old code or just use this part for keeping straight and make new for only rotation
		SwerveCompensate.setTolerance(7);
		if (!SwerveCompensate.onTarget()) {
			SwerveCompensate.setPID(0.013, SmartDashboard.getNumber("CompensateI", 0.0), SmartDashboard.getNumber("CompensateD", 0.0));
		} else {
			SwerveCompensate.setPID(0.015, SmartDashboard.getNumber("CompensateI", 0.0), SmartDashboard.getNumber("CompensateD", 0.0));
		}

		robotRotation = SwerveCompensate.performPID();

		RCW = (robotRotation);
	}

	/**
	 * This function is run when the robot is first started up and should be used for any initialization code.
	 */
	public void robotInit() {
		// LABEL robot init
//		compressor = new Compressor(0);

		// Load the wheel offset file from the roborio
		try {
			FileInputStream in = new FileInputStream(offsets);
			application.load(in);
		} catch (IOException e) {
			e.printStackTrace();
		}

		Time.start();

		SwerveDrivetrain.loadPorts();

		SmartDashboard.putNumber("2018 SRX Test", 0);

		SmartDashboard.putNumber("CompensateP", 0.02);
		SmartDashboard.putNumber("CompensateI", 0.0);
		SmartDashboard.putNumber("CompensateD", 0.0);

		SmartDashboard.putNumber("Autonomous Delay", 0);

		autoChooser = new SendableChooser<>();
		autoChooser.addDefault("Middle", 1);
		autoChooser.addObject("Left", 2);
		autoChooser.addObject("Right", 3);
		SmartDashboard.putData("Autonomous Mode Chooser", autoChooser);

		log = new RRLogger();

		// Connect to jetson
		try {
			jet = new JetsonServer((short) 5800, (short) 5801);
			t = new Thread(jet);
			t.start();
			jet.setDisabled();
		} catch (IOException e) {
			throw new RuntimeException(e);
		}

		driveTrain = new SwerveDrivetrain();
		loadOffsets();

		imu = new IMU();
		imu.IMUInit();

		SwerveCompensate = new PIDController(0.015, 0.00, 0.00);
		SwerveCompensate.setContinuous(true);
		SwerveCompensate.setOutputRange(-1.0, 1.0);
		SwerveCompensate.setInputRange(0.0, 360.0);
		SwerveCompensate.setTolerance(1.0);

		AutoTranslate = new PIDController(01.000, 0.0, 0.0);
		AutoTranslate.setContinuous(false);
		AutoTranslate.setOutputRange(-1.0, 1.0);
		AutoTranslate.setInputRange(0.0, 250.0);

		SwerveCompensate.enable();
		AutoTranslate.enable();
	}

	public void autonomousInit() {
		// LABEL autonomous init
		jet.setAuto(); // this line is important because it does clock synchronization

		String gameData = m_ds.getGameSpecificMessage();
		char switchSide = gameData.charAt(0);
		char scaleSide = gameData.charAt(1);

		timeCheck = true;
		imu.reset();

		autonomousChoice = autoChooser.getSelected();

		String choice;

		if (autonomousChoice == 1) {
			if (switchSide == 'L') {
				if (scaleSide == 'L') {
					choice = "/home/lvuser/MidSwitchLScaleL.csv";
				} else {
					choice = "/home/lvuser/MidSwitchLScaleR.csv";
				}
			} else {
				if (scaleSide == 'L') {
					choice = "/home/lvuser/MidSwitchRScaleL.csv";
				} else {
					choice = "/home/lvuser/MidSwitchRScaleR.csv";
				}
			}

		} else if (autonomousChoice == 2) {
			if (switchSide == 'L') {
				if (scaleSide == 'L') {
					choice = "/home/lvuser/LeftSwitchLScaleL.csv";
				} else {
					choice = "/home/lvuser/LeftSwitchLScaleR.csv";
				}
			} else {
				if (scaleSide == 'L') {
					choice = "/home/lvuser/LeftSwitchRScaleL.csv";
				} else {
					choice = "/home/lvuser/LeftSwitchRScaleR.csv";
				}
			}

		} else if (autonomousChoice == 3) {
			if (switchSide == 'L') {
				if (scaleSide == 'L') {
					choice = "/home/lvuser/RightSwitchLScaleL.csv";
				} else {
					choice = "/home/lvuser/RightSwitchLScaleR.csv";
				}
			} else {
				if (scaleSide == 'L') {
					choice = "/home/lvuser/RightSwitchRScaleL.csv";
				} else {
					choice = "/home/lvuser/RightSwitchRScaleR.csv";
				}
			}

		} else {
			choice = "/home/lvuser/Stopped.csv";
		}

		SmartDashboard.putString("Autonomous File", choice);

		time = 0;
		arrayIndex = 0;

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
				currentDistanceFR = SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getDistance();
				currentDistanceFL = SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getDistance();
				currentDistanceBL = SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getDistance();
				currentDistanceBR = SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getDistance();

				previousDistanceFR = currentDistanceFR;
				previousDistanceFL = currentDistanceFL;
				previousDistanceBL = currentDistanceBL;
				previousDistanceBR = currentDistanceBR;

				break;

			case 1:

				SmartDashboard.putNumber("Array Index", arrayIndex);
				SmartDashboard.putNumber("IMU Angle", imu.getAngle());

				/*
				 * 0, 1,  2 = FR Power, Angle, Dist
				 * 3, 4,  5 = FL Power, Angle, Dist
				 * 6, 7,  8 = BL Power, Angle, Dist
				 * 9, 10, 11 = BR Power, Angle, Dist
				 * 12 = Robot Heading
				 */
				currentDistanceFR = SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getDistance();
				currentDistanceFL = SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getDistance();
				currentDistanceBL = SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getDistance();
				currentDistanceBR = SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getDistance();

				SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setSpeedCommand(commands[arrayIndex][0]);
				SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setAngleCommand(commands[arrayIndex][1]);

				SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setSpeedCommand(commands[arrayIndex][3]);
				SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setAngleCommand(commands[arrayIndex][4]);

				SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setSpeedCommand(commands[arrayIndex][6]);
				SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setAngleCommand(commands[arrayIndex][7]);

				SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setSpeedCommand(commands[arrayIndex][9]);
				SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setAngleCommand(commands[arrayIndex][10]);

				System.out.println(commands[arrayIndex][0]);
				SmartDashboard.putNumber("Array Index", arrayIndex);

				if ((Math.abs(currentDistanceFR - previousDistanceFR) >= commands[arrayIndex][2]) || commands[arrayIndex][2] == 0) {
					FRDone = true;
				}
				if ((Math.abs(currentDistanceFL - previousDistanceFL) >= commands[arrayIndex][5]) || commands[arrayIndex][5] == 0) {
					FLDone = true;
				}
				if ((Math.abs(currentDistanceBL - previousDistanceBL) >= commands[arrayIndex][8]) || commands[arrayIndex][8] == 0) {
					BLDone = true;
				}
				if ((Math.abs(currentDistanceBR - previousDistanceBR) >= commands[arrayIndex][11]) || commands[arrayIndex][11] == 0) {
					BRDone = true;
				}

				try {
					imuOffset = commands[arrayIndex][12];
				} catch (NullPointerException e) {}

//				if (robotBackwards) {
//					driveTrain.drive(new Vector(-STR, -FWD), -RCW);
//				} else {
//					driveTrain.drive(new Vector(STR, FWD), RCW);
//				}

				if (FRDone && FLDone && BLDone && BRDone) {
					arrayIndex++;
					previousDistanceFR = currentDistanceFR;
					previousDistanceFL = currentDistanceFL;
					previousDistanceBL = currentDistanceBL;
					previousDistanceBR = currentDistanceBR;

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

//		log.newLine();
//		log.newPowerLine();
//
//		log.addPower("BR", PowerPanel.j());
//		log.addPower("BL", PowerPanel.k());
//		log.addPower("FL", PowerPanel.f());
//		log.addPower("FR", PowerPanel.h());

		SmartDashboard.putNumber("Distance", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getDistance());

		if (xbox1.Back()) {
			imu.reset(); // robot should be perpendicular to field when pressed.
		}

		// forward command (-1.0 to 1.0)
		FWD = -xbox1.LStickY() / 10.5 * Ds.getBatteryVoltage();

		// strafe command (-1.0 to 1.0)
		STR = xbox1.LStickX() / 10.5 * Ds.getBatteryVoltage();

		// rotate clockwise command (-1.0 to 1.0)
		// Limited to half speed because of wheel direction calculation issues when rotating quickly
		RCW = xbox1.RStickX() * 0.5;

		// Deadbands for joysticks
		if (Math.abs(xbox1.LStickY()) <= 0.11) {
			FWD = 0.0;
		} else if (FWD > 1.0) {
			FWD = 1.0;
		} else if (FWD < -1.0) {
			FWD = -1.0;
		}

		if (Math.abs(xbox1.LStickX()) <= 0.079) {
			STR = 0.0;
		} else if (STR > 1.0) {
			STR = 1.0;
		} else if (STR < -1.0) {
			STR = -1.0;
		}

		if (Math.abs(xbox1.RStickX()) <= 0.016) {
			RCW = 0.0;
		}

		// Let robot rotate at full speed if it is not translating
		if (FWD + STR == 0.0) {
			RCW = xbox1.RStickX();
		}

		// Increase the time it takes for the robot to accelerate
		currentRampTime = Time.get();
		if (FWD != 0.0 || STR != 0.0 || RCW != 0.0) {
			if (wheelRamp < 1.0) {
				rampRate = currentRampTime - prevRampTime;

				//rampRate is x if this is set to a different equation later
				wheelRamp = rampRate;
			} else {
				wheelRamp = 1.0;
			}

			FWD *= wheelRamp;
			STR *= wheelRamp;
			RCW *= wheelRamp;
		} else {
			prevRampTime = currentRampTime;
		}

		if (imu.collisionDetected()) {
			xbox1.rumble();
		} else {
			xbox1.stopRumble();
		}

		SmartDashboard.putNumber("FWD", FWD);
		SmartDashboard.putNumber("STR", STR);
		SmartDashboard.putNumber("RCW", RCW);
		SmartDashboard.putNumber("IMU Angle", imu.getAngle());

		double headingDeg = imu.getAngle();
		double headingRad = MathUtils.degToRad(headingDeg);

		currentSlowButton = xbox1.X();
		if (currentSlowButton && !previousSlowButton) {
			slow = !slow;

		}
		previousSlowButton = currentSlowButton;

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
			STR = -0.85;
			RCW = 0.25;
		}

		if (xbox1.RStickButton()) {
			FWD = 0.0;
			STR = 0.95/2;
			RCW = -0.52/2;
		}

		SmartDashboard.putNumber("FWD", FWD);
		SmartDashboard.putNumber("STR", STR);
		SmartDashboard.putNumber("RCW", RCW);

		if (slow) {
			STR /= 2;
			FWD /= 2;
		}

		keepAngle();

		if (robotBackwards) {
			driveTrain.drive(new Vector(-STR, -FWD), -RCW); // x = str, y = fwd, rotation = rcw
		} else {
			driveTrain.drive(new Vector(STR, FWD), RCW); // x = str, y = fwd, rotation = rcw
		}

		SmartDashboard.putNumber("Pot Test", test.getValue());

	}

	public void disabledInit() {
		jet.setDisabled();
		autoMove = 0;

		// When robot is turned on, disabledInit is called once
		if (disabled < 1) {
			System.out.println("Hello");
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
//		LiveWindow.run();

		double speed = (xbox1.RStickX() * 0.3);

		if (xbox1.DPad() != -1) {
			dx = xbox1.DPad();
		}

		System.out.println(xbox1.DPad());

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
