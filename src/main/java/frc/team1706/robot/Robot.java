package frc.team1706.robot;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;
import java.util.Properties;

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

import edu.wpi.first.wpilibj.Compressor;
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

	private Compressor compressor;

	private XboxController xbox1 = new XboxController(0);

	private int autonomousChoice;
	private JetsonServer jet;
	private Thread t;
	private SwerveDrivetrain driveTrain;
	private IMU imu;
	private RRLogger log;

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
	private double previousDistance;
	private double currentDistance;
	private double currentTime = 0;
	private boolean driveDone;
	private boolean turnDone;
	private boolean timeDone;
	private boolean collisionDone;
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
	}

	private void autonomousAngle(double angle) {
		// LABEL autonomous angle

		SwerveCompensate.setInput(imu.getAngle());
		SwerveCompensate.setSetpoint(angle);

		// FIXME try having move to next step immediately, like old code
		SwerveCompensate.setTolerance(7);
		if (!SwerveCompensate.onTarget()) {
			SwerveCompensate.setPID(0.005, SmartDashboard.getNumber("CompensateI", 0.0), SmartDashboard.getNumber("CompensateD", 0.0));
		} else {
			SwerveCompensate.setPID(0.04, SmartDashboard.getNumber("CompensateI", 0.0), SmartDashboard.getNumber("CompensateD", 0.0));
		}

		robotRotation = SwerveCompensate.performPID();

		RCW = (robotRotation);
	}

	/**
	 * This function is run when the robot is first started up and should be used for any initialization code.
	 */
	public void robotInit() {
		// LABEL robot init
		compressor = new Compressor(0);

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
		autoChooser.addDefault("Shoot Only", 1);
		autoChooser.addObject("RGear Only", 2);
		SmartDashboard.putData("Autonomous Mode Chooser", autoChooser);

		log = new RRLogger();

		// Connect to jetson
		try {
			jet = new JetsonServer((short) 5800);
			t = new Thread(jet);
			t.start();
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

		SwerveCompensate.enable();
	}

	public void autonomousInit() {
		// LABEL autonomous init

		timeCheck = true;
		imu.reset();

		autonomousChoice = autoChooser.getSelected();

		String choice;

		if (autonomousChoice == 1) {
			choice = "/home/lvuser/FILENAME.csv";
		} else if (autonomousChoice == 2) {
			choice = "/home/lvuser/FILENAME2.csv";
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
				String[] parts = line.split(",");
				double[] linel = new double[parts.length];
				int i = 0;
				for (String part : parts) {
					linel[i++] = Double.parseDouble(part);
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
			System.out.println(timeBase);
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
				currentDistance = SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getDistance();
				previousDistance = currentDistance;

				break;

			case 1:

				SmartDashboard.putNumber("Array Index", arrayIndex);
				SmartDashboard.putNumber("IMU Angle", imu.getAngle());

				/*
				 * 0 = translate speed, 1 = rotate speed, 2 = direction to translate, 3 = direction to face,
				 * 4 = distance(in), 6 = time out(seconds), 7 = check for collision,
				 * 10 = imu offset, create new
				 */
				currentDistance = SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getDistance();

				tSpeed = commands[arrayIndex][0];
				rSpeed = commands[arrayIndex][1];

				if (commands[arrayIndex][2] != -1) {
					FWD = Math.cos(MathUtils.degToRad(commands[arrayIndex][2]));
					STR = Math.sin(MathUtils.degToRad(commands[arrayIndex][2]));
				} else {
					FWD = 0;
					STR = 0;
				}

				autonomousAngle = commands[arrayIndex][3];

				if (commands[arrayIndex][4] != -1) {
					AutoTranslate.setInput(Math.abs(currentDistance - previousDistance));
					AutoTranslate.setSetpoint(commands[arrayIndex][4]);
					PIDSpeed = AutoTranslate.performPID();
				}

				SmartDashboard.putNumber("Array Index", arrayIndex);
				SmartDashboard.putNumber("Vel X", imu.getVelocityX());
				SmartDashboard.putNumber("Vel Y", imu.getVelocityY());

				Vector driveCommands;
				driveCommands = MathUtils.convertOrientation(MathUtils.degToRad(imu.getAngle()), FWD, STR);
				FWD = driveCommands.getY() * tSpeed * PIDSpeed;
				STR = driveCommands.getX() * tSpeed * PIDSpeed;

				autonomousAngle = commands[arrayIndex][3];

				// FIXME
				if (autonomousAngle != -1) {
					autonomousAngle(autonomousAngle);
				}

				RCW *= rSpeed;

				if ((Math.abs(currentDistance - previousDistance) >= commands[arrayIndex][4]) || commands[arrayIndex][4] == 0) {
					driveDone = true;
					STR = 0;
					FWD = 0;
				}

				SmartDashboard.putNumber("Auto Distance Gone", Math.abs(currentDistance - previousDistance));
				SmartDashboard.putNumber("Auto Distance Command", commands[arrayIndex][4]);

				SwerveCompensate.setTolerance(1);
				if (SwerveCompensate.onTarget() || commands[arrayIndex][3] == -1) {
					turnDone = true;
					RCW = 0;
				}

				if (Time.get() > timeBase + commands[arrayIndex][6] && commands[arrayIndex][6] != 0) {
					timeDone = true;
					collisionDone = true;
					driveDone = true;
					turnDone = true;
				} else if (commands[arrayIndex][6] == 0) {
					timeDone = true;
				}

				if (commands[arrayIndex][7] == 1) {
					if (imu.collisionDetected()) {
						collisionDone = true;
						driveDone = true;
						turnDone = true;
						timeDone = true;
					}
				} else {
					collisionDone = true;
				}

				imuOffset = commands[arrayIndex][10];

				SmartDashboard.putNumber("FWD", FWD);
				SmartDashboard.putNumber("STR", STR);
				SmartDashboard.putNumber("RCW", RCW);

				driveTrain.drive(new Vector(STR, FWD), RCW);

//				System.out.println("Drive: " + driveDone);
//				System.out.println("Turn: " + turnDone);
//				System.out.println("Coll: " + collisionDone);
//				System.out.println("Time: " + timeDone);

				if (driveDone && turnDone && collisionDone) {
					arrayIndex++;
					driveDone = false;
					previousDistance = currentDistance;
					turnDone = false;
					timeDone = false;
					collisionDone = false;
					timeBase = Time.get();

				}
				break;
		}
	}

	public void teleopInit() {

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
			// TODO this may change depending on robot
			FWD *= -1;
			STR *= -1;
		}

		SmartDashboard.putBoolean("Field Oriented", fieldOriented);

		SmartDashboard.putNumber("FWD", FWD);
		SmartDashboard.putNumber("STR", STR);
		SmartDashboard.putNumber("RCW", RCW);

		if (slow) {
			STR /= 2;
			FWD /= 2;
		}

		keepAngle();

		driveTrain.drive(new Vector(STR, FWD), RCW); // x = str, y = fwd, rotation = rcw

	}

	public void disabledInit() {
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

	public void disabledPeriodic() {

	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		// LABEL test
		LiveWindow.run();

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
