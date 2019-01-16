package frc.team1706.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.team1706.robot.utilities.MathUtils;

class SwerveMotor {
    private static final double STEERING_COUNTS_PER_REV = 10.0;

    private CANSparkMax clockwiseMotor;
    private CANSparkMax counterMotor;

    private CANEncoder clockwiseEncoder;
    private CANEncoder counterEncoder;

    /**
     *
     * @param canPortC Port of the motor that moves the wheel Clockwise
     * @param canPortCC Port of the motor that moves the wheel CounterClockwise
     */
    SwerveMotor(int canPortC, int canPortCC){
        clockwiseMotor = new CANSparkMax(canPortC, CANSparkMaxLowLevel.MotorType.kBrushless);
        counterMotor = new CANSparkMax(canPortCC, CANSparkMaxLowLevel.MotorType.kBrushless);
        clockwiseEncoder = new CANEncoder(clockwiseMotor);
        counterEncoder = new CANEncoder(counterMotor);

        clockwiseMotor.setInverted(false);
        counterMotor.setInverted(true);
    }

    /**
     *
     * @param speedCommand Moves wheel forward
     * @param rotationCommand Moves wheel clockwise
     */
    void set(double speedCommand, double rotationCommand){
        double clockwiseCommand = speedCommand + rotationCommand;
        double counterCommand = speedCommand - rotationCommand;
        double clockwiseOvershoot = Math.abs(clockwiseCommand) - 1;
        double counterOvershoot = Math.abs(counterCommand) - 1;

        if (clockwiseOvershoot > 0) {
            clockwiseCommand -= clockwiseOvershoot * Math.signum(clockwiseCommand);
            counterCommand -= clockwiseOvershoot * Math.signum(clockwiseCommand);
        }

        if (counterOvershoot > 0) {
            counterCommand -= counterOvershoot * Math.signum(counterCommand);
            clockwiseCommand -= counterOvershoot * Math.signum(counterCommand);
        }

        clockwiseMotor.set(clockwiseCommand);
        counterMotor.set(counterCommand);
    }

    /**
     * @return sum of both encoders, wrapped from 0 to 360
     */
    double getAngle() {
        return MathUtils.resolveAngleNative(getDistance(), STEERING_COUNTS_PER_REV)*36.0;
    }

    /**
     * @return Distance the module has translated
     */
    double getDistance() {
        return clockwiseEncoder.getPosition() - counterEncoder.getPosition();
    }

    public void reset() {
//        clockwiseEncoder.;
//        counterEncoder.;
    }
}
