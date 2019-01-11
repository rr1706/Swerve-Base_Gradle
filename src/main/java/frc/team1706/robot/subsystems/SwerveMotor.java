package frc.team1706.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.team1706.robot.utilities.MathUtils;

class SwerveMotor {
    private static final int POSITION_MOTION_MAGIC_IDX = 0;
    private static final int REMOTE_0 = 0;
    private static final int TIMEOUT = 10;
    public static final int STEERING_COUNTS_PER_REV = 800;

    private WPI_TalonSRX clockwiseMotor;
    private WPI_TalonSRX counterMotor;

    /**
     *
     * @param canPortC Port of the motor that moves the wheel Clockwise
     * @param canPortCC Port of the motor that moves the wheel CounterClockwise
     */
    SwerveMotor(int canPortC, int canPortCC){
        //FIXME switch to spark max when available
        clockwiseMotor = new WPI_TalonSRX(canPortC);
        counterMotor = new WPI_TalonSRX(canPortCC);

        //set (M1_ENC + M2_ENC)/2 to be feedback sensor on M2
        clockwiseMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,POSITION_MOTION_MAGIC_IDX, TIMEOUT);

        counterMotor.configRemoteFeedbackFilter(clockwiseMotor.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, REMOTE_0, TIMEOUT);
        counterMotor.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, TIMEOUT);
        counterMotor.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, TIMEOUT);
        counterMotor.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, POSITION_MOTION_MAGIC_IDX, TIMEOUT);
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
     * @return (sum of both encoders) % (the number of ticks per rev), so that the position wraps around
     */
    int getAngle() {
        return MathUtils.resolveHalfAngleNative(counterMotor.getSelectedSensorPosition(POSITION_MOTION_MAGIC_IDX), STEERING_COUNTS_PER_REV);
    }

    /**
     * @return Distance the module has translated
     */
    int getDistance() {
        return counterMotor.getSelectedSensorPosition(POSITION_MOTION_MAGIC_IDX);
    }

    public void reset() {
        clockwiseMotor.getSensorCollection().setQuadraturePosition(0, TIMEOUT);
        counterMotor.getSensorCollection().setQuadraturePosition(0, TIMEOUT);
    }
}
