package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class IntakeEjectAction extends Action {

    private final double kPeriod1 = 1.0 ;
    private final double kPeriod2 = 0.5 ;
    private final double kPeriod3 = 1.0 ;
    private final double kPower = 0.7 ;

    private enum State {
        Forward,
        Paused,
        Reverse
    } ;

    private IntakeShooterSubsystem sub_ ;
    private MotorEncoderPowerAction start_ ;
    private MotorEncoderPowerAction power1f_ ;
    private MotorEncoderPowerAction power2f_ ;
    private MotorEncoderPowerAction power1r_ ;
    private MotorEncoderPowerAction power2r_ ;
    private double curlim1_ ;
    private double curlim2_ ;
    private double start_time_ ;
    private State state_ ;

    public IntakeEjectAction(IntakeShooterSubsystem sub) throws MissingParameterException, BadParameterTypeException, BadMotorRequestException, MotorRequestFailedException {
        super(sub.getRobot()) ;

        sub_ = sub ;
        start_ = new MotorEncoderPowerAction(sub_.getFeeder(), -0.5) ;

        power1f_ = new MotorEncoderPowerAction(sub_.getShooter1(), -kPower) ;
        power2f_ = new MotorEncoderPowerAction(sub_.getShooter2(), -kPower) ;      
        power1r_ = new MotorEncoderPowerAction(sub_.getShooter1(), kPower) ;
        power2r_ = new MotorEncoderPowerAction(sub_.getShooter2(), kPower) ;           

        curlim1_ = sub_.getShooter1().getMotorController().getCurrentLimit() ;
        curlim2_ = sub_.getShooter2().getMotorController().getCurrentLimit() ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.getFeeder().setAction(start_, true) ;
        start_time_ = sub_.getRobot().getTime() ;
        state_ = State.Forward ;

        sub_.getShooter1().setAction(power1f_, true) ;
        sub_.getShooter2().setAction(power2f_, true) ;
        sub_.setHoldingNote(false);
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        double elapsed = sub_.getRobot().getTime() - start_time_ ;

        if (elapsed > kPeriod1 && state_ == State.Forward) {
            state_ = State.Paused ;
            sub_.getShooter1().setPower(0.0);
            sub_.getShooter2().setPower(0.0);            
        }
        else if (elapsed > kPeriod1 + kPeriod2 && state_ == State.Paused) {
            sub_.getShooter1().setAction(power1r_, true) ;
            sub_.getShooter2().setAction(power2r_, true) ;
            state_ = State.Reverse ;
        }
        else if (elapsed > kPeriod1 + kPeriod2 + kPeriod3 && state_ == State.Reverse) {
            sub_.getFeeder().setPower(0.0) ;
            sub_.getShooter1().setPower(0.0);
            sub_.getShooter2().setPower(0.0);

            sub_.getShooter1().getMotorController().setCurrentLimit(curlim1_);
            sub_.getShooter2().getMotorController().setCurrentLimit(curlim2_);

            setDone() ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "IntakeEjectAction" ;
    }
}
