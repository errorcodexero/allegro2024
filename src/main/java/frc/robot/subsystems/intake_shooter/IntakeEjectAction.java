package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class IntakeEjectAction extends Action {
    private IntakeShooterSubsystem sub_ ;
    private MotorEncoderPowerAction start_ ;
    private MCVelocityAction shooter1_ ;
    private MCVelocityAction shooter2_ ;
    private XeroTimer timer_ ;

    public IntakeEjectAction(IntakeShooterSubsystem sub) throws MissingParameterException, BadParameterTypeException, BadMotorRequestException, MotorRequestFailedException {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;
        start_ = new MotorEncoderPowerAction(sub_.getFeeder(), -0.5) ;
        timer_ = new XeroTimer(sub_.getRobot(), "eject-timer", 1.0) ;

        shooter1_ = new MCVelocityAction(sub.getShooter1(), "pids:velocity", -25.0, 1, false) ;
        shooter2_ = new MCVelocityAction(sub.getShooter2(), "pids:velocity", -25.0, 1, false) ;        
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.getFeeder().setAction(start_, true) ;
        timer_.start() ;

        sub_.getShooter1().setAction(shooter1_, true) ;
        sub_.getShooter2().setAction(shooter2_, true) ;
        sub_.setHoldingNote(false);
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (timer_.isExpired()) {
            sub_.getFeeder().setPower(0.0) ;
            sub_.getShooter1().setPower(0.0);
            sub_.getShooter2().setPower(0.0);
            setDone() ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "IntakeEjectAction" ;
    }
}
