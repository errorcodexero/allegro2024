package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

public class ManualShootAction extends Action {
    private final double kTiltPosition = -37.0;
    private final double kUpDownPosition = 90.0; 
    private final double kShooterVelocity = 30.0;
    private final double kShooterThreshold = 5.0 ;

    private IntakeShooterSubsystem intake_;
    private MCMotionMagicAction tilt_action_ ;
    private MCMotionMagicAction up_down_action_ ;
    private MCVelocityAction shooter1_action_ ;
    private MCVelocityAction shooter2_action_ ;
    private MotorEncoderPowerAction feeder_action_ ;

    private boolean shooting_ ;
    private boolean tilt_ready_ ;
    private boolean updown_ready_ ;

    public ManualShootAction(IntakeShooterSubsystem intake) throws Exception {
        super(intake.getRobot().getMessageLogger());
        intake_ = intake;

        tilt_action_ = new MCMotionMagicAction(intake.getTilt(), "pids:position", kTiltPosition, 5.0, 5.0) ;
        up_down_action_ = new MCMotionMagicAction(intake_.getUpDown(), "pids:position", kUpDownPosition, 5.0, 5.0);
        shooter1_action_ = new MCVelocityAction(intake_.getShooter1(), "pids:velocity", kShooterVelocity, false) ;
        shooter2_action_ = new MCVelocityAction(intake_.getShooter2(), "pids:velocity", kShooterVelocity, false) ;
        feeder_action_ = new MotorEncoderPowerAction(intake_.getFeeder(), 0.5, 2.0) ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        intake_.getTilt().setAction(tilt_action_, true) ;
        intake_.getUpDown().setAction(up_down_action_, true) ;
        intake_.getShooter1().setAction(shooter1_action_, true) ;
        intake_.getShooter2().setAction(shooter2_action_, true) ;

        tilt_ready_ = false ;
        updown_ready_ = false ;
        shooting_ = false ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        if (!shooting_) {

            boolean s1ready = false ;
            boolean s2ready = false ;

            if (tilt_action_.isDone()) {
                tilt_ready_ = true ;
            }

            if (up_down_action_.isDone()) {
                updown_ready_ = true ;
            }

            if (Math.abs(intake_.getShooter1().getVelocity() - kShooterVelocity) < kShooterThreshold) {
                s1ready = true ;
            }

            if (Math.abs(intake_.getShooter2().getVelocity() - kShooterVelocity) < kShooterThreshold) {
                s2ready = true ;
            }

            if (s1ready && s2ready && tilt_ready_ && updown_ready_) {
                shooting_ = true ;
                intake_.getFeeder().setAction(feeder_action_, true) ;
            }
        }
        else {
            if (feeder_action_.isDone()) {
                intake_.getShooter1().setPower(0.0) ;
                intake_.getShooter2().setPower(0.0) ;
                setDone() ;
            }
        }   
    }

    @Override
    public void cancel() {
        super.cancel();
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "ManualShootAction";
    }
}
