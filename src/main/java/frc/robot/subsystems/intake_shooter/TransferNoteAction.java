package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;

public class TransferNoteAction extends Action {
    private IntakeShooterSubsystem sub_ ;
    private MCVelocityAction spinner_feeder_xfer_ ;
    private MCVelocityAction shooter1_xfer_ ;
    private MCVelocityAction shooter2_xfer_ ;
    private XeroTimer timer_ ;

    public TransferNoteAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;
        spinner_feeder_xfer_ = new MCVelocityAction(sub_.getFeeder(), "pids:velocity", "targets:transfer") ;
        shooter1_xfer_ = new MCVelocityAction(sub_.getShooter1(), "pids:velocity", "targets:transfer") ;
        shooter2_xfer_ = new MCVelocityAction(sub_.getShooter2(), "pids:velocity", "targets:transfer") ;

        double duration = sub.getSettingsValue("actions:transfer:duration").getDouble() ;
        timer_ = new XeroTimer(sub.getRobot(), "intake-transfer", duration);
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.getFeeder().setAction(spinner_feeder_xfer_);
        sub_.getShooter1().setAction(shooter1_xfer_);
        sub_.getShooter2().setAction(shooter2_xfer_);
        timer_.start();
    }

    @Override 
    public void run() throws Exception {
        if (timer_.isExpired()) {
            sub_.getFeeder().setPower(0.0) ;
            sub_.getShooter1().setPower(0.0);
            sub_.getShooter2().setPower(0.0);
            setDone();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "TransferNoteAction" ;
    }
}
