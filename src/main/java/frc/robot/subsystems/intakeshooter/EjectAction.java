package frc.robot.subsystems.intakeshooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;

public class EjectAction extends Action {
    private IntakeShooterSubsystem sub_ ;

    private MCVelocityAction shooter1_action_ ;
    private MCVelocityAction shooter2_action_ ;
    private MCVelocityAction spinner_feeder_action_ ;
    private MCMotionMagicAction tilt_action_ ;
    private MCMotionMagicAction updown_action_ ;
    private XeroTimer timer_ ;

    public EjectAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;

        updown_action_ = new MCMotionMagicAction(sub_.updown(), "pids:position", "target:eject", 1, 1) ;
        tilt_action_ = new MCMotionMagicAction(sub_.tilt(), "pids:position", "targets:eject", 1, 1);
        shooter1_action_ = new MCVelocityAction(sub_.shooter1(), "pids:velocity", "target:eject") ;
        shooter2_action_ = new MCVelocityAction(sub_.shooter2(), "pids:velocity", "target:eject") ;
        spinner_feeder_action_ = new MCVelocityAction(sub_.spinner_feeder(), "pids:velocity", "target:eject") ;

        timer_ = new XeroTimer(sub.getRobot(), "intake-eject", sub.getSettingsValue("actions:eject:duration").getDouble());
    }   

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.updown().setAction(updown_action_);
        sub_.tilt().setAction(tilt_action_) ;
        sub_.shooter1().setAction(shooter1_action_) ;
        sub_.shooter2().setAction(shooter2_action_) ;
        sub_.spinner_feeder().setAction(spinner_feeder_action_) ;
        timer_.start();
    }

    @Override
    public void run() throws Exception {
        if (timer_.isExpired()) {
            sub_.shooter1().setPower(0.0);
            sub_.shooter2().setPower(0.0);
            sub_.spinner_feeder().setPower(0.0);

            //
            // TODO: If we can just go up, we can do this here, otherwise if we have to sequence 
            //       the tilt and up/down we need a new action
            //

            setDone() ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "EjectAction" ;
    }
}
