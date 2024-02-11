package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.misc.MessageType;

public class ButchStartCollectAction extends Action {
    private enum CollectState {
        Idle,
        TiltMoving,
        BothMoving,
        WaitingForNote,
        WaitingForCollect,
        Stowing,
        Done
    } ;

    private IntakeShooterSubsystem sub_ ;
    private CollectState state_ ;
    private double tilt_threshold_ ;
    private double feeder_power_ ;

    private XeroTimer timer_ ;

    private MCMotionMagicAction intake_collect_ ;
    private MCMotionMagicAction intake_stow_ ;
    private MCMotionMagicAction tilt_collect_ ;
    private MCMotionMagicAction tilt_stow_ ;
    private MotorEncoderPowerAction spinner_feeder_on_ ;
    private MotorEncoderPowerAction spinner_feeder_off_ ;

    public ButchStartCollectAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;


        feeder_power_ = sub.getSettingsValue("actions:butch-start-collect:feeder-power").getDouble() ;
        tilt_threshold_ = sub.getSettingsValue("actions:butch-start-collect:tilt-threshold").getDouble() ;

        intake_collect_ = new MCMotionMagicAction(sub_.getUpDown(), "pids:position", "targets:collect", 1, 1) ;
        intake_stow_ = new MCMotionMagicAction(sub_.getUpDown(), "pids:position", "targets:stow", 1, 1) ;
        spinner_feeder_on_ = new MotorEncoderPowerAction(sub_.getFeeder(), feeder_power_) ;
        spinner_feeder_off_ = new MotorEncoderPowerAction(sub_.getFeeder(), 0.0) ;
        tilt_collect_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position", "targets:collect", 1, 1) ;
        tilt_stow_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position", "targets:stow", 1, 1) ;

        double t = sub.getSettingsValue("actions:butch-start-collect:delay-time").getDouble() ;
        timer_ = new XeroTimer(sub.getRobot(), "collect-timer", t) ;
        state_ = CollectState.Idle ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.getFeeder().setAction(spinner_feeder_on_, true) ;
        sub_.getTilt().setAction(tilt_collect_, true) ;
        state_ = CollectState.TiltMoving ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        CollectState prev = state_ ;

        switch(state_) {
            case Idle:
                break ;

            case TiltMoving:
                if (sub_.getTilt().getPosition() > tilt_threshold_)  {
                    sub_.getUpDown().setAction(intake_collect_, true) ;
                    state_ = CollectState.BothMoving ;
                }
                break ;

            case BothMoving:
                if (sub_.isNoteCurrentlyDetected()) {
                    sub_.setHoldingNote(true) ;
                    state_ = CollectState.WaitingForCollect ;
                    timer_.start() ;
                }
                else if (tilt_collect_.isDone() && intake_collect_.isDone()) {
                    state_ = CollectState.WaitingForNote ;
                }
                break ;

            case WaitingForNote:
                if (sub_.isNoteCurrentlyDetected()) {
                    sub_.setHoldingNote(true) ;
                    state_ = CollectState.WaitingForCollect ;
                    timer_.start() ;
                }
                break ;

            case WaitingForCollect:
                if (timer_.isExpired()) {
                    sub_.getFeeder().setAction(spinner_feeder_off_, true) ;
                    sub_.getUpDown().setAction(intake_stow_, true) ;
                    sub_.getTilt().setAction(tilt_stow_, true) ;
                    state_ = CollectState.Stowing;
                }
                break;

            case Stowing:
                if (tilt_stow_.isDone() && intake_stow_.isDone()) {
                    setDone() ;
                    state_ = CollectState.Done ;
                }
                break ;

            case Done:
                break ;
        }

        if (state_ != prev) {
            sub_.getRobot().getMessageLogger().startMessage(MessageType.Debug, sub_.getLoggerID()) ;
            sub_.getRobot().getMessageLogger().add("state changed: ");
            sub_.getRobot().getMessageLogger().add(prev.toString()) ;
            sub_.getRobot().getMessageLogger().add(" ->");            
            sub_.getRobot().getMessageLogger().add(state_.toString()) ;            
            sub_.getRobot().getMessageLogger().endMessage();
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;
    }

    public String toString(int indent) {
        return spaces(indent) + "ButchStartCollectAction" ;
    }    
}
