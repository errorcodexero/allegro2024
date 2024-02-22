package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public class ButchStartCollectAction extends CollectAction {
    private enum CollectState {
        Idle,
        TiltMoving,
        BothMoving,
        WaitingForNote,
        WaitingForCollect,
        Stowing
    } ;

    private CollectState state_ ;
    private double tilt_down_threshold_ ;
    private double feeder_power_ ;

    private boolean collecting_note_ ;

    private XeroTimer timer_ ;

    private MCMotionMagicAction intake_collect_ ;
    private MCMotionMagicAction intake_stow_ ;
    private MCMotionMagicAction tilt_collect_ ;
    private MotorEncoderPowerAction spinner_feeder_on_ ;
    private MotorEncoderPowerAction spinner_feeder_off_ ;

    public ButchStartCollectAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub) ;

        feeder_power_ = sub.getSettingsValue("actions:butch-start-collect:feeder-power").getDouble() ;
        tilt_down_threshold_ = sub.getSettingsValue("actions:butch-start-collect:tilt-down-threshold").getDouble() ;


        intake_collect_ = new MCMotionMagicAction(getSubsystem().getUpDown(), "pids:position", "targets:collect", 1, 1) ;
        intake_stow_ = new MCMotionMagicAction(getSubsystem().getUpDown(), "pids:position", "targets:stow", 1, 1) ;
        spinner_feeder_on_ = new MotorEncoderPowerAction(getSubsystem().getFeeder(), feeder_power_) ;
        spinner_feeder_off_ = new MotorEncoderPowerAction(getSubsystem().getFeeder(), 0.0) ;
        tilt_collect_ = new MCMotionMagicAction(getSubsystem().getTilt(), "pids:position", "targets:collect", 1, 1) ;


        double t = sub.getSettingsValue("actions:butch-start-collect:delay-time").getDouble() ;
        timer_ = new XeroTimer(sub.getRobot(), "collect-timer", t) ;
        state_ = CollectState.Idle ;
    }

    public boolean isCollectingNote() {
        return collecting_note_ ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        if (getSubsystem().isHoldingNote()) {
            MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Info);
            logger.add("collect aborted - note already present") ;
            logger.endMessage();
            setDone() ;
        }
        else {
            collecting_note_ = false ;
            getSubsystem().getFeeder().setAction(spinner_feeder_on_, true) ;
            getSubsystem().getTilt().setAction(tilt_collect_, true) ;
            state_ = CollectState.TiltMoving ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        CollectState prev = state_ ;

        switch(state_) {
            case Idle:
                break ;

            case TiltMoving:
                if (getSubsystem().getTilt().getPosition() > tilt_down_threshold_)  {
                    getSubsystem().getUpDown().setAction(intake_collect_, true) ;
                    state_ = CollectState.BothMoving ;
                }
                break ;

            case BothMoving:
                if (getSubsystem().isNoteCurrentlyDetected()) {
                    timer_.start();
                    state_ = CollectState.WaitingForCollect;
                }
                else if (tilt_collect_.isDone() && intake_collect_.isDone()) {
                    state_ = CollectState.WaitingForNote ;
                }
                break ;

            case WaitingForNote:
                if (getSubsystem().isNoteCurrentlyDetected()) {
                    collecting_note_ = true ;
                    state_ = CollectState.WaitingForCollect ;
                    timer_.start() ;
                }
                break ;

            case WaitingForCollect:
                if (timer_.isExpired()) {
                    getSubsystem().getFeeder().setAction(spinner_feeder_off_, true) ;
                    getSubsystem().getUpDown().setAction(intake_stow_, true) ;
                    getSubsystem().setHoldingNote(true);
                    startStow();
                    state_ = CollectState.Stowing ;
                }
                break;

            case Stowing:
                //
                // The base class will call setDone()
                //
                runStow() ;
                break ;
        }

        if (state_ != prev) {
            getSubsystem().getRobot().getMessageLogger().startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
            getSubsystem().getRobot().getMessageLogger().add("state changed: ");
            getSubsystem().getRobot().getMessageLogger().add(prev.toString()) ;
            getSubsystem().getRobot().getMessageLogger().add(" ->");            
            getSubsystem().getRobot().getMessageLogger().add(state_.toString()) ;            
            getSubsystem().getRobot().getMessageLogger().endMessage();
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
