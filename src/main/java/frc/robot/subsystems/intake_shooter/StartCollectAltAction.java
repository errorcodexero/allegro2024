package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.oi.Gamepad;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public class StartCollectAltAction extends CollectBaseAltAction {
    private enum CollectState {
        Idle,
        MovingTilt,
        Moving,
        WaitingForNote,
        WaitingForCollect,
        Stowing
    } ;

    private CollectState state_ ;
    private double feeder_power_ ;
    private double updown_target_ ;
    private double tilt_target_ ;

    private boolean collecting_note_ ;

    private XeroTimer timer_ ;

    private MCMotionMagicAction updown_action_ ;
    private MCMotionMagicAction tilt_only_action_ ;
    private MCMotionMagicAction tilt_action_ ;
    private MotorEncoderPowerAction spinner_feeder_on_ ;
    private MotorEncoderPowerAction spinner_feeder_off_ ;

    public StartCollectAltAction(IntakeShooterSubsystem sub, double stowupdown, double stowtilt) throws Exception {
        super(sub, stowupdown, stowtilt) ;

        feeder_power_ = sub.getSettingsValue("actions:butch-start-collect:feeder-power").getDouble() ;

        spinner_feeder_on_ = new MotorEncoderPowerAction(getSubsystem().getFeeder(), feeder_power_) ;
        spinner_feeder_off_ = new MotorEncoderPowerAction(getSubsystem().getFeeder(), 0.0) ;

        updown_target_ = sub.getUpDown().getSettingsValue("targets:collect").getDouble() ;
        tilt_target_ = sub.getTilt().getSettingsValue("targets:collect").getDouble() ;

        updown_action_ = new MCMotionMagicAction(sub.getUpDown(), "pids:position" , updown_target_, 3.0 , 1);
        tilt_action_ = new MCMotionMagicAction(sub.getTilt(), "pids:position" , tilt_target_, 3.0 , 1);

        double t = sub.getSettingsValue("actions:butch-start-collect:delay-time").getDouble() ;
        timer_ = new XeroTimer(sub.getRobot(), "collect-timer", t) ;
        state_ = CollectState.Idle ;
    }

    public StartCollectAltAction(IntakeShooterSubsystem sub) throws Exception {
        this(sub, Double.NaN, Double.NaN) ;
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

            double updownpos = getSubsystem().getUpDown().getPosition() ;
            double updownstow = getSubsystem().getUpDown().getSettingsValue("targets:stow").getDouble() ;
            double tiltstow = getSubsystem().getTilt().getSettingsValue("targets:stow").getDouble() ;

            if (updownpos < updownstow && updown_target_ < updownpos) {
                MessageLogger logger = getSubsystem().getRobot().getMessageLogger();
                logger.startMessage(MessageType.Info).add("pos < stow").add("pos", updownpos).add("stow", updownstow).endMessage() ;

                //
                // The updown is not in the stowed position, we must move the tilt to a compatible
                // position, before we start the synchronous movement.
                //
                double ttarget = tiltstow + updownstow - updownpos ;
                tilt_only_action_ = new MCMotionMagicAction(getSubsystem().getTilt(), "pids:position", ttarget, 3.0, 1.0) ;
                getSubsystem().getTilt().setAction(tilt_only_action_, true) ;
                state_ = CollectState.MovingTilt ;
            }
            else 
            {
                getSubsystem().getUpDown().setAction(updown_action_, true);
                getSubsystem().getTilt().setAction(tilt_action_, true);
                state_ = CollectState.Moving ;
            }
        }
    }

    private void rumbleGamepad() {
        Gamepad gp = getSubsystem().getRobot().getRobotSubsystem().getOI().getGamePad();
        if (gp != null) {
            gp.rumble(1.0, 2.0);
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        CollectState prev = state_ ;

        switch(state_) {
            case Idle:
                break ;

            case MovingTilt:
                if (tilt_only_action_.isDone()) {
                    getSubsystem().getUpDown().setAction(updown_action_, true);
                    getSubsystem().getTilt().setAction(tilt_action_, true);
                    state_ = CollectState.Moving ;
                }
                break ;

            case Moving:
                if (getSubsystem().isNoteCurrentlyDetected()) {
                    getSubsystem().setHoldingNote(true);                    
                    rumbleGamepad() ;
                    collecting_note_ = true ;                    
                    timer_.start();
                    state_ = CollectState.WaitingForCollect;
                }
                else if (updown_action_.isDone() && tilt_action_.isDone()) {
                    state_ = CollectState.WaitingForNote ;
                }
                break ;

            case WaitingForNote:
                if (getSubsystem().isNoteCurrentlyDetected()) {
                    getSubsystem().setHoldingNote(true);
                    rumbleGamepad() ;
                    collecting_note_ = true ;
                    state_ = CollectState.WaitingForCollect ;
                    timer_.start() ;
                }
                break ;

            case WaitingForCollect:
                if (timer_.isExpired()) {
                    getSubsystem().getFeeder().setAction(spinner_feeder_off_, true) ;
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
        return spaces(indent) + "StartCollectAction" ;
    }    
}
