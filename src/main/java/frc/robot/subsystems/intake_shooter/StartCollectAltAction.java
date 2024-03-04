package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public class StartCollectAltAction extends CollectBaseAltAction {
    private enum CollectState {
        Idle,
        Moving,
        WaitingForNote,
        WaitingForCollect,
        Stowing
    } ;

    private CollectState state_ ;
    private double feeder_power_ ;

    private boolean collecting_note_ ;

    private XeroTimer timer_ ;

    private IntakeGotoNamedPositionAction collect_ ;
    private MotorEncoderPowerAction spinner_feeder_on_ ;
    private MotorEncoderPowerAction spinner_feeder_off_ ;

    public StartCollectAltAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub) ;

        feeder_power_ = sub.getSettingsValue("actions:butch-start-collect:feeder-power").getDouble() ;

        spinner_feeder_on_ = new MotorEncoderPowerAction(getSubsystem().getFeeder(), feeder_power_) ;
        spinner_feeder_off_ = new MotorEncoderPowerAction(getSubsystem().getFeeder(), 0.0) ;

        double v1 = sub.getUpDown().getSettingsValue("targets:collect").getDouble() ;
        double v2 = sub.getTilt().getSettingsValue("targets:collect").getDouble() ;
        collect_ = new IntakeGotoNamedPositionAction(sub, v1, v2) ;

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
            getSubsystem().setAction(collect_, true) ;
            state_ = CollectState.Moving ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        CollectState prev = state_ ;

        switch(state_) {
            case Idle:
                break ;

            case Moving:
                if (getSubsystem().isNoteCurrentlyDetected()) {
                    getSubsystem().setHoldingNote(true);                    
                    timer_.start();
                    state_ = CollectState.WaitingForCollect;
                }
                else if (collect_.isDone()) {
                    state_ = CollectState.WaitingForNote ;
                }
                break ;

            case WaitingForNote:
                if (getSubsystem().isNoteCurrentlyDetected()) {
                    getSubsystem().setHoldingNote(true);                    
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
