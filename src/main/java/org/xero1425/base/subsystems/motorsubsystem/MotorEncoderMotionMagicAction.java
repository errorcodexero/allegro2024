package org.xero1425.base.subsystems.motorsubsystem;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motors.IMotorController.XeroPidType;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public class MotorEncoderMotionMagicAction extends MotorAction {

    private double SetDoneDelay = 0.0 ;

    private enum State {
        Waiting,
        Running,
        Delaying,
        Complete
    }

    private static final double NearEndpoint = 3000 ;
    private static final double EndVelocity = 1500 ;

    public enum HoldType
    {
        None,
        AtCurrentPosition,
        AtTargetPosition
    } ;

    private double start_ ;
    private double target_ ;
    private HoldType hold_ ;
    private State state_ ;
    private double start_pos_ ;
    private XeroTimer delay_timer_ ;

    private double kp_ ;
    private double ki_ ;
    private double kd_ ;
    private double kv_ ;
    private double ka_ ;
    private double ks_ ;
    private double kg_ ;
    private double maxv_ ;
    private double maxa_ ;
    private double jerk_ ;

    public MotorEncoderMotionMagicAction(MotorEncoderSubsystem sub, double target, double maxa, double maxv, HoldType holdtype) throws Exception {
        super(sub);

        target_ = target ;
        hold_ = holdtype ;
        maxa_ = maxa ;
        maxv_ = maxv ;

        kp_ = sub.getSettingsValue("magic:kp").getDouble() ;
        ki_ = sub.getSettingsValue("magic:ki").getDouble() ;
        kd_ = sub.getSettingsValue("magic:kd").getDouble() ;
        kv_ = sub.getSettingsValue("magic:kv").getDouble() ;
        ka_ = sub.getSettingsValue("magic:ka").getDouble() ;
        ks_ = sub.getSettingsValue("magic:ks").getDouble() ;
        kg_ = sub.getSettingsValue("magic:kg").getDouble() ;

        maxv_ = sub.getSettingsValue("magic:maxv").getDouble() ;
        maxa_ = sub.getSettingsValue("magic:maxa").getDouble() ;
        jerk_ = sub.getSettingsValue("magic:jerk").getDouble() ;                
    }

    public double getDistance() {
        MotorEncoderSubsystem me = (MotorEncoderSubsystem)getSubsystem();
        return Math.abs(me.getPosition() - start_pos_) ;
    }

    public boolean isWaiting() {
        return state_ == State.Waiting ;
    }

    public boolean isRunning() {
        return state_ == State.Running ;
    }

    public boolean isComplete() {
        return state_ == State.Complete ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        getSubsystem().getMotorController().setPID(XeroPidType.MotionMagic, kp_, ki_, kd_, kv_, ka_, kg_, ks_, 1.0);
        getSubsystem().getMotorController().setMotionMagicParams(maxv_, maxa_, jerk_) ;

        start_ = getSubsystem().getRobot().getTime() ;
        state_ = State.Waiting ;
        tryStart() ;
    }

    @Override
    public void run() throws Exception  {
        super.run() ;

        MessageLogger logger = getSubsystem().getRobot().getMessageLogger();
        State old = state_ ;

        tryStart();

        MotorEncoderSubsystem me = (MotorEncoderSubsystem)getSubsystem();
        double mcvel = me.getMotorController().getVelocity() ;
        double delta = Math.abs(target_ - me.getPosition()) ;

        if (state_ == State.Running) {
            logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
            logger.add("MotionMagic Pos") ;
            logger.add("time", getSubsystem().getRobot().getTime() - start_);
            logger.add("target", target_, "%.0f");
            logger.add("actual", me.getPosition(), "%.0f") ;
            logger.add("velocity", mcvel, "%.0f") ;
            logger.add("delta", delta, "%.0f") ;
            logger.add("state", state_.toString()) ;
            logger.endMessage();
        }

        if (state_ == State.Delaying && delay_timer_.isExpired())
        {
            state_ = State.Complete ;
            setDone() ;
        }
        else if (state_ == State.Running && delta < NearEndpoint && Math.abs(mcvel) < EndVelocity) {
            state_ = State.Complete ;
            // logger.startMessage(MessageType.Info) ;
            // logger.add("Motion magic duration ") ;
            // logger.add(getSubsystem().getRobot().getTime() - start_) ;
            // logger.add("delta", delta) ;
            // logger.endMessage();

            if (SetDoneDelay > 0.0) 
            {
                state_ = State.Delaying ;
                delay_timer_ = new XeroTimer(getSubsystem().getRobot(), "delaytimer", SetDoneDelay);
                delay_timer_.start() ;
            }
            else 
            {
                setDone() ;
            }
        }

        if (state_ != old) {
            logger.startMessage(MessageType.Info) ;
            logger.add("Motion Magic: ").add(old.toString()).add(" -> ").add(state_.toString()).endMessage();
        }
    }

    @Override
    public String toString(int indent) {
        String ret ;

        ret = spaces(indent) + "MotorEncoderMotionMagicAction (" + getSubsystem().getName() + ")";
        ret += " target=" + target_ + ", hold=" + hold_.toString();

        return ret ;
    }

    private void tryStart() throws BadMotorRequestException, MotorRequestFailedException {
        if (state_ == State.Waiting) {
            MotorEncoderSubsystem me = (MotorEncoderSubsystem)getSubsystem() ;
            me.getMotorController().set(XeroPidType.MotionMagic, target_);
            state_ = State.Running ;

            MessageLogger logger = me.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Info) ;
            logger.add("MotionMagic: ").add("accel", maxa_, "%.0f").add("velocity", maxv_, "%.0f").add("jerk", jerk_, "%0.f")
                    .add("target", target_, "%.0f").endMessage() ;
        }
    }
}
