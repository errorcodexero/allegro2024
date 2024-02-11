package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public class ManualShootAction extends Action {
    private double kTiltPosition ;
    private double kTiltPositionThreshold ;
    private double kTiltVelocityThreshold ;
    private double kUpDownPosition ;
    private double kUpDownPositionThreshold ;
    private double kUpDownVelocityThreshold ;

    private double kShooterVelocity ;
    private double kShooterThreshold ;

    private IntakeShooterSubsystem intake_;
    private MCMotionMagicAction tilt_action_ ;
    private MCMotionMagicAction up_down_action_ ;
    private MCVelocityAction shooter1_action_ ;
    private MCVelocityAction shooter2_action_ ;
    private MotorEncoderPowerAction feeder_action_ ;

    private boolean shooting_ ;
    private boolean tilt_ready_ ;
    private boolean updown_ready_ ;

    public ManualShootAction(IntakeShooterSubsystem intake, String location) throws Exception {
        super(intake.getRobot().getMessageLogger());
        intake_ = intake;

        kTiltPosition = intake.getSettingsValue("actions:manual-shoot:" + location + ":tilt").getDouble();
        kTiltPositionThreshold = intake.getSettingsValue("actions:manual-shoot:" + location + ":tilt-pos-threshold").getDouble();
        kTiltVelocityThreshold  = intake.getSettingsValue("actions:manual-shoot:" + location + ":tilt-velocity-threshold").getDouble();
        tilt_action_ = new MCMotionMagicAction(intake.getTilt(), "pids:position", kTiltPosition, kTiltPositionThreshold, kTiltVelocityThreshold);

        kUpDownPosition = intake.getSettingsValue("actions:manual-shoot:" + location + ":up-down-pos").getDouble();
        kUpDownPositionThreshold = intake.getSettingsValue("actions:manual-shoot:" + location + ":up-down-pos-threshold").getDouble();
        kUpDownVelocityThreshold = intake.getSettingsValue("actions:manual-shoot:" + location + ":up-down-velocity-threshold").getDouble();
        up_down_action_ = new MCMotionMagicAction(intake_.getUpDown(), "pids:position", kUpDownPosition, kUpDownPositionThreshold, kUpDownVelocityThreshold) ;

        kShooterVelocity = intake.getSettingsValue("actions:manual-shoot:" + location + ":shooter-velocity").getDouble();
        kShooterThreshold = intake.getSettingsValue("actions:manual-shoot:" + location + ":shooter-velocity-threshold").getDouble();
        shooter1_action_ = new MCVelocityAction(intake_.getShooter1(), "pids:velocity", kShooterVelocity, kShooterThreshold, false) ;
        shooter2_action_ = new MCVelocityAction(intake_.getShooter2(), "pids:velocity", kShooterVelocity, kShooterThreshold, false) ;

        feeder_action_ = new MotorEncoderPowerAction(intake_.getFeeder(), 0.5, 2.0) ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        if (!intake_.isHoldingNote()) {
            MessageLogger logger = intake_.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("ManualShootAction started with no note in intake").endMessage();
            setDone() ;
        }
        else {
            intake_.getTilt().setAction(tilt_action_, true) ;
            intake_.getUpDown().setAction(up_down_action_, true) ;
            intake_.getShooter1().setAction(shooter1_action_, true) ;
            intake_.getShooter2().setAction(shooter2_action_, true) ;

            tilt_ready_ = false ;
            updown_ready_ = false ;
            shooting_ = false ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run();

        if (!shooting_) {
            if (tilt_action_.isDone()) {
                tilt_ready_ = true ;
            }

            if (up_down_action_.isDone()) {
                updown_ready_ = true ;
            }

            if (shooter1_action_.isAtVelocity() && shooter2_action_.isAtVelocity() && tilt_ready_ && updown_ready_) {
                shooting_ = true ;
                intake_.getFeeder().setAction(feeder_action_, true) ;
            }
        }
        else {
            if (feeder_action_.isDone()) {
                intake_.setHoldingNote(false) ;
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
