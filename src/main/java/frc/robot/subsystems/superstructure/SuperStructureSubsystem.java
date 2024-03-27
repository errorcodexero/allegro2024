package frc.robot.subsystems.superstructure;

import org.xero1425.base.LoopType;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import frc.robot.subsystems.amp_trap.AmpTrapSubsystem;
import frc.robot.subsystems.intake_shooter.IntakeShooterSubsystem;

public class SuperStructureSubsystem extends Subsystem {
    private IntakeShooterSubsystem is_ ;
    private AmpTrapSubsystem at_ ;
    private MotorEncoderSubsystem climber_ ;

    public SuperStructureSubsystem(Subsystem parent) throws Exception {
        super(parent, "superstructure");

        try {
            is_ = new IntakeShooterSubsystem(this);
            addChild(is_);
        }
        catch(Exception ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("exception creating intake-shooter subsystem -  ").add(ex.getMessage()).endMessage();
            logger.logStackTrace(ex.getStackTrace());
            is_ = null ;
        }

        try {
            at_ = new AmpTrapSubsystem(this);
            addChild(at_);
        }
        catch(Exception ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("exception creating amp-trap subsystem -  ").add(ex.getMessage()).endMessage();
            logger.logStackTrace(ex.getStackTrace());
            at_ = null ;
        }        

        try {
            climber_ = new MotorEncoderSubsystem(this, "climber", false);
            addChild(climber_) ;
        }
        catch(Exception ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("exception creating climber subsystem -  ").add(ex.getMessage()).endMessage();
            logger.logStackTrace(ex.getStackTrace());
            climber_ = null ;
        }
    }
    
    @Override
    public void init(LoopType prev, LoopType next) {
        super.init(prev, next);

        if (prev == LoopType.Disabled && next == LoopType.Autonomous && !getRobot().isTestMode()) {
            is_.setHoldingNote(true);
        }
    }    

    public IntakeShooterSubsystem getIntakeShooter() {
        return is_;
    }

    public AmpTrapSubsystem getAmpTrap() {
        return at_;
    }

    public MotorEncoderSubsystem getClimber() {
        return climber_;
    }
}
