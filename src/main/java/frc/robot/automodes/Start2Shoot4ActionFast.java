package frc.robot.automodes;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.swerve.SwerveHolonomicPathFollowerAction;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import frc.robot.subsystems.intake_shooter.IntakeGotoNamedPositionAction;
import frc.robot.subsystems.intake_shooter.IntakeManualShootAction;
import frc.robot.subsystems.intake_shooter.StartCollectAltAction;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Start2Shoot4ActionFast extends Action {

    private static final double kPath4Delay = 0.0; 
    private static final double kPath2Delay = 1.2 ;
    private static final double kPath6Delay = 1.2 ;

    private enum State {
        Idle,
        Delay,
        Shoot1,
        Shoot2,
        Shoot3,
        Shoot4,
        Path1,          // Subwoofer to second note
        Path2,          // second note to subwoofer
        Path3,          // Subwoofer to third note
        Path4,          // Third note to subwoofer
        Path5,          // Subwoofer to fourth note
        Path6,          // Fourth note to subwoofer
        Done,
        StowState,
    }

    private AllegroRobot2024 robot_ ;
    private State state_ ;
    private XeroTimer delay_timer_ ;
    private double state_start_time_ ;

    private IntakeGotoNamedPositionAction stow_ ;
    private IntakeManualShootAction manual_shoot_ ;
    private StartCollectAltAction start_collect_ ;

    private SwerveHolonomicPathFollowerAction p1_ ;
    private SwerveHolonomicPathFollowerAction p2_ ;
    private SwerveHolonomicPathFollowerAction p3_ ;
    private SwerveHolonomicPathFollowerAction p4_ ;
    private SwerveHolonomicPathFollowerAction p5_ ;
    private SwerveHolonomicPathFollowerAction p6_ ;    
    private SwerveHolonomicPathFollowerAction p7_ ;                 
    
    public Start2Shoot4ActionFast(AllegroRobot2024 robot, boolean mirror, double mvalue) throws Exception {
        super(robot.getRobot().getMessageLogger()) ;

        robot_ = robot ;

        double v1 = robot_.getIntakeShooter().getUpDown().getSettingsValue("targets:stow").getDouble() ;
        double v2 = robot_.getIntakeShooter().getTilt().getSettingsValue("targets:stow").getDouble() ;         
        stow_ = new IntakeGotoNamedPositionAction(robot_.getIntakeShooter(), v1, v2) ;

        double updown = robot_.getIntakeShooter().getSettingsValue("actions:manual-shoot:subwoofer-center-low:updown").getDouble() ;
        double tilt = robot_.getIntakeShooter().getSettingsValue("actions:manual-shoot:subwoofer-center-low:tilt").getDouble() ;

        start_collect_ = new StartCollectAltAction(robot_.getIntakeShooter(), updown, tilt) ;
        manual_shoot_ = new IntakeManualShootAction(robot_.getIntakeShooter(), "subwoofer-center-low") ;

        p1_ = new SwerveHolonomicPathFollowerAction(robot.getSwerve(), "S2S4-P1", false, 0.2, mirror, mvalue);
        p2_ = new SwerveHolonomicPathFollowerAction(robot.getSwerve(), "S2S4-P2", false, 0.2, mirror, mvalue);
        p3_ = new SwerveHolonomicPathFollowerAction(robot.getSwerve(), "S2S4-P3", true, 0.2, mirror, mvalue);
        p4_ = new SwerveHolonomicPathFollowerAction(robot.getSwerve(), "S2S4-P4", false, 0.2, mirror, mvalue);
        p5_ = new SwerveHolonomicPathFollowerAction(robot.getSwerve(), "S2S4-P5", false, 0.2, mirror, mvalue);
        p6_ = new SwerveHolonomicPathFollowerAction(robot.getSwerve(), "S2S4-P6", false, 0.2, mirror, mvalue);                
        p7_ = new SwerveHolonomicPathFollowerAction(robot.getSwerve(), "S2S4-P7", false, 0.2, mirror, mvalue);

        delay_timer_ = new XeroTimer(robot.getRobot(), "auto-delay", 0.6) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        state_start_time_ = robot_.getRobot().getTime() ;

        state_ = State.Shoot1 ;
        robot_.getIntakeShooter().setHoldingNote(true);        
        robot_.getIntakeShooter().setAction(manual_shoot_, true) ;
    }

    //
    // Shooting the preloaded note
    //
    private void shoot1State() {
        if (manual_shoot_.isDone()) {
            robot_.getIntakeShooter().setAction(start_collect_, true) ;              
            delay_timer_.start() ;
            state_ = State.Delay ;
        }
    }

    private void delayState() {
        if (delay_timer_.isExpired()) {
            robot_.getSwerve().setAction(p3_, true) ;
            state_ = State.Path1 ;            
        }
    }

    //
    // Driving from the subwoofer to collect a second note (the first collected note)
    //
    private void path1State() {
        if (p3_.isDone()) {
            robot_.getSwerve().setAction(p4_, true) ;
            state_ = State.Path2 ;
        }
    }

    //
    // Driving back to the subwoofer after collecting the second note (first collected note)
    //
    private void path2State() {
        if (robot_.getRobot().getTime() - state_start_time_ > kPath4Delay && robot_.getIntakeShooter().getAction() != manual_shoot_ && start_collect_.isDone())  {
            robot_.getIntakeShooter().setAction(manual_shoot_, true) ;
            state_ = State.Shoot2 ;            
        }
    }

    //
    // Shooting the second note (the first collected note)
    //
    private void shoot2State() {
        if (manual_shoot_.isDone()) {
            robot_.getIntakeShooter().setAction(start_collect_, true) ;            
            robot_.getSwerve().setAction(p1_, true) ;
            state_ = State.Path3 ;            
        }
    }

    //
    // Driving to collect a third note (the second collected note)
    //
    private void path3State() {
        if (p1_.isDone()) {
            robot_.getSwerve().setAction(p2_, true) ;
            state_ = State.Path4 ;
        }
    }

    //
    // Driving back to the subwoofer after collecting the third note (the second collected note)
    //
    private void path4State() {
        if (robot_.getRobot().getTime() - state_start_time_ > kPath2Delay && robot_.getIntakeShooter().getAction() != manual_shoot_ && start_collect_.isDone())  {
            robot_.getIntakeShooter().setAction(manual_shoot_, true) ;
            state_ = State.Shoot3 ;            
        }
    }

    //
    // Shooting the third note (the second collected note)
    //
    private void shoot3State() {
        if (manual_shoot_.isDone()) {
            robot_.getIntakeShooter().setAction(start_collect_, true) ;            
            robot_.getSwerve().setAction(p5_, true) ;
            state_ = State.Path5 ;            
        }
    }

    //
    // Driving to collect a fourth note (the third collected note)
    //
    private void path5State() {
        if (p5_.isDone()) {
            robot_.getSwerve().setAction(p6_, true) ;
            state_ = State.Path6 ;
        }
    }

    //
    // Driving back to the subwoofer after collecting the fourth note (the third collected note)
    //
    private void path6State() {
        if (robot_.getRobot().getTime() - state_start_time_ > kPath6Delay && robot_.getIntakeShooter().getAction() != manual_shoot_ && start_collect_.isDone())  {
            robot_.getIntakeShooter().setAction(manual_shoot_, true) ;
            state_ = State.Shoot4 ;
        }        
    }

    //
    // Shooting the fourth note (the third collected note)
    //
    private void shoot4State() {
        if (manual_shoot_.isDone()) {
            robot_.getSwerve().setAction(p7_, true) ;
            robot_.getIntakeShooter().setAction(stow_, true) ;
            state_ = State.StowState ;
        }
    }

    private void stowState() {
        if (stow_.isDone()) {
            state_ = State.Done ;
        }
    }        

    public void run() throws Exception {
        super.run() ;

        State prev = state_ ;

        switch(state_) {
        case Idle:
            break ;
        
        case Delay:
            delayState() ;
            break ;

        case Shoot1:
            shoot1State() ;
            break ;

        case Shoot2:
            shoot2State() ;
            break ;

        case Shoot3:
            shoot3State() ;
            break ;

        case Shoot4:
            shoot4State() ;
            break ;

        case Path1:
            path1State() ;
            break ;

        case Path2:
            path2State() ;
            break ;
        
        case Path3:
            path3State() ;
            break ;

        case Path4:
            path4State() ;
            break ;

        case Path5:
            path5State() ;
            break ;

        case Path6:
            path6State() ;
            break ;

        case StowState:
            stowState() ;
            break ;

        case Done:
            break ;
        }

        if (state_ != prev) {
            state_start_time_ = robot_.getRobot().getTime() ;

            MessageLogger logger = robot_.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Info) ;
            logger.add("automode: " + prev.toString() + " -> " + state_.toString()) ;
            logger.endMessage();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "Start2Shoot4ActionFast" ;
    }
}
