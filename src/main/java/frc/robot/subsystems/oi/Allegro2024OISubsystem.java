package frc.robot.subsystems.oi;

import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.oi.Gamepad;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.base.subsystems.oi.SwerveDriveGamepad;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.intake_shooter.ButchStartCollectAction;
import frc.robot.subsystems.intake_shooter.ButchStopCollectionAction;
import frc.robot.subsystems.intake_shooter.IntakeShooterSubsystem;
import frc.robot.subsystems.intake_shooter.ManualShootAction;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Allegro2024OISubsystem extends OISubsystem {
    private final static String OIHIDIndexName = "panel:index" ;

    private final static Gamepad.Button[] resetButtons = { Gamepad.Button.Y, Gamepad.Button.B} ;

    private ButchStartCollectAction startCollectAction_ ;
    private ButchStopCollectionAction stopCollectAction_ ;
    private ButchOIPanel oipanel_ ;

    private ManualShootAction manual_ ;

    public Allegro2024OISubsystem(Subsystem parent, DriveBaseSubsystem db, IntakeShooterSubsystem intake) throws Exception {
        super (parent,"allegro2024oi",GamePadType.Swerve, db, true);

        startCollectAction_ = new ButchStartCollectAction(intake);
        stopCollectAction_ = new ButchStopCollectionAction(intake);

        int index ;
        MessageLogger logger = getRobot().getMessageLogger() ;

        try {
            index = getSettingsValue(OIHIDIndexName).getInteger() ;
        } catch (BadParameterTypeException e) {
            logger.startMessage(MessageType.Error) ;
            logger.add("parameter ").addQuoted(OIHIDIndexName) ;
            logger.add(" exists, but is not an integer").endMessage();
            index = -1 ;
        } catch (MissingParameterException e) {
            logger.startMessage(MessageType.Error) ;
            logger.add("parameter ").addQuoted(OIHIDIndexName) ;
            logger.add(" does not exist").endMessage();
            index = -1 ;            
        }

        if (index != -1) {
            try {
                oipanel_ = new ButchOIPanel(this, index) ;
                addHIDDevice(oipanel_) ;
            }
            catch(Exception ex) {
                logger.startMessage(MessageType.Error) ;
                logger.add("OI HID device was not created - ") ;
                logger.add(ex.getMessage()).endMessage(); ;
            }
        }
    }

    public ButchOIPanel getPanel() {
        return oipanel_ ;
    }

    @Override
    public boolean isCoastMode() {
        return oipanel_.getCoastValue() == 0 ;
    }

    public void startCollect() {
        if (oipanel_.isIdle()) {        
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem();
            IntakeShooterSubsystem intake = robot.getIntakeShooter();
            intake.setAction(startCollectAction_) ;
        }
        else {
            MessageLogger logger = getRobot().getMessageLogger() ;            
            logger.startMessage(MessageType.Debug, getLoggerID()) ;
            logger.add("collect requested while OI Panel was not in idle state") ;
            logger.endMessage() ;
        }
    }

    public void stopCollect() {
        //
        // If the panel is not idle, then it has detected a note and is processing
        // what comes next.
        //
        if (oipanel_.isIdle()) {
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem();
            IntakeShooterSubsystem intake = robot.getIntakeShooter();
            intake.setAction(stopCollectAction_) ;
        }
    }

    public void targetLockMode(boolean enable) {
        AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem();
        robot.getSwerve().setSWRotationControl(enable);
        robot.getTargetTracker().feedTargetToDB(enable);
    }

    public void manualShoot(String name) {
        if (!getPanel().isIdle() || manual_ != null)
            return ;

        try {
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem();
            IntakeShooterSubsystem intake = robot.getIntakeShooter();
            getPanel().setBusyExternal(true);
            manual_ = new ManualShootAction(intake, name) ;
            intake.setAction(manual_);
        }
        catch(Exception ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("exception thrown in manualShoot - " + ex.getMessage()).endMessage();
            logger.logStackTrace(ex.getStackTrace());
        }
    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();

        if(manual_ != null && manual_.isDone()) {
            getPanel().setBusyExternal(false);
            manual_ = null ;
        }
    }

    @Override
    protected void gamePadCreated(Gamepad gp) {
        SwerveDriveGamepad swgp = (SwerveDriveGamepad)gp ;
        if (swgp != null) {
            swgp.bindButtons(resetButtons, ()->swgp.resetSwerveDriveDirection(), null);            
            swgp.bindButton(Gamepad.Button.LBack, ()-> swgp.startDriveBaseX(), ()->swgp.stopDriveBaseX());   
            swgp.bindButton(Gamepad.Button.RBack, ()->startCollect(), ()->stopCollect());
            // swgp.bindButton(Gamepad.Button.LTrigger, ()->targetLockMode(true), null);
            // swgp.bindButton(Gamepad.Button.RTrigger, ()->targetLockMode(false), null);
            swgp.bindButton(Gamepad.Button.A, ()->manualShoot("subwoofer"), null);
            swgp.bindButton(Gamepad.Button.X, ()->manualShoot("podium"), null);
        }
    }    
}
