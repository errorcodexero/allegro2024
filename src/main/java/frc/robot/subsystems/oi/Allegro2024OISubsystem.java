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
        AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem();
        IntakeShooterSubsystem intake = robot.getIntakeShooter();

        intake.setAction(startCollectAction_) ;
    }

    public void stopCollect() {
        AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem();
        IntakeShooterSubsystem intake = robot.getIntakeShooter();

        if (!intake.isHoldingNote()) {
            intake.setAction(stopCollectAction_) ;
        }
    }

    public void targetLockMode(boolean enable) {
        AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem();
        robot.getSwerve().setSWRotationControl(enable);
        robot.getTargetTracker().sendTargetInfoToDB(enable);
    }

    public void manualShootSubwoofer() {
        try {
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem();
            IntakeShooterSubsystem intake = robot.getIntakeShooter();
            intake.setAction(new ManualShootAction(intake, "subwoofer"));    
        }
        catch(Exception ex) {
        }
    }

    public void manualShootPodium() {
        try {
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem();
            IntakeShooterSubsystem intake = robot.getIntakeShooter();
            intake.setAction(new ManualShootAction(intake, "podium"));    
        }
        catch(Exception ex) {
        }
    }    

    @Override
    protected void gamePadCreated(Gamepad gp) {
        SwerveDriveGamepad swgp = (SwerveDriveGamepad)gp ;
        if (swgp != null) {
            swgp.bindButtons(resetButtons, ()->swgp.resetSwerveDriveDirection(), null);            
            swgp.bindButton(Gamepad.Button.LBack, ()-> swgp.startDriveBaseX(), ()->swgp.stopDriveBaseX());   
            swgp.bindButton(Gamepad.Button.RBack, ()->startCollect(), ()->stopCollect());
            swgp.bindButton(Gamepad.Button.LTrigger, ()->targetLockMode(false), null);
            swgp.bindButton(Gamepad.Button.RTrigger, ()->targetLockMode(true), null);
            swgp.bindButton(Gamepad.Button.A, ()->manualShootSubwoofer(), null);
            swgp.bindButton(Gamepad.Button.A, ()->manualShootPodium(), null);
        }
    }    
}
