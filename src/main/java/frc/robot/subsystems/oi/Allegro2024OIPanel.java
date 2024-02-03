package frc.robot.subsystems.oi;

import org.xero1425.base.subsystems.oi.OIPanel;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.base.subsystems.oi.OIPanelButton.ButtonType;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Allegro2024OIPanel extends OIPanel {

    private enum State {
        Empty,
        NotInShooter,
        Shooting,
        Ejecting,
        Transfering
    }

    private int climb_up_prep_gadget_ ;
    private int climb_up_exec_gadget_ ;
    private int shoot_gadget_ ;
    private int abort_gadget_ ;
    private int climb_down_prep_gadget_ ;
    private int climb_down_exec_gadget_ ;
    private int xfer_gadget_ ;
    private int speaker_gadget_ ;
    private int amp_gadget_ ;
    private int trap_gadget_ ;

    private int climb_up_prep_led_ ;
    private int climb_up_exec_led_ ;
    private int shoot_led_ ;
    private int climb_down_prep_led_ ;
    private int climb_down_exec_led_ ;
    private int xfer_led_ ;

    public Allegro2024OIPanel(OISubsystem sub, String name, int index) throws BadParameterTypeException, MissingParameterException {
        super(sub, name, index) ;
    }   

    @Override
    public void computeState() throws Exception {
        super.computeState();
    }

    @Override
    public void generateActions() {
        super.generateActions();

        AllegroRobot2024 robot = (AllegroRobot2024)getSubsystem().getRobot().getRobotSubsystem() ;
        if (robot.getIntakeShooter().isNotePresent()) {
            //
            // The intake has a note.  Valid Actions ...
            // - shoot
            // - transfer
            // - eject
            //
        }
        else if (robot.getAmpTrap().hasNote()) {
            //
            // The amptrap has a note, the only valid actions are prepareAmp, prepareTrap, or transfer
            //
        }
        else {
            //
            // There are no notes in the robot, the only valid actions is collect
            //
        }
    }
       
    @Override
    public void createStaticActions() throws Exception {
    }

    @Override
    protected void initializeLEDs() throws BadParameterTypeException, MissingParameterException {
        super.initializeLEDs();

        climb_up_prep_led_ = getSubsystem().getSettingsValue("panel:leds:climb-up-prep").getInteger();
        climb_up_exec_led_ = getSubsystem().getSettingsValue("panel:leds:climb-up-exec").getInteger();
        shoot_led_ = getSubsystem().getSettingsValue("panel:leds:shoot").getInteger();
        climb_down_prep_led_ = getSubsystem().getSettingsValue("panel:leds:climb-down-prep").getInteger();
        climb_down_exec_led_ = getSubsystem().getSettingsValue("panel:leds:climb-down-exec").getInteger();
        xfer_led_ = getSubsystem().getSettingsValue("panel:leds:xfer").getInteger();
    }
    
    @Override
    protected void initializeGadgets() throws BadParameterTypeException, MissingParameterException {
        super.initializeGadgets();
        int num;    

        num = getSubsystem().getSettingsValue("panel:gadgets:climb-up-prep").getInteger();
        climb_up_prep_gadget_ = mapButton(num, ButtonType.LowToHigh);

        num = getSubsystem().getSettingsValue("panel:gadgets:climb-up-exec").getInteger();
        climb_up_exec_gadget_ = mapButton(num, ButtonType.LowToHigh);

        num = getSubsystem().getSettingsValue("panel:gadgets:shoot").getInteger();
        shoot_gadget_ = mapButton(num, ButtonType.LowToHigh);

        num = getSubsystem().getSettingsValue("panel:gadgets:abort").getInteger();
        abort_gadget_ = mapButton(num, ButtonType.LowToHigh);

        num = getSubsystem().getSettingsValue("panel:gadgets:climb-down-prep").getInteger();
        climb_down_prep_gadget_ = mapButton(num, ButtonType.LowToHigh);

        num = getSubsystem().getSettingsValue("panel:gadgets:climb-down-exec").getInteger();
        climb_down_exec_gadget_ = mapButton(num, ButtonType.LowToHigh);

        num = getSubsystem().getSettingsValue("panel:gadgets:xfer").getInteger();
        xfer_gadget_ = mapButton(num, ButtonType.LowToHigh);

        num = getSubsystem().getSettingsValue("panel:gadgets:speaker").getInteger();
        speaker_gadget_ = mapButton(num, ButtonType.Level);
        
        num = getSubsystem().getSettingsValue("panel:gadgets:amp").getInteger();
        amp_gadget_ = mapButton(num, ButtonType.Level);

        num = getSubsystem().getSettingsValue("panel:gadgets:trap").getInteger();
        trap_gadget_ = mapButton(num, ButtonType.Level);
    }        
}
