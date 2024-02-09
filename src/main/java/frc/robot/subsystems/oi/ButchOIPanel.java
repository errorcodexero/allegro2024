package frc.robot.subsystems.oi;

import org.xero1425.base.subsystems.oi.OILed;
import org.xero1425.base.subsystems.oi.OIPanel;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.base.subsystems.oi.OIPanelButton.ButtonType;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class ButchOIPanel extends OIPanel {
    //
    // Panel gadgets
    //
    private int target_toggle1_gadget_ ;                    // Speaker, Amp, or Trap
    private int target_toggle2_gadget_ ;                    // Speaker, Amp, or Trap    
    private int amp_purpose_target1_gadget_ ;               // Amplification, Cooperition, None
    private int amp_purpose_target2_gadget_ ;               // Amplification, Cooperition, None    
    private int climb_up_prep_gadget_ ;                     // Prepare to climb up
    private int climb_up_exec_gadget_ ;                     // Perform the climb up
    private int climb_down_prep_gadget_ ;                   // Prepare to climb down
    private int climb_down_exec_gadget_ ;                   // Perform the climb down
    private int shoot_gadget_ ;                             // Shoot the note
    private int handoff_gadget_ ;                           // Handoff the note

    //
    // LEDs
    //
    private OILed db_ready_led_ ;
    private OILed shooter_velocity_ready_led_ ;
    private OILed shooter_tilt_ready_led_ ;
    private OILed shooter_april_tag_led_ ;
    private OILed note_in_manipulator_led_ ;
    private OILed climb_up_prep_enabled_led_ ;
    private OILed climb_up_exec_enabled_led_ ;
    private OILed climb_down_prep_enabled_led_ ;
    private OILed climb_down_exec_enabled_led_ ;

    public ButchOIPanel(OISubsystem oi, int index) throws BadParameterTypeException, MissingParameterException {
        super(oi, "oipanel", index);
    }

    @Override
    public void computeState() throws Exception {
        super.computeState();
    }

    @Override
    public void generateActions() {
        super.generateActions();
    }

    @Override
    public void createStaticActions() throws Exception {
    }    

    @Override
    protected void initializeLEDs() throws BadParameterTypeException, MissingParameterException {
        super.initializeLEDs();
        int num ;

        num = getSubsystem().getSettingsValue("panel:leds:db-ready").getInteger() ;
        db_ready_led_ = createLED(num) ;
        
        num = getSubsystem().getSettingsValue("panel:leds:shooter-velocity-ready").getInteger() ;
        shooter_velocity_ready_led_ = createLED(num) ;

        num = getSubsystem().getSettingsValue("panel:leds:shooter-tilt-ready").getInteger() ;
        shooter_tilt_ready_led_ = createLED(num) ;

        num = getSubsystem().getSettingsValue("panel:leds:shooter-april-tag").getInteger() ;
        shooter_april_tag_led_ = createLED(num) ;

        num = getSubsystem().getSettingsValue("panel:leds:note-in-manipulator").getInteger() ;
        note_in_manipulator_led_ = createLED(num) ;

        num = getSubsystem().getSettingsValue("panel:leds:climb-up-prep-enabled").getInteger() ;
        climb_up_prep_enabled_led_ = createLED(num) ;

        num = getSubsystem().getSettingsValue("panel:leds:climb-up-exec-enabled").getInteger() ;
        climb_up_exec_enabled_led_ = createLED(num) ;

        num = getSubsystem().getSettingsValue("panel:leds:climb-down-prep-enabled").getInteger() ;
        climb_down_prep_enabled_led_ = createLED(num) ;

        num = getSubsystem().getSettingsValue("panel:leds:climb-down-exec-enabled").getInteger() ;
        climb_down_exec_enabled_led_ = createLED(num) ;
    }

    @Override
    protected void initializeGadgets() throws BadParameterTypeException, MissingParameterException {
        super.initializeGadgets();
        int num ;

        num = getSubsystem().getSettingsValue("panel:gadgets:target_toggle1").getInteger() ;
        target_toggle1_gadget_ = mapButton(num, ButtonType.Level) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:target_toggle2").getInteger() ;
        target_toggle2_gadget_ = mapButton(num, ButtonType.Level) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:amp_purpose1").getInteger() ;
        amp_purpose_target1_gadget_ = mapButton(num, ButtonType.Level) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:amp_purpose2").getInteger() ;
        amp_purpose_target2_gadget_ = mapButton(num, ButtonType.Level) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:climb_up_prep").getInteger() ;
        climb_up_prep_gadget_ = mapButton(num, ButtonType.LowToHigh) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:climb_up_exec").getInteger() ;
        climb_up_exec_gadget_ = mapButton(num, ButtonType.LowToHigh) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:climb_down_prep").getInteger() ;
        climb_down_prep_gadget_ = mapButton(num, ButtonType.LowToHigh) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:climb_down_exec").getInteger() ;
        climb_down_exec_gadget_ = mapButton(num, ButtonType.LowToHigh) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:shoot").getInteger() ;
        shoot_gadget_ = mapButton(num, ButtonType.LowToHigh) ;
        
        num = getSubsystem().getSettingsValue("panel:gadgets:handoff").getInteger() ;
        handoff_gadget_ = mapButton(num, ButtonType.LowToHigh) ;
    }
}
