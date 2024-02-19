package frc.robot.subsystems.oi;

import org.xero1425.base.subsystems.oi.OILed;
import org.xero1425.base.subsystems.oi.OIPanel;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.base.subsystems.oi.OILed.LEDState;
import org.xero1425.base.subsystems.oi.OIPanelButton.ButtonType;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class ButchOIPanel extends OIPanel {
    public enum NoteTarget {
        Invalid,
        Trap,
        Amp,
        Speaker
    } ;


    //
    // Panel gadgets
    //
    private int target_toggle1_gadget_ ;                    // Speaker, Amp, or Trap
    private int target_toggle2_gadget_ ;                    // Speaker, Amp, or Trap     
    private int climb_up_prep_gadget_ ;                     // Prepare to climb up
    private int climb_up_exec_gadget_ ;                     // Perform the climb up
    private int shoot_gadget_ ;                             // Shoot the note
    private int abort_gadget_ ;                             // Abort mode
    private int coast1_gadget_ ;
    private int coast2_gadget_ ;

    private int climb_down_gadget_ ;                        // Prepare to climb down
    private int eject_gadget_ ;                             // Eject the note
    private int turtle_gadget_ ;                            // Turtle mode

    //
    // LEDs
    //
    private OILed db_ready_led_ ;
    private OILed shooter_velocity_ready_led_ ;
    private OILed shooter_tilt_ready_led_ ;
    private OILed shooter_april_tag_led_ ;

    private OILed climb_up_prep_enabled_led_ ;
    private OILed climb_up_exec_enabled_led_ ;
    private OILed climb_down_exec_enabled_led_ ;

    //////////////////////////////////////////////////////////////////////
    // Actions
    //////////////

    public ButchOIPanel(OISubsystem oi, int index) throws BadParameterTypeException, MissingParameterException {
        super(oi, "oipanel", index);
    }

    public void setVelocityLED(OILed.LEDState st) {
        shooter_velocity_ready_led_.setState(st) ;
    }

    public void setTiltLED(OILed.LEDState st) {
        shooter_tilt_ready_led_.setState(st) ;
    }

    public void setAprilTagLED(OILed.LEDState st) {
        shooter_april_tag_led_.setState(st) ;
    }

    public void setDBLED(OILed.LEDState st) {
        db_ready_led_.setState(st) ;
    }

    public void setClimbUpPrepLED(OILed.LEDState st) {
        climb_up_prep_enabled_led_.setState(st) ;
    }

    public void setClimbUpExecLED(OILed.LEDState st) {
        climb_up_exec_enabled_led_.setState(st) ;
    }

    public void setClimbDownLED(OILed.LEDState st) {
        climb_down_exec_enabled_led_.setState(st) ;
    }

    @Override
    protected void initializeLEDs() throws BadParameterTypeException, MissingParameterException {
        super.initializeLEDs();
        int num ;

        num = getSubsystem().getSettingsValue("panel:leds:db-ready").getInteger() ;
        db_ready_led_ = createLED(num, false) ;
        db_ready_led_.setState(LEDState.OFF);
        
        num = getSubsystem().getSettingsValue("panel:leds:shooter-velocity-ready").getInteger() ;
        shooter_velocity_ready_led_ = createLED(num, false) ;
        shooter_velocity_ready_led_.setState(LEDState.OFF);        

        num = getSubsystem().getSettingsValue("panel:leds:shooter-tilt-ready").getInteger() ;
        shooter_tilt_ready_led_ = createLED(num, false) ;
        shooter_tilt_ready_led_.setState(LEDState.OFF);

        num = getSubsystem().getSettingsValue("panel:leds:shooter-april-tag").getInteger() ;
        shooter_april_tag_led_ = createLED(num, false) ;
        shooter_april_tag_led_.setState(LEDState.OFF);

        num = getSubsystem().getSettingsValue("panel:leds:climb-up-prep-enabled").getInteger() ;
        climb_up_prep_enabled_led_ = createLED(num, false) ;
        climb_up_prep_enabled_led_.setState(LEDState.OFF);

        num = getSubsystem().getSettingsValue("panel:leds:climb-up-exec-enabled").getInteger() ;
        climb_up_exec_enabled_led_ = createLED(num, false) ;
        climb_up_exec_enabled_led_.setState(LEDState.OFF);

        num = getSubsystem().getSettingsValue("panel:leds:climb-down-exec-enabled").getInteger() ;
        climb_down_exec_enabled_led_ = createLED(num, false) ;
        climb_down_exec_enabled_led_.setState(LEDState.OFF);        
    }

    @Override
    protected void initializeGadgets() throws BadParameterTypeException, MissingParameterException {
        super.initializeGadgets();
        int num ;

        num = getSubsystem().getSettingsValue("panel:gadgets:target_toggle1").getInteger() ;
        target_toggle1_gadget_ = mapButton(num, ButtonType.Level) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:target_toggle2").getInteger() ;
        target_toggle2_gadget_ = mapButton(num, ButtonType.Level) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:coast1").getInteger() ;
        coast1_gadget_ = mapButton(num, ButtonType.Level) ;        

        num = getSubsystem().getSettingsValue("panel:gadgets:coast2").getInteger() ;
        coast2_gadget_ = mapButton(num, ButtonType.Level) ;           

        num = getSubsystem().getSettingsValue("panel:gadgets:climb_up_prep").getInteger() ;
        climb_up_prep_gadget_ = mapButton(num, ButtonType.LowToHigh) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:climb_up_exec").getInteger() ;
        climb_up_exec_gadget_ = mapButton(num, ButtonType.LowToHigh) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:climb_down").getInteger() ;
        climb_down_gadget_ = mapButton(num, ButtonType.LowToHigh) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:shoot").getInteger() ;
        shoot_gadget_ = mapButton(num, ButtonType.LowToHigh) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:turtle").getInteger() ;
        turtle_gadget_ = mapButton(num, ButtonType.LowToHigh) ;
        
        num = getSubsystem().getSettingsValue("panel:gadgets:abort").getInteger() ;
        abort_gadget_ = mapButton(num, ButtonType.LowToHigh) ;
        
        num = getSubsystem().getSettingsValue("panel:gadgets:eject").getInteger() ;
        eject_gadget_ = mapButton(num, ButtonType.LowToHigh) ;
    }

    public NoteTarget getNoteTarget() {
        NoteTarget ret = NoteTarget.Invalid ;

        if (getValue(target_toggle1_gadget_) == 1 && getValue(target_toggle2_gadget_) == 0) {
            ret = NoteTarget.Speaker ;
        }
        else if (getValue(target_toggle1_gadget_) == 0 && getValue(target_toggle2_gadget_) == 0) {
            ret = NoteTarget.Amp ;
        }
        else if (getValue(target_toggle1_gadget_) == 0 && getValue(target_toggle2_gadget_) == 1) {
            ret = NoteTarget.Trap ;
        }

        return ret ;
    }

    public boolean isClimbUpPrepPressed() {
        return getValue(climb_up_prep_gadget_) == 1 ;
    }

    public boolean isClimbUpExecPressed() {
        return getValue(climb_up_exec_gadget_) == 1 ;
    }

    public boolean isClimbDownPressed() {
        return getValue(climb_down_gadget_) == 1 ;
    }

    public boolean isShootPressed() {
        return getValue(shoot_gadget_) == 1 ;
    }

    public boolean isAbortPressed() {
        return getValue(abort_gadget_) == 1 ;
    }

    public boolean isEjectPressed() {
        return getValue(eject_gadget_) == 1 ;
    }

    public boolean isTurtlePressed() {
        return getValue(turtle_gadget_) == 1 ;
    }

    public int getCoastValue() {
        return getValue(coast1_gadget_) + getValue(coast2_gadget_) * 2 ;
    }
}
