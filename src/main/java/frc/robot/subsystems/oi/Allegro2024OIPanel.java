package frc.robot.subsystems.oi;

import org.xero1425.base.subsystems.oi.OIPanel;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class Allegro2024OIPanel extends OIPanel {
    
    public Allegro2024OIPanel(OISubsystem sub, String name, int index) throws BadParameterTypeException, MissingParameterException {
        super(sub, name, index) ;
    }   

    @Override
    public void computeState() throws Exception {
        super.computeState();
    }
       
    @Override
    public void createStaticActions() throws Exception {
    }    
}
