package frc.models;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.simulator.models.ISimMotorController;

import edu.wpi.first.hal.simulation.DIODataJNI;

public class IntakeShooterModel extends SimulationModel {
    ISimMotorController spinner_ ;
    ISimMotorController updown_ ;
    ISimMotorController shooter1_ ;
    ISimMotorController shooter2_ ;    
    ISimMotorController tilt_ ;
    int note_sensor_io_ ;
    
    public IntakeShooterModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst) ;
    }

    @Override
    public boolean create(SimulationEngine engine) throws Exception {        
        spinner_ = createSimulatedMotor(engine, "spinner");
        updown_ = createSimulatedMotor(engine, "updown");
        shooter1_ = createSimulatedMotor(engine, "shooter1");
        shooter2_ = createSimulatedMotor(engine, "shooter2");
        tilt_ = createSimulatedMotor(engine, "tilt");

        note_sensor_io_ = getProperty("note-sensor").getInteger() ;
        DIODataJNI.setIsInput(note_sensor_io_, true);
        DIODataJNI.setValue(note_sensor_io_, false) ;

        setCreated();
        return true;
    }

    @Override
    public boolean processEvent(String name, SettingsValue value) {
        if (name.equals("note-present")) {
            if (!value.isBoolean()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a boolean").endMessage();                
            }

            try {
                DIODataJNI.setValue(note_sensor_io_, value.getBoolean());
            }
            catch(BadParameterTypeException ex) {
                //
                // Will never happen since we check the value type above
                //
            }
        }
        return false ;
    }

    @Override
    public void run(double dt) {
        spinner_.run(dt) ;
        updown_.run(dt) ;
        shooter1_.run(dt) ;
        shooter2_.run(dt) ;        
        tilt_.run(dt) ;
    }
}
