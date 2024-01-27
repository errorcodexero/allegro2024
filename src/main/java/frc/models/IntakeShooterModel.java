package frc.models;

import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.simulator.models.ISimMotorController;

public class IntakeShooterModel extends SimulationModel {
    ISimMotorController spinner_ ;
    ISimMotorController updown_ ;
    ISimMotorController feeder_ ;
    ISimMotorController shooter1_ ;
    ISimMotorController shooter2_ ;    
    ISimMotorController tilt_ ;
    
    public IntakeShooterModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst) ;
    }

    @Override
    public boolean create(SimulationEngine engine) throws Exception {
        final String bus = "" ;
        boolean exists = true ;

        for(int i = 1 ; i <= 6 ; i++) {
            //
            // This is a hard coded check to ensure the motors with CAN ids 1 - 6
            // exist.  This is assumed to be the motors for the shooter/intake subsystem.  If any
            // of them do not exists, then the assocaited modelsa are not created.
            //
            if (engine.getRobot().getMotorFactory().getMotorController(bus, 1) == null) {
                exists = false ;
                break ;
            }
        }

        if (exists) {
            //
            // Create the models for the motors
            //
            spinner_ = createSimulatedMotor(engine, "spinner");
            updown_ = createSimulatedMotor(engine, "updown");
            feeder_ = createSimulatedMotor(engine, "feeder");
            shooter1_ = createSimulatedMotor(engine, "shooter1");
            shooter2_ = createSimulatedMotor(engine, "shooter2");
            tilt_ = createSimulatedMotor(engine, "tilt");
        }

        setCreated();

        return false;
    }

    @Override
    public boolean processEvent(String name, SettingsValue value) {
        return false ;
    }

    @Override
    public void run(double dt) {
        if (spinner_ != null) {
            spinner_.run(dt) ;
            updown_.run(dt) ;
            feeder_.run(dt) ;
            shooter1_.run(dt) ;
            shooter2_.run(dt) ;        
            tilt_.run(dt) ;
        }
    }
}
