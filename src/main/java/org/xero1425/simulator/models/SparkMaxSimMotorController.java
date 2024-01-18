package org.xero1425.simulator.models;

import org.xero1425.simulator.engine.SimulationEngine;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import org.xero1425.base.motors.IMotorController;
import org.xero1425.base.motors.SparkMaxMotorController;

public class SparkMaxSimMotorController implements ISimMotorController {
    private static final double kTicksPerRev = 42 ;
    private SparkMaxMotorController motor_ ;
    private DCMotor dcmotor_ ;
    private DCMotorSim sim_ ;
    
    
    public SparkMaxSimMotorController(SimulationEngine engine, String bus, int canid, int count, double moment, double gearning) throws Exception {
        if (!bus.isEmpty()) {
            throw new Exception("SparkMax controllers must have an empty bus");
        }

        IMotorController ctrl = engine.getRobot().getMotorFactory().getMotorController(bus, canid) ;
        if (!(ctrl instanceof SparkMaxMotorController)) {
            throw new Exception("motor on bus '" + bus + "', can id " + canid + " is not a CANSparkMax motor");
        }

        motor_ = (SparkMaxMotorController)ctrl ;
        dcmotor_ = DCMotor.getNEO(count) ;
        sim_ = new DCMotorSim(dcmotor_, 1.0, moment);
    }

    @Override
    public void run(double dt) {
        SparkMaxMotorController.SimState state = motor_.getSimState() ;

        sim_.setInputVoltage(state.getMotorVoltage());

        state.setPosition(sim_.getAngularPositionRotations() * kTicksPerRev);
        state.setVelocity(sim_.getAngularVelocityRPM() * 60.0 * kTicksPerRev);
    }

    @Override
    public double ticksPerRev() {
        return kTicksPerRev ;
    }
} ;
