package org.gnuromancer;

import simbad.gui.Simbad;
import simbad.sim.Agent;
import simbad.sim.EnvironmentDescription;
import simbad.sim.RangeSensorBelt;
import simbad.sim.RobotFactory;

import javax.vecmath.Vector3d;

public class RoverSim {

    public static class Rover extends Agent {

        private RangeSensorBelt sensorBelt;

        public Rover(Vector3d position, String name) {
            super(position, name);

            sensorBelt = RobotFactory.addSonarBeltSensor(this, 1);
        }

        public void initBehavior() {
        }

        public void performBehavior() {
            setTranslationalVelocity(1.0);
            double d = sensorBelt.getMeasurement(0);
            if (d < 0.05) {
                setRotationalVelocity(Math.PI / 2 * (0.5 - Math.random()));
            }
            System.out.println(d);
        }
    }

    public static class Space extends EnvironmentDescription {
        public Space() {
            EnvironmentFactory.createSquareSpace(16, this);
            add(new Rover(new Vector3d(0, 0, 0), "rover"));
        }
    }

    public static void main(String[] args) {
        Simbad simbad = new Simbad(new Space(), false);
    }
}