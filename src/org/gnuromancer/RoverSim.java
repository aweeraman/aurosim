package org.gnuromancer;

import simbad.gui.Simbad;
import simbad.sim.Agent;
import simbad.sim.EnvironmentDescription;

import javax.vecmath.Vector3d;

public class RoverSim {

    public static class Rover extends Agent {

        public Rover(Vector3d position, String name) {
            super(position, name);

            // TODO: Initialize sensors
        }

        public void initBehavior() {
        }

        public void performBehavior() {
            // TODO: Move rover
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