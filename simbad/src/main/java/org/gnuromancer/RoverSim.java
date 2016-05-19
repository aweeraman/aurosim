package org.gnuromancer;

import simbad.gui.Simbad;
import simbad.sim.EnvironmentDescription;

public class RoverSim {

    public static class Space extends EnvironmentDescription {
        public Space() {
            EnvironmentFactory.createSquareSpace(16, this);
            /*
            add(new Rover(new Vector3d(0, 0, 0), "rover1"));
            add(new Rover(new Vector3d(0, 0, 0), "rover2"));
            add(new Rover(new Vector3d(0, 0, 0), "rover3"));
            add(new Rover(new Vector3d(0, 0, 0), "rover4"));
            add(new Rover(new Vector3d(0, 0, 0), "rover5"));
            */
        }
    }

    public static void main(String[] args) {
        Simbad simbad = new Simbad(new Space(), false);
    }
}