import simulation.Moon;

public class Main {

    public static void main(String[] args) {
        Beresheet beresheet = new Beresheet();
        // starting point:
        double dt = 1; // sec
        double vs, hs;
        double dvs, dhs, gas;

        PID vs_controller = new PID(0.04, 0.0003, 0.2, 100);
        PID hs_controller = new PID(0.001, 0.00001, 0.01, 1000);

        // ***** main simulation loop ******
        while (beresheet.getAlt() > 0) {
            // current spacecraft values
            vs = beresheet.getVs();
            hs = beresheet.getHs();

            // desired values for PID
            dvs = beresheet.getDesiredVs();
            dhs = beresheet.getDesiredHs();

            // compute updates
            gas = vs_controller.update(vs - dvs, dt);
            beresheet.addPower(gas);
            gas = hs_controller.update(hs - dhs, dt);
            beresheet.addPower(gas);

            if (beresheet.getAlt() < 700 || beresheet.getAng() > 60) { // rotate to vertical position.
                beresheet.addAng(-3 * dt);
            }
            if (beresheet.getTime() % 10 == 0 || beresheet.getAlt() < 100) {
                System.out.println(beresheet);
            }
            beresheet.computeStep(dt);
        }
    }
}
