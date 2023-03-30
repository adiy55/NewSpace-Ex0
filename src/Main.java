import simulation.Moon;

public class Main {

    public static void main(String[] args) {
        Beresheet beresheet = new Beresheet();
        // starting point:
        double dt = 1; // sec
        double vs, hs, change_ang, ang;
        double dvs, dhs, gas, dang;

        PID vsPID = new PID(0.04, 0.0, 0.2, 100);
        PID hsPID = new PID(0.0001, 0.00002, 0.001, 950);
        PID angPID = new PID(0.0006, 0.00001, 0.0004, 90);

        // ***** main simulation loop ******
        while (beresheet.getAlt() > 0) {
            // current spacecraft values
            vs = beresheet.getVs();
            hs = beresheet.getHs();
            ang = beresheet.getAng();

            // desired values for PID
            dvs = beresheet.getDesiredVs();
            dhs = beresheet.getDesiredHs();
            dang = beresheet.getDesiredAng();

            // compute updates
            gas = vsPID.update(vs - dvs, dt);
            beresheet.addPower(gas);

            if (beresheet.getAlt() < 500) {
                beresheet.addAng(-3);
            } // rotate to vertical position.

//            gas = hsPID.update(hs - dhs, dt);
//            beresheet.addPower(gas);
//
//            change_ang = angPID.update(ang - dang, dt);
//            beresheet.addAng(change_ang);

            if (beresheet.getTime() % 10 == 0 || beresheet.getAlt() < 100) {
                System.out.println(beresheet);
            }
            beresheet.computeStep(dt);
        }
    }
}
