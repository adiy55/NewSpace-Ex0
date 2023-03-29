import simulation.Moon;

public class Main {

    public static void main(String[] args) {
        Beresheet beresheet = new Beresheet();
        // starting point:
        double dt = 1; // sec
        double vs, hs, change_ang, ang;
        double dvs, dhs, gas;

        PID vsPID = new PID(0.04, 0.0003, 0.2, 100);
        PID angPID = new PID(0.0006, 0.0, 0.0, 100);

        // ***** main simulation loop ******
        while (beresheet.getAlt() > 0) {
            beresheet.computeStep(dt);
            vs = beresheet.getVs();
            hs = beresheet.getHs();

            dvs = beresheet.getDesiredVs();
            gas = vsPID.update(vs - dvs, dt);
            beresheet.addPower(gas);

            dhs = beresheet.getDesiredHs();
            change_ang = angPID.update(hs - dhs, dt);
            beresheet.updateAng(change_ang, dt);

            if (beresheet.getTime() % 10 == 0 || beresheet.getAlt() < 100) {
                System.out.println(beresheet);
            }
        }
    }
}
