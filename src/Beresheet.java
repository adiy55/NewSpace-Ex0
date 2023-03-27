import simulation.Moon;

public class Beresheet {
    public static final double WEIGHT_EMP = 165; // kg
    public static final double WEIGHT_FULE = 420; // kg
    public static final double WEIGHT_FULL = WEIGHT_EMP + WEIGHT_FULE; // kg
    // https://davidson.weizmann.ac.il/online/askexpert/%D7%90%D7%99%D7%9A-%D7%9E%D7%98%D7%99%D7%A1%D7%99%D7%9D-%D7%97%D7%9C%D7%9C%D7%99%D7%AA-%D7%9C%D7%99%D7%A8%D7%97
    public static final double MAIN_ENG_F = 430; // N
    public static final double SECOND_ENG_F = 25; // N
    public static final double MAIN_BURN = 0.15; //liter per sec, 12 liter per m'
    public static final double SECOND_BURN = 0.009; //liter per sec 0.6 liter per m'
    public static final double ALL_BURN = MAIN_BURN + 8 * SECOND_BURN;

    public static double accMax(double weight) {
        return acc(weight, true, 8);
    }

    public static double acc(double weight, boolean main, int seconds) {
        double t = 0;
        if (main) {
            t += MAIN_ENG_F;
        }
        t += seconds * SECOND_ENG_F;
        return t / weight;
    }

    public static double desired_vs(double alt) {
        double max_alt = 30000;
        if (alt > max_alt) {
            return 0;
        }
        if (alt > 1000) {
            return 23;
        }
        if (alt > 500) {
            return 13 + 10 * (alt - 500) / 500;
        }
        if (alt > 70) {
            return 5 + 8 * (alt - 70) / 430;
        }
        return 2;
    }

    public static double desired_hs(double alt) {
        double min_alt = 2000, max_alt = 30000;
        if (alt < min_alt) {
            return 0;
        }
        if (alt > max_alt) {
            return Moon.EQ_SPEED;
        }
        double norm = (alt - min_alt) / (max_alt - min_alt);
        norm = Math.pow(norm, 0.70); // [0,1]
        return norm * Moon.EQ_SPEED;
    }

    public static double updatePower(double curr_power, double gas) {
        double power = curr_power + gas;
        if (power < 0) {
            return 0;
        } else if (power > 1) {
            return 1;
        }
        return power;
    }

    // 14095, 955.5, 24.8, 2.0
    public static void main(String[] args) {
        System.out.println("Simulating Bereshit's Landing:");
        // starting point:
        double vs = 24.8;
        double hs = 932;
        double dist = 181 * 1000;
        double ang = 58.3; // zero is vertical (as in landing)
        double alt = 13748; // 2:25:40 (as in the simulation) // https://www.youtube.com/watch?v=JJ0VfRL9AMs
        double time = 0;
        double dt = 1; // sec
        double acc = 0; // Acceleration rate (m/s^2)
        double fuel = 121; //
        double weight = WEIGHT_EMP + fuel;

        PID vsPID = new PID(0.04, 0.0003, 0.2, 100);
        double dvs, gas;

        System.out.println("time, vs, hs, dist, alt, ang, weight, acc");
        double power = 0.7; // rate[0,1]
        // ***** main simulation loop ******

        while (alt > 0) {
            if (time % 10 == 0 || alt < 100) {
                System.out.println(time + "," + vs + "," + hs + "," + dist + "," + alt + "," + ang + "," + weight + "," + acc);
            }

            dvs = desired_vs(alt);
            gas = vsPID.update(vs - dvs, dt);
            power = updatePower(power, gas);

            if (Math.abs(hs) < 3) {
                if (ang > 3) {
                    ang -= 3;
                } else {
                    ang = 0;
                }
            }

            // main computations
            double ang_rad = Math.toRadians(ang);
            double h_acc = Math.sin(ang_rad) * acc;
            double v_acc = Math.cos(ang_rad) * acc;
            double vacc = Moon.getAcc(hs);
            time += dt;
            double dw = dt * ALL_BURN * power;
            if (fuel > 0) {
                fuel -= dw;
                weight = WEIGHT_EMP + fuel;
                acc = power * accMax(weight);
            } else { // ran out of fuel
                acc = 0;
            }

            v_acc -= vacc;
            if (hs > 2.5) {
                hs -= h_acc * dt;
            } else {
                hs = 0;
            }
            dist -= hs * dt;
            vs -= v_acc * dt;
            alt -= dt * vs;
        }
    }
}

