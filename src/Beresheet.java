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

<<<<<<< HEAD
=======
    public static double updateAngle(double curr_angle, double change_angle, double dt) {
        double angle = curr_angle + change_angle;
        double angular_momentum = 3;
        if (angle >= curr_angle + 3) {
            if (curr_angle + angular_momentum * dt > 90)
            {
                return 90;
            }
            return curr_angle + angular_momentum * dt;
        }
        else if (angle <= curr_angle - 3) {
            if (curr_angle - angular_momentum * dt < 0)
            {
                return 0;
            }
            return curr_angle - angular_momentum * dt;
        }
        else if (angle > 90)
        {
            return 90;
        }
        else if (angle < 0)
        {
            return 0;
        }
        return angle;
    }

>>>>>>> c427576 (Init commit)
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
<<<<<<< HEAD
        double ang = 58.3; // zero is vertical (as in landing)
        double alt = 13748; // 2:25:40 (as in the simulation) // https://www.youtube.com/watch?v=JJ0VfRL9AMs
        double time = 0;
        double dt = 1; // sec
=======
        double ang = 60; // zero is vertical (as in landing)
        double alt = 13748; // 2:25:40 (as in the simulation) // https://www.youtube.com/watch?v=JJ0VfRL9AMs
        double time = 0;
        double dt = 0.1; // sec
>>>>>>> c427576 (Init commit)
        double acc = 0; // Acceleration rate (m/s^2)
        double fuel = 121; //
        double weight = WEIGHT_EMP + fuel;

<<<<<<< HEAD
        PID vsPID = new PID(0.04, 0.0003, 0.2, 100);
        double dvs, gas;

=======
        PID vsPID = new PID(0.3, 0.01, 9, 100);
        double dvs, gas;

        PID hsPID = new PID(0.06, 0.00002, 4, 100);
        double dhs, change_ang;

>>>>>>> c427576 (Init commit)
        System.out.println("time, vs, hs, dist, alt, ang, weight, acc");
        double power = 0.7; // rate[0,1]
        // ***** main simulation loop ******

        while (alt > 0) {
<<<<<<< HEAD
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
=======
            //if (time % 10 == 0 || alt < 100) {
                System.out.println(time + "," + vs + "," + hs + "," + dist + "," + alt + "," + ang + "," + weight + "," + acc + ", " + power);
            //}

            dvs = desired_vs(alt);
            dhs = desired_hs(alt);

            double epsilon = 1;


//            if (Math.abs(vs - dvs) > epsilon || Math.abs(hs - hvs) > epsilon) {
//                change_ang = hsPID.update(hs - hvs, dt);
//                ang = updateAngle(ang, change_ang, dt);
//            }
            dvs = desired_vs(alt);
            gas = vsPID.update(vs - dvs, dt);
            power = updatePower(power, gas);

            dhs = desired_hs(alt);
            change_ang = hsPID.update((hs - dhs), dt);
            ang = updateAngle(ang, change_ang, dt);

//            if (Math.abs(hs) < 3) {
//                if (ang > 3) {
//                    ang -= 3;
//                } else {
//                    ang = 0;
//                }
//            }
//            else {
//                hvs = desired_hs(alt);
//                change_ang = hsPID.update(hs - hvs, dt);
//                ang = updateAngle(ang, change_ang, dt);
//            }
>>>>>>> c427576 (Init commit)

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
<<<<<<< HEAD
            if (hs > 2.5) {
                hs -= h_acc * dt;
            } else {
                hs = 0;
            }
=======
            hs -= h_acc * dt;
>>>>>>> c427576 (Init commit)
            dist -= hs * dt;
            vs -= v_acc * dt;
            alt -= dt * vs;
        }
    }
}

