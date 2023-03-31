import simulation.Moon;

public class Beresheet {
    public static final double WEIGHT_EMP = 165; // kg
    public static final double WEIGHT_FUEL = 420; // kg
    public static final double WEIGHT_FULL = WEIGHT_EMP + WEIGHT_FUEL; // kg
    // https://davidson.weizmann.ac.il/online/askexpert/%D7%90%D7%99%D7%9A-%D7%9E%D7%98%D7%99%D7%A1%D7%99%D7%9D-%D7%97%D7%9C%D7%9C%D7%99%D7%AA-%D7%9C%D7%99%D7%A8%D7%97
    public static final double MAIN_ENG_F = 430; // N // Main engine
    public static final double SECOND_ENG_F = 25; // N // Side engine
    public static final double MAIN_BURN = 0.15; // liter per sec, 12 liter per m'
    public static final double SECOND_BURN = 0.009; // liter per sec 0.6 liter per m'
    public static final double ALL_BURN = MAIN_BURN + 8 * SECOND_BURN; // Total fuel consumption // 8 side engines
    public static final double TOTAL_THRUST = MAIN_ENG_F + 8 * SECOND_ENG_F;

    private double _hs, _vs, _alt, _NN, _ang, _dist, _time, _acc, _fuel, _weight;
    private boolean _first; // flag for toString function

    Beresheet() {
        _hs = 932;
        _vs = 24.8;
        _alt = 13748; // 2:25:40 (as in the simulation) // https://www.youtube.com/watch?v=JJ0VfRL9AMs
        _NN = 0.7; // rate[0,1]
        _ang = 58.3; // zero is vertical (as in landing)
        _dist = 181 * 1000;
        _time = 0;
        _acc = 0; // Acceleration rate (m/s^2)
        _fuel = 121; //
        _weight = WEIGHT_EMP + _fuel;
        _first = true; // for toString method
    }

    @Override
    public String toString() {
        if (_first) { // print only once
            System.out.println("Simulating Beresheet's Landing:");
            System.out.println("time, vs, hs, dist, alt, ang, weight, acc, fuel");
            _first = false;
        }
        return String.format("%f, %f, %f, %f, %f, %f, %f, %f, %f", _time, _vs, _hs, _dist, _alt, _ang, _weight, _acc, _fuel);
    }

    public double getHs() {
        return _hs;
    }

    public double getVs() {
        return _vs;
    }

    public double getAlt() {
        return _alt;
    }

    public double getTime() {
        return _time;
    }

    public double getAng() {
        return _ang;
    }

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

    public double getDesiredVs() {
        double max_alt = 30000;
        if (_alt > max_alt) {
            return 30;
        }
        if (_alt > 2000) {
            return 25;
        }
        if (_alt > 1000) {
            return 23;
        }
        if (_alt > 500) {
            return 13 + 10 * (_alt - 500) / 500;
        }
        if (_alt > 70) {
            return 5 + 8 * (_alt - 70) / 430;
        }
        return 2;
    }

    public double getDesiredHs() {
        double min_alt = 2000, max_alt = 30000;
        if (_alt < min_alt) {
            return 0;
        }
        if (_alt > max_alt) {
            return Moon.EQ_SPEED;
        }
        double norm = (_alt - min_alt) / (max_alt - min_alt);
        norm = Math.pow(norm, 0.70); // [0,1]
        return norm * Moon.EQ_SPEED;
    }

    public void addPower(double gas) {
        double tmp = _NN + gas;
        if (tmp >= 0 && tmp <= 1) {
            _NN = tmp;
        }
        if (tmp > 1) {
            _NN = 1;
        }
        if (tmp < 0) {
            _NN = 0;
        }
    }

    public void addAng(double val) {
        double tmp = _ang + val;
        if (tmp >= 0 && tmp <= 90) {
            _ang = tmp;
        }
        if (tmp > 90) {
            _ang = 90;
        }
        if (tmp <= 3) {
            _ang = 0;
        }
    }

    public void computeStep(double dt) {
        // main computations
        double ang_rad = Math.toRadians(_ang);
        double h_acc = Math.sin(ang_rad) * _acc;
        double v_acc = Math.cos(ang_rad) * _acc;
        double vacc = Moon.getAcc(_hs);
        _time += dt;
        double dw = dt * ALL_BURN * _NN;
        if (_fuel > 0) {
            _fuel -= dw;
            _weight = WEIGHT_EMP + _fuel;
            _acc = _NN * accMax(_weight);
        } else { // ran out of fuel
            _acc = 0;
        }
        v_acc -= vacc;
        if (_hs <= 2.5) {
            _hs = 0;
        }
        if (_hs > 0) {
            _hs -= h_acc * dt;
        }
        _dist -= _hs * dt;
        _vs -= v_acc * dt;
        _alt -= dt * _vs;
    }

}

