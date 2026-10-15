// quadsim - runs src/quad.h on its own (no game) and prints how the quad flies: hover throttle, rate
// response (step, overshoot, settling), a flip, top speed, a fall, battery drain. For tuning.
//   cl /std:c++20 /O2 /EHsc tests\quadsim\quadsim.cpp && quadsim
#include "../../src/kamikaze.h"
#include <cstdio>
#include <cstdlib>
#include <string>

using namespace quad;
static const double kDt = 0.001, kD2r = 3.14159265358979 / 180;

static Build s_build;  // FPV Kamikaze Drone's defaults; "quadsim <prop> <pitch> <kv> <cells> <grams>" for others

static Quad Make() {
    Quad q;
    q.p = FromBuild(s_build);
    const V3 motors[4] = {{0.076, -0.118, 0.034}, {0.076, 0.118, 0.034}, {-0.082, -0.109, 0.010}, {-0.082, 0.109, 0.010}};
    q.Setup(motors);
    q.pos = {0, 0, 10};
    q.Arm(true);
    return q;
}
static void Run(Quad& q, Inputs in, double secs) { for (double t = 0; t < secs; t += kDt) q.Step(in, kDt); }

// FPV Kamikaze Drone's model (kamikaze.h): hover, climb, speed at a pitch, turn rates, fall, battery.
static void Kamikaze() {
    using kamikaze::Drone;
    using kamikaze::Sticks;
    const double h = 1.0 / 90;  // a VR frame
    auto make = [] {
        Quad q;
        q.pos = {0, 0, 100};
        return q;
    };
    printf("--- FPV Kamikaze Drone model\n");
    printf("curves: thrust at 0/30/60/90/102 km/h %.2f %.2f %.2f %.2f %.2f; ground effect at 0/5/10/30/60/117 cm %.2f %.2f %.2f %.2f %.2f %.2f\n",
           kamikaze::Eval(kamikaze::kThrustCurve, 0), kamikaze::Eval(kamikaze::kThrustCurve, 30), kamikaze::Eval(kamikaze::kThrustCurve, 60),
           kamikaze::Eval(kamikaze::kThrustCurve, 90), kamikaze::Eval(kamikaze::kThrustCurve, 102), kamikaze::Eval(kamikaze::kGroundCurve, 0),
           kamikaze::Eval(kamikaze::kGroundCurve, 5), kamikaze::Eval(kamikaze::kGroundCurve, 10), kamikaze::Eval(kamikaze::kGroundCurve, 30),
           kamikaze::Eval(kamikaze::kGroundCurve, 60), kamikaze::Eval(kamikaze::kGroundCurve, 117));
    double lo = 0, hi = 1;
    for (int it = 0; it < 30; it++) {
        double mid = (lo + hi) / 2;
        Quad q = make();
        Drone d;
        for (double t = 0; t < 2; t += h) d.Step(q, {mid, 0, 0, 0}, 0, h);
        (q.vel.z > 0 ? hi : lo) = mid;
    }
    printf("hover: throttle %.3f\n", (lo + hi) / 2);
    {
        Quad q = make();
        Drone d;
        double t = 0, at1 = 0;
        for (; t < 6; t += h) { d.Step(q, {1, 0, 0, 0}, 0, h); if (t < 1 && t + h >= 1) at1 = q.vel.z; }
        printf("full throttle straight up: %.1f m/s after 1 s, %.1f m/s (%.0f km/h) after 6 s\n", at1, q.vel.z, q.vel.z * 3.6);
    }
    for (double pitch : {20.0, 35.0, 50.0, 65.0, 80.0}) {
        Quad q = make();
        Drone d;
        double a = pitch * kD2r;
        q.att = {0, std::sin(a / 2), 0, std::cos(a / 2)};  // nose down by `pitch`
        for (double t = 0; t < 30; t += h) d.Step(q, {1, 0, 0, 0}, 0, h);
        printf("full throttle, %2.0f deg nose down: %.1f m/s forward (%.0f km/h), %+.1f m/s up\n", pitch, q.vel.x, q.vel.x * 3.6, q.vel.z);
    }
    {
        Quad q = make();
        Drone d;
        d.Step(q, {0.5, 1, 0, 0}, 0, h);
        V3 r = q.rate;
        d.Step(q, {0.5, 0, 1, 1}, 0, h);
        V3 r2 = q.rate;
        d.Step(q, {0.5, 0, 0, 0}, 0, h);
        printf("full stick at %.0f fps: roll %.0f deg/s, pitch %.0f, yaw %.0f; stick released: %.0f deg/s\n", d.p.frameRate, -r.x / kD2r,
               r2.y / kD2r, r2.z / kD2r, Len(q.rate) / kD2r);
    }
    // Carrying a payload (two grenades): heavier, and draggier with it hanging under the drone.
    for (double add : {0.06, 0.08, 0.10}) {
        for (double drag : {0.25, 0.5}) {
            auto load = [&](Drone& d) { d.p.mass += add; d.p.linearDamping += drag; };
            double lo2 = 0, hi2 = 1;
            for (int it = 0; it < 30; it++) {
                double mid = (lo2 + hi2) / 2;
                Quad q = make();
                Drone d;
                load(d);
                for (double t = 0; t < 2; t += h) d.Step(q, {mid, 0, 0, 0}, 0, h);
                (q.vel.z > 0 ? hi2 : lo2) = mid;
            }
            Quad q = make();
            Drone d;
            load(d);
            for (double t = 0; t < 4; t += h) d.Step(q, {1, 0, 0, 0}, 0, h);
            double v0 = q.vel.z, z0 = q.pos.z;
            while (q.vel.z > 0) d.Step(q, {0, 0, 0, 0}, 0, h);
            Quad f = make();
            Drone e;
            load(e);
            double a = 50 * kD2r;
            f.att = {0, std::sin(a / 2), 0, std::cos(a / 2)};
            for (double t = 0; t < 30; t += h) e.Step(f, {1, 0, 0, 0}, 0, h);
            printf("payload +%.0f g, drag +%.2f: hover %.0f%%, full climb %.1f m/s, cut: coasts %.1f m (%.0f ft); 50 deg flat out %.0f km/h\n",
                   add * 1000, drag, (lo2 + hi2) / 2 * 100, v0, q.pos.z - z0, (q.pos.z - z0) * 3.281, f.vel.x * 3.6);
        }
    }
    // Full throttle straight up for 1, 2 or 4 s, then cut: how far it coasts up, at a few dampings.
    for (double damp : {0.25, 0.5, 1.0, 2.0}) {
        for (double climb : {1.0, 2.0, 4.0}) {
            Quad q = make();
            Drone d;
            d.p.linearDamping = damp;
            for (double t = 0; t < climb; t += h) d.Step(q, {1, 0, 0, 0}, 0, h);
            double v0 = q.vel.z, z0 = q.pos.z, t = 0;
            while (q.vel.z > 0 && t < 10) { d.Step(q, {0, 0, 0, 0}, 0, h); t += h; }
            printf("damping %.2f: full throttle up %.0f s -> %4.1f m/s, cut: coasts %4.1f m (%3.0f ft) up in %.2f s\n", damp, climb, v0,
                   q.pos.z - z0, (q.pos.z - z0) * 3.281, t);
        }
    }
    {
        Quad q = make();
        Drone d;
        for (double t = 0; t < 30; t += h) d.Step(q, {0, 0, 0, 0}, 0, h);
        printf("zero throttle fall: %.1f m/s after 30 s\n", -q.vel.z);
        Drone e;
        double t = 0;
        for (; e.controlled && t < 600; t += h) e.Step(q, {0.35, 0, 0, 0}, 0, h);
        printf("battery: flat after %.0f s\n", t);
    }
}

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "kamikaze") { Kamikaze(); return 0; }
    if (argc > 5) {
        s_build.propInches = atof(argv[1]); s_build.pitchInches = atof(argv[2]); s_build.kv = atof(argv[3]);
        s_build.cells = atoi(argv[4]); s_build.grams = atof(argv[5]);
    }
    {
        Quad q = Make();
        const Params& p = q.p;
        printf("build: %.1fx%.1f\" props, %.0f KV, %dS %.0f mAh, %.0f g -> thrust/weight %.1f, %.1f N a motor, pitch speed %.0f m/s,\n"
               "  arm %.0f mm, inertia (%.4f %.4f %.4f), motor lag %.0f/%.0f ms, %.0f W a motor flat out\n",
               s_build.propInches, s_build.pitchInches, s_build.kv, p.cells, p.capacity, p.mass * 1000, p.thrustToWeight, q.TmaxFull(), p.pitchSpeed,
               p.armLength * 1000, p.inertia.x, p.inertia.y, p.inertia.z, p.motorUp * 1000, p.motorDown * 1000, p.motorWatts);
    }
    // Hover throttle: the throttle that holds height on a full pack.
    double lo = 0, hi = 1;
    for (int it = 0; it < 30; it++) {
        double mid = (lo + hi) / 2;
        Quad q = Make();
        Run(q, {mid, 0, 0, 0}, 1.5);
        (q.vel.z > 0 ? hi : lo) = mid;
    }
    double hover = (lo + hi) / 2;
    {
        Quad q = Make();
        Run(q, {hover, 0, 0, 0}, 2);
        printf("hover: throttle %.3f, %.1f A at %.2f V (%.2f V/cell)\n", hover, q.bat.amps, q.bat.volts, q.bat.volts / q.p.cells);
    }
    // Roll step to full stick, then back to centre.
    for (int axis = 0; axis < 3; axis++) {
        Quad q = Make();
        Run(q, {hover, 0, 0, 0}, 0.5);
        Inputs in{hover, 0, 0, 0};
        (axis == 0 ? in.roll : axis == 1 ? in.pitch : in.yaw) = 1;
        double want = (axis == 0 ? -1 : 1) * Controller::Rate(axis == 2 ? q.fc.yawRates : q.fc.rollRates, 1) * kD2r;
        double peak = 0, t63 = -1, settle = -1;
        for (double t = 0; t < 0.6; t += kDt) {
            q.Step(in, kDt);
            double r = axis == 0 ? q.rate.x : axis == 1 ? q.rate.y : q.rate.z;
            peak = std::max(peak, r / want);
            if (t63 < 0 && r / want > 0.63) t63 = t;
            if (std::fabs(r / want - 1) > 0.05) settle = t;
        }
        Inputs back{hover, 0, 0, 0};
        double over = 0;
        for (double t = 0; t < 0.4; t += kDt) {
            q.Step(back, kDt);
            double r = axis == 0 ? q.rate.x : axis == 1 ? q.rate.y : q.rate.z;
            over = std::max(over, -r / want);
        }
        double rest = axis == 0 ? q.rate.x : axis == 1 ? q.rate.y : q.rate.z;
        printf("%s step to %.0f deg/s: 63%% in %.0f ms, peak %.0f%%, within 5%% after %.0f ms; on release overshoot %.0f%%, left %.1f deg/s\n",
               axis == 0 ? "roll " : axis == 1 ? "pitch" : "yaw  ", want / kD2r, t63 * 1000, peak * 100, settle * 1000, over * 100, rest / kD2r);
    }
    // A knock (as from leaving the ground) then hands off: the rate loop must stop the turn and hold.
    {
        Quad q = Make();
        q.rate = {-7 * kD2r, 2 * kD2r, 0};
        for (int i = 0; i < 12; i++) {
            Run(q, {i < 4 ? 0.75 : 0.51, 0, 0, 0}, 0.25);
            double r, p;
            q.Tilt(r, p);
            printf("  knock t=%.2f roll %.2f pitch %.2f rates (%.2f %.2f %.2f) deg/s iterm (%.2f %.2f %.2f) sat %d\n", (i + 1) * 0.25, r / kD2r,
                   p / kD2r, q.rate.x / kD2r, q.rate.y / kD2r, q.rate.z / kD2r, q.fc.iterm.x, q.fc.iterm.y, q.fc.iterm.z, q.fc.saturated);
        }
    }
    // A flip: full roll until 360 degrees, then level.
    {
        Quad q = Make();
        Run(q, {hover, 0, 0, 0}, 0.5);
        double z0 = q.pos.z, turned = 0, t = 0;
        while (turned < 2 * 3.14159265358979 - 0.6 && t < 2) { q.Step({0.7, 1, 0, 0}, kDt); turned += -q.rate.x * kDt; t += kDt; }
        double tflip = t;
        Run(q, {hover, 0, 0, 0}, 0.3);
        double r, p;
        q.Tilt(r, p);
        printf("flip: 360 deg in %.0f ms, height change %.2f m, then level within %.1f deg roll\n", tflip * 1000, q.pos.z - z0, r / kD2r);
    }
    // Top speed: angle mode, full forward and full throttle.
    {
        Quad q = Make();
        q.fc.angleMode = true;
        Run(q, {1, 0, 1, 0}, 8);
        double r, p;
        q.Tilt(r, p);
        printf("top speed (angle mode, 55 deg, full throttle): %.1f m/s (%.0f km/h), climbing %.1f m/s, pitch %.0f, %.0f A, %.2f V/cell\n",
               std::sqrt(q.vel.x * q.vel.x + q.vel.y * q.vel.y), std::sqrt(q.vel.x * q.vel.x + q.vel.y * q.vel.y) * 3.6, q.vel.z, p / kD2r,
               q.bat.amps, q.bat.volts / q.p.cells);
    }
    // Punch out: full throttle straight up.
    {
        Quad q = Make();
        Run(q, {1, 0, 0, 0}, 3);
        printf("punch-out: %.1f m/s up after 3 s, %.0f m climbed, %.0f A\n", q.vel.z, q.pos.z - 10, q.bat.amps);
    }
    // Falling, disarmed.
    {
        Quad q = Make();
        q.Arm(false);
        Run(q, {0, 0, 0, 0}, 6);
        printf("disarmed fall: %.1f m/s after 6 s\n", -q.vel.z);
    }
    // Battery: hover until a cell sags to 3.3 V.
    {
        Quad q = Make();
        double t = 0;
        while (t < 1200 && q.bat.volts / q.p.cells > 3.3) {
            Inputs in{hover, 0, 0, 0};
            // hold height: nudge throttle with the climb rate
            in.throttle = Clamp(hover + (0 - q.vel.z) * 0.08 + (10 - q.pos.z) * 0.02 + (1 - q.bat.volts / (q.p.cells * 4.0)) * 0.3, 0, 1);
            q.Step(in, kDt);
            t += kDt;
        }
        printf("hovering: %.0f s (%.1f min) to 3.3 V/cell, %.0f mAh used of %.0f\n", t, t / 60, q.bat.used, q.p.capacity);
    }
    return 0;
}
