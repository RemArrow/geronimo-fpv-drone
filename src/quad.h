// quad.h - an FPV quadcopter: rigid body, four motors with propellers, a LiPo pack and a
// Betaflight-style flight controller (acro rate mode, angle mode, airmode). Pure maths, no engine:
// the plugin (FPVDrone.cpp) feeds it the sticks and the ground, does the collisions and draws it;
// tests/quadsim runs it on its own to check the tune.
//
// Units SI (m, s, kg, N, rad) in Unreal's axes: x forward, y right, z up. Unreal is left-handed, but all
// the vector maths here (cross products, quaternion rotation) is the plain textbook form applied to those
// numbers, so the equations stay self-consistent; only the stick signs need care (see Controller).
#pragma once
#include <algorithm>
#include <cmath>

namespace quad {

struct V3 {
    double x = 0, y = 0, z = 0;
    V3 operator+(V3 b) const { return {x + b.x, y + b.y, z + b.z}; }
    V3 operator-(V3 b) const { return {x - b.x, y - b.y, z - b.z}; }
    V3 operator*(double s) const { return {x * s, y * s, z * s}; }
    V3 operator-() const { return {-x, -y, -z}; }
};
inline double Dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 Cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double Len(V3 a) { return std::sqrt(Dot(a, a)); }
inline V3 Mul(V3 a, V3 b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
inline double Clamp(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }

struct Q {  // unit quaternion, body -> world
    double x = 0, y = 0, z = 0, w = 1;
};
inline Q Mul(Q a, Q b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
inline Q Normalize(Q q) {
    double l = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return l > 0 ? Q{q.x / l, q.y / l, q.z / l, q.w / l} : Q{};
}
inline V3 Rotate(Q q, V3 v) {
    V3 u{q.x, q.y, q.z};
    V3 t = Cross(u, v) * 2;
    return v + t * q.w + Cross(u, t);
}
inline V3 Unrotate(Q q, V3 v) { return Rotate({-q.x, -q.y, -q.z, q.w}, v); }
// Turning by body rates w (rad/s, body axes) for dt.
inline Q Integrate(Q q, V3 w, double dt) {
    double a = Len(w) * dt;
    if (a < 1e-12) return q;
    V3 n = w * (1 / Len(w));
    double s = std::sin(a / 2);
    return Normalize(Mul(q, Q{n.x * s, n.y * s, n.z * s, std::cos(a / 2)}));
}

// ------------------------------------------------------------------ the airframe

struct Params {
    double mass = 0.249;                 // kg (a DJI Mini 4K weighs 249 g, so does a sub-250 FPV quad)
    V3 inertia{0.00072, 0.00080, 0.0013};  // kg m2 (roll, pitch, yaw)
    double thrustToWeight = 4.0;         // all four motors flat out on a full pack
    double propDiameter = 0.0762;        // m (3")
    double torqueRatio = 0.02;           // a propeller's drag torque per newton of thrust, m
    double motorUp = 0.022, motorDown = 0.040;  // motor speed time constants, s (spinning down is slower)
    double idle = 0.055;                 // motor idle when armed, fraction of full speed (DShot idle 5.5%)
    double pitchSpeed = 45;              // m/s of inflow at which a prop stops making thrust (30k rpm x 3.5" pitch)
    // body drag area x Cd (m2): front, side, top
    V3 dragArea{0.006, 0.007, 0.016};
    double rotorDrag = 0.012;            // in-plane drag per newton of thrust per m/s (blade flapping, H-force)
    double angularDrag = 0.00012;        // N m s
    // battery
    int cells = 4;
    double capacity = 650;               // mAh
    double cellResistance = 0.012;       // ohm (a fresh 650 mAh 95C cell)
    double motorWatts = 150;             // one motor flat out (a 4S sub-250 pulls about 40 A punched out)
    double avionicsWatts = 4;            // FC, VTX, receiver, camera
    double gravity = 9.81;               // m/s2
    double armLength = 0;                // m, centre to each motor (0: where Setup is told they are)
};

// A drone from its parts, as FPV Kamikaze Drone's (HFM Games) drone builder has them, with that game's
// physics settings and defaults. FromBuild turns it into the airframe above.
struct Build {
    double propInches = 7, pitchInches = 4;  // propeller diameter and pitch (a 7040)
    double kv = 1300;                        // motor rpm per volt
    int cells = 6;
    double grams = 2600;                     // all up: frame, pack and a warhead
    double capacity = 3000;                  // mAh
    double minThrottle = 0;                  // motor speed at zero throttle (0: they stop)
    double gravity = 9.82;
    double instantPower = 1.2;      // how fast the motors spin up and down (x what the props' size gives)
    double airFriction = 0.45;      // body drag (Cd)
    double airGrip = 0.8;           // how hard the spinning props bite the air sideways (rotor drag)
    double relativeAirSpeed = 1.0;  // how much the air rushing through the props costs thrust at speed
};

inline Params FromBuild(const Build& b) {
    Params p;
    double D = std::max(b.propInches, 1.0), P = std::max(b.pitchInches, 0.5);
    p.mass = std::max(b.grams, 50.0) / 1000;
    p.gravity = b.gravity;
    p.cells = std::max(b.cells, 1);
    p.capacity = std::max(b.capacity, 50.0);
    // Motor speed at full throttle: KV x a full pack, a fifth less under load.
    double rpm = b.kv * p.cells * 4.2 * 0.78;
    // Static thrust, Abbott's propeller formula (D, P in inches, N):
    //   T = 4.392e-8 rpm D^3.5 / sqrt(P) (4.233e-4 rpm P - V0), V0 the air speed into the prop (m/s).
    double pitchSpeed = 4.2333e-4 * rpm * P;
    double thrust = 4.392399e-8 * rpm * std::pow(D, 3.5) / std::sqrt(P) * pitchSpeed;
    p.thrustToWeight = 4 * thrust / (p.mass * p.gravity);
    p.pitchSpeed = b.relativeAirSpeed > 0.01 ? pitchSpeed / b.relativeAirSpeed : 1e9;
    p.propDiameter = D * 0.0254;
    p.armLength = 0.875 * p.propDiameter;  // a frame for its props: motor to motor about 1.75 props
    double ma2 = p.mass * p.armLength * p.armLength;
    p.inertia = {0.25 * ma2, 0.27 * ma2, 0.45 * ma2};  // pack and payload in the middle
    p.torqueRatio = 0.0035 * D;
    double tau = 0.0045 * std::pow(D, 1.1) / std::max(b.instantPower, 0.1);  // bigger props spin up slower
    p.motorUp = tau;
    p.motorDown = tau * 1.8;
    p.idle = Clamp(b.minThrottle, 0, 0.5);
    double size = (D / 7) * (D / 7);
    p.dragArea = V3{0.09, 0.10, 0.16} * (size * b.airFriction);  // frame, pack and the payload under it
    p.rotorDrag = 0.075 * b.airGrip;
    p.angularDrag = 0.00012 * std::pow(D / 3, 4);
    // One motor flat out: momentum theory at that thrust, figure of merit 0.55.
    double disc = 3.14159265358979 * std::pow(p.propDiameter / 2, 2);
    p.motorWatts = std::pow(thrust, 1.5) / std::sqrt(2 * 1.225 * disc) / 0.55;
    p.cellResistance = 7.8 / p.capacity;  // 12 mOhm for 650 mAh
    p.avionicsWatts = 8;
    return p;
}

// A LiPo cell's resting voltage at a state of charge (0..1).
inline double CellVoltage(double soc) {
    static const double v[] = {3.27, 3.50, 3.62, 3.69, 3.73, 3.77, 3.81, 3.86, 3.93, 4.03, 4.20};
    soc = Clamp(soc, 0, 1) * 10;
    int i = std::min(9, (int)soc);
    return v[i] + (v[i + 1] - v[i]) * (soc - i);
}

struct Motor { V3 pos; double spin; double speed = 0; double cmd = 0; double thrust = 0; };  // spin: +1/-1 about body z

struct Battery {
    double soc = 1, used = 0;  // mAh used
    double volts = 16.8, amps = 0;
};

// ------------------------------------------------------------------ the flight controller

struct Rates { double center = 30, max = 220, expo = 0.3; };  // Betaflight "Actual": deg/s at centre, at full stick, expo

struct Inputs { double throttle = 0, roll = 0, pitch = 0, yaw = 0; };  // throttle 0..1, others -1..1

struct Controller {
    Rates rollRates, pitchRates, yawRates{30, 160, 0.2};
    double kp[3] = {55, 55, 35};  // rate loop, 1/s: angular acceleration per rad/s of error
    double ki[3] = {35, 35, 25};  // 1/s2
    double kd[3] = {0.9, 0.9, 0};
    double kff[3] = {0.4, 0.4, 0.4};  // feed-forward of the setpoint's change (low: it eases into a turn)
    double angleGain = 6, maxAngle = 55;  // angle mode: 1/s, deg
    bool angleMode = false;
    bool airmode = true;
    V3 iterm, lastSp, lastRate, spSmooth, spIn;
    double smoothing = 0.09;  // s: RC smoothing of the sticks - a heavy quad eases into turns rather than snapping
    bool saturated = false;  // the mixer ran out of motor last step: the I-term holds
    bool airborne = false;   // the throttle has been past 25% since arming (Betaflight: airmode start throttle)

    // Betaflight's ACTUAL rates.
    static double Rate(const Rates& r, double s) {
        double expof = std::fabs(s) * (std::pow(s, 5) * r.expo + s * (1 - r.expo));
        return s * r.center + std::max(0.0, r.max - r.center) * expof;
    }
    void Reset() { iterm = {}; lastSp = {}; lastRate = {}; spSmooth = {}; spIn = {}; saturated = false; airborne = false; }
};

// ------------------------------------------------------------------ the quad

struct Quad {
    Params p;
    Controller fc;
    Motor m[4];
    Battery bat;
    V3 pos, vel;       // m, m/s world
    Q att;             // body -> world
    V3 rate;           // rad/s body
    bool armed = false;
    double groundHeight = 99;  // m from the props' plane to the ground below (for ground effect)
    double washDisturb = 0;    // prop wash: how hard the air is pushing back, 0..1
    unsigned noise = 12345;
    double mix[4][4];  // [motor][collective, roll, pitch, yaw] -> thrust

    double TmaxFull() const { return p.thrustToWeight * p.mass * p.gravity / 4; }

    // Motor layout in body metres; spin +1 = turning about +z (body up).
    void Setup(const V3 pos4[4]) {
        static const double spins[4] = {+1, -1, -1, +1};  // FL, FR, BL, BR: diagonals turn the same way
        double arm = 0;
        for (int i = 0; i < 4; i++) arm += std::sqrt(pos4[i].x * pos4[i].x + pos4[i].y * pos4[i].y) / 4;
        double k = p.armLength > 0 && arm > 0 ? p.armLength / arm : 1;  // the frame its props need
        for (int i = 0; i < 4; i++) { m[i].pos = V3{pos4[i].x * k, pos4[i].y * k, pos4[i].z}; m[i].spin = spins[i]; }
        // Thrusts T -> (sum T, roll torque, pitch torque, yaw torque) is linear: invert it once.
        double a[4][8] = {};
        for (int i = 0; i < 4; i++) {
            a[0][i] = 1;
            a[1][i] = m[i].pos.y;                  // tau_x = y T
            a[2][i] = -m[i].pos.x;                 // tau_y = -x T
            a[3][i] = -m[i].spin * p.torqueRatio;  // a prop's drag twists the body the other way
        }
        for (int r = 0; r < 4; r++) a[r][4 + r] = 1;
        for (int c = 0; c < 4; c++) {  // Gauss-Jordan
            int piv = c;
            for (int r = c + 1; r < 4; r++) if (std::fabs(a[r][c]) > std::fabs(a[piv][c])) piv = r;
            for (int k = 0; k < 8; k++) std::swap(a[c][k], a[piv][k]);
            double d = a[c][c];
            for (int k = 0; k < 8; k++) a[c][k] /= d;
            for (int r = 0; r < 4; r++) {
                if (r == c) continue;
                double f = a[r][c];
                for (int k = 0; k < 8; k++) a[r][k] -= f * a[c][k];
            }
        }
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) mix[i][j] = a[i][4 + j];
        bat = Battery{};
        bat.volts = p.cells * CellVoltage(1);
    }

    double Random() {  // -1..1
        noise = noise * 1664525u + 1013904223u;
        return (noise >> 8) / double(1 << 24) * 2 - 1;
    }

    // Roll and pitch of the body against the horizon, rad (roll + = right side down, pitch + = nose up).
    void Tilt(double& roll, double& pitch) const {
        V3 fwd = Rotate(att, {1, 0, 0}), right = Rotate(att, {0, 1, 0});
        pitch = std::asin(Clamp(fwd.z, -1, 1));
        roll = -std::asin(Clamp(right.z, -1, 1));
    }

    // One fixed step (about 1 kHz): flight controller, motors, battery, forces, motion. `extForce` (N,
    // world) and the caller's collisions come on top.
    void Step(const Inputs& in, double dt) {
        const double d2r = 3.14159265358979 / 180;
        // --- setpoints. Body rates: +x turns the right side up, +y the nose down, +z the nose right.
        V3 sp;
        double rollDeg = Controller::Rate(fc.rollRates, in.roll), pitchDeg = Controller::Rate(fc.pitchRates, in.pitch);
        double yawDeg = Controller::Rate(fc.yawRates, in.yaw);
        if (fc.angleMode) {
            double r, pch;
            Tilt(r, pch);
            double wantR = in.roll * fc.maxAngle * d2r, wantP = -in.pitch * fc.maxAngle * d2r;  // stick forward = nose down
            // Angle errors in the body frame (small-angle split is fine at these angles).
            rollDeg = (wantR - r) * fc.angleGain / d2r;
            pitchDeg = -(wantP - pch) * fc.angleGain / d2r;
        }
        sp = V3{-rollDeg, pitchDeg, yawDeg} * d2r;
        if (fc.smoothing > 0) fc.spIn = fc.spIn + (sp - fc.spIn) * std::min(1.0, dt / fc.smoothing);
        else fc.spIn = sp;
        sp = fc.spIn;

        // --- rate PID -> wanted angular acceleration -> torque.
        double throttle = Clamp(in.throttle, 0, 1);
        V3 torque;
        if (armed) {
            double e[3] = {sp.x - rate.x, sp.y - rate.y, sp.z - rate.z};
            double r[3] = {rate.x, rate.y, rate.z}, lr[3] = {fc.lastRate.x, fc.lastRate.y, fc.lastRate.z};
            double s[3] = {sp.x, sp.y, sp.z}, ls[3] = {fc.lastSp.x, fc.lastSp.y, fc.lastSp.z};
            double it[3] = {fc.iterm.x, fc.iterm.y, fc.iterm.z};
            double out[3];
            // Sitting on the ground the quad can't turn, so an I-term would only wind up against it and kick
            // on take-off: it waits until the throttle has been past 25% (Betaflight's airmode start throttle).
            if (throttle > 0.25) fc.airborne = true;
            bool idleGround = !fc.airborne || (throttle < 0.05 && !fc.airmode);
            // I-term relax (as Betaflight): while the stick moves fast the I-term holds, so flips and rolls
            // don't wind it up and bounce back; it also holds while the motors are maxed out.
            fc.spSmooth = fc.spSmooth + (sp - fc.spSmooth) * std::min(1.0, dt * 2 * 3.14159265358979 * 15);
            double hp[3] = {sp.x - fc.spSmooth.x, sp.y - fc.spSmooth.y, sp.z - fc.spSmooth.z};
            for (int k = 0; k < 3; k++) {
                double relax = Clamp(1 - std::fabs(hp[k]) / (40 * d2r), 0, 1);
                if (idleGround) it[k] = 0;
                else if (!fc.saturated) it[k] = Clamp(it[k] + fc.ki[k] * e[k] * relax * dt, -25, 25);
                double dterm = -fc.kd[k] * (r[k] - lr[k]) / dt;
                double ff = fc.kff[k] * (s[k] - ls[k]) / dt;
                ff = Clamp(ff, -400, 400);
                out[k] = fc.kp[k] * e[k] + it[k] + dterm + ff;  // rad/s2
            }
            fc.iterm = {it[0], it[1], it[2]};
            fc.lastRate = rate;
            fc.lastSp = sp;
            V3 alpha{out[0], out[1], out[2]};
            torque = Mul(alpha, p.inertia) + Cross(rate, Mul(p.inertia, rate));  // + gyroscopic term
        } else fc.Reset();

        // --- mixer (thrust space): collective from the throttle (thrust goes with speed squared, and
        // the throttle sets speed), torques on top; airmode keeps the torques by moving the collective.
        double sag = bat.volts / (p.cells * 4.2);
        double tmax = TmaxFull() * sag * sag;          // what a motor can do on this voltage
        double tmin = tmax * p.idle * p.idle;
        double collective = 4 * tmax * throttle * throttle;
        double t[4];
        if (armed) {
            double lo = 1e9, hi = -1e9, base[4], diff[4];
            for (int i = 0; i < 4; i++) {
                base[i] = mix[i][0] * collective;
                diff[i] = mix[i][1] * torque.x + mix[i][2] * torque.y + mix[i][3] * torque.z;
            }
            for (int i = 0; i < 4; i++) { lo = std::min(lo, diff[i]); hi = std::max(hi, diff[i]); }
            fc.saturated = hi - lo > tmax - tmin;
            if (fc.saturated) {  // more torque than the motors have: scale it down
                double k = (tmax - tmin) / (hi - lo);
                for (double& v : diff) v *= k;
            }
            double minT = 1e9, maxT = -1e9;
            for (int i = 0; i < 4; i++) { t[i] = base[i] + diff[i]; minT = std::min(minT, t[i]); maxT = std::max(maxT, t[i]); }
            double shift = 0;
            if (maxT > tmax) shift = tmax - maxT;
            if (minT + shift < tmin) shift = fc.airmode || throttle > 0.05 ? tmin - minT : shift;
            for (double& v : t) v = Clamp(v + shift, tmin, tmax);
            for (int i = 0; i < 4; i++) m[i].cmd = std::sqrt(t[i] / tmax);
        } else
            for (auto& mo : m) mo.cmd = 0;

        // --- motors, propellers, air.
        V3 up = Rotate(att, {0, 0, 1});
        V3 vb = Unrotate(att, vel);
        double inflow = vb.z;  // climbing along the thrust axis eats thrust
        double dyn = Clamp(1 - inflow / p.pitchSpeed, 0, 1.15);
        double ge = 1 + 0.2 * Clamp(1 - groundHeight / (2 * p.propDiameter), 0, 1);
        // Prop wash: sinking through your own downwash with little forward speed buffets the quad.
        double horiz = std::sqrt(vb.x * vb.x + vb.y * vb.y);
        double sink = -inflow;
        double wash = armed ? Clamp((sink - 2.5) / 4, 0, 1) * Clamp(1 - horiz / 6, 0, 1) : 0;
        washDisturb += (wash - washDisturb) * std::min(1.0, dt * 8);
        double totalT = 0, yawDrag = 0, watts = p.avionicsWatts;
        for (int i = 0; i < 4; i++) {
            Motor& mo = m[i];
            double target = armed ? std::max(mo.cmd, p.idle) : 0;
            double tau = target > mo.speed ? p.motorUp : p.motorDown;
            mo.speed += (target - mo.speed) * std::min(1.0, dt / tau);
            double T = tmax * mo.speed * mo.speed * dyn * ge * (1 - 0.25 * washDisturb * (1 + Random()) / 2);
            mo.thrust = T;
            totalT += T;
            yawDrag += -mo.spin * p.torqueRatio * T;
            watts += p.motorWatts * mo.speed * mo.speed * mo.speed;  // a propeller's power goes with speed cubed
        }
        V3 thrustTorque;
        for (int i = 0; i < 4; i++) thrustTorque = thrustTorque + Cross(m[i].pos, {0, 0, m[i].thrust});
        V3 bodyTorque = thrustTorque + V3{0, 0, yawDrag} - rate * p.angularDrag;
        if (washDisturb > 0.01) bodyTorque = bodyTorque + V3{Random(), Random(), 0} * (0.02 * washDisturb);

        // --- battery: the motors' power at the pack's sagging voltage.
        double ocv = p.cells * CellVoltage(bat.soc), rint = p.cells * p.cellResistance;
        double disc = ocv * ocv - 4 * watts * rint;
        bat.volts = disc > 0 ? (ocv + std::sqrt(disc)) / 2 : ocv / 2;
        bat.amps = watts / std::max(bat.volts, 1.0);
        bat.used += bat.amps * dt / 3.6;  // mAh
        bat.soc = std::max(0.0, 1 - bat.used / p.capacity);

        // --- forces: thrust, drag, gravity.
        V3 dragB{-p.dragArea.x * std::fabs(vb.x) * vb.x, -p.dragArea.y * std::fabs(vb.y) * vb.y, -p.dragArea.z * std::fabs(vb.z) * vb.z};
        dragB = dragB * (0.5 * 1.225);
        dragB = dragB + V3{-vb.x, -vb.y, 0} * (p.rotorDrag * totalT);
        V3 force = up * totalT + Rotate(att, dragB) + V3{0, 0, -p.gravity * p.mass};
        vel = vel + force * (dt / p.mass);
        pos = pos + vel * dt;
        // Euler: I w' = tau - w x Iw
        V3 iw = Mul(p.inertia, rate);
        V3 wdot = bodyTorque - Cross(rate, iw);
        rate = rate + V3{wdot.x / p.inertia.x, wdot.y / p.inertia.y, wdot.z / p.inertia.z} * dt;
        att = Integrate(att, rate, dt);
    }

    // A hit at world point `at` (m) on a surface with normal `n`: an impulse with restitution and friction.
    // `spin` is how much of the turn a rigid body would get from it: arms and props flex and soak up part
    // of it. Returns the speed it hit with (m/s, along n).
    double Impact(V3 at, V3 n, double restitution, double friction, double spin = 1) {
        V3 r = at - pos;
        V3 wWorld = Rotate(att, rate);
        V3 vc = vel + Cross(wWorld, r);
        double vn = Dot(vc, n);
        if (vn >= 0) return 0;
        auto invI = [&](V3 v) {  // world-space I^-1 v
            V3 b = Unrotate(att, v);
            return Rotate(att, V3{b.x / p.inertia.x, b.y / p.inertia.y, b.z / p.inertia.z});
        };
        double k = 1 / p.mass + Dot(n, Cross(invI(Cross(r, n)), r));
        double j = -(1 + restitution) * vn / k;
        V3 impulse = n * j;
        // Coulomb friction, at most what stops the sliding.
        V3 vt = vc - n * vn;
        double vtl = Len(vt);
        if (vtl > 1e-6) {
            V3 tdir = vt * (1 / vtl);
            double kt = 1 / p.mass + Dot(tdir, Cross(invI(Cross(r, tdir)), r));
            double jt = std::min(friction * j, vtl / kt);
            impulse = impulse - tdir * jt;
        }
        vel = vel + impulse * (1 / p.mass);
        rate = rate + Unrotate(att, invI(Cross(r, impulse))) * spin;
        return -vn;
    }

    void Arm(bool on) {
        armed = on;
        fc.Reset();
        if (!on) for (auto& mo : m) mo.cmd = 0;
    }
};

}  // namespace quad
