// kamikaze.h - FPV Kamikaze Drone's (HFM Games) flight model, as its kamikaze drone Blueprint
// (BP_Drone_FPV, on BP_Drone_Base) and the Chaos physics it runs on do it. Read from the game's own
// assets; the numbers below are its defaults.
//
//   Turning    no inertia and no PID: while a stick is off centre the drone's angular velocity is set,
//              every frame, to stick x RotationRate (180, 180, 250 deg/s for roll, pitch, yaw) about its
//              own axes, and the moment a stick returns to centre it is set to zero. Chaos then applies the
//              body's angular damping (20) once per frame, (1 - 20 dt), before it turns the drone, so the
//              rate it really turns at depends on the frame rate: 120 deg/s of roll at 60 fps, 140 at 90.
//   Thrust     Thrust (350 kg cm/s2 = 3.5 N) x throttle x ThrustCurve(speed along the thrust axis, km/h)
//              x GroundeffectCurve(height, cm), along the drone's up axis. Throttle is the stick clamped
//              to 0..1 - linear, no motors to spin up. The drone weighs 0.1 kg (its physics asset's mass
//              override), so it hovers at about 35% and has 2.9 x its weight at the line, 3.6 x at
//              30 km/h climbing; the curve falls to nothing at 102 km/h along the thrust axis, which is
//              what limits its speed.
//   Air        linear damping 0.25 (Chaos: v *= 1 - 0.25 dt each frame) and nothing else of note: the
//              Blueprint's drag term pushes along the nose with (world X speed / 1000)^2 x 0.5 x 1.225 x
//              0.063 x 0.03, which is next to nothing; it is kept as the Blueprint has it.
//   Battery    100% drains evenly over 120 s; at 0 the drone loses control.
//   Impacts    the body bounces off things (Chaos); only a fitted warhead turns a hit into a detonation.
#pragma once
#include "quad.h"

namespace kamikaze {

using quad::V3;

// Unreal's FRichCurve with cubic keys: between two keys a Bezier whose inner control points sit a third
// of the way along their tangents; constant before the first key and after the last.
struct Key { double t, v, arrive, leave; };
template <int N>
inline double Eval(const Key (&k)[N], double t) {
    if (t <= k[0].t) return k[0].v;
    if (t >= k[N - 1].t) return k[N - 1].v;
    int i = 0;
    while (t > k[i + 1].t) i++;
    double span = k[i + 1].t - k[i].t, a = (t - k[i].t) / span, b = 1 - a;
    double p0 = k[i].v, p1 = p0 + k[i].leave * span / 3, p3 = k[i + 1].v, p2 = p3 - k[i + 1].arrive * span / 3;
    return b * b * b * p0 + 3 * b * b * a * p1 + 3 * b * a * a * p2 + a * a * a * p3;
}
// Curve_ThrustDrone: thrust against speed along the thrust axis, km/h.
inline const Key kThrustCurve[] = {
    {-0.1, 0.8, 0, 0}, {30.2, 1.0000001, -0.00083333254, -0.00083333254}, {59.9, 0.75000006, -0.01390821, -0.01390821}, {102.1, 0, 0, 0}};
// Curve_GroundEffect: thrust against the height a 10 m ray down finds, cm (no ground in reach reads 0).
inline const Key kGroundCurve[] = {
    {0.1, 1, 0.051369887, 0}, {10.1, 1.9999999, 0.00028834687, 0.00028834687}, {59.9, 1.2499995, -0.009389674, -0.009389674}, {116.6, 0.9999995, 0, 0}};

struct Params {
    V3 rotationRate{180, 180, 250};      // deg/s: roll, pitch, yaw at full stick
    double thrust = 3.5;                 // N at full throttle (Thrust 350, Unreal force units)
    double mass = 0.1;                   // kg
    double linearDamping = 0.25;         // 1/s
    double angularDamping = 20;          // 1/s
    double gravity = 9.8;                // m/s2 (Unreal's default)
    double dragK = 0.5 * 1.225 * 0.063 * 0.03;  // factor x p x C_D x A
    double frameRate = 60;               // the frame rate its turn rate is taken at (see Turning)
    double batterySeconds = 120;
};

struct Sticks { double throttle = 0, roll = 0, pitch = 0, yaw = 0; };  // throttle 0..1, others -1..1

struct Drone {
    Params p;
    bool controlled = true;   // false once the battery is flat (or the caller cuts it)
    bool held = false;        // a stick was off centre last frame
    double charge = 1;        // battery, 0..1
    double curThrottle = 0;   // what the motor sound follows (FInterpTo at 2)
    double force = 0;         // N of thrust last step

    // The turn rate (rad/s, quad.h's body axes: +x right side up, +y nose down, +z nose right) the sticks
    // set, after Chaos's once-a-frame damping.
    V3 StickRate(const Sticks& s) const {
        const double d2r = 3.14159265358979 / 180;
        double k = std::max(0.0, 1 - p.angularDamping / std::max(1.0, p.frameRate));
        return V3{-s.roll * p.rotationRate.x, s.pitch * p.rotationRate.y, s.yaw * p.rotationRate.z} * (d2r * k);
    }

    // One step. `groundCm`: what a 10 m ray straight down from its middle finds, cm, 0 for nothing.
    void Step(quad::Quad& q, Sticks s, double groundCm, double dt) {
        if (!controlled) s = {};
        charge = std::max(0.0, charge - dt / p.batterySeconds);
        if (charge <= 0) controlled = false;
        // Turning: set while a stick is off centre, zeroed when they come back, free (damped) otherwise.
        bool anyStick = controlled && (s.roll != 0 || s.pitch != 0 || s.yaw != 0);
        if (anyStick) q.rate = StickRate(s);
        else if (held) q.rate = {};
        else q.rate = q.rate * std::max(0.0, 1 - p.angularDamping * dt);
        held = anyStick;
        // Thrust along the up axis, by the curves.
        V3 up = quad::Rotate(q.att, {0, 0, 1}), fwd = quad::Rotate(q.att, {1, 0, 0});
        double throttle = quad::Clamp(s.throttle, 0, 1);
        double kmh = quad::Dot(q.vel, up) * 3.6;
        force = controlled ? p.thrust * throttle * Eval(kThrustCurve, kmh) * Eval(kGroundCurve, groundCm) : 0;
        V3 f = up * force;
        double vx = q.vel.x * 0.1;  // the Blueprint's (cm/s x 0.001)
        if (controlled) f = f + fwd * (p.dragK * vx * vx * 0.01);  // kg cm/s2 -> N
        // Chaos: forces and gravity, then the damping, then the move.
        q.vel = q.vel + (f * (1 / p.mass) + V3{0, 0, -p.gravity}) * dt;
        q.vel = q.vel * std::max(0.0, 1 - p.linearDamping * dt);
        q.pos = q.pos + q.vel * dt;
        q.att = quad::Integrate(q.att, q.rate, dt);
        // What the props and the sound follow.
        curThrottle += (throttle * (controlled ? 1 : 0) - curThrottle) * std::min(1.0, dt * 2);
        for (auto& m : q.m) { m.speed = curThrottle; m.thrust = force / 4; }
    }
};

}  // namespace kamikaze
