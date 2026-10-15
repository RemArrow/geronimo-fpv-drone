// FPV Drone - GML plugin (com.remarrow.fpvdrone).
//
// An FPV quad (the DJI Mini 4K model) you fly through DJI Goggles 2, both 3DMA models built into this DLL
// (encrypted).
//
//   Goggles   a new entry in the gear menu's Night Vision list: DJI Goggles 2 on a G24 mount. It is the
//             game's own NVG entry underneath (the GPNVG-18's: its actor, flip lever and helmet mount),
//             built at runtime and never saved as itself (see "saves"). On the helmet the GPNVG-18's
//             meshes are hidden and the goggles hang on the G24 in their place; they flip up and down
//             with the game's lever, about the G24's own hinge. Flipping them down does not turn on
//             night vision: it puts the goggles' screen in front of your eyes instead.
//   Drone     hold the right thumbstick in (half a second; a tap still kicks) while wearing the goggles
//             and the quad appears on the ground in front of you, disarmed. Hold it again (or Start on a
//             gamepad) to call it back there with a fresh pack. Fly it with a gamepad or an FPV radio in
//             USB joystick mode, mode 2, as in Bodycam's and FPV Kamikaze Drone's acro: arm with the
//             throttle down (A, or the radio's arm switch), Y for angle mode, B to flip it back over.
//   Flight    real physics (quad.h): a rigid body with four motors (spin-up lag, thrust with speed
//             squared, prop torque), a Betaflight-style rate PID with airmode at 1 kHz, a 4S LiPo that
//             sags and drains, drag, thrust lost to inflow, ground effect, prop wash. Hits are impulses
//             at the point of contact: it bounces and tumbles, and a hard one disarms it.
//   Screen    the FPV camera (fixed, up-tilted) renders into a texture (a scene capture, only while the
//             goggles are down); a Betaflight OSD in a blocky font of our own and the goggles' own readouts
//             are drawn over it. The goggles show it on a screen 30 cm in front of your eyes, everything
//             around it black, like the real goggles' displays. The radio link fades with distance and walls.
//
// Settings: GML\config\com.remarrow.fpvdrone.cfg. Build: build.bat [install].
// Assets: tools\prepare_assets.py writes Assets\ from the two 3DMA downloads (not in the repo);
// tools\pack encrypts them into the payload that src\FPVDrone.rc builds into the DLL (src\payload.h).
#define NOMINMAX
#include <GML/GML.hpp>
#include <windows.h>
#include <Xinput.h>
#include <mmsystem.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cwctype>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>
#include "payload.h"
#include "payload_key.h"  // generated per build by tools\pack (build\gen), never committed
#include "version.h"
#include "kamikaze.h"  // and quad.h
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "xinput.lib")
#pragma comment(lib, "winmm.lib")
#pragma comment(lib, "advapi32.lib")

using namespace gml;

GML_PLUGIN("com.remarrow.fpvdrone", "FPV Drone", FPVDRONE_VERSION);

// ------------------------------------------------------------------ math (UE conventions: cm, X fwd, Y right, Z up)

struct V3 { double x = 0, y = 0, z = 0; };
static V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static V3 operator*(V3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
static double Dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static V3 Cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
static double Len(V3 a) { return std::sqrt(Dot(a, a)); }
static V3 Norm(V3 a) { double l = Len(a); return l > 1e-9 ? a * (1 / l) : V3{0, 0, 1}; }
struct V2 { double x = 0, y = 0; };
struct Rot { double pitch = 0, yaw = 0, roll = 0; };
struct Quat { double x = 0, y = 0, z = 0, w = 1; };
struct Color { float r = 0, g = 0, b = 0, a = 1; };
static constexpr double kPi = 3.14159265358979323846, kDeg = kPi / 180;

static double Clamp(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }
static double Lerp(double a, double b, double t) { return a + (b - a) * t; }
static double Smooth(double cur, double target, double tau, double dt) { return target + (cur - target) * std::exp(-dt / std::max(tau, 1e-4)); }
static double WrapDeg(double a) { a = std::fmod(a + 180, 360); return a < 0 ? a + 180 : a - 180; }

static Quat ToQuat(Rot r) {  // FRotator::Quaternion
    double sp = std::sin(r.pitch * kDeg / 2), cp = std::cos(r.pitch * kDeg / 2);
    double sy = std::sin(r.yaw * kDeg / 2), cy = std::cos(r.yaw * kDeg / 2);
    double sr = std::sin(r.roll * kDeg / 2), cr = std::cos(r.roll * kDeg / 2);
    return {cr * sp * sy - sr * cp * cy, -cr * sp * cy - sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy};
}
static Rot ToRot(Quat q) {  // FQuat::Rotator
    double sing = q.z * q.x - q.w * q.y;
    double yawY = 2 * (q.w * q.z + q.x * q.y), yawX = 1 - 2 * (q.y * q.y + q.z * q.z);
    Rot r;
    r.yaw = std::atan2(yawY, yawX) / kDeg;
    if (sing < -0.4999995) { r.pitch = -90; r.roll = WrapDeg(-r.yaw - 2 * std::atan2(q.x, q.w) / kDeg); }
    else if (sing > 0.4999995) { r.pitch = 90; r.roll = WrapDeg(r.yaw - 2 * std::atan2(q.x, q.w) / kDeg); }
    else {
        r.pitch = std::asin(2 * sing) / kDeg;
        r.roll = std::atan2(-2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) / kDeg;
    }
    return r;
}
static Quat Mul(Quat a, Quat b) {  // a * b: b first, then a (UE order)
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
static Quat Normalized(Quat q) {
    double l = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return l > 0 ? Quat{q.x / l, q.y / l, q.z / l, q.w / l} : Quat{};
}
static Quat AxisAngle(V3 axis, double rad) {
    double s = std::sin(rad / 2);
    return {axis.x * s, axis.y * s, axis.z * s, std::cos(rad / 2)};
}
static V3 Rotate(Quat q, V3 v) {  // FQuat::RotateVector
    V3 u{q.x, q.y, q.z};
    V3 t = Cross(u, v) * 2;
    return v + t * q.w + Cross(u, t);
}
static V3 Unrotate(Quat q, V3 v) { return Rotate({-q.x, -q.y, -q.z, q.w}, v); }

// ------------------------------------------------------------------ engine helpers

static int XfSize() {
    static int size = 0;
    if (!size) {
        GML_PropInfo ri;
        API->GetPropertyInfo(API->FindProperty((GUStruct*)API->FindFunction(Lib("KismetMathLibrary").Struct(), "MakeTransform"), "ReturnValue"), &ri);
        size = ri.size;
    }
    return size;
}
static std::vector<uint8_t> Transform(V3 loc, Rot rot, V3 scale = {1, 1, 1}) {
    Params p(Lib("KismetMathLibrary"), "MakeTransform");
    p.Set("Location", loc).Set("Rotation", rot).Set("Scale", scale).Call();
    auto* s = (uint8_t*)p.Ptr("ReturnValue");
    return {s, s + XfSize()};
}

static Object SpawnActor(const char* cls, V3 loc, Rot rot) {
    auto xf = Transform(loc, rot);
    Params b(Lib("GameplayStatics"), "BeginDeferredActorSpawnFromClass");
    b.SetObj("WorldContextObject", Object(API->WorldContext())).Set("ActorClass", FindClass(cls));
    std::memcpy(b.Ptr("SpawnTransform"), xf.data(), xf.size());
    b.Set("CollisionHandlingOverride", (uint8_t)1).Call();
    Object a = b.Return<GUObject*>();
    if (!a) return {};
    Params f(Lib("GameplayStatics"), "FinishSpawningActor");
    f.SetObj("Actor", a);
    std::memcpy(f.Ptr("SpawnTransform"), xf.data(), xf.size());
    f.Call();
    return a;
}

static void Attach(Object c, Object parent, uint8_t rule) {  // rule: 0 KeepRelative, 1 KeepWorld, 2 SnapToTarget
    Params at(c, "K2_AttachToComponent");
    at.SetObj("Parent", parent).Set("SocketName", uint64_t(0)).Set("LocationRule", rule).Set("RotationRule", rule)
        .Set("ScaleRule", rule).Set("bWeldSimulatedBodies", false).Call();
}
static void SetRel(Object c, V3 loc, Rot rot) {
    Params p(c, "K2_SetRelativeLocationAndRotation");
    p.Set("NewLocation", loc).Set("NewRotation", rot).Set("bSweep", false).Set("bTeleport", true).Call();
}
static void SetRelRot(Object c, Rot rot) {
    Params p(c, "K2_SetRelativeRotation");
    p.Set("NewRotation", rot).Set("bSweep", false).Set("bTeleport", true).Call();
}
static void SetWorldRot(Object c, Rot rot) {
    Params p(c, "K2_SetWorldRotation");
    p.Set("NewRotation", rot).Set("bSweep", false).Set("bTeleport", true).Call();
}
static void SetVisible(Object c, bool on) {
    if (!c || c.GetBool("bVisible") == on) return;
    Params v(c, "SetVisibility");
    v.Set("bNewVisibility", on).Set("bPropagateToChildren", false).Call();
}
static V3 CompLoc(Object c) { Params p(c, "K2_GetComponentLocation"); p.Call(); return p.Return<V3>(); }
static Rot CompRot(Object c) { Params p(c, "K2_GetComponentRotation"); p.Call(); return p.Return<Rot>(); }
static Object GetComp(Object actor, const char* cls) {
    GUClass* c = FindClass(cls);
    if (!actor || !c) return {};
    Params p(actor, "GetComponentByClass");
    p.Set("ComponentClass", c).Call();
    return p.Return<GUObject*>();
}

// A component of `cls` on `actor`, attached to `parent` (or made the root) at a relative transform.
static Object AddComp(Object actor, const char* cls, Object parent, V3 loc = {}, Rot rot = {}) {
    auto xf = Transform({}, {});
    Params add(actor, "AddComponentByClass");
    add.Set("Class", FindClass(cls)).Set("bManualAttachment", (bool)parent);
    std::memcpy(add.Ptr("RelativeTransform"), xf.data(), xf.size());
    add.Set("bDeferredFinish", false).Call();
    Object c = add.Return<GUObject*>();
    if (!c) return {};
    Params mob(c, "SetMobility");
    mob.Set("NewMobility", (uint8_t)2).Call();  // Movable
    if (parent) Attach(c, parent, 0);
    SetRel(c, loc, rot);
    return c;
}

// A runtime mesh on `actor`, hung on `parent` at a relative transform; drawn only for its owner if asked.
static GUObject* s_mats[4];  // material ID i of every mesh here: MI_G24, MI_Goggles, MI_Drone, MI_Bomb
static Object AddMesh(Object actor, GUObject* mesh, Object parent, V3 loc = {}, Rot rot = {}, GUObject* const* mats = s_mats,
                      int count = 4) {
    if (!mesh) return {};
    Object c = API->AddDynamicMeshComponent(actor.ptr, mesh, mats, count);
    if (!c) return {};
    if (parent) Attach(c, parent, 2);
    SetRel(c, loc, rot);
    return c;
}

static Object PlayerPawn() {
    Params p(Lib("GameplayStatics"), "GetPlayerPawn");
    p.SetObj("WorldContextObject", Object(API->WorldContext())).Set("PlayerIndex", (int32_t)0).Call();
    return p.Return<GUObject*>();
}
static Object PlayerController() {
    Params p(Lib("GameplayStatics"), "GetPlayerController");
    p.SetObj("WorldContextObject", Object(API->WorldContext())).Set("PlayerIndex", (int32_t)0).Call();
    return p.Return<GUObject*>();
}
static Object VRCamera(Object pawn) { return pawn ? pawn.GetObj("VRReplicatedCamera") : Object(); }
static Object GearManager(Object pawn) { return GetComp(pawn, "BP_GearManagerComponent_C"); }

// Multiplayer: this machine is a client in someone else's game. The host owns every actor there, the
// drone's grenade too: what a client's drone does goes to the host (see "multiplayer" below).
static bool NetClient() {
    Params s(Lib("KismetSystemLibrary"), "IsServer");
    s.SetObj("WorldContextObject", Object(API->WorldContext())).Call();
    return !s.Return<bool>();
}
static uint64_t NameOf(const char* s) { return API->MakeName(s); }

// ---- multiplayer
//
// The host owns every actor, the drone's grenade too: a client can't move its drone there or set its payload
// off. So what a client's drone does goes to the host, whose copy of this plugin does it, and the game sends
// everyone the result (the grenade's movement and its explosion replicate). The client sends it with its
// PlayerController's ServerSetSpectatorLocation (unreliable; a full-precision location and a rotation), the
// location raised by kMsg times the kind: nothing the game sends is ever that high, and the host's plugin takes
// those and keeps them from the game (OnClientMessage).
//   fly    where the drone's grenade is and how it is turned, 30 times a second while it flies
//   boom   the payload going off: where, and the number of grenades as the rotation's roll
//   fresh  a fresh drone into a drone pouch of theirs
enum Msg { MsgFly = 1, MsgBoom = 2, MsgFresh = 3 };
static constexpr double kMsg = 1e8;  // cm: 1000 km
static void SendToHost(Msg kind, V3 at, Rot rot) {
    Object pc = PlayerController();
    if (!pc) return;
    Params p(pc, "ServerSetSpectatorLocation");
    p.Set("NewLoc", V3{at.x, at.y, at.z + kMsg * (int)kind}).Set("NewRot", rot).Call();
}

// A box sweep (half size `box`, turned by `orient`) or, with no box, a line trace, on the Visibility
// channel, ignoring `ignore`.
struct Hit { bool hit = false, startPenetrating = false; V3 location, impact, normal; double time = 1, depth = 0; };
static Hit Sweep(V3 from, V3 to, V3 box, Rot orient, const std::vector<GUObject*>& ignore) {
    bool line = box.x <= 0;
    Params t(Lib("KismetSystemLibrary"), line ? "LineTraceSingle" : "BoxTraceSingle");
    t.SetObj("WorldContextObject", Object(API->WorldContext())).Set("Start", from).Set("End", to)
        .Set("TraceChannel", (uint8_t)0).Set("bTraceComplex", false).Set("DrawDebugType", (uint8_t)0).Set("bIgnoreSelf", true);
    if (!line) t.Set("HalfSize", box).Set("Orientation", orient);
    *(GML_TArray*)t.Ptr("ActorsToIgnore") = GML_TArray{(void*)ignore.data(), (int32_t)ignore.size(), (int32_t)ignore.size()};
    t.Call();
    Hit h;
    if (!t.Return<bool>()) return h;
    static GML_PropInfo loc, imp, nrm, tim, pen, dep;
    static bool init = false;
    if (!init) {
        GML_PropInfo hit;
        API->GetPropertyInfo(API->FindProperty((GUStruct*)API->FindFunction(Lib("KismetSystemLibrary").Struct(), "LineTraceSingle"), "OutHit"), &hit);
        API->GetPropertyInfo(API->FindProperty(hit.structType, "Location"), &loc);
        API->GetPropertyInfo(API->FindProperty(hit.structType, "ImpactPoint"), &imp);
        API->GetPropertyInfo(API->FindProperty(hit.structType, "ImpactNormal"), &nrm);
        API->GetPropertyInfo(API->FindProperty(hit.structType, "Time"), &tim);
        API->GetPropertyInfo(API->FindProperty(hit.structType, "bStartPenetrating"), &pen);
        API->GetPropertyInfo(API->FindProperty(hit.structType, "PenetrationDepth"), &dep);
        init = true;
    }
    auto* o = (uint8_t*)t.Ptr("OutHit");
    h.hit = true;
    std::memcpy(&h.location, o + loc.offset, sizeof(V3));
    std::memcpy(&h.impact, o + imp.offset, sizeof(V3));
    std::memcpy(&h.normal, o + nrm.offset, sizeof(V3));
    h.time = *(float*)(o + tim.offset);
    h.startPenetrating = (o[pen.offset + pen.boolByteOffset] & pen.boolFieldMask) != 0;
    h.depth = *(float*)(o + dep.offset);
    return h;
}

// ------------------------------------------------------------------ settings

static ConfigEntry<bool> s_cfgGoggles;
static ConfigEntry<float> s_cfgScreenDist, s_cfgScreenFov, s_cfgHold, s_cfgCamFov, s_cfgUptilt, s_cfgZoom, s_cfgVolume,
    s_cfgCrashSpeed, s_cfgOsdBright, s_cfgSensitivity, s_cfgDeadzone, s_cfgSmoothing;
static ConfigEntry<float> s_cfgProp, s_cfgPitch, s_cfgKV, s_cfgGrams, s_cfgCapacity, s_cfgMinThrottle;  // [Build]
static ConfigEntry<float> s_cfgGravity, s_cfgInstantPower, s_cfgAirFriction, s_cfgAirGrip, s_cfgRelAirSpeed;  // [Physics]
static ConfigEntry<int> s_cfgResX, s_cfgResY;
static ConfigEntry<std::string> s_cfgMode, s_cfgArming, s_cfgDevice, s_cfgPadThrottle, s_cfgRadioAxes, s_cfgRadioInvert, s_cfgRadioArm,
    s_cfgRatesRP, s_cfgRatesYaw;
static ConfigEntry<bool> s_cfgVRSticks, s_cfgOsd, s_cfgCaptureHDR;
static ConfigEntry<int> s_cfgGamepadMode, s_cfgCells;
static ConfigEntry<std::string> s_cfgModel;
static ConfigEntry<bool> s_cfgPayload;
static ConfigEntry<int> s_cfgGrenades;
static ConfigEntry<float> s_cfgDetonateSpeed, s_cfgArmDistance;
static bool Kamikaze() { return s_cfgModel.Value() != "Sim"; }  // FPV Kamikaze Drone's flight model (kamikaze.h)
static bool AutoArm() { return Kamikaze() || s_cfgArming.Value() != "Switch"; }

// ------------------------------------------------------------------ assets

static GUObject* s_meshGogMount = nullptr;
static GUObject* s_meshGogFold = nullptr;
static GUObject* s_meshBody = nullptr;
static GUObject* s_meshCamera = nullptr;
static GUObject* s_meshProps[4] = {};
static GUObject* s_meshRelease = nullptr;  // the payload release under the body, and the bomb it holds
static GUObject* s_meshBomb = nullptr;
static GUObject* s_meshScreen = nullptr;  // generated: the goggles' screen and the black around it
static GUObject* s_meshShroud = nullptr;
static GUObject* s_icon = nullptr;
static GUObject* s_droneIcon = nullptr;  // the drone pouch's tile in the kit station
static GUObject* s_unlit = nullptr;       // the engine's glTF unlit material
static GUObject* s_midScreen = nullptr;
static GUObject* s_midBlack = nullptr;
static bool s_assetsTried = false, s_assetsOk = false;
static const char* kPropNames[4] = {"FL", "FR", "BL", "BR"};
static const double kPropSpin[4] = {-1, 1, 1, -1};  // seen from above: front left clockwise, as on a DJI

// fpv.txt, written by prepare_assets.py: pivots and sizes in cm.
static V3 s_hinge{11.5, 0, 8.6};  // the G24's hinge, from the eye, camera axes
static V3 s_camPos{6.9, 0, -1.2}, s_propPos[4], s_bombPos{-1.7, 0, -6.6};
static double s_payloadBottom = -9.2;  // the payload's underside below the drone's centre, cm
static double s_bottom = -3.5;    // the drone's underside below its centre

static bool Unpack(const char* name, std::vector<uint8_t>& out) {
    static Blob blob = Resource("DATA");
    uint8_t key[32];
    for (int i = 0; i < 32; i++) key[i] = kPayloadKeyA[i] ^ kPayloadKeyB[i];
    bool ok = blob && payload::Extract(blob.data, blob.size, key, name, out);
    SecureZeroMemory(key, sizeof key);
    if (!ok) Error("embedded asset '{}' is missing or damaged", name);
    return ok;
}

static void ReadManifest() {
    std::vector<uint8_t> buf;
    if (!Unpack("fpv", buf)) return;
    std::istringstream in(std::string(buf.begin(), buf.end()));
    payload::Wipe(buf);
    std::string key;
    while (in >> key) {
        V3 v;
        if (key == "drone.bottom" || key == "drone.payload_bottom") { in >> v.x; (key == "drone.bottom" ? s_bottom : s_payloadBottom) = v.x; continue; }
        in >> v.x >> v.y >> v.z;
        if (key == "goggles.hinge") s_hinge = v;
        else if (key == "drone_camera") s_camPos = v;
        else if (key == "drone_bomb") s_bombPos = v;
        for (int i = 0; i < 4; i++)
            if (key == std::string("drone_prop_") + kPropNames[i]) s_propPos[i] = v;
    }
}

static GUObject* ImportMesh(const char* name, GML_Axis axis = GML_AXIS_BLENDER_OBJ, const std::string* text = nullptr) {
    static const char* groups[] = {"MI_G24", "MI_Goggles", "MI_Drone", "MI_Bomb"};
    GML_MeshImport o{};
    o.size = sizeof o;
    o.axis = axis;
    o.flipWinding = -1;
    o.materialCount = 4;
    o.materialNames = groups;
    o.materials = s_mats;
    GUObject* m = nullptr;
    if (text) return API->ImportDynamicMeshFromMemory(text->data(), text->size(), name, &o);
    std::vector<uint8_t> buf;
    if (Unpack(name, buf)) m = API->ImportDynamicMeshFromMemory(buf.data(), buf.size(), name, &o);
    payload::Wipe(buf);
    if (!m) Error("could not build mesh {}", name);
    return m;
}

static GUObject* Texture(const char* name) {
    std::vector<uint8_t> buf;
    GUObject* t = Unpack(name, buf) ? API->ImportTextureFromMemory(buf.data(), buf.size(), name) : nullptr;
    payload::Wipe(buf);
    return t;
}

// The screen: a 16:9 quad `dist` cm ahead of the eye spanning `fov` degrees, and a black box around the
// eye that hides everything else. Unreal axes, cm, both windings (seen from either side).
// (The back faces get vertices of their own: sharing them would make the mesh non-manifold, which
// Geometry Scripting refuses.)
static std::string ScreenObj(double dist, double fov) {
    double w = 2 * dist * std::tan(fov * kDeg / 2), h = w * 9 / 16;
    std::ostringstream s;
    s << "o screen\nusemtl MI_G24\n";
    for (int side = 0; side < 2; side++) {
        s << "v " << dist << " " << -w / 2 << " " << h / 2 << "\nv " << dist << " " << w / 2 << " " << h / 2 << "\n";
        s << "v " << dist << " " << w / 2 << " " << -h / 2 << "\nv " << dist << " " << -w / 2 << " " << -h / 2 << "\n";
    }
    s << "vt 0 1\nvt 1 1\nvt 1 0\nvt 0 0\n";
    s << "f 1/1 2/2 3/3\nf 1/1 3/3 4/4\nf 7/3 6/2 5/1\nf 8/4 7/3 5/1\n";
    return s.str();
}
static std::string ShroudObj(double dist) {
    double r = dist * 1.3, front = dist + 1;
    std::ostringstream s;
    s << "o shroud\nusemtl MI_G24\n";
    for (int side = 0; side < 2; side++)
        for (int i = 0; i < 8; i++) s << "v " << ((i & 1) ? front : -r) << " " << ((i & 2) ? r : -r) << " " << ((i & 4) ? r : -r) << "\n";
    s << "vt 0 0\n";
    static const int faces[6][4] = {{1, 2, 4, 3}, {5, 7, 8, 6}, {1, 5, 6, 2}, {3, 4, 8, 7}, {1, 3, 7, 5}, {2, 6, 8, 4}};
    for (auto& f : faces) {
        s << "f " << f[0] << "/1 " << f[1] << "/1 " << f[2] << "/1\nf " << f[0] << "/1 " << f[2] << "/1 " << f[3] << "/1\n";
        s << "f " << f[2] + 8 << "/1 " << f[1] + 8 << "/1 " << f[0] + 8 << "/1\nf " << f[3] + 8 << "/1 " << f[2] + 8 << "/1 " << f[0] + 8 << "/1\n";
    }
    return s.str();
}

static bool BuildAssets() {
    if (s_assetsTried) return s_assetsOk;
    s_assetsTried = true;
    if (API->size < offsetof(GML_API, ImportDynamicMeshFromMemory) + sizeof(void*)) {
        Error("this plugin needs GML 2.2 or newer (assets embedded in the DLL)");
        return false;
    }
    Object shader = LoadObject("/Game/Art/Common/M_DefaultShader.M_DefaultShader");
    if (!shader) { Error("the game's M_DefaultShader is missing"); return false; }
    const char* sets[4] = {"G24", "Goggles", "Drone", "Bomb"};
    for (int i = 0; i < 4; i++) {
        s_mats[i] = API->CreateMaterialInstance(shader.ptr);
        if (!s_mats[i]) { Error("could not create material {}", sets[i]); return false; }
        API->KeepAlive(s_mats[i]);
        for (const char* param : {"Diffuse", "Normal", "ORM"}) {
            std::string res = std::string(sets[i]) + "_" + param;
            GUObject* tex = Texture(res.c_str());
            if (!tex || !API->SetMaterialTexture(s_mats[i], param, tex)) Warn("texture {} missing", res);
        }
    }
    ReadManifest();
    s_meshGogMount = ImportMesh("goggles_mount");
    s_meshGogFold = ImportMesh("goggles_fold");
    s_meshBody = ImportMesh("drone_body");
    s_meshCamera = ImportMesh("drone_camera");
    for (int i = 0; i < 4; i++) s_meshProps[i] = ImportMesh((std::string("drone_prop_") + kPropNames[i]).c_str());
    s_meshRelease = ImportMesh("drone_release");
    s_meshBomb = ImportMesh("drone_bomb");
    s_icon = Texture("Icon_Goggles");
    s_droneIcon = Texture("Icon_Drone");
    if (s_droneIcon) API->KeepAlive(s_droneIcon);
    if (s_icon) API->KeepAlive(s_icon);

    s_unlit = LoadObject("/InterchangeAssets/gltf/M_Unlit.M_Unlit").ptr;
    if (!s_unlit) Error("the engine's unlit material is missing - the goggles can't show the drone's camera");
    else {
        s_midScreen = API->CreateMaterialInstance(s_unlit);
        s_midBlack = API->CreateMaterialInstance(s_unlit);
        API->KeepAlive(s_midScreen);
        API->KeepAlive(s_midBlack);
        float black[4] = {0, 0, 0, 1};
        API->SetMaterialVector(s_midBlack, "BaseColorFactor", black);
    }
    std::string screen = ScreenObj(s_cfgScreenDist.Value(), s_cfgScreenFov.Value());
    std::string shroud = ShroudObj(s_cfgScreenDist.Value());
    s_meshScreen = ImportMesh("fpv_screen", GML_AXIS_UNREAL, &screen);
    s_meshShroud = ImportMesh("fpv_shroud", GML_AXIS_UNREAL, &shroud);
    s_assetsOk = s_meshGogMount && s_meshGogFold && s_meshBody && s_meshCamera && s_meshProps[0] && s_meshProps[1] &&
                 s_meshProps[2] && s_meshProps[3];
    Log("assets {}: hinge ({:.2f}, {:.2f}, {:.2f}) camera ({:.2f}, {:.2f}, {:.2f}) underside {:.2f}", s_assetsOk ? "ready" : "INCOMPLETE",
        s_hinge.x, s_hinge.y, s_hinge.z, s_camPos.x, s_camPos.y, s_camPos.z, s_bottom);
    return s_assetsOk;
}

// ------------------------------------------------------------------ the gear menu entry

static constexpr uint8_t kCatHelmet = 5, kCatNightVision = 19;  // EGearItem
static const char* kDonor = "/Game/Characters/Data/DA_NightVision_GPNVG18_Green.DA_NightVision_GPNVG18_Green";
static const char* kDisplayName = "DJI Goggles 2 (FPV)";
static const char* kSaveTag = "FPVGoggles";  // SelectedColor of the donor entry that stands for ours in a save
static GUObject* s_item = nullptr;           // our OutfitItem_NightVision_C

static Object Donor() {
    static GUObject* d = nullptr;
    if (!d || !API->IsValid(d)) d = LoadObject(kDonor).ptr;
    return d;
}
static bool IsOurs(GUObject* item) { return item && item == s_item; }
// The goggles as worn: our entry, or the GPNVG-18 entry with SelectedColor "FPVGoggles". The second is what
// the outfit really holds (OnSetItem): a save and the network carry only asset paths and names, and ours is
// built at runtime, so every machine and every save sees that pair and this plugin makes it the goggles.
static bool IsOursPair(GUObject* data, uint64_t color) {
    static uint64_t tag = NameOf(kSaveTag);
    return data && (data == s_item || (data == Donor().ptr && color == tag));
}

// FGearItemStruct: {UOutfitItem_C* Data; FName SelectedColor}, Blueprint-mangled names.
static int GearDataOff() {
    static int off = -1;
    if (off < 0) {
        GUStruct* s = (GUStruct*)API->FindObject("/Game/Gameplay/Customization/GearItemStruct.GearItemStruct");
        GML_PropInfo i;
        off = s && API->GetPropertyInfo(API->FindProperty(s, "Data"), &i) ? i.offset : 0;
    }
    return off;
}
static int GearColorOff() {
    static int off = -1;
    if (off < 0) {
        GUStruct* s = (GUStruct*)API->FindObject("/Game/Gameplay/Customization/GearItemStruct.GearItemStruct");
        GML_PropInfo i;
        off = s && API->GetPropertyInfo(API->FindProperty(s, "SelectedColor"), &i) ? i.offset : 8;
    }
    return off;
}

// The value for `key` in a TMap<EGearItem, V> property of `obj`, or nullptr.
static uint8_t* MapValue(Object obj, const char* prop, uint8_t key) {
    GFProperty* mp = obj ? API->FindProperty(obj.Struct(), prop) : nullptr;
    void* map = obj ? obj.PropPtr(prop) : nullptr;
    if (!mp || !map) return nullptr;
    struct Ctx { uint8_t key; uint8_t* found; } ctx{key, nullptr};
    API->MapForEach(map, mp, [](void* u, void* k, void* v) -> int {
        auto* c = (Ctx*)u;
        if (*(uint8_t*)k != c->key) return 0;
        c->found = (uint8_t*)v;
        return 1;
    }, &ctx);
    return ctx.found;
}

static GML_TArray* NightVisionList(Object gi) {
    uint8_t* v = MapValue(gi, "OutfitItems", kCatNightVision);
    if (!v) return nullptr;
    GUStruct* s = (GUStruct*)API->FindObject("/Game/Gameplay/Customization/OutfitItemList.OutfitItemList");
    GML_PropInfo i;
    if (!s || !API->GetPropertyInfo(API->FindProperty(s, "Items"), &i)) return nullptr;
    return (GML_TArray*)(v + i.offset);
}

static void AddGoggles(Object gi) {
    if (!s_cfgGoggles.Value()) return;
    GML_TArray* raw = NightVisionList(gi);
    if (!raw) { Warn("the gear menu's Night Vision list isn't there yet"); return; }
    TArrayView<GUObject*> list{raw};
    if (s_item && API->IsValid(s_item) && list.Contains(s_item)) return;
    Object donor = Donor();
    GUClass* cls = FindClass("OutfitItem_NightVision_C");
    if (!donor || !cls) { Error("{} or OutfitItem_NightVision_C missing", kDonor); return; }
    Object item = NewObject(cls, gi);
    if (!item) { Error("could not construct the goggles' gear entry"); return; }
    for (const char* p : {"Mesh", "EnabledMesh", "NVGClass"}) item.Set(p, donor.Get<GUObject*>(p));
    item.Set("PPMaterial", (GUObject*)nullptr);  // no night vision: flipping them down shows the drone's camera
    item.Set("Category", kCatNightVision);
    item.Set("CustomIconSize", donor.Get<double>("CustomIconSize"));
    if (BuildAssets() && s_icon) item.Set("CustomIcon", s_icon);
    uint8_t text[16];
    if (API->MakeText(kDisplayName, text)) std::memcpy(item.PropPtr("DisplayName"), text, 16);
    if (API->MakeText("FPV drone goggles", text)) std::memcpy(item.PropPtr("Subtitle"), text, 16);
    int before = list.Num();
    if (!list.Add(item.ptr)) { Error("could not add the goggles to the gear menu"); return; }
    s_item = item.ptr;
    API->KeepAlive(s_item);
    Log("added '{}' to the Night Vision gear ({} -> {})", kDisplayName, before, list.Num());
}

// The menu shows an item's icon by its mesh's name (Thumb_<mesh>), which for ours would be the
// GPNVG-18's: the goggles' own icon goes in instead.
static void IconFor(Object widget) {
    if (!IsOurs(widget.Get<GUObject*>("OutfitItem")) || !s_icon) return;
    Object image = widget.GetObj("Image_30");
    if (!image) return;
    Params p(image, "SetBrushFromTexture");
    p.SetObj("Texture", Object(s_icon)).Set("bMatchSize", false).Call();
}

// ------------------------------------------------------------------ saves
//
// An outfit is saved as its items' asset paths, and ours has none (it is built at runtime). So while the
// game saves an outfit, the goggles' slot holds the GPNVG-18 entry with SelectedColor "FPVGoggles"; when
// an outfit is loaded, that pair becomes the goggles again. Without this plugin the save simply loads a
// GPNVG-18.

static uint8_t* s_savedSlot = nullptr;
static void BeforeSave(Object gm) {
    s_savedSlot = nullptr;
    uint8_t* v = MapValue(gm, "GearByCategory", kCatNightVision);
    if (!v || !IsOurs(*(GUObject**)(v + GearDataOff()))) return;
    *(GUObject**)(v + GearDataOff()) = Donor().ptr;
    *(uint64_t*)(v + GearColorOff()) = API->MakeName(kSaveTag);
    s_savedSlot = v;
}
static void AfterSave(Object gm) {
    uint8_t* v = s_savedSlot ? MapValue(gm, "GearByCategory", kCatNightVision) : nullptr;
    if (v && v == s_savedSlot) {
        *(GUObject**)(v + GearDataOff()) = s_item;
        *(uint64_t*)(v + GearColorOff()) = 0;
        Log("outfit saved with the FPV goggles");
    }
    s_savedSlot = nullptr;
}
// SetOutfitItem_Internal's GearItem, before it runs: our entry (picked in the gear menu) goes in as the
// GPNVG-18 entry marked "FPVGoggles", the pair every machine understands (IsOursPair). In multiplayer the
// outfit goes to the host and back to every player as asset paths and names; ours, built at runtime, would
// arrive as nothing (no goggles, and no NVG actor for them).
static void OnSetItem(uint8_t* gearItem) {
    if (!gearItem || !s_item) return;
    GUObject*& data = *(GUObject**)(gearItem + GearDataOff());
    uint64_t& color = *(uint64_t*)(gearItem + GearColorOff());
    char cname[64] = "";
    API->NameToString(color, cname, sizeof cname);
    Log("gear item set: {} colour '{}'", data ? Object(data).Name() : "none", cname);
    if (data && data == s_item) {
        data = Donor().ptr;
        color = NameOf(kSaveTag);
        Log("the FPV goggles go on (as the GPNVG-18 entry marked '{}')", kSaveTag);
    } else if (IsOursPair(data, color)) Log("outfit loaded with the FPV goggles");
}

static void BuildDrone(Object a, bool item);
static void DestroyDrone();
struct Drone;
static void BuildLook(Drone& d, Object a, bool item);
static Object AddSound(Object actor, Object parent);
template <class T> static void Call1(Object o, const char* fn, const char* p, T v);
template <class A, class B> static void Call2(Object o, const char* fn, const char* p, A a, const char* q, B b);

// Held in a hand (2), or stored in a pouch (1, attached to it): the game moves it then. Else 0.
static int Carried(Object a) {
    // VRExpansion keeps it in the actor's VRGripInterfaceSettings (FBPInterfaceProperties::bIsHeld).
    bool held = false;
    if (uint8_t* s = (uint8_t*)a.PropPtr("VRGripInterfaceSettings")) {
        static int off = -1;
        if (off < 0) {
            GML_PropInfo pi, bi;
            if (API->GetPropertyInfo(API->FindProperty(a.Struct(), "VRGripInterfaceSettings"), &pi) && pi.structType &&
                API->GetPropertyInfo(API->FindProperty(pi.structType, "bIsHeld"), &bi))
                off = (int)bi.offset;
        }
        if (off >= 0) held = s[off] != 0;
    }
    if (held) return 2;
    Params at(a, "GetAttachParentActor");
    at.Call();
    return at.Return<GUObject*>() ? 1 : 0;
}

// ------------------------------------------------------------------ the goggles on the helmet

struct Goggles {
    GUObject* actor = nullptr;  // the GPNVG-18 actor the game hung on the VR camera
    Object root, lever, mount, pivot, fold, screen, shroud;
    V3 eye;                     // the camera, in the actor's root frame
    bool down = false;
};
static Goggles s_gog;

static bool GogglesValid() { return s_gog.actor && API->IsValid(s_gog.actor); }

// The GPNVG-18's own meshes: hidden, its lever (the grip that flips them) kept working.
static void HideOriginal(Object nvg) {
    for (const char* c : {"Mount", "NVG", "Lenses", "LocalMesh"}) SetVisible(nvg.GetObj(c), false);
}

// How far up the lever is turned: 0 flipped down (in front of the eyes), 1 up. The game's lever rests at
// 0 degrees flipped up and toggles (powers night vision) near its limit, flipped down.
static double LeverUp(Object lever) {
    double a = std::fabs(lever.Get<float>("CurrentLeverAngle"));
    double lim = std::max(std::fabs(lever.Get<float>("LeverLimitNegative")), std::fabs(lever.Get<float>("LeverLimitPositive")));
    return lim > 1 ? Clamp(1 - a / lim, 0, 1) : 0;
}

static std::set<GUObject*> s_remoteGog;  // other players' NVG actors with the goggles fitted (multiplayer)

// The goggles on an NVG actor: yours (`local`: with their screen, seen only by you), or another player's (their
// look only: the G24 and the goggles, on their helmet, folding with their lever).
static void Dress(Object nvg, Object pawn = PlayerPawn(), bool local = true) {
    if (!BuildAssets()) return;
    Object cam = VRCamera(pawn);
    Object root = nvg.GetObj("RootComponent");
    Object lever = nvg.GetObj("NVG");
    if (!cam || !root || !lever) { Warn("the NVG actor isn't complete - goggles not fitted"); return; }
    Goggles g;
    g.actor = nvg.ptr;
    g.root = root;
    g.lever = lever;
    g.eye = Unrotate(ToQuat(CompRot(root)), CompLoc(cam) - CompLoc(root));
    HideOriginal(nvg);
    // The G24's helmet half on the actor (it follows the head), its folding half on a pivot at the hinge.
    g.mount = AddMesh(nvg, s_meshGogMount, root, g.eye + s_hinge);
    g.pivot = AddComp(nvg, "SceneComponent", root, g.eye + s_hinge);
    g.fold = AddMesh(nvg, s_meshGogFold, g.pivot);
    // The folding half rides the game's lever (the GPNVG-18's, hidden), the part a hand grabs: what you see
    // flips exactly as what you hold, and pushing it down brings the screen down. It is fitted at the
    // lever's down angle - set there for a moment, without its events - where the goggles sit before the eyes.
    if (g.pivot) {
        // First the lever's own pivot goes onto the G24's hinge (it sits 4.7 cm behind and 1 cm below it), so
        // the goggles fold about their own hinge and stay on the mount: its current place and the
        // InitialRelativeTransform its angle is applied to (FTransform: rotation, then translation at +32).
        Params gp(lever, "GetAttachParent");
        gp.Call();
        Object parent = gp.Return<GUObject*>();
        if (parent) {
            V3 dw = CompLoc(g.pivot) - CompLoc(lever);
            V3 dp = Unrotate(ToQuat(CompRot(parent)), dw);
            if (auto* init = (uint8_t*)lever.PropPtr("InitialRelativeTransform")) {
                double* t = (double*)(init + 32);
                t[0] += dp.x, t[1] += dp.y, t[2] += dp.z;
            }
            Params sl(lever, "K2_SetRelativeLocation");
            sl.Set("NewLocation", lever.Get<V3>("RelativeLocation") + dp).Set("bSweep", false).Set("bTeleport", true).Call();
            Log("goggles' lever pivot moved onto the G24 hinge by ({:.2f}, {:.2f}, {:.2f}) cm", dp.x, dp.y, dp.z);
        }
        float a0 = lever.Get<float>("CurrentLeverAngle");
        float lim = lever.Get<float>("LeverLimitNegative");
        float down = (lim > 0 ? -1.f : 1.f) * std::fabs(lim);  // the lever turns negative towards its negative limit
        auto setLever = [&](float angle) {
            Params p(lever, "SetLeverAngle");
            p.Set("NewAngle", angle).Set("DualAxisForwardVector", V3{1, 0, 0}).Set("bAllowThrowingEvents", false).Call();
        };
        setLever(down);
        Attach(g.pivot, lever, 1);  // keep its place: fitted, down
        setLever(a0);
    }
    for (Object c : {g.mount, g.fold}) {
        if (!c) continue;
        if (local) { Params o(c, "SetOnlyOwnerSee"); o.Set("bNewOnlyOwnerSee", true).Call(); }
        c.SetBool("bHiddenInSceneCapture", true);
        Params sh(c, "SetCastShadow"); sh.Set("NewCastShadow", false).Call();
    }
    if (!local) {
        s_remoteGog.insert(nvg.ptr);
        Log("{}'s goggles fitted on {}", pawn.Name(), nvg.Name());
        return;
    }
    // The screen and the dark around it, hidden until the goggles come down. They exist on this machine
    // only, and the game's spectator view (the PC monitor, a scene capture) shows them too, so whoever
    // watches sees the drone's picture; the drone's own camera hides them (HideFromDrone).
    if (s_midScreen && s_meshScreen && s_meshShroud) {
        g.screen = AddMesh(nvg, s_meshScreen, root, g.eye, {}, &s_midScreen, 1);
        g.shroud = AddMesh(nvg, s_meshShroud, root, g.eye, {}, &s_midBlack, 1);
        for (Object c : {g.screen, g.shroud}) {
            if (!c) continue;
            Params sh(c, "SetCastShadow"); sh.Set("NewCastShadow", false).Call();
            SetVisible(c, false);
        }
    }
    g.down = LeverUp(lever) < 0.05;
    s_gog = g;
    Log("goggles fitted on {} (lever at {:.1f} deg, {}); eye at ({:.2f}, {:.2f}, {:.2f}) in the actor", nvg.Name(),
        lever.Get<float>("CurrentLeverAngle"), g.down ? "down" : "up", g.eye.x, g.eye.y, g.eye.z);
}

// Whether a player wears the goggles: their gear manager's own map, where the outfit was set on this machine,
// else the replicated gear (what the host sent: a client's own outfit, or another player's).
static bool WearsGoggles(Object gm) {
    if (!gm) return false;
    if (uint8_t* v = MapValue(gm, "GearByCategory", kCatNightVision))
        return IsOursPair(*(GUObject**)(v + GearDataOff()), *(uint64_t*)(v + GearColorOff()));
    GFProperty* prop = API->FindProperty(gm.Struct(), "ReplicatedGear");
    auto* arr = (GML_TArray*)gm.PropPtr("ReplicatedGear");
    GML_PropInfo ai{}, ei{};
    if (!prop || !arr || !API->GetPropertyInfo(prop, &ai) || !ai.inner || !API->GetPropertyInfo(ai.inner, &ei)) return false;
    for (int i = 0; i < arr->Num; i++) {  // FGearItemStruct each
        uint8_t* e = (uint8_t*)arr->Data + (size_t)i * ei.size;
        if (IsOursPair(*(GUObject**)(e + GearDataOff()), *(uint64_t*)(e + GearColorOff()))) return true;
    }
    return false;
}

// Called after anything that may have spawned, swapped or kept the local player's NVG actor.
static void CheckGoggles() {
    Object pawn = PlayerPawn();
    Object gm = GearManager(pawn);
    if (!gm) return;
    Object nvg = gm.GetObj("NVG");
    bool ours = WearsGoggles(gm);
    if (nvg && ours && s_gog.actor != nvg.ptr) Dress(nvg);
    else if (nvg && !ours && s_gog.actor == nvg.ptr) {
        // The same actor class now carries the real GPNVG-18: let the game build a clean one. (On a client the
        // NVG actor is the host's, which builds the new one.)
        Log("goggles taken off - the game's NVG goes back on");
        s_gog = Goggles{};
        if (NetClient()) return;
        Params d(nvg, "K2_DestroyActor"); d.Call();
        gm.Set("NVG", (GUObject*)nullptr);
        Params c(gm, "CheckNVG"); c.Call();
    } else if (!nvg && s_gog.actor) s_gog = Goggles{};
}

// ------------------------------------------------------------------ input
//
// Mode 2, as on FPV radios and in FPV games (Bodycam, FPV Kamikaze Drone): left stick throttle and yaw,
// right stick pitch and roll. Gamepad: the left stick's whole travel is the throttle (down 0%, centred 50%,
// about a hover); A arms and disarms (throttle down to arm), Y switches acro / angle, B held flips the
// drone back over after a crash, LB / RB tilt the camera, Start calls the drone back to you. A radio's
// sticks and arm switch are set in [Radio].

struct Sticks { double throttle = 0, yaw = 0, pitch = 0, roll = 0, zoom = 0; bool any = false; };  // throttle, zoom 0..1
struct Buttons { bool arm = false, mode = false, flip = false, tiltUp = false, tiltDown = false, recall = false; };
static Sticks s_sticks;
static Buttons s_buttons, s_buttonsPrev;
static int s_armSwitch = -1;  // a radio's arm switch: -1 none, 0 off, 1 on
static std::string s_inputName = "none";

static bool s_devInput = false;  // FPV_Dev "input": the harness drives the sticks
static Sticks s_devSticks;
static Buttons s_devButtons;

static double DeadZone(double v, double dz) { return std::fabs(v) < dz ? 0 : (v - (v > 0 ? dz : -dz)) / (1 - dz); }

static bool ReadGamepad(Sticks& s, Buttons& b) {
    static int slot = -1;
    static ULONGLONG nextScan = 0;
    XINPUT_STATE st{};
    if (slot >= 0 && XInputGetState(slot, &st) != ERROR_SUCCESS) slot = -1;
    if (slot < 0) {
        if (GetTickCount64() < nextScan) return false;
        nextScan = GetTickCount64() + 2000;  // asking empty slots is slow
        for (int i = 0; i < 4 && slot < 0; i++)
            if (XInputGetState(i, &st) == ERROR_SUCCESS) { slot = i; Log("gamepad {} connected", i); }
        if (slot < 0) return false;
    }
    auto ax = [](SHORT v) { return std::max(-1.0, v / 32767.0); };
    double lx = ax(st.Gamepad.sThumbLX), ly = ax(st.Gamepad.sThumbLY), rx = ax(st.Gamepad.sThumbRX), ry = ax(st.Gamepad.sThumbRY);
    if (s_cfgGamepadMode.Value() == 1) std::swap(ly, ry);  // mode 1: throttle on the right
    double dz = Clamp(s_cfgDeadzone.Value(), 0, 0.5);
    if (s_cfgPadThrottle.Value() == "Full") {
        s.throttle = Clamp((ly + 1) / 2, 0, 1);
        if (s.throttle < 0.02) s.throttle = 0;
    } else s.throttle = Clamp(DeadZone(ly, dz), 0, 1);  // the middle is zero, below it too
    s.yaw = DeadZone(lx, dz);
    s.pitch = DeadZone(ry, dz);
    s.roll = DeadZone(rx, dz);
    s.zoom = std::max(st.Gamepad.bLeftTrigger, st.Gamepad.bRightTrigger) / 255.0;
    WORD w = st.Gamepad.wButtons;
    b.arm = w & XINPUT_GAMEPAD_A;
    b.mode = w & XINPUT_GAMEPAD_Y;
    b.flip = w & XINPUT_GAMEPAD_B;
    b.tiltUp = w & XINPUT_GAMEPAD_RIGHT_SHOULDER;
    b.tiltDown = w & XINPUT_GAMEPAD_LEFT_SHOULDER;
    b.recall = w & XINPUT_GAMEPAD_START;
    return true;
}

// winmm calls every joystick "Microsoft PC-joystick driver"; the product name is in the registry.
static std::wstring JoystickName(UINT id, const JOYCAPSW& caps, std::wstring& oemKey) {
    std::wstring settings = L"System\\CurrentControlSet\\Control\\MediaResources\\Joystick\\" + std::wstring(caps.szRegKey) +
                            L"\\CurrentJoystickSettings";
    std::wstring value = L"Joystick" + std::to_wstring(id + 1) + L"OEMName";
    wchar_t buf[256];
    DWORD size = sizeof buf;
    if (RegGetValueW(HKEY_CURRENT_USER, settings.c_str(), value.c_str(), RRF_RT_REG_SZ, nullptr, buf, &size) != ERROR_SUCCESS) return caps.szPname;
    oemKey = buf;
    std::wstring oem = L"System\\CurrentControlSet\\Control\\MediaProperties\\PrivateProperties\\Joystick\\OEM\\" + oemKey;
    for (HKEY root : {HKEY_CURRENT_USER, HKEY_LOCAL_MACHINE}) {
        size = sizeof buf;
        if (RegGetValueW(root, oem.c_str(), L"OEMName", RRF_RT_REG_SZ, nullptr, buf, &size) == ERROR_SUCCESS) return buf;
    }
    return oemKey;
}
static std::string Utf8(const std::wstring& w) {
    char out[256];
    WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, out, sizeof out, nullptr, nullptr);
    return out;
}
// A gamepad that XInput already reads (Xbox and compatible pads show up in winmm too).
static bool IsXInputPad(const JOYCAPSW& caps, const std::wstring& name, const std::wstring& oemKey) {
    std::wstring n = name, k = oemKey;
    for (auto& c : n) c = (wchar_t)towlower(c);
    for (auto& c : k) c = (wchar_t)towupper(c);
    return caps.wMid == 0x045E || k.find(L"IG_") != std::wstring::npos || n.find(L"xbox") != std::wstring::npos ||
           n.find(L"xinput") != std::wstring::npos || n.rfind(L"controller (", 0) == 0;
}

// An FPV radio in USB joystick mode is a plain HID joystick, which Unreal doesn't read: winmm does.
static bool ReadRadio(Sticks& s) {
    static int dev = -1;
    static JOYCAPSW caps{};
    static ULONGLONG nextScan = 0;
    JOYINFOEX info{};
    info.dwSize = sizeof info;
    info.dwFlags = JOY_RETURNALL;
    if (dev >= 0 && joyGetPosEx(dev, &info) != JOYERR_NOERROR) { Log("radio disconnected"); dev = -1; }
    if (dev < 0) {
        if (GetTickCount64() < nextScan) return false;
        nextScan = GetTickCount64() + 2000;
        static std::vector<std::wstring> skipped;  // logged once each
        UINT n = joyGetNumDevs();
        for (UINT i = 0; i < n && dev < 0; i++) {
            JOYCAPSW c{};
            if (joyGetDevCapsW(i, &c, sizeof c) != JOYERR_NOERROR || joyGetPosEx(i, &info) != JOYERR_NOERROR) continue;
            std::wstring oemKey;
            std::wstring name = JoystickName(i, c, oemKey);
            if (IsXInputPad(c, name, oemKey)) {  // gamepads go through XInput
                if (std::find(skipped.begin(), skipped.end(), name) == skipped.end()) {
                    skipped.push_back(name);
                    Log("joystick '{}' ({}) is a gamepad - read through XInput", Utf8(name), Utf8(oemKey));
                }
                continue;
            }
            dev = (int)i;
            caps = c;
            Log("radio '{}' ({}) connected, {} axes, {} buttons - axes {} are roll, pitch, throttle, yaw; arm {} (see [Radio])", Utf8(name),
                Utf8(oemKey), c.wNumAxes, c.wNumButtons, s_cfgRadioAxes.Value(), s_cfgRadioArm.Value());
        }
        if (dev < 0) return false;
    }
    auto axis = [&](char a) -> double {
        DWORD v = 0, lo = 0, hi = 65535;
        switch (a) {
        case 'X': v = info.dwXpos; lo = caps.wXmin; hi = caps.wXmax; break;
        case 'Y': v = info.dwYpos; lo = caps.wYmin; hi = caps.wYmax; break;
        case 'Z': v = info.dwZpos; lo = caps.wZmin; hi = caps.wZmax; break;
        case 'R': v = info.dwRpos; lo = caps.wRmin; hi = caps.wRmax; break;
        case 'U': v = info.dwUpos; lo = caps.wUmin; hi = caps.wUmax; break;
        case 'V': v = info.dwVpos; lo = caps.wVmin; hi = caps.wVmax; break;
        default: return 0;
        }
        return hi > lo ? Clamp(2.0 * (double(v) - lo) / double(hi - lo) - 1, -1, 1) : 0;
    };
    std::string map = s_cfgRadioAxes.Value(), inv = s_cfgRadioInvert.Value();
    while (map.size() < 4) map += ' ';
    auto get = [&](int i) {
        char a = (char)toupper(map[i]);
        double v = axis(a);
        return inv.find(a) != std::string::npos || inv.find((char)tolower(a)) != std::string::npos ? -v : v;
    };
    s.roll = DeadZone(get(0), 0.01);
    s.pitch = DeadZone(get(1), 0.01);
    s.throttle = Clamp((get(2) + 1) / 2, 0, 1);  // 0 at the bottom
    s.yaw = DeadZone(get(3), 0.01);
    // Arm switch: an axis letter (on above the middle) or B<n> for a button.
    std::string arm = s_cfgRadioArm.Value();
    s_armSwitch = -1;
    if (arm.size() == 1) s_armSwitch = axis((char)toupper(arm[0])) > 0 ? 1 : 0;
    else if (arm.size() > 1 && (arm[0] == 'B' || arm[0] == 'b')) {
        int n = atoi(arm.c_str() + 1);
        if (n >= 1 && n <= 32) s_armSwitch = (info.dwButtons >> (n - 1)) & 1;
    }
    s_inputName = "radio";
    return true;
}

static uint64_t KeyName(const char* k) {
    static std::map<std::string, uint64_t> cache;
    auto it = cache.find(k);
    return it != cache.end() ? it->second : cache[k] = API->MakeName(k);
}
static double VRAxis(Object pc, std::initializer_list<const char*> keys) {
    for (const char* k : keys) {
        Params p(pc, "GetInputAnalogKeyState");
        *(uint64_t*)p.Ptr("Key") = KeyName(k);
        p.Call();
        double v = p.Return<float>();
        if (std::fabs(v) > 0.001) return v;
    }
    return 0;
}
static bool ReadVRSticks(Sticks& s) {
    Object pc = PlayerController();
    if (!pc) return false;
    s.roll = DeadZone(VRAxis(pc, {"OculusTouch_Right_Thumbstick_X", "ValveIndex_Right_Thumbstick_X", "MixedReality_Right_Thumbstick_X"}), 0.1);
    s.pitch = DeadZone(VRAxis(pc, {"OculusTouch_Right_Thumbstick_Y", "ValveIndex_Right_Thumbstick_Y", "MixedReality_Right_Thumbstick_Y"}), 0.1);
    s.yaw = DeadZone(VRAxis(pc, {"OculusTouch_Left_Thumbstick_X", "ValveIndex_Left_Thumbstick_X", "MixedReality_Left_Thumbstick_X"}), 0.1);
    s.throttle = Clamp((VRAxis(pc, {"OculusTouch_Left_Thumbstick_Y", "ValveIndex_Left_Thumbstick_Y", "MixedReality_Left_Thumbstick_Y"}) + 1) / 2, 0, 1);
    s_inputName = "VR sticks";
    return true;
}

static void ReadInput() {
    s_buttonsPrev = s_buttons;
    s_buttons = {};
    Sticks s;
    std::string dev = s_cfgDevice.Value();
    bool ok = false;
    s_armSwitch = -1;
    if (s_devInput) { s = s_devSticks; s_buttons = s_devButtons; s_inputName = "dev"; ok = true; }
    if (!ok && dev != "Gamepad") ok = ReadRadio(s);
    Buttons pad;
    Sticks padSticks;
    if (ReadGamepad(padSticks, pad) && dev != "Radio") {
        if (!ok) { s = padSticks; s_inputName = "gamepad"; ok = true; }
        if (!s_devInput) s.zoom = padSticks.zoom;
        s_buttons.arm |= pad.arm; s_buttons.mode |= pad.mode; s_buttons.flip |= pad.flip;  // a gamepad's buttons work with a radio too
        s_buttons.tiltUp |= pad.tiltUp; s_buttons.tiltDown |= pad.tiltDown; s_buttons.recall |= pad.recall;
    }
    if (!ok && s_cfgVRSticks.Value()) ok = ReadVRSticks(s);
    s.any = ok;
    if (!ok) s_inputName = "none";
    if (!s_devInput) {  // the stick curve: |x|^sensitivity, so above 1 is softer round the middle
        double k = Clamp(s_cfgSensitivity.Value(), 0.2, 4);
        for (double* v : {&s.roll, &s.pitch, &s.yaw}) *v = std::copysign(std::pow(std::fabs(*v), k), *v);
    }
    static std::string lastName;
    if (s_inputName != lastName) { lastName = s_inputName; Log("flying with: {}", s_inputName); }
    s_sticks = s;
}
static bool Pressed(bool Buttons::*b) { return s_buttons.*b && !(s_buttonsPrev.*b); }

// The right thumbstick, from the player character's own input events (pressed = event 0: it kicks).
static bool s_rightStick = false;
static double s_rightStickHeld = 0;
static bool s_rightStickFired = false;

// ------------------------------------------------------------------ the drone

struct Drone {
    GUObject* actor = nullptr;
    Object root, body, camPivot, camMesh, capture, props[4], propPivots[4], sound;
    GUObject* rt = nullptr;
    quad::Quad q;               // the physics (metres)
    kamikaze::Drone k;          // FPV Kamikaze Drone's flight model on top of q (FlightModel = Kamikaze)
    double accum = 0;           // physics time not stepped yet
    double propAngle[4] = {};
    bool crashed = false;       // the last hit disarmed it
    double flyTime = 0;         // armed time (OSD timer)
    double sinceSpawn = 0;
    std::string warn;           // a warning that stays up for a while
    double warnFor = 0;
    double tilt = 25;           // camera up-tilt, deg
    int lastArmSwitch = -1;
    bool turtle = false;        // crash flip under way
    int turtleDir = 1;          // which way it rolls over
    Object release, bomb;       // the payload under the body
    bool payload = false;       // carrying it: heavier, and it goes off on impact
    bool fuseArmed = false;     // the fuse is live (once it has been ArmDistance from you)
    bool item = false;          // built on a game item (an M67 from a drone pouch on the vest)
    V3 anchor;                  // where that item's root (the grenade) sits, in the drone's frame: at its bomb
    // FPV Kamikaze Drone's model flies on the engine's physics (MakeFlightBody): the item's root is its body.
    bool chaos = false;         // the body is FPV Kamikaze Drone's and simulating
    Object capsule;             // the capsule welded to it
    GUObject* itemMesh = nullptr;  // the item's own mesh, its mass and damping, back when it is carried
    float itemMass = 0, itemLinDamp = 0, itemAngDamp = 0;
    double batteryTick = 0;     // s to the next once-a-second battery step
    V3 lastBodyAt;              // its body's outline's middle last frame, and its speed then (the hit test)
    double lastKmh = 0;
    bool haveLast = false;
    bool lostControl = false;   // the battery ran flat (ControlLost)
    GUObject* pouch = nullptr;  // that pouch (Pouch_C)
    bool free = false;          // out of hand and pouch: it flies (else the game moves it)
    int carry = -1;             // 0 free, 1 in its pouch, 2 in a hand (Carried)
    double carryFor = 0;        // s like that
    Quat seat;                  // d.root in the item's root: on its pouch belly in, kept as it was when taken
    Quat lastAtt;               // d.root's world rotation, last frame
    bool detonate = false;      // hit hard enough this frame: boom (handled after the collisions)
    V3 detonateAt, detonateNormal;
    double fov = 0;             // the camera's field of view now (zoom)
    double stillFor = 0;        // s sitting still on the ground (Arming = Auto)
    bool fullAtArm = true;
    // the radio and video links
    double rssi = -18, lq = 100, lostFor = 0, failsafeFor = 0, bitrate = 25.4, latency = 25;
    bool rxLoss = false, videoLost = false;
    quad::V3 home;              // where it was armed, m
    // multiplayer
    GUObject* pilot = nullptr;  // another player's drone (s_remote): theirs
    bool hostFlown = false;     // on the host: moved by its pilot's messages (physics off), see "multiplayer"
    bool noRepMove = false;     // on a client, its own drone in flight: the host's copy of its movement ignored
    double sentAgo = 0;         // s since its pilot last sent where it is
    V3 lastSeenAt;              // another player's drone: where it was last tick (its propellers spin while it moves)
};
static Drone s_drone;

static bool DroneValid() { return s_drone.actor && API->IsValid(s_drone.actor); }

// What it bumps into the world with: a box around the frame and the propeller tips, cm, turned with it
// (down to the bomb while it carries the payload).
static V3 Box() { return {12.5, 16.5, std::max(2.0, -(s_drone.payload ? s_payloadBottom : s_bottom))}; }
static V3 ToUE(quad::V3 v) { return {v.x * 100, v.y * 100, v.z * 100}; }
static quad::V3 ToM(V3 v) { return {v.x / 100, v.y / 100, v.z / 100}; }
static Rot AttRot(const quad::Q& q) { return ToRot({q.x, q.y, q.z, q.w}); }

// Not destroyed (or being): the engine's IsValid, which a destroyed actor fails while its object lives on.
static bool Alive(GUObject* o) {
    if (!o || !API->IsValid(o)) return false;
    Params v(Lib("KismetSystemLibrary"), "IsValid");
    v.SetObj("Object", Object(o)).Call();
    return v.Return<bool>();
}

// ------------------------------------------------------------------ the drone on the vest
//
// The drone is carried like a grenade. In the kit station it is a pouch beside the grenade, flashbang and
// slapcharge pouches (BP_GameInstance.Pouches, explosives): take it and put it anywhere on your vest or
// belt. A pouch on the vest (Pouch_C) spawns its PouchData's Pouchable into itself and stores it there
// (CheckSpawnStorable). Ours is the grenade pouch's own, the M67 (BP_M67_C, a GrenadeActor: grippable,
// storable, its grip, its explosion), and DressItem makes it the drone: the grenade hidden where the
// drone's bomb is, so a hand holds the drone by it, and the drone on it. Its pin can't be pulled; the
// payload going off is the grenade's own Explode. The pouch's own mesh (the grenade pouch's, kept for its
// collision and attach socket) is hidden, so the drone hangs on the vest belly in, nose up. Take it in a
// hand, put it back (let go of it at its pouch), or let go of it anywhere else and it flies. With more than
// one drone pouch, the drone taken last is the one you fly; the others wait where they are.
// A kit is saved with its pouches' asset paths, and ours has none: while a kit is saved the drone's pouch
// is the grenade pouch with SelectedColor "FPVDrone", and GetKitSetup turns that back into ours. Without
// this plugin such a kit has a grenade pouch there.

static const char* kDroneName = "DJI Mini 4K FPV";
static const char* kGrenadeClass = "/Game/Gameplay/Throwables/M67/BP_M67.BP_M67_C";
static const char* kGrenadePouch = "/Game/Characters/Data/Pouches/Grenade.Grenade";
static const char* kPouchTag = "FPVDrone";  // SelectedColor of the grenade pouch that stands for ours in a save
static constexpr uint8_t kExplosives = 2;   // EPouchType: grenades, flashbangs, slapcharge
static ConfigEntry<bool> s_cfgPouch;
static GUObject* s_pouchData = nullptr;     // our PouchData_C
static std::vector<GUObject*> s_editables;  // kit-station pouches (EditablePouch_C) showing the drone
static std::vector<Drone> s_parked;         // drones built on the vest or set down, not the one flying

// Where the drone sits on its pouch, in the pouch mesh's frame (the grenade pouch's: +Y out of the vest,
// +Z up, its back at Y -0.3): belly against the vest, nose up.
static const Quat kMount{-0.5, -0.5, -0.5, 0.5};  // the drone's X (nose) along +Z, its top along +Y
static V3 MountLoc() {
    double belly = -(s_cfgPayload.Value() ? s_payloadBottom : s_bottom);  // down to the bomb
    return {0.3, belly + 0.2, 0.4};
}

// FPouchStruct {UPouchData_C* Data; ...; FName SelectedColor; ...}, Blueprint-mangled names.
static int PouchStructOff(const char* name, int fallback) {
    GUStruct* s = (GUStruct*)API->FindObject("/Game/Gameplay/Customization/PouchStruct.PouchStruct");
    GML_PropInfo i;
    return s && API->GetPropertyInfo(API->FindProperty(s, name), &i) ? i.offset : fallback;
}
static int PouchDataOff() { static int off = PouchStructOff("Data", 0); return off; }
static int PouchColorOff() { static int off = PouchStructOff("SelectedColor", 0x78); return off; }
static int PouchStructSize() {
    static int size = 0;
    GML_PropInfo i;
    if (!size) size = API->GetPropertyInfo(API->FindProperty((GUStruct*)FindClass("Pouch_C"), "Data"), &i) ? i.size : 0xA0;
    return size;
}
static GUObject* PouchDataOf(Object actor) {  // a Pouch_C's or EditablePouch_C's Data.Data
    uint8_t* p = actor ? (uint8_t*)actor.PropPtr("Data") : nullptr;
    return p ? *(GUObject**)(p + PouchDataOff()) : nullptr;
}
static GUObject* GrenadePouch() {
    static GUObject* g = nullptr;
    if (!g || !API->IsValid(g)) g = LoadObject(kGrenadePouch).ptr;
    return g;
}
// Ours, or the grenade pouch with SelectedColor "FPVDrone": the form a kit is saved in, and the one the vest
// really carries (ToNetForm): in multiplayer a kit goes to the host and every pouch to every player as asset
// paths and names, and our PouchData, built at runtime, would arrive as nothing. Both spawn the same M67.
// The game recolours a kit's pouches to its camo (SelectedColor), so a vest pouch is remembered once seen as ours,
// and the host gives it its mark back (KeepPouchTags) for every other machine to see.
static std::set<GUObject*> s_knownPouches;
static bool IsDronePouch(Object actor) {
    static uint64_t tag = NameOf(kPouchTag);
    if (!actor) return false;
    if (s_knownPouches.count(actor.ptr)) return true;
    uint8_t* p = (uint8_t*)actor.PropPtr("Data");
    GUObject* data = p ? *(GUObject**)(p + PouchDataOff()) : nullptr;
    if (!data) return false;
    bool ours = (s_pouchData && data == s_pouchData) || (data == GrenadePouch() && *(uint64_t*)(p + PouchColorOff()) == tag);
    if (ours && actor.IsA("Pouch_C")) s_knownPouches.insert(actor.ptr);
    return ours;
}
// The host (or alone): our vest pouches marked "FPVDrone" again where the game recoloured them.
static void KeepPouchTags() {
    static uint64_t tag = NameOf(kPouchTag);
    for (auto it = s_knownPouches.begin(); it != s_knownPouches.end();) {
        if (!Alive(*it)) { it = s_knownPouches.erase(it); continue; }
        uint8_t* p = (uint8_t*)Object(*it).PropPtr("Data");
        if (p && *(GUObject**)(p + PouchDataOff()) == GrenadePouch() && *(uint64_t*)(p + PouchColorOff()) != tag)
            *(uint64_t*)(p + PouchColorOff()) = tag;
        ++it;
    }
}
// What a pouch holds: its Item, else the M67 attached to it (a client's own pouches don't get their Item).
static Object PouchItem(Object pouch) {
    Object item = pouch ? pouch.GetObj("Item") : Object();
    if (item && Alive(item.ptr)) return item;
    if (!pouch) return {};
    Params ga(pouch, "GetAttachedActors");
    ga.Set("bResetArray", true).Set("bRecursivelyIncludeAttachedActors", false).Call();
    auto* arr = (GML_TArray*)ga.Ptr("OutActors");
    GUClass* m67 = LoadClass(kGrenadeClass);
    for (int i = 0; arr && i < arr->Num; i++) {
        Object a(((GUObject**)arr->Data)[i]);
        if (a && a.Class() == m67 && Alive(a.ptr)) return a;
    }
    return {};
}
// A kit-station pouch (EditablePouch_C) of ours, made the grenade pouch marked "FPVDrone" (see IsDronePouch).
static void ToNetForm(Object pouch) {
    uint8_t* p = pouch ? (uint8_t*)pouch.PropPtr("Data") : nullptr;
    if (!p || !s_pouchData || *(GUObject**)(p + PouchDataOff()) != s_pouchData || !GrenadePouch()) return;
    *(GUObject**)(p + PouchDataOff()) = GrenadePouch();
    *(uint64_t*)(p + PouchColorOff()) = NameOf(kPouchTag);
}
static std::vector<Drone> s_remote;  // other players' drones (multiplayer): their look only, see DressRemote
static bool IsDroneActor(GUObject* a) {
    if (!a) return false;
    if (a == s_drone.actor) return true;
    for (const Drone& p : s_parked)
        if (p.actor == a) return true;
    for (const Drone& p : s_remote)
        if (p.actor == a) return true;
    return false;
}

static bool MakePouchData() {
    if (s_pouchData && API->IsValid(s_pouchData)) return true;
    Object grenade = LoadObject(kGrenadePouch);
    GUClass* cls = FindClass("PouchData_C");
    GUClass* m67 = LoadClass(kGrenadeClass);
    if (!grenade || !cls || !m67) { Error("the grenade pouch, PouchData_C or the M67 is missing - no drone pouch"); return false; }
    Object d = NewObject(cls, Object(API->WorldContext()));
    if (!d) { Error("could not construct the drone's pouch"); return false; }
    d.Set("Mesh", grenade.Get<GUObject*>("Mesh"));
    d.Set("Type", kExplosives);
    d.Set("Pouchable", (GUObject*)m67);
    d.Set("AttachType", grenade.Get<uint8_t>("AttachType"));
    d.Set("AttachPointTag", grenade.Get<uint64_t>("AttachPointTag"));
    if (BuildAssets() && s_droneIcon) d.Set("IconOverride", s_droneIcon);
    uint8_t text[16];
    if (API->MakeText(kDroneName, text)) std::memcpy(d.PropPtr("DisplayName"), text, 16);
    s_pouchData = d.ptr;
    API->KeepAlive(s_pouchData);
    return true;
}

static void AddDronePouch(Object gi) {
    if (!s_cfgPouch.Value()) return;
    uint8_t* v = MapValue(gi, "Pouches", kExplosives);
    if (!v) { Warn("the kit's explosives pouches aren't there yet"); return; }
    TArrayView<GUObject*> list{(GML_TArray*)v};  // FPouchDataList's one member, TArray<UPouchData_C*>
    if (s_pouchData && API->IsValid(s_pouchData) && list.Contains(s_pouchData)) return;
    if (!MakePouchData()) return;
    int before = list.Num();
    if (!list.Add(s_pouchData)) { Error("could not add the drone's pouch to the kit"); return; }
    Log("added '{}' to the kit's explosives pouches ({} -> {})", kDroneName, before, list.Num());
}

// The drone's meshes under `at`: body, camera, propellers and (with the payload) the release and bomb.
static void AddLook(Object a, Object at) {
    AddMesh(a, s_meshBody, at);
    AddMesh(a, s_meshCamera, AddComp(a, "SceneComponent", at, s_camPos, Rot{s_cfgUptilt.Value(), 0, 0}));
    for (int i = 0; i < 4; i++) AddMesh(a, s_meshProps[i], AddComp(a, "SceneComponent", at, s_propPos[i]));
    if (s_cfgPayload.Value() && s_meshBomb) {
        AddMesh(a, s_meshRelease, at);
        AddMesh(a, s_meshBomb, at, s_bombPos);
    }
}

// The kit station's pouch for ours (the one you take and put on the vest) shows the drone where it will
// hang instead of the grenade pouch, and a hand takes it by a box the drone's size.
static void DressEditable(Object ep) {
    if (!ep || !IsDronePouch(ep) || std::find(s_editables.begin(), s_editables.end(), ep.ptr) != s_editables.end()) return;
    Object root = ep.GetObj("RootComponent");
    if (!root || !BuildAssets()) return;
    std::erase_if(s_editables, [](GUObject* e) { return !Alive(e); });
    s_editables.push_back(ep.ptr);
    SetVisible(root, false);
    AddLook(ep, AddComp(ep, "SceneComponent", root, MountLoc(), ToRot(kMount)));
    if (Object box = AddComp(ep, "BoxComponent", root, MountLoc(), ToRot(kMount))) {
        V3 b = Box();
        Params e(box, "SetBoxExtent");
        e.Set("InBoxExtent", b).Set("bUpdateOverlaps", true).Call();
        Params pn(root, "GetCollisionProfileName");
        pn.Call();
        Params sp(box, "SetCollisionProfileName");
        sp.Set("InCollisionProfileName", pn.Return<uint64_t>()).Set("bUpdateOverlaps", true).Call();
        Params qo(box, "SetCollisionEnabled");  // for a hand to find, never pushing anything (a physics box this big shoves off what it overlaps)
        qo.Set("NewType", (uint8_t)1).Call();  // QueryOnly
    }
}

// A kit is saved from the kit station's pouches: ours stand in as the grenade pouch while it is.
static std::vector<GUObject*> s_savingPouches;
static void BeforeKitSave() {
    s_savingPouches.clear();
    Object grenade = LoadObject(kGrenadePouch);
    for (GUObject* e : s_editables) {
        if (!grenade || !Alive(e) || !s_pouchData || PouchDataOf(Object(e)) != s_pouchData) continue;  // (in the saved form already: as is)
        uint8_t* p = (uint8_t*)Object(e).PropPtr("Data");
        *(GUObject**)(p + PouchDataOff()) = grenade.ptr;
        *(uint64_t*)(p + PouchColorOff()) = API->MakeName(kPouchTag);
        s_savingPouches.push_back(e);
    }
}
static void AfterKitSave() {
    for (GUObject* e : s_savingPouches) {
        if (!Alive(e)) continue;
        uint8_t* p = (uint8_t*)Object(e).PropPtr("Data");
        *(GUObject**)(p + PouchDataOff()) = s_pouchData;
        *(uint64_t*)(p + PouchColorOff()) = 0;
    }
    if (!s_savingPouches.empty()) Log("kit saved with {} drone pouch(es)", s_savingPouches.size());
    s_savingPouches.clear();
}
// GetKitSetup's kit as loaded from its slot (the cast it returns, read from its locals: the out parameter
// itself is written through the caller's frame). Its drone pouches stay the grenade pouch marked "FPVDrone",
// which IsDronePouch takes for ours: that kit goes to the host as it is in multiplayer.
static void OnKitLoaded(GUObject* loaded) {
    if (!loaded || !s_cfgPouch.Value()) return;
    auto* arr = (GML_TArray*)Object(loaded).PropPtr("Pouches");
    Object grenade = LoadObject(kGrenadePouch);
    if (!arr || !grenade) return;
    uint64_t tag = API->MakeName(kPouchTag);
    int n = 0;
    for (int i = 0; i < arr->Num; i++) {
        uint8_t* p = (uint8_t*)arr->Data + (size_t)i * PouchStructSize();
        if (*(GUObject**)(p + PouchDataOff()) == grenade.ptr && *(uint64_t*)(p + PouchColorOff()) == tag) n++;
    }
    if (n) Log("kit loaded with {} drone pouch(es)", n);
}

// BP_StorableComponent's StoreToPouch, before it runs: drones go only into the drone's pouches, and nothing
// else does (a real grenade neither; else the pouch argument is cleared and nothing is stored), except the
// grenade a drone pouch is spawning for itself. Into one, square to it with the drone at MountLoc: the
// grenade's PouchRelativeTransform, which is relative to the pouch mesh's AttachSocket.
static GUObject* s_filling = nullptr;  // the drone pouch in its CheckSpawnStorable
static void BeforeStore(Object storable, uint8_t* pouchArg) {
    if (!pouchArg || !s_pouchData || !*(GUObject**)pouchArg) return;
    Object pouch(*(GUObject**)pouchArg);
    Params go(storable, "GetOwner");
    go.Call();
    Object item = go.Return<GUObject*>();
    bool ours = IsDronePouch(pouch);
    bool drone = IsDroneActor(item.ptr) || (pouch.ptr == s_filling && item && item.Class() == LoadClass(kGrenadeClass));
    if (ours != drone) {
        *(GUObject**)pouchArg = nullptr;
        return;
    }
    Object mesh = pouch.GetObj("StaticMesh");
    if (!ours || !mesh || !BuildAssets()) return;
    Params gs(mesh, "GetSocketTransform");
    gs.Set("InSocketName", API->MakeName("AttachSocket")).Set("TransformSpace", (uint8_t)2).Call();  // RTS_Component
    const double* s = (const double*)gs.Ptr("ReturnValue");
    double* t = (double*)storable.PropPtr("PouchRelativeTransform");
    if (!s || !t) return;
    Quat sq{s[0], s[1], s[2], s[3]};
    V3 loc = Unrotate(sq, MountLoc() + Rotate(kMount, s_bombPos) - V3{s[4], s[5], s[6]});  // the grenade at the bomb
    double rel[11] = {-sq.x, -sq.y, -sq.z, sq.w, loc.x, loc.y, loc.z, 0, 1, 1, 1};
    std::memcpy(t, rel, sizeof rel);
}

// The drone on its item, turned by `seat` and placed so that the item's root (the grenade) is at the
// drone's bomb (d.anchor, in the drone's frame).
static void PlaceOnItem(Drone& d) {
    SetRelRot(d.root, ToRot(d.seat));
    V3 at = Rotate(d.seat, d.anchor) * -1;
    if (Object root = Object(d.actor).GetObj("RootComponent")) {  // a parent's scale scales the offset (the flight body's box)
        Params s(root, "K2_GetComponentScale");
        s.Call();
        V3 k = s.Return<V3>();
        at = {at.x / std::max(1e-3, k.x), at.y / std::max(1e-3, k.y), at.z / std::max(1e-3, k.z)};
    }
    Params p(d.root, "K2_SetRelativeLocation");
    p.Set("NewLocation", at).Set("bSweep", false).Set("bTeleport", true).Call();
}

// Turns the drone on its item (the grenade at its bomb) so that on its pouch it hangs as kMount says,
// square to the pouch's mesh, and in a hand it stays as it was when taken (the grenade snaps to the hand's
// grip; for a moment, while the game settles it there).
static void UpdateSeat(Drone& d, Object root, int carry) {
    if (!root || !d.root || !carry) return;
    Quat qr = ToQuat(CompRot(root));
    Quat want = d.lastAtt;
    if (carry == 1) {
        Params gp(root, "GetAttachParent");
        gp.Call();
        Object mesh = gp.Return<GUObject*>();
        if (!mesh) return;
        want = Mul(ToQuat(CompRot(mesh)), kMount);
        d.lastAtt = want;  // as a hand will find it
    } else if (d.carryFor >= 0.15) return;
    Quat seat = Normalized(Mul({-qr.x, -qr.y, -qr.z, qr.w}, want));
    if (std::fabs(seat.x - d.seat.x) + std::fabs(seat.y - d.seat.y) + std::fabs(seat.z - d.seat.z) + std::fabs(seat.w - d.seat.w) > 1e-4) {
        d.seat = seat;
        PlaceOnItem(d);
    }
}

// A drone that is not the one flying: its motors quiet, and, if it isn't on the vest or in a hand, left to
// the engine's physics (it falls and lies where it lands, and a hand can still take it).
static void Park(Drone& d) {
    if (d.sound) { Params v(d.sound, "SetVolumeMultiplier"); v.Set("VolumeMultiplier", 0.0f).Call(); }
    Object a(d.actor);
    Object root = a ? a.GetObj("RootComponent") : Object();
    if (root && !Carried(a)) { Params p(root, "SetSimulatePhysics"); p.Set("bSimulate", true).Call(); }
    d.free = false;
}

// ---- FPV Kamikaze Drone's body (PHYS_Drone_FPV, BP_Drone_FPV): what flies is the engine's physics, as there.
// Its physics asset: one body of a box 22.2 wide, 21.4 long and 3.8 cm high, 2 cm under the middle, and a
// capsule 5 cm round and 15 long along the nose, 4 cm over it; 0.1 kg, CCD, hit events, no physical
// material (the engine's default). BeginPlay sets linear damping 0.25 and angular 20.
static const V3 kBoxSize{21.424, 22.235, 3.817};
static const V3 kBoxAt{0, 0, -1.985}, kCapsuleAt{0, 0, 4.024};  // in the drone's frame
static constexpr double kCapsuleRadius = 5, kCapsuleLength = 15;
static const V3 kOutlineAt{0, 0, 2.55}, kOutlineHalf{12.5, 11.12, 6.46};  // a box round both, for the hit test

static GUObject* CubeMesh() {
    static GUObject* c = nullptr;
    if (!c) c = LoadObject("/Engine/BasicShapes/Cube.Cube").ptr;  // 100 cm, a box collision
    return c;
}
template <class T> static void CallOn(Object o, const char* fn, const char* p, T v) { Params c(o, fn); c.Set(p, v).Call(); }

// A component's own body settings, written in: on a welded one the setters (SetMassOverrideInKg,
// SetNotifyRigidBodyCollision) act on the body it is welded to, while the welded body's mass and its shapes'
// hit events are worked out from each component's own.
static void SetOwnBody(Object comp, float massKg, bool notify) {
    GML_PropInfo bi, f;
    uint8_t* b = comp ? (uint8_t*)comp.PropPtr("BodyInstance") : nullptr;
    if (!b || !API->GetPropertyInfo(API->FindProperty(comp.Struct(), "BodyInstance"), &bi) || !bi.structType) return;
    if (API->GetPropertyInfo(API->FindProperty(bi.structType, "MassInKgOverride"), &f)) *(float*)(b + f.offset) = massKg;
    for (const char* flag : {"bOverrideMass", "bNotifyRigidBodyCollision"}) {
        if (!API->GetPropertyInfo(API->FindProperty(bi.structType, flag), &f)) continue;
        uint8_t& byte = b[f.offset + f.boolByteOffset];
        bool on = flag[1] == 'O' || notify;
        byte = on ? (byte | f.boolFieldMask) : (byte & ~f.boolFieldMask);
    }
}

// Let go (out of hand and pouch): the item's root becomes that body - the engine's cube scaled to the box,
// the capsule welded on, the mass shared by volume as one body's is - where the drone is, at the speed it
// was let go at, and simulating.
static void MakeFlightBody(Drone& d, V3 vel) {
    Object a(d.actor), root = a ? a.GetObj("RootComponent") : Object();
    if (!root || !CubeMesh()) return;
    V3 at = CompLoc(d.root);
    Quat qa = ToQuat(CompRot(d.root));
    { Params m(root, "SetStaticMesh"); m.SetObj("NewMesh", Object(CubeMesh())).Call(); }
    CallOn(root, "SetWorldScale3D", "NewScale", kBoxSize * 0.01);
    SetVisible(root, false);
    d.seat = Quat{};
    d.anchor = kBoxAt;
    PlaceOnItem(d);
    { Params t(root, "K2_SetWorldLocationAndRotation");
      t.Set("NewLocation", at + Rotate(qa, kBoxAt)).Set("NewRotation", ToRot(qa)).Set("bSweep", false).Set("bTeleport", true).Call(); }
    { Params s(root, "SetSimulatePhysics"); s.Set("bSimulate", true).Call(); }
    double vb = kBoxSize.x * kBoxSize.y * kBoxSize.z;
    double vc = kPi * kCapsuleRadius * kCapsuleRadius * kCapsuleLength + 4.0 / 3 * kPi * std::pow(kCapsuleRadius, 3);
    double mass = d.k.p.mass;
    if (Object c = AddComp(a, "CapsuleComponent", root)) {  // welded once the root simulates
        d.capsule = c;
        Params ab(c, "SetAbsolute");
        ab.Set("bNewAbsoluteLocation", false).Set("bNewAbsoluteRotation", false).Set("bNewAbsoluteScale", true).Call();
        CallOn(c, "SetWorldScale3D", "NewScale", V3{1, 1, 1});
        Params sz(c, "SetCapsuleSize");
        sz.Set("InRadius", (float)kCapsuleRadius).Set("InHalfHeight", (float)(kCapsuleLength / 2 + kCapsuleRadius)).Set("bUpdateOverlaps", false).Call();
        Params pn(root, "GetCollisionProfileName");
        pn.Call();
        Params sp(c, "SetCollisionProfileName");
        sp.Set("InCollisionProfileName", pn.Return<uint64_t>()).Set("bUpdateOverlaps", false).Call();
        CallOn(c, "SetCollisionEnabled", "NewType", (uint8_t)3);  // QueryAndPhysics
        SetOwnBody(c, (float)(mass * vc / (vb + vc)), true);
        Params at2(c, "K2_AttachToComponent");
        at2.SetObj("Parent", root).Set("SocketName", uint64_t(0)).Set("LocationRule", (uint8_t)0).Set("RotationRule", (uint8_t)0)
            .Set("ScaleRule", (uint8_t)2).Set("bWeldSimulatedBodies", true).Call();
        V3 off = kCapsuleAt - kBoxAt, k = kBoxSize * 0.01;  // its place in the scaled root, along the nose
        Params rl(c, "K2_SetRelativeLocationAndRotation");
        rl.Set("NewLocation", V3{off.x / k.x, off.y / k.y, off.z / k.z}).Set("NewRotation", Rot{90, 0, 0}).Set("bSweep", false).Set("bTeleport", true).Call();
    }
    if (d.capsule) SetOwnBody(d.capsule, (float)(mass * vc / (vb + vc)), true);  // its share and its hits; the root's calls redo the body
    Params rm(root, "SetMassOverrideInKg");
    rm.Set("BoneName", uint64_t(0)).Set("MassInKg", (float)(mass * vb / (vb + vc))).Set("bOverrideMass", true).Call();
    CallOn(root, "SetLinearDamping", "InDamping", (float)d.k.p.linearDamping);
    CallOn(root, "SetAngularDamping", "InDamping", (float)d.k.p.angularDamping);
    { Params c(root, "SetUseCCD"); c.Set("InUseCCD", true).Set("BoneName", uint64_t(0)).Call(); }
    CallOn(root, "SetNotifyRigidBodyCollision", "bNewNotifyRigidBodyCollision", true);
    { Params v(root, "SetPhysicsLinearVelocity"); v.Set("NewVel", vel).Set("bAddToCurrent", false).Set("BoneName", uint64_t(0)).Call(); }
    { Params w(root, "SetPhysicsAngularVelocityInDegrees"); w.Set("NewAngVel", V3{}).Set("bAddToCurrent", false).Set("BoneName", uint64_t(0)).Call(); }
    d.k.held = false;
    d.haveLast = false;
    d.chaos = true;
    Params gm(root, "GetMass");
    gm.Call();
    Params cmass(d.capsule ? d.capsule : root, "GetMass");
    cmass.Call();
    Log("flight body: {:.3f} kg (capsule {:.3f}), want {:.3f}", gm.Return<float>(), d.capsule ? cmass.Return<float>() : 0.f, mass);
}

// Taken up again (a hand, a pouch): the grenade back as it was.
static void DropFlightBody(Drone& d) {
    if (!d.chaos) return;
    d.chaos = false;
    if (d.capsule) { Params x(d.capsule, "K2_DestroyComponent"); x.SetObj("Object", d.capsule).Call(); d.capsule = {}; }
    Object root = Object(d.actor).GetObj("RootComponent");
    if (!root) return;
    if (d.itemMesh) { Params m(root, "SetStaticMesh"); m.SetObj("NewMesh", Object(d.itemMesh)).Call(); }
    CallOn(root, "SetWorldScale3D", "NewScale", V3{1, 1, 1});
    SetVisible(root, false);
    if (d.itemMass > 0) {
        Params rm(root, "SetMassOverrideInKg");
        rm.Set("BoneName", uint64_t(0)).Set("MassInKg", d.itemMass).Set("bOverrideMass", true).Call();
    }
    CallOn(root, "SetLinearDamping", "InDamping", d.itemLinDamp);
    CallOn(root, "SetAngularDamping", "InDamping", d.itemAngDamp);
    d.anchor = s_bombPos;
    PlaceOnItem(d);
}

static void Alert(Drone& d, const char* w, double secs);

// BP_Drone_Base's ControlLost (the battery flat): SetDefaultDamping - linear 0.01, angular 0 - and a kick
// of spin, TransformLocation of a random (r, r, r) as a torque, an acceleration (the Blueprint's own quirk).
static void LoseControl(Drone& d, Object root) {
    d.lostControl = true;
    d.k.controlled = false;
    CallOn(root, "SetLinearDamping", "InDamping", 0.01f);
    CallOn(root, "SetAngularDamping", "InDamping", 0.0f);
    double r = 10000 * d.q.Random();  // RandomFloatInRange(-10000, 10000)
    V3 torque = CompLoc(root) + Rotate(ToQuat(CompRot(root)), V3{r, r, r});
    Params t(root, "AddTorqueInDegrees");
    t.Set("Torque", torque).Set("BoneName", uint64_t(0)).Set("bAccelChange", true).Call();
    Alert(d, "BATTERY EMPTY", 99);
    Log("battery flat: control lost");
}

// One frame of FPV Kamikaze Drone (BP_Drone_FPV's tick and stick events) on the body; the engine's physics
// does the rest - integration, damping, gravity, contacts - at the game's frame rate, as there.
static void FlyChaos(Drone& d, const quad::Inputs& in, double dt, const std::vector<GUObject*>& ignore) {
    quad::Quad& q = d.q;
    kamikaze::Drone& k = d.k;
    Object root = Object(d.actor).GetObj("RootComponent");
    if (!root) return;
    Params gv(root, "GetPhysicsLinearVelocity");
    gv.Set("BoneName", uint64_t(0)).Call();
    V3 v = gv.Return<V3>();  // cm/s
    Params gw(root, "GetPhysicsAngularVelocityInRadians");
    gw.Set("BoneName", uint64_t(0)).Call();
    V3 w = gw.Return<V3>();
    Quat qa = ToQuat(CompRot(d.root));
    q.pos = ToM(CompLoc(d.root));
    q.att = {qa.x, qa.y, qa.z, qa.w};
    q.vel = ToM(v);
    V3 rb = Unrotate(qa, w);
    q.rate = {rb.x, rb.y, rb.z};
    // The hit (FPV Kamikaze Drone's body hit event, the warhead's CheckHit): this frame's way swept with the
    // body's outline a centimetre over; something there, at more than DetonateSpeed, and the payload goes off.
    V3 now = CompLoc(d.root) + Rotate(qa, kOutlineAt);
    double kmh = Len(v) * 0.01 * 3.6;
    if (d.payload && d.fuseArmed && !d.detonate && d.haveLast && std::max(kmh, d.lastKmh) > s_cfgDetonateSpeed.Value()) {
        Hit h = Sweep(d.lastBodyAt, now, kOutlineHalf + V3{1, 1, 1}, ToRot(qa), ignore);
        if (h.hit) {
            d.detonate = true;
            d.detonateAt = h.impact;
            d.detonateNormal = h.normal;
            Log("payload hit something at {:.0f} km/h", std::max(kmh, d.lastKmh));
        }
    }
    d.lastBodyAt = now;
    d.lastKmh = kmh;
    d.haveLast = true;
    // The battery: BatteryCharge steps down once a second (ConsumeBattery); flat, control is lost.
    if (!d.lostControl && (d.batteryTick -= dt) <= 0) {
        d.batteryTick += 1;
        k.charge = std::max(0.0, k.charge - 1 / k.p.batterySeconds);
        if (k.charge <= 0) LoseControl(d, root);
    }
    if (!q.armed) q.Arm(true);  // no arming in FPV Kamikaze Drone: under control until the battery is flat
    bool controlled = !d.lostControl;
    k.controlled = controlled;
    // The sticks (IA_Roll, IA_Pitch, IA_Yaw): while one is off centre its turn is set, in its own axes, at
    // RotationRate; when they come back, stopped (their Completed events).
    bool any = controlled && (in.roll != 0 || in.pitch != 0 || in.yaw != 0);
    if (any) {
        const double d2r = kPi / 180;
        V3 rate{-in.roll * k.p.rotationRate.x * d2r, in.pitch * k.p.rotationRate.y * d2r, in.yaw * k.p.rotationRate.z * d2r};
        Params s(root, "SetPhysicsAngularVelocityInRadians");
        s.Set("NewAngVel", Rotate(qa, rate)).Set("bAddToCurrent", false).Set("BoneName", uint64_t(0)).Call();
    } else if (k.held) {
        Params s(root, "SetPhysicsAngularVelocityInRadians");
        s.Set("NewAngVel", V3{}).Set("bAddToCurrent", false).Set("BoneName", uint64_t(0)).Call();
    }
    k.held = any;
    // Its tick: thrust (Thrust x throttle x ThrustCurve(speed along its up, km/h) x GroundeffectCurve(the
    // 10 m ray's distance, cm)) along its up, and the drag term along its nose - while controlled.
    double throttle = controlled ? quad::Clamp(in.throttle, 0, 1) : 0;
    k.force = 0;
    if (controlled) {
        Quat qb = ToQuat(CompRot(root));
        V3 up = Rotate(qb, {0, 0, 1}), fwd = Rotate(qb, {1, 0, 0});
        V3 b = CompLoc(root);
        Hit g = Sweep(b, b - V3{0, 0, 1000}, {}, {}, ignore);
        double groundCm = g.hit ? Len(g.impact - b) : 0;
        double kmh = Dot(v, up) * 0.01 * 3.6;
        double force = k.p.thrust * 100 * throttle * kamikaze::Eval(kamikaze::kThrustCurve, kmh) * kamikaze::Eval(kamikaze::kGroundCurve, groundCm);
        double vx = v.x * 0.001;
        V3 f = up * force + fwd * (k.p.dragK * vx * vx);
        Params af(root, "AddForce");
        af.Set("Force", f).Set("BoneName", uint64_t(0)).Set("bAccelChange", false).Call();
        k.force = force / 100;
        q.groundHeight = g.hit ? groundCm / 100 : 99;
    }
    k.curThrottle += (throttle - k.curThrottle) * std::min(1.0, dt * 2);  // CurThrottle, FInterpTo at 2 (the sound)
    for (auto& m : q.m) { m.speed = k.curThrottle; m.thrust = k.force / 4; }
    q.bat.soc = k.charge;
    q.bat.volts = q.p.cells * (3.2 + 1.0 * k.charge);
    q.bat.used = q.p.capacity * (1 - k.charge);
    q.bat.amps = 0.4 + 60 * std::pow(k.curThrottle, 1.5);
}

// A hand takes the drone by its grenade, and by a box the drone's size round it (colliding as the
// grenade does), on the drone.
static void AddGrabBox(Drone& d) {
    Object a(d.actor), root = a ? a.GetObj("RootComponent") : Object();
    Object box = root && d.root ? AddComp(a, "BoxComponent", d.root) : Object();
    if (!box) return;
    V3 b = Box();
    Params e(box, "SetBoxExtent");
    e.Set("InBoxExtent", b).Set("bUpdateOverlaps", true).Call();
    Params pn(root, "GetCollisionProfileName");
    pn.Call();
    Params sp(box, "SetCollisionProfileName");
    sp.Set("InCollisionProfileName", pn.Return<uint64_t>()).Set("bUpdateOverlaps", true).Call();
    Params qo(box, "SetCollisionEnabled");  // for a hand to find, never pushing anything (a physics box this big shoves off what it overlaps)
    qo.Set("NewType", (uint8_t)1).Call();  // QueryOnly
}

// The M67 one of our pouches spawned (or took back) becomes the drone: the grenade hidden (its mesh, pin,
// pin ring and spoon; the pin can't be taken, so its fuse never burns) at the drone's bomb, the drone on
// it. Its grip, storing, physics and explosion stay the grenade's. The first one is the one you fly; with
// that one about, a new one waits, built, in its pouch.
// The grenade under a drone: its own meshes hidden, its pin kept for its physics (the grenade hangs it on a
// constraint when held) but no hand may take it, so its fuse never burns.
static void PrepItem(Object a) {
    for (const char* c : {"StaticMeshComponent", "Pin", "PinBase", "Spoon"}) SetVisible(a.GetObj(c), false);
    if (Object pin = a.GetObj("Pin")) {
        GML_PropInfo pi, di;
        uint8_t* s = (uint8_t*)pin.PropPtr("VRGripInterfaceSettings");
        if (s && API->GetPropertyInfo(API->FindProperty(pin.Struct(), "VRGripInterfaceSettings"), &pi) && pi.structType &&
            API->GetPropertyInfo(API->FindProperty(pi.structType, "bDenyGripping"), &di))
            s[di.offset] = 1;
        pin.SetBool("bPinCanExit", false);
    }
    if (Object st = GetComp(a, "BP_StorableComponent_C"))
        if (s_droneIcon) st.Set("Icon", s_droneIcon);
}

// Another player's drone (multiplayer): on this machine only its look, on their grenade. Their own machine
// flies it; the host moves its grenade (see "multiplayer") and the game moves it everywhere else.
static void DressRemote(Object a, Object pouch, Object pilot) {
    if (!a || IsDroneActor(a.ptr) || !BuildAssets() || !a.GetObj("RootComponent")) return;
    PrepItem(a);
    Drone d;
    BuildLook(d, a, true);
    d.pouch = pouch.ptr;
    d.pilot = pilot.ptr;
    d.lastSeenAt = CompLoc(d.root);
    d.sound = AddSound(a, d.root);  // its motors, while it moves (TrackDrones)
    AddGrabBox(d);  // their hand finds it on the host too
    s_remote.push_back(std::move(d));
    Log("{}'s drone dressed on {}", pilot.Name(), a.Name());
}

static void DressItem(Object a, Object pouch) {
    if (!a || IsDroneActor(a.ptr) || !BuildAssets()) return;
    Object root = a.GetObj("RootComponent");
    if (!root) return;
    PrepItem(a);
    Drone flying;
    bool park = DroneValid();
    if (park) {
        flying = std::move(s_drone);
        s_drone = Drone{};
        s_drone.rt = flying.rt;
    }
    BuildDrone(a, true);
    s_drone.pouch = pouch.ptr;
    s_drone.itemMesh = root.Get<GUObject*>("StaticMesh");  // the grenade as it is, for when it is taken up again
    { Params m(root, "GetMass"); m.Call(); s_drone.itemMass = m.Return<float>(); }
    { Params l(root, "GetLinearDamping"); l.Call(); s_drone.itemLinDamp = l.Return<float>(); }
    { Params g(root, "GetAngularDamping"); g.Call(); s_drone.itemAngDamp = g.Return<float>(); }
    AddGrabBox(s_drone);
    if (!park) return;
    Park(s_drone);
    s_parked.push_back(std::move(s_drone));
    s_drone = std::move(flying);
}

// The player whose vest a pouch is on (its owner, else what it hangs on).
static Object PouchPilot(Object pouch) {
    Object o = pouch;
    for (int guard = 0; guard < 6 && o; guard++) {
        if (o.IsA("Pawn")) return o;
        Params go(o, "GetOwner");
        go.Call();
        Object owner = go.Return<GUObject*>();
        if (!owner) {
            Params ap(o, "GetAttachParentActor");
            ap.Call();
            owner = ap.Return<GUObject*>();
        }
        o = owner;
    }
    return {};
}

// Pouch_C's CheckSpawnStorable, after it ran: ours has its grenade pouch hidden and its new drone dressed (in
// multiplayer, on the host, a pouch may be another player's: their drone gets its look, see DressRemote).
static void OnPouchFilled(Object pouch) {
    if (!IsDronePouch(pouch)) return;
    SetVisible(pouch.GetObj("StaticMesh"), false);
    Object item = PouchItem(pouch);
    if (!item || !Alive(item.ptr)) return;
    Object pilot = PouchPilot(pouch);
    if (pilot && pilot.ptr != PlayerPawn().ptr) DressRemote(item, pouch, pilot);
    else DressItem(item, pouch);
}

// The drone pouches on the vest (the gear manager's Pouches).
static std::vector<Object> DronePouches() {
    std::vector<Object> out;
    Object gm = GearManager(PlayerPawn());
    auto* arr = gm ? (GML_TArray*)gm.PropPtr("Pouches") : nullptr;
    for (int i = 0; arr && i < arr->Num; i++) {
        Object p(((GUObject**)arr->Data)[i]);
        if (p && Alive(p.ptr) && IsDronePouch(p)) out.push_back(p);
    }
    return out;
}

static void DestroyDrone() {
    if (DroneValid()) {
        Params d(Object(s_drone.actor), "K2_DestroyActor");
        d.Call();
    }
    GUObject* rt = s_drone.rt;
    s_drone = Drone{};
    s_drone.rt = rt;  // the render target is reused
}

// Each tick: the parked drones that are gone go from the list, those on the vest keep their seat, and one
// that a hand takes becomes the one you fly (the one flying till then is parked in its stead). A drone
// pouch whose drone the hooks missed is dressed here.
static void TrackDrones(double dt) {
    std::erase_if(s_parked, [](const Drone& p) { return !Alive(p.actor); });
    for (size_t i = 0; i < s_parked.size(); i++) {
        Drone& p = s_parked[i];
        Object a(p.actor);
        int carry = Carried(a);
        if (carry == 1) UpdateSeat(p, a.GetObj("RootComponent"), 1);
        else if (carry == 0 && p.root) p.lastAtt = ToQuat(CompRot(p.root));  // lying about: as a hand will find it
        if (carry != 2) continue;
        GUObject* rt = s_drone.rt;
        std::swap(s_drone, p);
        s_drone.rt = rt;
        s_drone.carry = -1;  // Fly takes it from here, as just taken
        if (Alive(p.actor)) Park(p);
        else s_parked.erase(s_parked.begin() + i);
        Log("flying the drone just taken ({} waiting)", s_parked.size());
        break;
    }
    // Other players' drones: seated on their pouches as theirs are, their propellers turning while they move.
    std::erase_if(s_remote, [](const Drone& p) { return !Alive(p.actor); });
    for (Drone& p : s_remote) {
        Object a(p.actor);
        int carry = Carried(a);
        if (carry != p.carry) p.carry = carry, p.carryFor = 0;
        else p.carryFor += dt;
        if (carry) p.hostFlown = false;  // in a hand or its pouch again: the game moves it
        UpdateSeat(p, a.GetObj("RootComponent"), carry);
        if (carry == 0 && p.root) p.lastAtt = ToQuat(CompRot(p.root));
        V3 at = p.root ? CompLoc(p.root) : V3{};
        double speed = dt > 0 ? Len(at - p.lastSeenAt) / dt : 0;  // cm/s
        bool moving = carry == 0 && speed > 50;  // over 0.5 m/s
        p.lastSeenAt = at;
        p.stillFor = moving ? 0 : p.stillFor + dt;
        if (p.sound) {  // its motors as it flies: their blade-pass pitch rises with its speed
            bool on = carry == 0 && p.stillFor < 1.0;
            double s = Clamp(0.35 + speed / 4000, 0, 1);
            float semis = (float)(12 * std::log2((300 + 700 * s) / 261.63));
            Call2(p.sound, "SetOscSemitones", "OscIndex", (int32_t)0, "Semitones", semis);
            Call2(p.sound, "SetOscSemitones", "OscIndex", (int32_t)1, "Semitones", semis);
            Call1(p.sound, "SetVolumeMultiplier", "VolumeMultiplier", (float)(on ? s_cfgVolume.Value() * (0.25 + 0.75 * s) : 0.0));
        }
        if (!moving) continue;
        for (int i = 0; i < 4; i++) {
            p.propAngle[i] = std::fmod(p.propAngle[i] + (i % 2 ? 1 : -1) * 2400 * dt, 360.0);
            if (p.propPivots[i]) SetRelRot(p.propPivots[i], {0, p.propAngle[i], 0});
        }
    }
    static double checkT = 0;
    if ((checkT += dt) < 0.5) return;
    checkT = 0;
    if (!NetClient()) KeepPouchTags();
    for (Object pouch : DronePouches()) {
        Object item = PouchItem(pouch);
        if (item && Alive(item.ptr) && !IsDroneActor(item.ptr)) OnPouchFilled(pouch);
    }
    // Multiplayer: every other player's drone pouches (their gear managers' Pouches, replicated), theirs dressed.
    Object me = PlayerPawn();
    std::erase_if(s_remoteGog, [](GUObject* g) { return !Alive(g); });
    for (auto& gm : FindAllOf("BP_GearManagerComponent_C")) {
        Object pawn(API->GetOuter(gm.ptr));
        if (!pawn || pawn.ptr == me.ptr || !pawn.IsA("Pawn")) continue;
        Object nvg = gm.GetObj("NVG");  // their goggles, on their helmet
        if (nvg && Alive(nvg.ptr) && !s_remoteGog.count(nvg.ptr) && WearsGoggles(gm)) Dress(nvg, pawn, false);
        auto* arr = (GML_TArray*)gm.PropPtr("Pouches");
        for (int i = 0; arr && i < arr->Num; i++) {
            Object p(((GUObject**)arr->Data)[i]);
            if (!p || !Alive(p.ptr) || !IsDronePouch(p)) continue;
            SetVisible(p.GetObj("StaticMesh"), false);
            Object item = PouchItem(p);
            if (item && Alive(item.ptr) && !IsDroneActor(item.ptr)) DressRemote(item, p, pawn);
        }
    }
}

static GUObject* RenderTarget() {
    if (s_drone.rt && API->IsValid(s_drone.rt)) return s_drone.rt;
    Params p(Lib("KismetRenderingLibrary"), "CreateRenderTarget2D");
    p.SetObj("WorldContextObject", Object(API->WorldContext())).Set("Width", (int32_t)s_cfgResX.Value()).Set("Height", (int32_t)s_cfgResY.Value())
        .Set("Format", (uint8_t)(s_cfgCaptureHDR.Value() ? 6 : 3)).Set("ClearColor", Color{0, 0, 0, 1})
        .Set("bAutoGenerateMipMaps", false).Set("bSupportUAVs", false).Call();
    s_drone.rt = p.Return<GUObject*>();
    if (s_drone.rt) {
        API->KeepAlive(s_drone.rt);
        if (s_midScreen) API->SetMaterialTexture(s_midScreen, "BaseColorTexture", s_drone.rt);
    }
    return s_drone.rt;
}

static void Alert(Drone& d, const char* w, double secs = 1.5) { d.warn = w; d.warnFor = secs; }
static bool s_everArmed = false;  // until the first arm, the OSD says how

// ---- sound
//
// The game has no drone sound, so a small synth (the engine's ModularSynthComponent) rides on the drone:
// a saw and a square, slightly detuned, through a low-pass, at the propellers' blade-pass frequency
// (two blades: about 300 Hz at idle to 1 kHz flat out on a small quad), spatialised with the game's
// small-vehicle falloff.
template <class T> static void Call1(Object o, const char* fn, const char* p, T v) { Params c(o, fn); c.Set(p, v).Call(); }
template <class A, class B> static void Call2(Object o, const char* fn, const char* p, A a, const char* q, B b) {
    Params c(o, fn);
    c.Set(p, a).Set(q, b).Call();
}
static Object AddSound(Object actor, Object parent) {
    if (!FindClass("ModularSynthComponent")) return {};
    Object s = AddComp(actor, "ModularSynthComponent", parent);
    if (!s) return {};
    if (Object att = LoadObject("/Game/Audio/Helicopter/ATT_Small.ATT_Small")) s.Set("AttenuationSettings", att.ptr);
    s.SetBool("bAllowSpatialization", true);
    Call1(s, "SetEnablePolyphony", "bEnablePolyphony", false);
    Call2(s, "SetOscType", "OscIndex", (int32_t)0, "OscType", (uint8_t)1);  // saw
    Call2(s, "SetOscType", "OscIndex", (int32_t)1, "OscType", (uint8_t)3);  // square
    Call2(s, "SetOscGain", "OscIndex", (int32_t)0, "OscGain", 0.7f);
    Call2(s, "SetOscGain", "OscIndex", (int32_t)1, "OscGain", 0.3f);
    Call2(s, "SetOscCents", "OscIndex", (int32_t)1, "Cents", 18.0f);
    Call1(s, "SetFilterType", "FilterType", (uint8_t)0);  // low-pass
    Call1(s, "SetFilterFrequency", "FilterFrequencyHz", 3200.0f);
    Call1(s, "SetFilterQ", "FilterQ", 1.2f);
    Call1(s, "SetAttackTime", "AttackTimeMsec", 40.0f);
    Call1(s, "SetSustainGain", "SustainGain", 1.0f);
    Call1(s, "SetReleaseTime", "ReleaseTimeMsec", 250.0f);
    Call1(s, "SetGainDb", "GainDb", -8.0f);
    Call1(s, "SetVolumeMultiplier", "VolumeMultiplier", 0.0f);
    Params st(s, "Start"); st.Call();
    Params on(s, "NoteOn"); on.Set("Note", 60.0f).Set("Velocity", (int32_t)100).Set("Duration", -1.0f).Call();  // C4, held
    return s;
}
static void UpdateSound(Drone& d) {
    if (!d.sound) return;
    double speed = 0;
    for (auto& m : d.q.m) speed += m.speed / 4;  // 0..1 of full rpm
    double hz = 300 + 700 * speed;
    float semis = (float)(12 * std::log2(hz / 261.63));
    Call2(d.sound, "SetOscSemitones", "OscIndex", (int32_t)0, "Semitones", semis);
    Call2(d.sound, "SetOscSemitones", "OscIndex", (int32_t)1, "Semitones", semis);
    Call1(d.sound, "SetVolumeMultiplier", "VolumeMultiplier", (float)(speed > 0.01 ? s_cfgVolume.Value() * (0.25 + 0.75 * speed) : 0.0));
}

static quad::Rates ParseRates(const std::string& s, quad::Rates def) {
    quad::Rates r = def;
    sscanf_s(s.c_str(), "%lf , %lf , %lf", &r.center, &r.max, &r.expo);
    r.expo = Clamp(r.expo, 0, 1);
    return r;
}

// A fresh drone (full battery, payload) in place of the one flying: in the pouch that one came from, or
// an empty drone pouch, or the first one (its drone goes); the pouch spawns it as it does at the start
// (CheckSpawnStorable) and OnPouchFilled dresses it. Without a drone pouch on the vest, on the ground in
// front of you.
static void SpawnDroneBare();
static void SpawnDrone() {
    if (!BuildAssets()) return;
    if (NetClient()) {  // the host spawns it into a drone pouch of ours, and this plugin dresses it when it arrives
        if (DronePouches().empty()) { Log("no drone pouch on the vest - in multiplayer a drone comes only into one"); return; }
        SendToHost(MsgFresh, {}, {});
        GUObject* rt = s_drone.rt;  // the host replaces the one flying
        s_drone = Drone{};
        s_drone.rt = rt;
        Log("a fresh drone asked of the host");
        return;
    }
    std::vector<Object> pouches = DronePouches();
    Object pouch;
    for (Object p : pouches)
        if (p.ptr == s_drone.pouch) pouch = p;
    for (Object p : pouches) {
        Object item = PouchItem(p);
        if (!pouch && !(item && Alive(item.ptr))) pouch = p;
    }
    if (!pouch && !pouches.empty()) pouch = pouches[0];
    if (!pouch) { SpawnDroneBare(); return; }
    DestroyDrone();
    Object old = PouchItem(pouch);
    if (old && Alive(old.ptr)) {  // a parked one in that pouch
        std::erase_if(s_parked, [&](const Drone& p) { return p.actor == old.ptr; });
        Params d(old, "K2_DestroyActor");
        d.Call();
    }
    pouch.Set("Item", (GUObject*)nullptr);
    Params c(pouch, "CheckSpawnStorable");
    c.Call();
    Log("a fresh drone in its pouch on the vest");
}
static void SpawnDroneBare() {
    if (!BuildAssets()) return;
    Object pawn = PlayerPawn();
    Object cam = VRCamera(pawn);
    if (!pawn || !cam) return;
    V3 eye = CompLoc(cam);
    double yaw = CompRot(cam).yaw;
    V3 fwd{std::cos(yaw * kDeg), std::sin(yaw * kDeg), 0};
    V3 at = eye + fwd * 120;
    Hit g = Sweep(at + V3{0, 0, 30}, at - V3{0, 0, 400}, {}, {}, {pawn.ptr});
    bool onGround = g.hit;
    double half = std::max(2.0, -(s_cfgPayload.Value() ? s_payloadBottom : s_bottom));  // on its legs, or on the bomb
    at.z = (onGround ? g.impact.z + half + 0.3 : eye.z) + s_bombPos.z;  // the grenade under it at its bomb
    DestroyDrone();
    // On an M67 as from a pouch (with no pouch): the same item, the same flight.
    GUClass* cls = LoadClass(kGrenadeClass);
    auto xf = Transform(at, {0, yaw, 0});
    Params b(Lib("GameplayStatics"), "BeginDeferredActorSpawnFromClass");
    b.SetObj("WorldContextObject", Object(API->WorldContext())).Set("ActorClass", cls).SetObj("Owner", pawn);
    std::memcpy(b.Ptr("SpawnTransform"), xf.data(), xf.size());
    b.Set("CollisionHandlingOverride", (uint8_t)1).Call();
    Object a = cls ? b.Return<GUObject*>() : nullptr;
    if (!a) { Error("could not spawn the drone"); return; }
    Params f(Lib("GameplayStatics"), "FinishSpawningActor");
    f.SetObj("Actor", a);
    std::memcpy(f.Ptr("SpawnTransform"), xf.data(), xf.size());
    f.Call();
    DressItem(a, Object());
    Log("drone on the {} in front of you", onGround ? "ground" : "air");
}

// Everything that makes an actor the drone: its meshes, the FPV camera, the sound, the payload and the
// flight model, starting where the actor is. One on an item (the M67 from a drone pouch, see DressItem)
// keeps the grenade as its root, at the drone's bomb, and the drone hangs on it at its own scale.
// What the drone looks like, on an actor: its frame, camera, propellers and payload. On an item (the M67 from a
// drone pouch) the grenade is its root, at the drone's bomb, and the drone hangs on it at its own scale.
static void BuildLook(Drone& d, Object a, bool item) {
    d.actor = a.ptr;
    d.item = item;
    d.root = AddComp(a, "SceneComponent", item ? a.GetObj("RootComponent") : Object());
    if (item && d.root) {
        Params ab(d.root, "SetAbsolute");
        ab.Set("bNewAbsoluteLocation", false).Set("bNewAbsoluteRotation", false).Set("bNewAbsoluteScale", true).Call();
        Params sc(d.root, "SetWorldScale3D");
        sc.Set("NewScale", V3{1, 1, 1}).Call();
        d.anchor = s_bombPos;
        PlaceOnItem(d);
    }
    d.body = AddMesh(a, s_meshBody, d.root);
    d.camPivot = AddComp(a, "SceneComponent", d.root, s_camPos);
    d.camMesh = AddMesh(a, s_meshCamera, d.camPivot);
    for (int i = 0; i < 4; i++) {
        d.propPivots[i] = AddComp(a, "SceneComponent", d.root, s_propPos[i]);
        d.props[i] = AddMesh(a, s_meshProps[i], d.propPivots[i]);
    }
    // The payload: the release under the body and the bomb in it.
    d.payload = s_cfgPayload.Value() && s_meshBomb;
    if (d.payload) {
        d.release = AddMesh(a, s_meshRelease, d.root);
        d.bomb = AddMesh(a, s_meshBomb, d.root, s_bombPos);
    }
}

static void BuildDrone(Object a, bool item) {
    Drone& d = s_drone;
    BuildLook(d, a, item);
    d.sound = AddSound(a, d.root);
    // The FPV camera: fixed to the frame with an up-tilt, the lens 1 cm ahead of the camera's pivot.
    d.capture = AddComp(a, "SceneCaptureComponent2D", d.camPivot, {1.2, 0, 0});
    if (d.capture) {
        d.capture.Set("FOVAngle", s_cfgCamFov.Value());
        d.capture.Set("CaptureSource", (uint8_t)(s_cfgCaptureHDR.Value() ? 0 : 2));  // SceneColorHDR / FinalColorLDR
        d.capture.SetBool("bCaptureEveryFrame", false);
        d.capture.SetBool("bCaptureOnMovement", false);
        d.capture.Set("bAlwaysPersistRenderingState", true);  // exposure and anti-aliasing carry over frames
        d.capture.Set("TextureTarget", RenderTarget());
        Params h(d.capture, "HideActorComponents"); h.SetObj("InActor", a).Set("bIncludeFromChildActors", true).Call();
    }
    // The quad: built from the [Build] parts with the [Physics] settings, its motors where the model's
    // propellers are (spread to the frame its props need).
    quad::Quad& q = d.q;
    quad::Build b;
    b.propInches = Clamp(s_cfgProp.Value(), 2, 15);
    b.pitchInches = Clamp(s_cfgPitch.Value(), 1, 10);
    b.kv = Clamp(s_cfgKV.Value(), 100, 10000);
    b.cells = std::clamp(s_cfgCells.Value(), 1, 12);
    b.capacity = std::max(100.0f, s_cfgCapacity.Value());
    b.grams = Clamp(s_cfgGrams.Value(), 50, 20000);
    b.minThrottle = s_cfgMinThrottle.Value();
    b.gravity = Clamp(s_cfgGravity.Value(), 0, 50);
    b.instantPower = s_cfgInstantPower.Value();
    b.airFriction = std::max(0.0f, s_cfgAirFriction.Value());
    b.airGrip = std::max(0.0f, s_cfgAirGrip.Value());
    b.relativeAirSpeed = std::max(0.0f, s_cfgRelAirSpeed.Value());
    q.p = quad::FromBuild(b);
    quad::V3 motors[4];
    for (int i = 0; i < 4; i++) motors[i] = ToM(s_propPos[i]);
    q.Setup(motors);
    q.fc.rollRates = q.fc.pitchRates = ParseRates(s_cfgRatesRP.Value(), {30, 220, 0.3});
    q.fc.yawRates = ParseRates(s_cfgRatesYaw.Value(), {30, 160, 0.2});
    q.fc.smoothing = Clamp(s_cfgSmoothing.Value(), 0, 0.5);
    q.fc.angleMode = s_cfgMode.Value() == "Angle";
    q.pos = ToM(CompLoc(d.root));
    Quat qa = ToQuat(CompRot(d.root));
    q.att = {qa.x, qa.y, qa.z, qa.w};
    d.lastAtt = qa;
    d.tilt = s_cfgUptilt.Value();
    d.home = q.pos;
    d.lastArmSwitch = s_armSwitch;
    if (AutoArm()) q.Arm(true);  // live straight away: zero throttle keeps the motors stopped
    if (Kamikaze()) {
        // FPV Kamikaze Drone's drone, its own numbers (kamikaze.h): not settings, and a warhead adds nothing
        // to them there, so the payload doesn't here.
        kamikaze::Drone& k = d.k;
        k = kamikaze::Drone{};
        d.batteryTick = 1;  // ConsumeBattery: once a second from the start
        double m = k.p.mass, x = 0.214, y = 0.222, z = 0.038;
        q.p.mass = m;
        q.p.inertia = {m / 12 * (y * y + z * z), m / 12 * (x * x + z * z), m / 12 * (x * x + y * y)};
        q.p.gravity = k.p.gravity;
    }
    V3 at = CompLoc(d.root);
    Log("drone spawned {} ({:.0f}, {:.0f}, {:.0f}): {:.0f}x{:.0f}\" props, {:.0f} KV, {}S {:.0f} mAh, {:.0f} g, thrust/weight {:.1f}, {}, {}",
        item ? "on the vest" : "bare", at.x, at.y, at.z, b.propInches, b.pitchInches, b.kv, q.p.cells, q.p.capacity, b.grams,
        q.p.thrustToWeight, Kamikaze() ? "FPV Kamikaze Drone model" : q.fc.angleMode ? "angle" : "acro", AutoArm() ? "live" : "disarmed");
    if (d.payload) Log("payload on: {} x M67, {:.0f} g all up, goes off above {:.0f} km/h", s_cfgGrenades.Value(), q.p.mass * 1000, s_cfgDetonateSpeed.Value());
}

// ---- flight

static bool TryArm(Drone& d) {
    quad::Quad& q = d.q;
    double r, p;
    q.Tilt(r, p);
    // Betaflight's arming checks
    const char* no = d.rxLoss ? "RXLOSS" : s_sticks.throttle > 0.05 ? "THROTTLE" : std::fabs(r) > 25 * kDeg || std::fabs(p) > 25 * kDeg ? "ANGLE" : nullptr;
    if (no) {
        Alert(d, no, 2.5);
        Log("arming refused: {} (input {}, throttle {:.2f}, roll {:.0f}, pitch {:.0f})", no, s_inputName, s_sticks.throttle, r / kDeg, p / kDeg);
        return false;
    }
    s_everArmed = true;
    q.Arm(true);
    d.crashed = false;
    d.home = q.pos;
    d.fullAtArm = q.bat.soc > 0.97;
    Alert(d, "ARMED", 1.2);
    Log("armed ({:.2f} V)", q.bat.volts);
    return true;
}
static void Disarm(Drone& d, const char* why) {
    if (!d.q.armed) return;
    d.q.Arm(false);
    Log("disarmed ({})", why);
}

// One world collision pass over the step the physics just took: sweep the drone's box (turned as it is)
// from where it was, and turn every hit into an impulse at the point of contact.
static void Collide(Drone& d, quad::V3 from, const std::vector<GUObject*>& ignore) {
    quad::Quad& q = d.q;
    Rot rot = AttRot(q.att);
    V3 a = ToUE(from), b = ToUE(q.pos);
    // Where the impulse goes in. The engine reports one point of a contact; for a quad lying flat on the
    // ground that is one corner one frame and another the next, which would set it spinning in place, so a
    // flat contact pushes through the middle of the face it lies on.
    auto contact = [&](const Hit& h, quad::V3 n) {
        quad::V3 up = quad::Rotate(q.att, {0, 0, 1});
        if (std::fabs(quad::Dot(up, n)) > 0.85) return q.pos - n * (Box().z / 100);
        return ToM(h.impact);
    };
    for (int i = 0; i < 3; i++) {
        bool still = Len(b - a) < 0.01;  // not moving: just make sure it hasn't turned into something
        Hit h = Sweep(a, still ? a - V3{0, 0, 0.05} : b, Box(), rot, ignore);
        if (!h.hit) break;
        V3 n = Norm(h.normal);
        quad::V3 nm{n.x, n.y, n.z};
        if (h.startPenetrating) {  // out the way the surface faces, by as far as it is in
            q.pos = ToM(a) + nm * (h.depth / 100 + 0.0005);
            q.Impact(contact(h, nm), nm, 0.1, 0.6, 0.5);
            return;
        }
        if (still) break;
        q.pos = ToM(h.location) + nm * 0.0005;
        // FPV Kamikaze Drone: Unreal's default material (restitution 0.3, friction 0.7) on a rigid body, and no
        // crash without a warhead. Sim: a crash cuts the motors as Betaflight's crash detection does.
        double kmh = quad::Len(q.vel) * 3.6;
        double hitSpeed = Kamikaze() ? q.Impact(contact(h, nm), nm, 0.3, 0.7, 1) : q.Impact(contact(h, nm), nm, 0.15, 0.6, 0.5);
        // The payload's impact fuse, as FPV Kamikaze Drone's warheads: any hit while going faster than
        // DetonateSpeed (the drone's own speed, km/h) sets it off. (Our own pawn is never swept against.)
        if (d.payload && d.fuseArmed && hitSpeed > 0.05 && kmh > s_cfgDetonateSpeed.Value()) {
            d.detonate = true;
            d.detonateAt = h.impact;
            d.detonateNormal = n;
            Log("payload hit something at {:.0f} km/h", kmh);
            return;
        }
        if (hitSpeed > s_cfgCrashSpeed.Value() && q.armed && !Kamikaze()) {
            d.crashed = true;
            Disarm(d, std::format("crash at {:.1f} m/s", hitSpeed).c_str());
            Alert(d, "CRASH", 3);
        }
        // Slide the rest of the way along the surface.
        a = ToUE(q.pos);
        V3 rest = b - a;
        rest = rest - n * Dot(rest, n);
        b = a + rest;
        q.pos = ToM(b);
    }
}

// ---- the payload going off: the game's own M67 grenades, spawned just off what it hit and set off at once,
// each a moment after the one before. They belong to the player, so what they kill counts as theirs.
struct Boom { V3 at; double in; GUObject* owner; };
static std::vector<Boom> s_booms;

static void Grenade(V3 at, Object owner) {
    static GUClass* cls = nullptr;
    if (!cls) cls = LoadClass("/Game/Gameplay/Throwables/M67/BP_M67.BP_M67_C");
    if (!cls) { Error("the game's M67 grenade is missing"); return; }
    auto xf = Transform(at, {});
    Params b(Lib("GameplayStatics"), "BeginDeferredActorSpawnFromClass");
    b.SetObj("WorldContextObject", Object(API->WorldContext())).Set("ActorClass", cls).SetObj("Owner", owner);
    std::memcpy(b.Ptr("SpawnTransform"), xf.data(), xf.size());
    b.Set("CollisionHandlingOverride", (uint8_t)1).Call();
    Object g = b.Return<GUObject*>();
    if (!g) { Warn("could not spawn a grenade"); return; }
    Params f(Lib("GameplayStatics"), "FinishSpawningActor");
    f.SetObj("Actor", g);
    std::memcpy(f.Ptr("SpawnTransform"), xf.data(), xf.size());
    f.Call();
    Params e(g, "Explode");  // effects and sound here, then the server's radial damage
    e.Call();
}

// The payload of a drone on its item, at `at` (just off what it hit), n grenades in all: the first is the drone's
// own grenade, moved there and set off with the grenade's Explode (its damage, effects, light, AI noise,
// OnExplode); the drone is gone and the grenade removes itself after, as a thrown one does. The rest are
// spawned beside it, theirs to `owner`. Returns whether the drone's own went off.
static bool Payload(Drone& d, V3 at, int n, Object owner) {
    int own = 0;
    if (d.item && n > 0 && d.actor && Alive(d.actor) && Object(d.actor).Class() == LoadClass(kGrenadeClass)) {
        Object g(d.actor);
        if (d.root) { Params h(d.root, "SetVisibility"); h.Set("bNewVisibility", false).Set("bPropagateToChildren", true).Call(); }
        if (d.sound) { Params v(d.sound, "SetVolumeMultiplier"); v.Set("VolumeMultiplier", 0.0f).Call(); }
        Params mv(g, "K2_SetActorLocation");
        mv.Set("NewLocation", at).Set("bSweep", false).Set("bTeleport", true).Call();
        Params e(g, "Explode");
        e.Call();
        own = 1;
    }
    for (int i = own; i < n; i++) {
        double a = i * 2.39996;  // spread a little, around the first
        V3 off = i ? V3{std::cos(a) * 12, std::sin(a) * 12, 0} : V3{};
        s_booms.push_back({at + off, i * 0.12, owner.ptr});
    }
    return own != 0;
}

static void Detonate(Drone& d) {
    V3 at = d.detonateAt + d.detonateNormal * 15;  // just off the surface it hit
    int n = std::clamp(s_cfgGrenades.Value(), 0, 8);
    if (NetClient()) {  // the host sets it off: its grenade, its damage (see "multiplayer")
        if (d.root) { Params h(d.root, "SetVisibility"); h.Set("bNewVisibility", false).Set("bPropagateToChildren", true).Call(); }
        if (d.sound) { Params v(d.sound, "SetVolumeMultiplier"); v.Set("VolumeMultiplier", 0.0f).Call(); }
        SendToHost(MsgBoom, at, {0, 0, (double)n});
        Log("payload detonated: {} grenade(s) at ({:.0f}, {:.0f}, {:.0f}), set off by the host", n, at.x, at.y, at.z);
        GUObject* rt = s_drone.rt;  // the host's grenade goes by itself; the drone is forgotten
        s_drone = Drone{};
        s_drone.rt = rt;
        return;
    }
    bool own = Payload(d, at, n, PlayerPawn());
    Log("payload detonated: {} grenade(s) at ({:.0f}, {:.0f}, {:.0f}){}", n, at.x, at.y, at.z, own ? ", the first the drone's own" : "");
    if (own) {  // the grenade goes by itself (its DestroyDelay); the drone is forgotten
        GUObject* rt = s_drone.rt;
        s_drone = Drone{};
        s_drone.rt = rt;
    } else DestroyDrone();
}

static void TickBooms(double dt) {
    for (size_t i = 0; i < s_booms.size();) {
        if ((s_booms[i].in -= dt) <= 0) {
            Boom b = s_booms[i];
            s_booms.erase(s_booms.begin() + i);
            Grenade(b.at, b.owner && Alive(b.owner) ? Object(b.owner) : PlayerPawn());
        } else i++;
    }
}

static void Fly(double dt) {
    Drone& d = s_drone;
    quad::Quad& q = d.q;
    Object pawn = PlayerPawn();
    std::vector<GUObject*> ignore{d.actor};
    if (pawn) {  // you and what you wear (vest, pouches, helmet): it neither hits nor rests on its own pilot
        ignore.push_back(pawn.ptr);
        Params ga(pawn, "GetAttachedActors");
        ga.Set("bResetArray", true).Set("bRecursivelyIncludeAttachedActors", true).Call();
        if (auto* worn = (GML_TArray*)ga.Ptr("OutActors"))
            for (int i = 0; i < worn->Num; i++) ignore.push_back(((GUObject**)worn->Data)[i]);
    }
    d.sinceSpawn += dt;
    if (d.warnFor > 0) d.warnFor -= dt;
    // The payload's fuse is safe at your feet; it goes live once the drone has been ArmDistance from you.
    if (d.payload && !d.fuseArmed) {
        Object cam = VRCamera(pawn);
        if (cam && Len(ToUE(q.pos) - CompLoc(cam)) / 100 >= s_cfgArmDistance.Value()) {
            d.fuseArmed = true;
            Alert(d, "ARMED", 1.5);
            Log("payload armed ({:.0f} m out)", Len(ToUE(q.pos) - CompLoc(cam)) / 100);
        }
    }

    // --- the pilot
    if (Pressed(&Buttons::arm) && !AutoArm()) {
        if (q.armed) { Disarm(d, "button"); Alert(d, "DISARMED", 1.2); }
        else TryArm(d);
    }
    if (s_armSwitch >= 0 && s_armSwitch != d.lastArmSwitch && !AutoArm()) {  // a radio's arm switch, on its edges
        if (s_armSwitch == 1 && !q.armed) TryArm(d);
        if (s_armSwitch == 0 && q.armed) { Disarm(d, "switch"); Alert(d, "DISARMED", 1.2); }
        d.lastArmSwitch = s_armSwitch;
    }
    if (Pressed(&Buttons::mode) && !Kamikaze()) {  // (in FPV Kamikaze Drone, Y respawns: see Tick)
        q.fc.angleMode = !q.fc.angleMode;
        Alert(d, q.fc.angleMode ? "ANGLE MODE" : "ACRO MODE", 1.2);
    }
    if (Pressed(&Buttons::tiltUp)) d.tilt = Clamp(d.tilt + 5, -10, 60);
    if (Pressed(&Buttons::tiltDown)) d.tilt = Clamp(d.tilt - 5, -10, 60);
    quad::Inputs in{s_sticks.throttle, s_sticks.roll, s_sticks.pitch, s_sticks.yaw};
    if (!s_sticks.any) in = {};

    // --- the radio link: failsafe after 0.3 s without it, as a receiver does (motors to idle, then off).
    if (d.rxLoss) {
        in = {};
        if ((d.failsafeFor += dt) > 1.0 && q.armed) { Disarm(d, "failsafe"); }
    } else d.failsafeFor = 0;

    // --- on its item (from a drone pouch): held in a hand or stored in its pouch the game moves it (its camera still
    // live, its motors stopped); let go anywhere else, it flies on from there, at the speed it was let go.
    bool carried = false;
    if (d.item) {
        Object root = Object(d.actor).GetObj("RootComponent");
        auto syncFromActor = [&] {
            q.pos = ToM(CompLoc(d.root));
            Quat qa = ToQuat(CompRot(d.root));
            q.att = {qa.x, qa.y, qa.z, qa.w};
            q.vel = {};
            q.rate = {};
        };
        int carry = Carried(Object(d.actor));
        carried = carry != 0;
        if (carry != d.carry) d.carry = carry, d.carryFor = 0;
        else d.carryFor += dt;
        UpdateSeat(d, root, carry);
        // A client flies its drone here and sends the host where it is (see "multiplayer"): the host's copy of the
        // grenade's movement, coming back, is ignored while it flies, and followed again once it is carried.
        if (bool want = !carried && NetClient(); want != d.noRepMove) {
            Params r(Object(d.actor), "SetReplicateMovement");
            r.Set("bInReplicateMovement", !want).Call();
            d.noRepMove = want;
        }
        if (carried) {
            d.free = false;
            DropFlightBody(d);
            syncFromActor();
            d.k.held = false;
            d.k.curThrottle = 0;
            for (auto& m : q.m) { m.speed = 0; m.thrust = 0; }
        } else {
            bool sim = false;
            if (root) { Params s(root, "IsSimulatingPhysics"); s.Set("BoneName", uint64_t(0)).Call(); sim = s.Return<bool>(); }
            if (!d.free) {
                d.free = true;
                syncFromActor();
                if (root && sim) { Params v(root, "GetPhysicsLinearVelocity"); v.Set("BoneName", uint64_t(0)).Call(); q.vel = ToM(v.Return<V3>()); }
                if (double s = quad::Len(q.vel); s > 25) q.vel = q.vel * (25 / s);  // a throw, not a physics glitch
                Log("let go: flying from {:.1f} m/s{}", quad::Len(q.vel), Kamikaze() ? " on FPV Kamikaze Drone's body" : "");
                if (Kamikaze()) MakeFlightBody(d, ToUE(q.vel));
            }
            if (root && sim && !d.chaos) { Params p(root, "SetSimulatePhysics"); p.Set("bSimulate", false).Call(); }  // ours to move
        }
    }

    if (!carried && d.chaos) {  // FPV Kamikaze Drone: the engine flies it
        FlyChaos(d, in, dt, ignore);
        if (d.detonate) { Detonate(d); return; }  // the drone is gone
    } else if (!carried) {
        // --- ground below (ground effect): from its middle, 10 m down as FPV Kamikaze Drone's ray, m
        Hit below = Sweep(ToUE(q.pos), ToUE(q.pos) - V3{0, 0, 1000}, {}, {}, ignore);
        q.groundHeight = below.hit ? (ToUE(q.pos).z - below.impact.z) / 100 : 99;

        quad::V3 from = q.pos;
        if (Kamikaze()) {
            // --- FPV Kamikaze Drone's flight model, in steps of at most 1/240 s
            kamikaze::Sticks ks{in.throttle, in.roll, in.pitch, in.yaw};
            d.k.controlled = d.k.charge > 0 && q.armed;
            double groundCm = below.hit ? q.groundHeight * 100 : 0;
            int n = std::max(1, (int)std::ceil(dt * 240));
            for (int i = 0; i < n; i++) d.k.Step(q, ks, groundCm, dt / n);
            // The OSD's battery: a 6S pack's voltage and capacity along the charge.
            q.bat.soc = d.k.charge;
            q.bat.volts = q.p.cells * (3.2 + 1.0 * d.k.charge);
            q.bat.used = q.p.capacity * (1 - d.k.charge);
            q.bat.amps = 0.4 + 60 * std::pow(d.k.curThrottle, 1.5);
            if (d.k.charge <= 0 && q.armed) { Disarm(d, "battery flat"); Alert(d, "BATTERY EMPTY", 99); }
        } else {
            // --- physics at 1 kHz
            d.accum = std::min(d.accum + dt, 0.05);
            while (d.accum >= 0.001) { q.Step(in, 0.001); d.accum -= 0.001; }
        }
        // Crash flip (turtle mode): B held while it lies on its back, disarmed - two motors spin backwards and it
        // rolls over the edge that stays on the ground, onto its legs.
        quad::V3 up = quad::Rotate(q.att, {0, 0, 1});
        // On the ground: within reach of it, the reach being how far its box goes down as it is turned (tipped
        // on an edge, as a crash flip leaves it, that is the edge, not its underside).
        V3 bx = Box();
        double reach = (std::fabs(quad::Rotate(q.att, {1, 0, 0}).z) * bx.x + std::fabs(quad::Rotate(q.att, {0, 1, 0}).z) * bx.y +
                        std::fabs(up.z) * bx.z) / 100;
        bool touching = below.hit && q.groundHeight < reach + 0.04;
        if (s_buttons.flip && (!q.armed || Kamikaze()) && below.hit && (d.turtle || (q.groundHeight < 0.1 && up.z < -0.3))) {
            if (!d.turtle) d.turtleDir = quad::Rotate(q.att, {0, 1, 0}).z > 0 ? -1 : 1;  // the shorter way over
            d.turtle = true;
            // A corner of its box (s picks which), from its middle in world axes, and the corner lowest down.
            auto corner = [&](quad::Q a, quad::V3 s) { return quad::Rotate(a, {s.x * bx.x / 100, s.y * bx.y / 100, s.z * bx.z / 100}); };
            auto lowest = [](quad::Q a) {
                return quad::V3{quad::Rotate(a, {1, 0, 0}).z > 0 ? -1.0 : 1.0, quad::Rotate(a, {0, 1, 0}).z > 0 ? -1.0 : 1.0,
                                quad::Rotate(a, {0, 0, 1}).z > 0 ? -1.0 : 1.0};
            };
            quad::V3 pivot = lowest(q.att), before = corner(q.att, pivot);
            q.att = quad::Integrate(q.att, {6.0 * d.turtleDir, 0, 0}, dt);  // about 340 deg/s about its roll axis
            quad::V3 after = corner(q.att, pivot);
            q.pos.x += before.x - after.x;  // it turns about the edge on the ground
            q.pos.y += before.y - after.y;
            q.pos.z = below.impact.z / 100 - corner(q.att, lowest(q.att)).z + 0.002;
            q.vel = {};
            q.rate = {};
            Alert(d, "CRASH FLIP", 0.3);
            if (quad::Rotate(q.att, {0, 0, 1}).z > 0.9) d.turtle = false;  // on its legs: it settles below
        } else d.turtle = false;
        Collide(d, from, ignore);
        if (d.detonate) { Detonate(d); return; }  // the drone is gone

        // Resting: slow and on the ground with less thrust than its weight -> settle flat (on its legs or its back).
        double thrust = 0;
        for (auto& m : q.m) thrust += m.thrust;
        up = quad::Rotate(q.att, {0, 0, 1});
        bool resting = touching && thrust < q.p.mass * q.p.gravity * 0.9 && !d.turtle;
        if (resting) {  // sliding and rolling on the ground lose energy fast
            q.vel = {q.vel.x * std::exp(-6 * dt), q.vel.y * std::exp(-6 * dt), q.vel.z};
            q.rate = q.rate * std::exp(-8 * dt);
        }
        // Arming = Auto: after a crash or a failsafe it is live again once it has sat still on the ground, more or
        // less on its legs, for half a second, with the link back and the throttle down.
        bool still = q.groundHeight < 0.3 && quad::Len(q.vel) < 0.3 && quad::Len(q.rate) < 0.5 && !d.turtle;
        d.stillFor = still ? d.stillFor + dt : 0;
        if (AutoArm() && !q.armed && d.stillFor > 0.5 && up.z > 0.7 && !d.rxLoss && s_sticks.throttle < 0.05 && !(Kamikaze() && d.k.charge <= 0)) {
            q.Arm(true);
            d.crashed = false;
            Log("live again");
        }
        if (resting && quad::Len(q.vel) < 0.5 && quad::Len(q.rate) < 4) {
            q.vel = {};
            q.rate = {};
            q.fc.iterm = {};
            Rot r = AttRot(q.att);
            bool inverted = up.z < 0;
            double yaw = r.yaw * kDeg;
            quad::Q flat{0, 0, std::sin(yaw / 2), std::cos(yaw / 2)};
            if (inverted) flat = quad::Mul(flat, quad::Q{1, 0, 0, 0});  // on its back
            q.att = flat;
            q.pos.z = below.impact.z / 100 + Box().z / 100 + 0.002;
        }
    }

    // --- show it
    if (!carried && !d.chaos) {  // the actor, so that the drone on it (turned by its seat, the grenade at its bomb) is where the flight has it
        Quat att{q.att.x, q.att.y, q.att.z, q.att.w};
        Quat a = d.item ? Normalized(Mul(att, {-d.seat.x, -d.seat.y, -d.seat.z, d.seat.w})) : att;
        Params mv(Object(d.actor), "K2_SetActorLocationAndRotation");
        mv.Set("NewLocation", ToUE(q.pos) + Rotate(att, d.anchor)).Set("NewRotation", ToRot(a)).Set("bSweep", false).Set("bTeleport", true).Call();
    }
    d.lastAtt = {q.att.x, q.att.y, q.att.z, q.att.w};
    for (int i = 0; i < 4; i++) {
        double spin = q.m[i].speed * 2400;  // what the eye sees, not the real 30k rpm
        d.propAngle[i] = std::fmod(d.propAngle[i] + q.m[i].spin * -1 * spin * dt, 360);
        if (d.propPivots[i]) SetRelRot(d.propPivots[i], {0, d.propAngle[i], 0});
    }
    if (d.camPivot) SetRelRot(d.camPivot, {d.tilt, 0, 0});
    // Zoom (a gamepad's triggers): the same picture through a longer lens.
    double zoom = 1 + (std::max(1.0f, s_cfgZoom.Value()) - 1) * s_sticks.zoom;
    double fov = 2 * std::atan(std::tan(s_cfgCamFov.Value() * kDeg / 2) / zoom) / kDeg;
    if (d.capture && std::fabs(fov - d.fov) > 0.05) { d.capture.Set("FOVAngle", (float)fov); d.fov = fov; }
    if (q.armed) d.flyTime += dt;
    UpdateSound(d);

    // --- the links (10 times a second): RSSI falls with distance and walls between you and the drone
    static double linkT = 0;
    if ((linkT -= dt) <= 0) {
        linkT = 0.1;
        Object cam = VRCamera(pawn);
        V3 eye = cam ? CompLoc(cam) : ToUE(q.pos);
        V3 at = ToUE(q.pos);
        double dist = std::max(1.0, Len(at - eye) / 100);
        int walls = 0;
        V3 from2 = eye;
        for (int w = 0; w < 4; w++) {  // count what is in between
            Hit h = Sweep(from2, at, {}, {}, ignore);
            if (!h.hit) break;
            walls++;
            from2 = h.impact + Norm(at - eye) * 30;
            if (Dot(at - from2, at - eye) <= 0) break;
        }
        double rssi = -18 - 20 * std::log10(dist) - 8 * walls + q.Random() * 1.5;
        d.rssi += (rssi - d.rssi) * 0.5;
        double lq = Clamp(100 - (-100 - d.rssi) * 8.5, 0, 100);  // a long-range link: full LQ down to -100 dBm, gone by -112
        d.lq = std::round(lq);
        if (d.lq < 15) d.lostFor += 0.1; else d.lostFor = 0;
        if (!d.rxLoss && d.lostFor > 0.3) { d.rxLoss = true; Alert(d, "RXLOSS", 99); Log("link lost ({:.0f} dBm, {} walls)", d.rssi, walls); }
        if (d.rxLoss && d.lq > 30) { d.rxLoss = false; d.warnFor = 0; Log("link back"); }
        d.bitrate = Clamp((d.rssi + 108) * 0.9, 1.5, 50);
        d.latency = 24 + (d.rssi < -85 ? (-85 - d.rssi) * 1.5 : 0) + q.Random() * 2;
        d.videoLost = d.rssi < -106;
    }
}

// ------------------------------------------------------------------ the goggles' screen: Betaflight OSD on DJI goggles
//
// The OSD is Betaflight's look over the drone's video: white block letters with a black edge on a coarse
// grid (50 x 18 cells, like an HD OSD), in a pixel font of our own made below (5 x 7 glyphs, drawn
// fat), uploaded once as a texture and drawn glyph by glyph. Around it the goggles' own readouts in their
// smooth font: link, voltage, recording time, latency, bitrate, card.

// 5 x 7 glyphs ('#' lit). Icons sit on control codes.
enum : char { kIconSignal = 1, kIconBattery, kIconBolt, kIconTriangle, kIconClock, kIconCard, kIconCamera };
struct GlyphDef { char c; const char* rows[7]; };
static const GlyphDef kGlyphs[] = {
    {'0', {".###.", "#...#", "#..##", "#.#.#", "##..#", "#...#", ".###."}},
    {'1', {"..#..", ".##..", "..#..", "..#..", "..#..", "..#..", ".###."}},
    {'2', {".###.", "#...#", "....#", "...#.", "..#..", ".#...", "#####"}},
    {'3', {"#####", "...#.", "..#..", "...#.", "....#", "#...#", ".###."}},
    {'4', {"...#.", "..##.", ".#.#.", "#..#.", "#####", "...#.", "...#."}},
    {'5', {"#####", "#....", "####.", "....#", "....#", "#...#", ".###."}},
    {'6', {"..##.", ".#...", "#....", "####.", "#...#", "#...#", ".###."}},
    {'7', {"#####", "....#", "...#.", "..#..", ".#...", ".#...", ".#..."}},
    {'8', {".###.", "#...#", "#...#", ".###.", "#...#", "#...#", ".###."}},
    {'9', {".###.", "#...#", "#...#", ".####", "....#", "...#.", ".##.."}},
    {'A', {".###.", "#...#", "#...#", "#####", "#...#", "#...#", "#...#"}},
    {'B', {"####.", "#...#", "#...#", "####.", "#...#", "#...#", "####."}},
    {'C', {".###.", "#...#", "#....", "#....", "#....", "#...#", ".###."}},
    {'D', {"###..", "#..#.", "#...#", "#...#", "#...#", "#..#.", "###.."}},
    {'E', {"#####", "#....", "#....", "####.", "#....", "#....", "#####"}},
    {'F', {"#####", "#....", "#....", "####.", "#....", "#....", "#...."}},
    {'G', {".###.", "#...#", "#....", "#.###", "#...#", "#...#", ".####"}},
    {'H', {"#...#", "#...#", "#...#", "#####", "#...#", "#...#", "#...#"}},
    {'I', {".###.", "..#..", "..#..", "..#..", "..#..", "..#..", ".###."}},
    {'J', {"..###", "...#.", "...#.", "...#.", "...#.", "#..#.", ".##.."}},
    {'K', {"#...#", "#..#.", "#.#..", "##...", "#.#..", "#..#.", "#...#"}},
    {'L', {"#....", "#....", "#....", "#....", "#....", "#....", "#####"}},
    {'M', {"#...#", "##.##", "#.#.#", "#.#.#", "#...#", "#...#", "#...#"}},
    {'N', {"#...#", "#...#", "##..#", "#.#.#", "#..##", "#...#", "#...#"}},
    {'O', {".###.", "#...#", "#...#", "#...#", "#...#", "#...#", ".###."}},
    {'P', {"####.", "#...#", "#...#", "####.", "#....", "#....", "#...."}},
    {'Q', {".###.", "#...#", "#...#", "#...#", "#.#.#", "#..#.", ".##.#"}},
    {'R', {"####.", "#...#", "#...#", "####.", "#.#..", "#..#.", "#...#"}},
    {'S', {".####", "#....", "#....", ".###.", "....#", "....#", "####."}},
    {'T', {"#####", "..#..", "..#..", "..#..", "..#..", "..#..", "..#.."}},
    {'U', {"#...#", "#...#", "#...#", "#...#", "#...#", "#...#", ".###."}},
    {'V', {"#...#", "#...#", "#...#", "#...#", "#...#", ".#.#.", "..#.."}},
    {'W', {"#...#", "#...#", "#...#", "#.#.#", "#.#.#", "#.#.#", ".#.#."}},
    {'X', {"#...#", "#...#", ".#.#.", "..#..", ".#.#.", "#...#", "#...#"}},
    {'Y', {"#...#", "#...#", ".#.#.", "..#..", "..#..", "..#..", "..#.."}},
    {'Z', {"#####", "....#", "...#.", "..#..", ".#...", "#....", "#####"}},
    {'v', {".....", ".....", "#...#", "#...#", "#...#", ".#.#.", "..#.."}},
    {'m', {".....", ".....", "##.#.", "#.#.#", "#.#.#", "#...#", "#...#"}},
    {'h', {"#....", "#....", "#.##.", "##..#", "#...#", "#...#", "#...#"}},
    {'.', {".....", ".....", ".....", ".....", ".....", ".##..", ".##.."}},
    {':', {".....", ".##..", ".##..", ".....", ".##..", ".##..", "....."}},
    {'-', {".....", ".....", ".....", "#####", ".....", ".....", "....."}},
    {'%', {"##...", "##..#", "...#.", "..#..", ".#...", "#..##", "...##"}},
    {'<', {"...#.", "..#..", ".#...", "#....", ".#...", "..#..", "...#."}},
    {'>', {".#...", "..#..", "...#.", "....#", "...#.", "..#..", ".#..."}},
    {'/', {".....", "....#", "...#.", "..#..", ".#...", "#....", "....."}},
    {'!', {"..#..", "..#..", "..#..", "..#..", "..#..", ".....", "..#.."}},
    {'\'', {"..#..", "..#..", ".#...", ".....", ".....", ".....", "....."}},
    {'"', {".#.#.", ".#.#.", "#.#..", ".....", ".....", ".....", "....."}},
    {kIconSignal, {".....", "....#", "....#", "..#.#", "..#.#", "#.#.#", "#.#.#"}},
    {kIconBattery, {".###.", "#####", "#...#", "#...#", "#####", "#####", "#####"}},
    {kIconBolt, {"...#.", "..#..", ".#...", "#####", "...#.", "..#..", ".#..."}},
    {kIconTriangle, {".....", "..#..", "..#..", ".###.", ".###.", "#####", "#####"}},
    {kIconClock, {".###.", "#.#.#", "#.#.#", "#.###", "#...#", "#...#", ".###."}},
    {kIconCard, {"###..", "#.##.", "#...#", "#...#", "#...#", "#...#", "#####"}},
    {kIconCamera, {".....", "####.", "#..##", "#..##", "#..##", "####.", "....."}},
};

// The atlas: every glyph twice, big (3 px per dot, 2 px edge) and small (2 px per dot, 1 px edge), in a
// 16 x 8 grid of cells each.
static constexpr int kBigScale = 3, kBigEdge = 2, kSmallScale = 2, kSmallEdge = 1;
static constexpr int kBigW = 5 * kBigScale + 2 * kBigEdge, kBigH = 7 * kBigScale + 2 * kBigEdge;          // 19 x 25
static constexpr int kSmallW = 5 * kSmallScale + 2 * kSmallEdge, kSmallH = 7 * kSmallScale + 2 * kSmallEdge;  // 12 x 16
static constexpr int kAtlasW = 512, kAtlasH = 512, kSmallTop = 8 * kBigH;

// A PNG (RGBA, stored deflate: no zlib needed) from raw pixels.
static std::vector<uint8_t> EncodePng(const std::vector<uint32_t>& rgba, int w, int h) {
    auto crc = [](const uint8_t* p, size_t n, uint32_t c = 0xffffffffu) {
        for (size_t i = 0; i < n; i++) {
            c ^= p[i];
            for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xedb88320u & (0u - (c & 1)));
        }
        return c;
    };
    std::vector<uint8_t> raw;
    for (int y = 0; y < h; y++) {
        raw.push_back(0);  // no filter
        for (int x = 0; x < w; x++) {
            uint32_t p = rgba[y * w + x];
            raw.insert(raw.end(), {uint8_t(p), uint8_t(p >> 8), uint8_t(p >> 16), uint8_t(p >> 24)});
        }
    }
    std::vector<uint8_t> z{0x78, 0x01};
    for (size_t at = 0; at < raw.size();) {
        size_t n = std::min<size_t>(65535, raw.size() - at);
        z.push_back(at + n == raw.size() ? 1 : 0);
        z.insert(z.end(), {uint8_t(n), uint8_t(n >> 8), uint8_t(~n), uint8_t(~n >> 8)});
        z.insert(z.end(), raw.begin() + at, raw.begin() + at + n);
        at += n;
    }
    uint32_t a = 1, b = 0;
    for (uint8_t c : raw) { a = (a + c) % 65521; b = (b + a) % 65521; }
    uint32_t adler = (b << 16) | a;
    z.insert(z.end(), {uint8_t(adler >> 24), uint8_t(adler >> 16), uint8_t(adler >> 8), uint8_t(adler)});
    std::vector<uint8_t> png{0x89, 'P', 'N', 'G', 13, 10, 26, 10};
    auto chunk = [&](const char* type, const std::vector<uint8_t>& data) {
        uint32_t n = (uint32_t)data.size();
        png.insert(png.end(), {uint8_t(n >> 24), uint8_t(n >> 16), uint8_t(n >> 8), uint8_t(n)});
        std::vector<uint8_t> body(type, type + 4);
        body.insert(body.end(), data.begin(), data.end());
        png.insert(png.end(), body.begin(), body.end());
        uint32_t c = ~crc(body.data(), body.size());
        png.insert(png.end(), {uint8_t(c >> 24), uint8_t(c >> 16), uint8_t(c >> 8), uint8_t(c)});
    };
    chunk("IHDR", {uint8_t(w >> 24), uint8_t(w >> 16), uint8_t(w >> 8), uint8_t(w), uint8_t(h >> 24), uint8_t(h >> 16), uint8_t(h >> 8), uint8_t(h), 8, 6, 0, 0, 0});
    chunk("IDAT", z);
    chunk("IEND", {});
    return png;
}

static GUObject* OsdFont() {
    static GUObject* tex = nullptr;
    static bool tried = false;
    if (tried) return tex;
    tried = true;
    std::vector<uint32_t> px(kAtlasW * kAtlasH, 0);
    auto put = [&](int cell, int scale, int edge, int cw, int ch, int top, const GlyphDef& g) {
        int ox = (cell % 16) * cw, oy = top + (cell / 16) * ch;
        auto lit = [&](int x, int y) {  // in pixels within the glyph box
            int gx = (x - edge) / scale, gy = (y - edge) / scale;
            if (x < edge || y < edge || gx >= 5 || gy >= 7) return false;
            return g.rows[gy][gx] == '#';
        };
        for (int y = 0; y < ch; y++)
            for (int x = 0; x < cw; x++) {
                uint32_t& p = px[(oy + y) * kAtlasW + ox + x];
                if (lit(x, y)) { p = 0xffffffffu; continue; }
                for (int dy = -edge; dy <= edge && !p; dy++)
                    for (int dx = -edge; dx <= edge && !p; dx++)
                        if (lit(x + dx, y + dy)) p = 0xff000000u;  // black edge
            }
    };
    for (const GlyphDef& g : kGlyphs) {
        int cell = (uint8_t)g.c;
        if (cell >= 128) continue;
        put(cell, kBigScale, kBigEdge, kBigW, kBigH, 0, g);
        put(cell, kSmallScale, kSmallEdge, kSmallW, kSmallH, kSmallTop, g);
    }
    std::vector<uint8_t> png = EncodePng(px, kAtlasW, kAtlasH);
    tex = API->ImportTextureFromMemory(png.data(), png.size(), "osd_font");
    if (tex) API->KeepAlive(tex);
    else Error("could not build the OSD font");
    return tex;
}

static GUObject* Font() {
    static GUObject* f = nullptr;
    if (!f) {
        if (Object e = FindFirstOf("GameEngine")) f = e.Get<GUObject*>("MediumFont");
        if (!f) f = LoadObject("/Engine/EngineFonts/Roboto.Roboto").ptr;
    }
    return f;
}

// The OSD grid: 50 x 18 cells over the picture.
struct Osd {
    Object canvas;
    V2 size;
    double cw = 0, ch = 0;
    float bright = 1;
    void Glyph(char c, double x, double y, bool tiny) {
        if (c == ' ') return;
        GUObject* font = OsdFont();
        if (!font) return;
        int cell = (uint8_t)c & 127;
        double gw = tiny ? kSmallW : kBigW, gh = tiny ? kSmallH : kBigH;
        double u = (cell % 16) * gw / kAtlasW, v = ((tiny ? kSmallTop : 0) + (cell / 16) * gh) / kAtlasH;
        double k = size.y / 720.0;  // the glyphs are made for a 720-line picture
        Params p(canvas, "K2_DrawTexture");
        p.SetObj("RenderTexture", Object(font)).Set("ScreenPosition", V2{x, y}).Set("ScreenSize", V2{gw * k, gh * k})
            .Set("CoordinatePosition", V2{u, v}).Set("CoordinateSize", V2{gw / kAtlasW, gh / kAtlasH})
            .Set("RenderColor", Color{bright, bright, bright, 1}).Set("BlendMode", (uint8_t)2).Set("Rotation", 0.0f).Set("PivotPoint", V2{0.5, 0.5}).Call();
    }
    // Big text in cells from (col, row); each character centred in its cell.
    void Text(double col, double row, const std::string& s) {
        double k = size.y / 720.0;
        for (size_t i = 0; i < s.size(); i++) Glyph(s[i], (col + i) * cw + (cw - kBigW * k) / 2, row * ch + (ch - kBigH * k) / 2, false);
    }
    // Small text (units, labels), `line` 0 = top half of the cell, 1 = bottom half.
    void Small(double col, double row, const std::string& s, int line = 1) {
        double k = size.y / 720.0;
        for (size_t i = 0; i < s.size(); i++) Glyph(s[i], col * cw + i * kSmallW * k, row * ch + line * ch / 2 + (ch / 2 - kSmallH * k) / 2, true);
    }
    void Centre(double row, const std::string& s) { Text(25 - s.size() / 2.0, row, s); }
    // The goggles' own smooth text, at pixel fractions of the picture.
    void Dji(const std::string& s, double fx, double fy, double scale = 1.0, bool right = false) {
        Params p(canvas, "K2_DrawText");
        std::wstring w(s.begin(), s.end());
        double k = size.y / 720.0;
        p.SetObj("RenderFont", Object(Font())).SetString("RenderText", w).Set("ScreenPosition", V2{fx * size.x, fy * size.y})
            .Set("Scale", V2{1.9 * scale * k, 1.9 * scale * k}).Set("RenderColor", Color{bright, bright, bright, 1}).Set("Kerning", 0.0f)
            .Set("ShadowColor", Color{0, 0, 0, 1}).Set("ShadowOffset", V2{1, 1}).Set("bCentreX", false).Set("bCentreY", false)
            .Set("bOutlined", true).Set("OutlineColor", Color{0, 0, 0, 0.9f}).Call();
        (void)right;
    }
    void DjiIcon(char icon, double fx, double fy) { Glyph(icon, fx * size.x, fy * size.y, false); }
};

static std::string Fmt2(double v) { char b[16]; snprintf(b, sizeof b, "%.2f", v); return b; }
static std::string Fmt1(double v) { char b[16]; snprintf(b, sizeof b, "%.1f", v); return b; }
static std::string Clock(double secs, bool dji = false) {
    int s = (int)secs;
    char b[16];
    if (dji) snprintf(b, sizeof b, "%02d'%02d\"", s / 60, s % 60);
    else snprintf(b, sizeof b, "%02d:%02d", s / 60, s % 60);
    return b;
}

static void DrawOsd(GUObject* rt, bool signal) {
    Params b(Lib("KismetRenderingLibrary"), "BeginDrawCanvasToRenderTarget");
    b.SetObj("WorldContextObject", Object(API->WorldContext())).SetObj("TextureRenderTarget", Object(rt)).Call();
    Osd o;
    o.canvas = b.Get<GUObject*>("Canvas");
    o.size = b.Get<V2>("Size");
    o.cw = o.size.x / 50;
    o.ch = o.size.y / 18;
    o.bright = s_cfgOsdBright.Value();
    GML_PropInfo ci;
    API->GetPropertyInfo(API->FindProperty((GUStruct*)API->FindFunction(Lib("KismetRenderingLibrary").Struct(), "BeginDrawCanvasToRenderTarget"), "Context"), &ci);
    std::vector<uint8_t> ctx((uint8_t*)b.Ptr("Context"), (uint8_t*)b.Ptr("Context") + ci.size);
    if (o.canvas) {
        const Drone& d = s_drone;
        if (!signal) {
            o.Dji("No Signal", 0.43, 0.47, 1.4);
        } else if (s_cfgOsd.Value()) {
            const quad::Quad& q = d.q;
            char t[32];
            // Betaflight elements, where the screenshot of a DJI O3 has them.
            snprintf(t, sizeof t, "%c%d", kIconSignal, (int)std::lround(d.rssi));
            o.Text(10, 1.5, t);
            o.Small(10, 2.8, "LQ", 0);
            o.Glyph(kIconSignal, 10 * o.cw, (2.8 + 0.5) * o.ch, true);
            snprintf(t, sizeof t, "2:%d", (int)d.lq);
            o.Text(11, 2.8, t);
            o.Text(35, 1.5, Fmt2(q.bat.amps));
            o.Small(35.0 + (double)Fmt2(q.bat.amps).size(), 1.5, "A");
            o.Small(35, 2.8, "TH", 0);
            o.Small(35, 2.8, "%", 1);
            snprintf(t, sizeof t, "%3d", (int)std::lround(s_sticks.throttle * 100));
            o.Text(37, 2.8, t);
            snprintf(t, sizeof t, "%d", (int)std::lround(q.bat.used));
            o.Text(14, 10.6, t);
            o.Small(14.0 + (double)strlen(t), 10.6, "mA", 0);
            o.Small(14.0 + (double)strlen(t) + 0.3, 10.6, "h", 1);
            double cell = q.bat.volts / q.p.cells;
            snprintf(t, sizeof t, "%c%sv", kIconBattery, Fmt2(cell).c_str());
            o.Text(11, 12, t);
            snprintf(t, sizeof t, "%c%sv", kIconBattery, Fmt1(q.bat.volts).c_str());
            o.Text(11, 13.2, t);
            o.Small(34, 13.2, "FL", 0);
            o.Small(34, 13.2, "MN", 1);
            o.Text(35, 13.2, Clock(d.flyTime));
            o.Text(23.5, 15.4, q.fc.angleMode ? "ANGL" : "ACRO");
            // Warnings, in Betaflight's order of importance.
            std::string warn;
            if (d.warnFor > 0) warn = d.warn;
            else if (d.rxLoss) warn = "RXLOSS";
            else if (cell < 3.3 && q.armed) warn = "LAND NOW";
            else if (cell < 3.5 && q.armed) warn = "LOW BATTERY";
            else if (!q.armed && q.bat.soc < 0.97 && !AutoArm()) warn = "BATT < FULL";
            else if (!q.armed) warn = d.crashed ? "CRASH" : "DISARMED";
            if (!warn.empty()) {
                std::string spaced;
                for (char c : warn) { spaced += c; spaced += ' '; }  // letters spaced, as in the goggles
                o.Centre(8.2, spaced.substr(0, spaced.size() - 1));
            }
            if (!s_sticks.any) o.Centre(10, "NO CONTROLLER");
            else if (!q.armed && !s_everArmed && !AutoArm()) o.Centre(10,s_armSwitch >= 0 ? "THROTTLE DOWN THEN ARM SWITCH" : "THROTTLE DOWN THEN A");
        }
        // The goggles' own readouts: link, voltage and recording time along the bottom, latency and
        // bitrate above them, the card's free time top right.
        int bars = d.rssi > -60 ? 4 : d.rssi > -75 ? 3 : d.rssi > -88 ? 2 : d.rssi > -98 ? 1 : 0;
        o.DjiIcon(kIconBolt, 0.018, 0.895);
        if (bars > 0) o.DjiIcon(kIconSignal, 0.04, 0.895);
        if (bars > 2) o.DjiIcon(kIconSignal, 0.052, 0.895);
        o.Dji("CH1", 0.078, 0.9);
        if (signal) {
            o.Dji(std::format("{:.0f}ms", d.latency), 0.8, 0.825, 0.9);
            o.Dji(std::format("{:.1f}Mb", d.bitrate), 0.885, 0.825, 0.9);
            o.DjiIcon(kIconTriangle, 0.665, 0.895);
            o.Dji(Fmt1(d.q.bat.volts) + "V", 0.69, 0.9);
            o.DjiIcon(kIconClock, 0.785, 0.895);
            o.Dji(Clock(d.sinceSpawn, true), 0.81, 0.9);
            if (d.videoLost) o.Dji("Weak signal", 0.43, 0.6, 1.1);
            if (d.payload)  // the payload, and whether its fuse is live yet
                o.Dji(std::format("{} x M67  {}", std::clamp(s_cfgGrenades.Value(), 0, 8), d.fuseArmed ? "ARMED" : "SAFE"), 0.43, 0.95, 0.9);
        }
        o.DjiIcon(kIconCard, 0.865, 0.21);
        o.Dji("150min", 0.89, 0.215, 0.9);
    }
    Params e(Lib("KismetRenderingLibrary"), "EndDrawCanvasToRenderTarget");
    e.SetObj("WorldContextObject", Object(API->WorldContext()));
    std::memcpy(e.Ptr("Context"), ctx.data(), ctx.size());
    e.Call();
}

// The drone's camera never sees the goggles' screen (it would film the picture it is making).
static void HideFromDrone() {
    static GUObject *doneCapture = nullptr, *doneScreen = nullptr;
    if (!s_drone.capture || !GogglesValid() || (doneCapture == s_drone.capture.ptr && doneScreen == s_gog.screen.ptr)) return;
    doneCapture = s_drone.capture.ptr;
    doneScreen = s_gog.screen.ptr;
    for (Object c : {s_gog.screen, s_gog.shroud})
        if (c) { Params h(s_drone.capture, "HideComponent"); h.SetObj("InComponent", c).Call(); }
}

static void UpdateScreen(double dt) {
    Goggles& g = s_gog;
    if (!GogglesValid()) return;
    // The folding half rides the game's lever (see Dress): down, the screen shows.
    double up = LeverUp(g.lever);
    HideOriginal(Object(g.actor));
    bool view = up < 0.05;  // all the way down
    SetVisible(g.fold, !view);
    SetVisible(g.screen, view);
    SetVisible(g.shroud, view);
    if (!view) return;
    GUObject* rt = RenderTarget();
    if (!rt) return;
    if (DroneValid() && s_drone.capture) {
        HideFromDrone();
        if (!s_drone.videoLost) {  // a lost video link freezes the last picture, as digital goggles do
            Params c(s_drone.capture, "CaptureScene");
            c.Call();
        }
        DrawOsd(rt, true);
    } else {
        Params c(Lib("KismetRenderingLibrary"), "ClearRenderTarget2D");
        c.SetObj("WorldContextObject", Object(API->WorldContext())).SetObj("TextureRenderTarget", Object(rt)).Set("ClearColor", Color{0, 0, 0, 1}).Call();
        DrawOsd(rt, false);
    }
    (void)dt;
}

// ------------------------------------------------------------------ tick

static bool s_scriptHooksOk = false, s_scriptHooksFailed = false;  // see "Blueprint-to-Blueprint calls"
static void ResolveScriptHooks();
static struct StoreOnDropAgain { GUObject* storable = nullptr; double in = 0; } s_storeOnDropAgain;  // FPV_Dev "drop away"

static void Tick(double dt) {
    dt = Clamp(dt, 0, 1.0 / 20);
    static double checkT = 0, resolveT = 0;
    if ((checkT -= dt) <= 0) { checkT = 0.25; CheckGoggles(); }
    if (!s_scriptHooksFailed && (resolveT -= dt) <= 0) { resolveT = 1; ResolveScriptHooks(); }
    ReadInput();
    // Hold the right stick in (or Start on a gamepad): a fresh drone, into its pouch on the vest (only with the goggles on).
    bool recall = Pressed(&Buttons::recall) || (Kamikaze() && Pressed(&Buttons::mode));  // FPV Kamikaze Drone: Y respawns
    if (s_rightStick) {
        s_rightStickHeld += dt;
        if (!s_rightStickFired && s_rightStickHeld >= s_cfgHold.Value()) { s_rightStickFired = true; recall = true; }
    }
    if (recall) {
        if (GogglesValid()) SpawnDrone();
        else Log("put the FPV goggles on to get a drone");
    }
    TickBooms(dt);
    if (s_storeOnDropAgain.storable && (s_storeOnDropAgain.in -= dt) <= 0) {
        if (API->IsValid(s_storeOnDropAgain.storable)) Object(s_storeOnDropAgain.storable).SetBool("bStoreToPouchOnDrop", true);
        s_storeOnDropAgain = {};
    }
    TrackDrones(dt);
    if (DroneValid() && Alive(s_drone.actor)) {
        Fly(dt);
        // A client's drone in flight: where it is, to the host, 30 times a second (see "multiplayer").
        if (DroneValid() && s_drone.free && s_drone.noRepMove && (s_drone.sentAgo += dt) >= 1.0 / 30) {
            s_drone.sentAgo = 0;
            Object a(s_drone.actor);
            Params l(a, "K2_GetActorLocation"); l.Call();
            Params r(a, "K2_GetActorRotation"); r.Call();
            SendToHost(MsgFly, l.Return<V3>(), r.Return<Rot>());
        }
    }
    else if (s_drone.actor) DestroyDrone();
    UpdateScreen(dt);
}

// ------------------------------------------------------------------ multiplayer: the host's side
//
// What a client's drone does, done by the host (see "multiplayer" at the top): the client sends it, this
// takes it (the hook on ServerSetSpectatorLocation in GML_AWAKE).

// A fresh drone for another player (their "fresh"), into a drone pouch of theirs - an empty one, else their
// first - as SpawnDrone does for ours; the drone they were flying (moved by their messages) goes.
static void FreshDroneFor(Object pilot) {
    Object gm = GearManager(pilot);
    auto* arr = gm ? (GML_TArray*)gm.PropPtr("Pouches") : nullptr;
    Object pouch, first;
    for (int i = 0; arr && i < arr->Num; i++) {
        Object p(((GUObject**)arr->Data)[i]);
        if (!p || !Alive(p.ptr) || !IsDronePouch(p)) continue;
        if (!first) first = p;
        Object item = PouchItem(p);
        if (!pouch && !(item && Alive(item.ptr))) pouch = p;
    }
    if (!pouch) pouch = first;
    if (!pouch) { Warn("{} asked for a fresh drone and has no drone pouch", pilot.Name()); return; }
    for (Drone& r : s_remote)
        if (r.pilot == pilot.ptr && r.hostFlown && Alive(r.actor)) { Params d(Object(r.actor), "K2_DestroyActor"); d.Call(); }
    Object old = PouchItem(pouch);
    if (old && Alive(old.ptr)) { Params d(old, "K2_DestroyActor"); d.Call(); }
    pouch.Set("Item", (GUObject*)nullptr);
    Params c(pouch, "CheckSpawnStorable");
    c.Call();
    Log("a fresh drone in {}'s pouch on their vest", pilot.Name());
}

static void OnClientMessage(Object pc, int kind, V3 at, Rot rot) {
    Params gp(pc, "K2_GetPawn");
    gp.Call();
    Object pilot = gp.Return<GUObject*>();
    if (!pilot) return;
    if (kind == MsgFresh) { FreshDroneFor(pilot); return; }
    // Their drone out of hand and pouch: the one already following their messages, else the nearest to where they say.
    Drone* d = nullptr;
    double best = 1e300;
    for (Drone& r : s_remote) {
        if (r.pilot != pilot.ptr || !Alive(r.actor) || Carried(Object(r.actor))) continue;
        Params l(Object(r.actor), "K2_GetActorLocation");
        l.Call();
        double dist = Len(l.Return<V3>() - at) - (r.hostFlown ? 1e9 : 0);
        if (dist < best) best = dist, d = &r;
    }
    if (!d) return;
    Object a(d->actor);
    if (!d->hostFlown) {  // the game let it fall when they let go: it is theirs to move now
        if (Object root = a.GetObj("RootComponent")) { Params s(root, "SetSimulatePhysics"); s.Set("bSimulate", false).Call(); }
        d->hostFlown = true;
        Log("{}'s drone ({}) flies: moved here as they say", pilot.Name(), a.Name());
    }
    if (kind == MsgFly) {
        Params m(a, "K2_SetActorLocationAndRotation");
        m.Set("NewLocation", at).Set("NewRotation", rot).Set("bSweep", false).Set("bTeleport", true).Call();
    } else if (kind == MsgBoom) {
        int n = std::clamp((int)std::lround(rot.roll), 0, 8);
        bool own = Payload(*d, at, n, pilot);
        Log("{}'s drone: payload detonated, {} grenade(s) at ({:.0f}, {:.0f}, {:.0f}){}", pilot.Name(), n, at.x, at.y, at.z,
            own ? ", the first the drone's own" : "");
        if (!own) { Params x(a, "K2_DestroyActor"); x.Call(); }
    }
}

// ------------------------------------------------------------------ dev hook (tests/FPVDroneDev)

static void LogScriptHooks();

// Where the drone is, for the harness's cameras: false without one.
extern "C" __declspec(dllexport) bool FPV_DroneLoc(double* out) {
    if (!DroneValid() || !s_drone.root) return false;
    V3 l = CompLoc(s_drone.root);
    out[0] = l.x, out[1] = l.y, out[2] = l.z;
    return true;
}

extern "C" __declspec(dllexport) void FPV_Dev(const char* cmd) {
    std::string c = cmd ? cmd : "";
    Drone& d = s_drone;
    if (c == "spawn") SpawnDrone();
    else if (c == "bare") SpawnDroneBare();
    else if (c == "grab" || c == "grab waiting" || c == "drop" || c == "drop away") {  // the right hand takes the drone (or a waiting one), or lets go of it
        // (away: not back into the pouch it was taken from, as letting go of it out at arm's length does)
        Object hand;
        if (Object pawn = PlayerPawn()) {
            Params gc(pawn, "K2_GetComponentsByClass");
            gc.Set("ComponentClass", FindClass("GripMotionControllerComponent")).Call();
            auto* arr = (GML_TArray*)gc.Ptr("ReturnValue");
            for (int i = 0; arr && i < arr->Num; i++) {
                Object m(((GUObject**)arr->Data)[i]);
                if (!hand || m.Name().find("Right") != std::string::npos) hand = m;
            }
        }
        bool waiting = c == "grab waiting";
        if (!hand || (waiting ? s_parked.empty() : !DroneValid())) { Warn("FPV_Dev {}: no hand or no drone", c); return; }
        Object a(waiting ? s_parked[0].actor : d.actor), root = a.GetObj("RootComponent");
        if (c.rfind("grab", 0) == 0) {
            auto xf = Transform(CompLoc(root), CompRot(root));
            Params p(hand, "GripObjectByInterface");
            p.SetObj("ObjectToGrip", a);
            std::memcpy(p.Ptr("WorldOffset"), xf.data(), xf.size());
            p.Set("bWorldOffsetIsRelative", false).Call();
            Log("grab {}: {}", hand.Name(), p.Return<bool>());
        } else {
            Object storable;
            if (c == "drop away" && (storable = GetComp(a, "BP_StorableComponent_C"))) storable.SetBool("bStoreToPouchOnDrop", false);
            Params p(hand, "DropObjectByInterface");
            p.SetObj("ObjectToDrop", a).Call();
            Log("drop{}: {}", storable ? " away" : "", p.Return<bool>());
            if (storable) s_storeOnDropAgain = {storable.ptr, 1.0};  // back as it was once it is clear of the pouch
        }
    } else if (c == "pouches") {  // what the kit station lists among the explosives
        auto* list = (GML_TArray*)MapValue(Object(API->WorldContext()), "Pouches", kExplosives);
        for (int i = 0; list && i < list->Num; i++) {
            Object p(((GUObject**)list->Data)[i]);
            char name[64] = "";
            if (void* txt = p.PropPtr("DisplayName")) API->TextToString(txt, name, sizeof name);
            GUObject* icon = p.Get<GUObject*>("IconOverride");
            Log("  pouch [{}] {} '{}' item {} icon {}{}", i, p.Name(), name, Object(p.Get<GUObject*>("Pouchable")).Name(),
                icon ? Object(icon).Name() : "none", p.ptr == s_pouchData ? " (ours)" : "");
        }
    } else if (c == "vestpouch") {  // a grenade pouch on the vest becomes a drone pouch, as a kit with one loads
        Object gm = GearManager(PlayerPawn());
        auto* arr = gm ? (GML_TArray*)gm.PropPtr("Pouches") : nullptr;
        Object grenade, other;  // a grenade pouch, else any ordinary one (not a holster or the tactical pouch)
        for (int i = 0; arr && i < arr->Num; i++) {
            Object p(((GUObject**)arr->Data)[i]);
            GUObject* data = PouchDataOf(p);
            if (!data || IsDronePouch(p) || !Alive(p.ptr)) continue;
            uint8_t type = Object(data).Get<uint8_t>("Type");
            if (type == kExplosives && !grenade) grenade = p;
            else if (type <= 4 && !other) other = p;
        }
        if (!grenade) grenade = other;
        if (!grenade || !MakePouchData()) { Warn("FPV_Dev vestpouch: no pouch on the vest to swap"); return; }
        std::vector<uint8_t> data(PouchStructSize());
        std::memcpy(data.data(), grenade.PropPtr("Data"), data.size());
        *(GUObject**)(data.data() + PouchDataOff()) = s_pouchData;
        Log("drone pouch where {} ({}) was", grenade.Name(), Object(PouchDataOf(grenade)).Name());
        Params d(grenade, "RequestDestroy");
        d.Call();
        Params a(gm, "AddPouch");
        std::memcpy(a.Ptr("PouchData"), data.data(), data.size());
        a.Call();
    } else if (c == "station") {  // the kit station's pouch for the drone, as the drone pouch on the vest has it
        Object gm = GearManager(PlayerPawn());
        Object cam = VRCamera(PlayerPawn());
        std::vector<Object> vest = DronePouches();
        if (!cam || vest.empty()) { Warn("FPV_Dev station: no drone pouch on the vest"); return; }
        double yaw = CompRot(cam).yaw * kDeg;
        V3 at = CompLoc(cam) + V3{std::cos(yaw) * 60, std::sin(yaw) * 60, 30};  // as the harness's StationLoc
        auto xf = Transform(at, Rot{0, CompRot(cam).yaw + 180, 0});
        Params b(Lib("GameplayStatics"), "BeginDeferredActorSpawnFromClass");
        b.SetObj("WorldContextObject", Object(API->WorldContext())).Set("ActorClass", FindClass("EditablePouch_C"));
        std::memcpy(b.Ptr("SpawnTransform"), xf.data(), xf.size());
        b.Set("CollisionHandlingOverride", (uint8_t)1).Call();
        Object ep = b.Return<GUObject*>();
        if (!ep) { Warn("FPV_Dev station: no EditablePouch_C"); return; }
        std::memcpy(ep.PropPtr("Data"), vest[0].PropPtr("Data"), PouchStructSize());
        Params f(Lib("GameplayStatics"), "FinishSpawningActor");
        f.SetObj("Actor", ep);
        std::memcpy(f.Ptr("SpawnTransform"), xf.data(), xf.size());
        f.Call();
        Object root = ep.GetObj("RootComponent");
        if (root) { Params s(root, "SetSimulatePhysics"); s.Set("bSimulate", false).Call(); }
        DressEditable(ep);  // in case its BeginPlay didn't come through the hooks
        Log("kit station pouch {} at ({:.0f}, {:.0f}, {:.0f}): {} (shows the drone: {})", ep.Name(), at.x, at.y, at.z,
            Object(PouchDataOf(ep)).Name(), std::find(s_editables.begin(), s_editables.end(), ep.ptr) != s_editables.end());
    } else if (c.rfind("kitsave ", 0) == 0 || c.rfind("kitload ", 0) == 0) {  // a kit in a test slot, from the station's pouches
        Object gm = GearManager(PlayerPawn());
        if (!gm) return;
        std::wstring slot(c.begin() + 8, c.end());
        if (c[3] == 's') {
            std::vector<GUObject*> eps;
            for (GUObject* e : s_editables)
                if (Alive(e)) eps.push_back(e);
            Params p(gm, "Save Kit Setup");
            *(GML_TArray*)p.Ptr("AttachedPouches") = GML_TArray{eps.data(), (int32_t)eps.size(), (int32_t)eps.size()};
            p.SetString("SlotName", slot.c_str()).Call();
            *(GML_TArray*)p.Ptr("AttachedPouches") = GML_TArray{};
            Log("kit saved to '{}' with {} station pouch(es); theirs now {}", c.substr(8), eps.size(),
                eps.empty() ? "-" : Object(PouchDataOf(Object(eps[0]))).Name());
        } else {
            Params p(gm, "GetKitSetup");
            p.SetString("SlotName", slot.c_str()).Call();
            uint8_t* out = (uint8_t*)p.Ptr("AsGVRKit Setup");
            if (!out) out = (uint8_t*)p.Ptr("AsGVRKit_Setup");
            Object kit(out ? *(GUObject**)out : nullptr);
            auto* arr = kit ? (GML_TArray*)kit.PropPtr("Pouches") : nullptr;
            int ours = 0;
            for (int i = 0; arr && i < arr->Num; i++) {  // ours: the grenade pouch marked "FPVDrone" (see IsDronePouch)
                uint8_t* p = (uint8_t*)arr->Data + (size_t)i * PouchStructSize();
                GUObject* data = *(GUObject**)(p + PouchDataOff());
                if (data == s_pouchData || (data == GrenadePouch() && *(uint64_t*)(p + PouchColorOff()) == NameOf(kPouchTag))) ours++;
            }
            Log("kit loaded from '{}': {} pouch(es), {} the drone's - {}", c.substr(8), arr ? arr->Num : 0, ours, ours ? "(OK)" : "NOT the drone's");
            if (ours) {  // and put on, as the kit station does
                Params l(gm, "LoadKitSetup_Server");
                auto* pa = (GML_TArray*)(l.Ptr("Pouches") ? l.Ptr("Pouches") : l.Ptr("Pouches_0"));
                if (!pa) { Warn("FPV_Dev kitload: LoadKitSetup_Server has no Pouches"); return; }
                *pa = *arr;
                l.Call();
                *pa = GML_TArray{};
                Log("kit put on: {} drone pouch(es) on the vest", DronePouches().size());
            }
        }
    }
    else if (c == "equip") {
        Object gm = GearManager(PlayerPawn());
        if (!gm || !s_item) { Warn("FPV_Dev equip: no gear manager or no goggles entry"); return; }
        std::vector<uint8_t> item(16, 0);
        *(GUObject**)(item.data() + GearDataOff()) = s_item;
        Params p(gm, "SetOutfitItem");
        std::memcpy(p.Ptr("GearItem"), item.data(), 16);
        p.Call();
    } else if (c == "down" || c == "up") {
        if (!GogglesValid()) { Warn("FPV_Dev {}: no goggles", c); return; }
        float lim = s_gog.lever.Get<float>("LeverLimitNegative");
        float sign = lim > 0 ? -1.f : 1.f;  // the lever turns negative towards its negative limit
        Params p(s_gog.lever, "SetLeverAngle");
        p.Set("NewAngle", c == "down" ? sign * std::fabs(lim) : 0.0f).Set("DualAxisForwardVector", V3{1, 0, 0}).Set("bAllowThrowingEvents", true).Call();
    } else if (c.rfind("lever ", 0) == 0) {
        if (!GogglesValid()) return;
        Params p(s_gog.lever, "SetLeverAngle");
        p.Set("NewAngle", (float)atof(c.c_str() + 6)).Set("DualAxisForwardVector", V3{1, 0, 0}).Set("bAllowThrowingEvents", true).Call();
    } else if (c.rfind("input ", 0) == 0) {  // "input <throttle 0..1> <yaw> <pitch> <roll>"
        s_devInput = true;
        sscanf_s(c.c_str() + 6, "%lf %lf %lf %lf", &s_devSticks.throttle, &s_devSticks.yaw, &s_devSticks.pitch, &s_devSticks.roll);
    } else if (c == "input off") s_devInput = false;
    else if (c.rfind("zoom ", 0) == 0) s_devSticks.zoom = atof(c.c_str() + 5);
    else if (c == "arm" || c == "disarm") {
        if (!DroneValid()) return;
        if (c == "arm") TryArm(d);
        else Disarm(d, "dev");
    } else if (c == "angle" || c == "acro") d.q.fc.angleMode = c == "angle";
    else if (c == "invert") {  // on its back where it is (for the crash flip test)
        if (!DroneValid()) return;
        if (!d.chaos) Disarm(d, "dev");
        double yaw = AttRot(d.q.att).yaw * kDeg;
        d.q.att = quad::Mul(quad::Q{0, 0, std::sin(yaw / 2), std::cos(yaw / 2)}, quad::Q{1, 0, 0, 0});
        d.q.vel = {};
        d.q.rate = {};
        d.haveLast = false;
        if (Object root = Object(d.actor).GetObj("RootComponent"); root && d.chaos) {  // the body, turned over where it is
            Params t(root, "K2_SetWorldLocationAndRotation");
            t.Set("NewLocation", CompLoc(root) + V3{0, 0, 10}).Set("NewRotation", AttRot(d.q.att)).Set("bSweep", false).Set("bTeleport", true).Call();
            Params v(root, "SetPhysicsLinearVelocity");
            v.Set("NewVel", V3{}).Set("bAddToCurrent", false).Set("BoneName", uint64_t(0)).Call();
        }
    } else if (c == "flip on") s_devButtons.flip = true;
    else if (c == "flip off") s_devButtons.flip = false;
    else if (c == "boom") {  // the payload goes off where the flying drone is (as on a hit)
        if (!DroneValid() || !d.root) { Warn("FPV_Dev boom: no drone"); return; }
        d.detonateAt = CompLoc(d.root);
        d.detonateNormal = {0, 0, 1};
        Detonate(d);
    } else if (c == "state") {
        const quad::Quad& q = d.q;
        Rot a = AttRot(q.att);
        double thrust = 0;
        for (auto& m : q.m) thrust += m.thrust;
        Log("STATE drone {} pos ({:.0f}, {:.0f}, {:.0f}) vel ({:.2f}, {:.2f}, {:.2f}) m/s att p{:.1f} y{:.1f} r{:.1f} rates ({:.0f}, {:.0f}, {:.0f}) deg/s "
            "thrust {:.2f} N armed {} crashed {} | bat {:.2f} V {:.1f} A {:.0f} mAh | link {:.0f} dBm LQ {:.0f}{} | goggles {} lever {:.1f} | input {} thr {:.2f} | on the vest {} free {} ({} more)",
            DroneValid(), q.pos.x * 100, q.pos.y * 100, q.pos.z * 100, q.vel.x, q.vel.y, q.vel.z, a.pitch, a.yaw, a.roll, q.rate.x / kDeg,
            q.rate.y / kDeg, q.rate.z / kDeg, thrust, q.armed, d.crashed, q.bat.volts, q.bat.amps, q.bat.used, d.rssi, d.lq, d.rxLoss ? " RXLOSS" : "",
            GogglesValid(), GogglesValid() ? s_gog.lever.Get<float>("CurrentLeverAngle") : 0.f, s_inputName, s_sticks.throttle, d.item, d.free, s_parked.size());
        if (GogglesValid() && s_gog.pivot && s_gog.mount) {  // the folding half's hinge against the mount's, and its turn
            Quat qr = ToQuat(CompRot(s_gog.root)), qp = ToQuat(CompRot(s_gog.pivot));
            Rot fold = ToRot(Mul({-qr.x, -qr.y, -qr.z, qr.w}, qp));
            Log("GOGGLES hinge gap {:.2f} cm, fold p{:.1f} y{:.1f} r{:.1f} in the actor", Len(CompLoc(s_gog.pivot) - CompLoc(s_gog.mount)), fold.pitch, fold.yaw, fold.roll);
        }
    } else if (c.rfind("away ", 0) == 0) {  // the flying drone N m out in front of you, 3 m up
        Object cam = VRCamera(PlayerPawn());
        if (!cam || !DroneValid() || !d.free) { Warn("FPV_Dev {}: no free drone", c); return; }
        double yaw = CompRot(cam).yaw * kDeg;
        d.q.pos = ToM(CompLoc(cam) + V3{std::cos(yaw), std::sin(yaw), 0} * (std::atof(c.c_str() + 5) * 100) + V3{0, 0, 300});
        d.q.vel = {};
        d.q.rate = {};
        d.haveLast = false;  // a jump, not a path
        if (Object root = Object(d.actor).GetObj("RootComponent"); root && d.chaos) {  // the body, there and still
            Params t(root, "K2_SetWorldLocation");
            t.Set("NewLocation", ToUE(d.q.pos)).Set("bSweep", false).Set("bTeleport", true).Call();
            Params v(root, "SetPhysicsLinearVelocity");
            v.Set("NewVel", V3{}).Set("bAddToCurrent", false).Set("BoneName", uint64_t(0)).Call();
        }
    } else if (c == "hooks") LogScriptHooks();
    else if (c == "hold") { s_rightStick = true; s_rightStickHeld = 0; s_rightStickFired = false; }
    else if (c == "release") s_rightStick = false;
    else Warn("FPV_Dev: unknown command '{}'", c);
}

// ------------------------------------------------------------------ Blueprint-to-Blueprint calls
//
// GML's hooks see what goes through ProcessEvent: events, delegates, input, RPCs and calls from native
// code. A Blueprint function calling another one skips it (the script VM runs the callee directly), and
// that is how the gear menu saves an outfit, loads one and fills in its tiles. Every script function, called
// either way, ends up in the VM's ProcessLocalScriptFunction(Context, Stack, Result): Blueprint calls go
// there directly, ProcessEvent through UObject::ProcessInternal (every Blueprint function's ExecFunction),
// which checks for a remote call and then calls it. So it is found by following that call, and gets a
// detour which picks out the few functions below by their UFunction and passes everything else through.

using ExecFn = void (*)(void* context, void* stack, void* result);
static ExecFn s_processInternal = nullptr;  // the original ProcessLocalScriptFunction (the trampoline)

// ProcessInternal's call to ProcessLocalScriptFunction. In this build it reads
//   test r15b, 2 / jz / mov r8, rdi / mov rdx, rbp / mov rcx, rbx / call ProcessLocalScriptFunction
// (Context, Stack, Result back into rcx, rdx, r8); otherwise the first direct call that lands on a function.
static void* FindLocalScriptFunction(uint8_t* pi) {
    static const uint8_t pattern[] = {0x4C, 0x8B, 0xC7, 0x48, 0x8B, 0xD5, 0x48, 0x8B, 0xCB, 0xE8};
    for (int i = 0; i < 200; i++)
        if (!memcmp(pi + i, pattern, sizeof pattern)) return pi + i + sizeof pattern + 4 + *(int32_t*)(pi + i + sizeof pattern);
    for (int i = 0; i < 200; i++) {
        if (pi[i] != 0xE8) continue;
        uint8_t* t = pi + i + 5 + *(int32_t*)(pi + i + 1);
        if (std::abs(t - pi) > 0x1000000 || t == pi) continue;
        if ((t[0] == 0x48 && t[1] == 0x89) || (t[0] == 0x40 && t[1] >= 0x53 && t[1] <= 0x57) || (t[0] == 0x48 && t[1] == 0x8B && t[2] == 0xC4) ||
            (t[0] == 0x4C && t[1] == 0x8B && t[2] == 0xDC))  // typical x64 prologues
            return t;
    }
    return nullptr;
}
struct ScriptHook {
    const char* cls;
    const char* fn;
    void (*pre)(Object self, uint8_t* locals, GUFunction* fn);
    void (*post)(Object self, uint8_t* locals, GUFunction* fn);
    std::atomic<GUFunction*> ufn{nullptr};  // read from any thread the VM runs on (animation runs on workers)
    int hits = 0;
};
static uint8_t* ParamIn(uint8_t* locals, GUFunction* fn, const char* name) {
    GML_PropInfo i;
    GFProperty* p = API->FindProperty((GUStruct*)fn, name);
    return p && API->GetPropertyInfo(p, &i) ? locals + i.offset : nullptr;
}
static ScriptHook s_scriptHooks[] = {
    {"BP_GameInstance_C", "GetItems", nullptr, [](Object gi, uint8_t*, GUFunction*) { AddGoggles(gi); AddDronePouch(gi); }},
    {"BP_GearManagerComponent_C", "Save Kit Setup", [](Object, uint8_t*, GUFunction*) { BeforeKitSave(); },
     [](Object, uint8_t*, GUFunction*) { AfterKitSave(); }},
    {"BP_GearManagerComponent_C", "GetKitSetup", nullptr, [](Object, uint8_t* l, GUFunction* f) {
         uint8_t* kit = ParamIn(l, f, "K2Node_DynamicCast_AsKit_Serializable");
         OnKitLoaded(kit ? *(GUObject**)kit : nullptr);
     }},
    {"Pouch_C", "CheckSpawnStorable", [](Object pouch, uint8_t*, GUFunction*) { s_filling = IsDronePouch(pouch) ? pouch.ptr : nullptr; },
     [](Object pouch, uint8_t*, GUFunction*) { s_filling = nullptr; OnPouchFilled(pouch); }},
    {"BP_StorableComponent_C", "StoreToPouch", [](Object st, uint8_t* l, GUFunction* f) { BeforeStore(st, ParamIn(l, f, "Pouch")); }, nullptr},
    {"EditablePouch_C", "ReceiveBeginPlay", nullptr, [](Object ep, uint8_t*, GUFunction*) { ToNetForm(ep); DressEditable(ep); }},
    {"EditablePouch_C", "SetData", nullptr, [](Object ep, uint8_t*, GUFunction*) { ToNetForm(ep); DressEditable(ep); }},
    {"EditablePouch_C", "OnRep_Data", nullptr, [](Object ep, uint8_t*, GUFunction*) { ToNetForm(ep); DressEditable(ep); }},
    {"BP_GearManagerComponent_C", "SaveOutfit", [](Object gm, uint8_t*, GUFunction*) { BeforeSave(gm); },
     [](Object gm, uint8_t*, GUFunction*) { AfterSave(gm); }},
    {"BP_GearManagerComponent_C", "SetOutfitItem_Internal", [](Object, uint8_t* l, GUFunction* f) { OnSetItem(ParamIn(l, f, "GearItem")); }, nullptr},
    {"BP_GearManagerComponent_C", "CheckNVG", nullptr, [](Object, uint8_t*, GUFunction*) { CheckGoggles(); }},
    {"BP_NightVisionBase_C", "RefreshPP", nullptr, [](Object nvg, uint8_t*, GUFunction*) { if (nvg.ptr == s_gog.actor || s_remoteGog.count(nvg.ptr)) HideOriginal(nvg); }},
    {"WB_CustomizationItem_C", "SetOutfitItem", nullptr, [](Object w, uint8_t*, GUFunction*) { IconFor(w); }},
};
static int s_scriptHits = 0;
static void LogScriptHooks() {
    for (auto& h : s_scriptHooks) Log("  hook {}:{} {} - {} calls", h.cls, h.fn, h.ufn.load() ? "watched" : "not found yet", h.hits);
}

static void ProcessInternalDetour(void* ctx, void* stack, void* result) {
    // FFrame (after FOutputDevice's 16 bytes): Node 0x10, Object 0x18, Code 0x20, Locals 0x28.
    GUFunction* node = *(GUFunction**)((uint8_t*)stack + 0x10);
    for (auto& h : s_scriptHooks) {
        if (h.ufn.load(std::memory_order_relaxed) != node || !node) continue;
        uint8_t* locals = *(uint8_t**)((uint8_t*)stack + 0x28);
        if (!s_scriptHits++ && *(void**)((uint8_t*)stack + 0x18) != ctx) Warn("script frame layout looks different - hook data may be off");
        h.hits++;
        if (h.pre) h.pre(Object((GUObject*)ctx), locals, node);
        s_processInternal(ctx, stack, result);
        if (h.post) h.post(Object((GUObject*)ctx), locals, node);
        return;
    }
    s_processInternal(ctx, stack, result);
}

// Finds the hooked functions as their classes load; installs the detour once a Blueprint function is seen.
static std::string s_dump;
static void ResolveScriptHooks() {
    for (auto& h : s_scriptHooks) {
        if (h.ufn.load()) continue;
        GUClass* c = FindClass(h.cls);
        GUFunction* f = c ? API->FindFunction((GUStruct*)c, h.fn) : nullptr;
        if (!f) continue;
        if (!s_scriptHooksOk && !s_scriptHooksFailed) {
            uint32_t flags = *(uint32_t*)((uint8_t*)f + 0xB0);       // UFunction::FunctionFlags
            void* exec = *(void**)((uint8_t*)f + 0xD8);              // UFunction::ExecFunction: UObject::ProcessInternal
            void* local = exec && !(flags & 0x400) ? FindLocalScriptFunction((uint8_t*)exec) : nullptr;
            if (!local) for (int i = 0; exec && i < 160; i++) s_dump += std::format("{:02X} ", ((uint8_t*)exec)[i]);
            if (!local || !API->HookNative(local, (void*)&ProcessInternalDetour, (void**)&s_processInternal)) {
                s_scriptHooksFailed = true;
                Warn("could not hook Blueprint-to-Blueprint calls (flags {:x}) - falling back to ProcessEvent hooks: saving outfits with "
                     "the goggles and their menu icon may not work. ProcessInternal: {}", flags, s_dump);
                for (auto& fb : s_scriptHooks) {  // what reaches ProcessEvent still gets them
                    std::string spec = std::string(fb.cls) + ":" + fb.fn;
                    auto pre = fb.pre;
                    auto post = fb.post;
                    Hook(spec.c_str(), pre ? std::function<bool(Object, GUFunction*, void*)>([pre](Object o, GUFunction* f, void* p) {
                             pre(o, (uint8_t*)p, f);
                             return false;
                         }) : nullptr,
                         post ? std::function<void(Object, GUFunction*, void*)>([post](Object o, GUFunction* f, void* p) { post(o, (uint8_t*)p, f); })
                              : nullptr);
                }
                return;
            }
            s_scriptHooksOk = true;
            Log("Blueprint calls hooked: ProcessLocalScriptFunction at RVA {:#x} (ProcessInternal {:#x})",
                (uint8_t*)local - (uint8_t*)API->ImageBase(), (uint8_t*)exec - (uint8_t*)API->ImageBase());
        }
        h.ufn.store(f);
        Log("watching {}:{}", h.cls, h.fn);
    }
}

// ------------------------------------------------------------------ plugin

GML_AWAKE() {
    s_cfgGoggles = Config.Bind("Goggles", "Enabled", true, "DJI Goggles 2 (FPV) in the gear menu's Night Vision list (takes effect at the next launch).");
    s_cfgScreenDist = Config.Bind("Goggles", "ScreenDistance", 30.0f, "How far in front of your eyes the goggles' screen seems to be, cm (next launch).");
    s_cfgScreenFov = Config.Bind("Goggles", "ScreenFOV", 52.0f, "How wide the screen looks, degrees across (DJI Goggles 2: 51). Next launch.");
    s_cfgHold = Config.Bind("Drone", "SpawnHold", 0.5f, "Seconds to hold the right thumbstick in to get the drone (a short press still kicks).");
    s_cfgVolume = Config.Bind("Drone", "Volume", 1.0f, "Loudness of the drone's motors (0 = silent).");
    s_cfgMode = Config.Bind("Drone", "Mode", "Acro", "Flight mode at spawn: Acro (rate mode) or Angle (self-levelling). Y on a gamepad switches.");
    s_cfgModel = Config.Bind("Drone", "FlightModel", "Kamikaze",
        "Kamikaze: FPV Kamikaze Drone exactly - its drone's body (shape, mass, damping) on the engine's physics, driven as its Blueprint\n"
        "does every frame: the sticks set the turn rate, thrust and ground-effect curves, a 120 s battery, no arming. Its numbers, not settings.\n"
        "Sim: a full quadcopter simulation instead (motors, PID, LiPo; [Build], [Physics], [Rates]).");
    s_cfgPouch = Config.Bind("Pouch", "Enabled", true,
        "The drone's pouch in the kit station, with the grenade, flashbang and slapcharge pouches: put it anywhere on your vest\n"
        "or belt; the drone hangs there. Take it in your hand, put it back, or let go of it anywhere else to fly it (next launch).");
    s_cfgPayload = Config.Bind("Payload", "Enabled", true, "The DJI model's payload release and bomb under the drone: a kamikaze charge. Heavier, and it goes off on impact.");
    s_cfgGrenades = Config.Bind("Payload", "Grenades", 2, "Grenade detonations (the game's M67) where it hits.");
    s_cfgDetonateSpeed = Config.Bind("Payload", "DetonateSpeed", 5.0f, "km/h: a hit faster than this sets it off (FPV Kamikaze Drone's warheads: 5).");
    s_cfgArmDistance = Config.Bind("Payload", "ArmDistance", 10.0f, "m: the fuse is safe until the drone has been this far from you, then it is live (0: live from the start).");
    s_cfgArming = Config.Bind("Drone", "Arming", "Auto",
        "Auto: as in FPV Kamikaze Drone, it is live as soon as it spawns (zero throttle stops the motors) and after a crash it is live again once it\n"
        "sits on its legs. Switch: as Betaflight, arm with A or the radio's arm switch, throttle down.");
    s_cfgCrashSpeed = Config.Bind("Drone", "CrashSpeed", 7.0f, "Hitting something faster than this (m/s) cuts the motors, as Betaflight's crash detection does. Slower hits bounce.");
    // The drone builder and physics of FPV Kamikaze Drone (HFM Games), with its defaults for the physics.
    s_cfgProp = Config.Bind("Build", "PropSize", 7.0f, "Propeller diameter, inches. The frame is sized to suit.");
    s_cfgPitch = Config.Bind("Build", "PropPitch", 4.0f, "Propeller pitch, inches: more is faster at the top end, less thrust off the line.");
    s_cfgKV = Config.Bind("Build", "MotorKV", 1300.0f, "Motor KV (rpm per volt).");
    s_cfgCells = Config.Bind("Build", "Cells", 6, "Battery cells in series.");
    s_cfgCapacity = Config.Bind("Build", "Capacity", 3000.0f, "Battery capacity, mAh. Calling the drone back gives it a fresh pack.");
    s_cfgGrams = Config.Bind("Build", "Weight", 2600.0f, "All-up weight, grams: frame, battery and payload (a warhead). Heavier is less floaty and slower.");
    s_cfgMinThrottle = Config.Bind("Build", "MinThrottle", 0.0f, "Motor speed at zero throttle, 0..0.5 (0: the motors stop; Betaflight idles at 0.055).");
    s_cfgGravity = Config.Bind("Physics", "Gravity", 9.82f, "m/s2.");
    s_cfgInstantPower = Config.Bind("Physics", "InstantPower", 1.2f, "How fast the motors spin up and down, times what their propellers' size gives.");
    s_cfgAirFriction = Config.Bind("Physics", "AirFriction", 0.45f, "Body drag: what slows it at speed and in a fall.");
    s_cfgAirGrip = Config.Bind("Physics", "AirGrip", 0.8f, "How hard the spinning propellers bite the air sideways (rotor drag): more carves through turns, less slides.");
    s_cfgRelAirSpeed = Config.Bind("Physics", "RelativeAirSpeed", 1.0f, "How much the air rushing into the propellers at speed costs thrust (0: none).");
    s_cfgRatesRP = Config.Bind("Rates", "RollPitch", "30,220,0.3",
        "Betaflight ACTUAL rates for roll and pitch: centre sensitivity, max rate (deg/s), expo. Gentle by default (a gamepad's short sticks);\n"
        "Betaflight's own default is 70,670,0.");
    s_cfgRatesYaw = Config.Bind("Rates", "Yaw", "30,160,0.2", "Betaflight ACTUAL rates for yaw.");
    s_cfgSmoothing = Config.Bind("Rates", "Smoothing", 0.09f, "Seconds the sticks are smoothed over (RC smoothing): more eases into turns, 0 snaps.");
    s_cfgCamFov = Config.Bind("Camera", "FOV", 120.0f, "The FPV camera's horizontal field of view, degrees.");
    s_cfgUptilt = Config.Bind("Camera", "Uptilt", 10.0f, "FPV camera up-tilt, degrees (LB / RB change it in 5s). Near 40 you fly where you look.");
    s_cfgZoom = Config.Bind("Camera", "Zoom", 3.0f, "Zoom with a gamepad's triggers fully in (1: none).");
    s_cfgResX = Config.Bind("Camera", "Width", 1280, "Camera resolution (next launch).");
    s_cfgResY = Config.Bind("Camera", "Height", 720, "Camera resolution (next launch).");
    s_cfgOsd = Config.Bind("Camera", "OSD", true, "Betaflight OSD over the picture.");
    s_cfgOsdBright = Config.Bind("Camera", "OSDBrightness", 1.4f, "OSD white level (the picture is HDR: above 1 stays white over a bright sky).");
    s_cfgCaptureHDR = Config.Bind("Camera", "HDR", true,
        "The camera hands over its picture before tone mapping, so the game tone-maps it once like everything else you see\n"
        "(natural colours). false: the camera tone-maps it itself (brighter, more saturated). Next launch.");
    s_cfgDevice = Config.Bind("Controls", "Device", "Auto", "Auto (a radio if one is plugged in, else a gamepad), Gamepad or Radio.");
    s_cfgVRSticks = Config.Bind("Controls", "VRSticks", false, "Also fly with the VR controllers' thumbsticks when no gamepad or radio is found (they also move you).");
    s_cfgGamepadMode = Config.Bind("Controls", "GamepadMode", 2, "Stick layout: 2 = throttle and yaw on the left (default), 1 = throttle on the right.");
    s_cfgPadThrottle = Config.Bind("Controls", "GamepadThrottle", "Centre",
        "Centre: the throttle stick's middle is zero throttle and up is full, so letting go cuts the motors (as FPV Kamikaze Drone can).\n"
        "Full: all the way down is zero and the middle is half, as on a radio.");
    s_cfgSensitivity = Config.Bind("Controls", "Sensitivity", 1.0f,
        "Stick curve for pitch, roll and yaw, as FPV Kamikaze Drone's: 1 is straight, 1.5 is softer round the middle, 0.5 sharper.");
    s_cfgDeadzone = Config.Bind("Controls", "Deadzone", 0.25f, "Gamepad stick dead zone, rescaled past it (FPV Kamikaze Drone, as Unreal's default: 0.25).");
    s_cfgRadioAxes = Config.Bind("Radio", "Axes", "XYZR",
        "Which joystick axes carry roll, pitch, throttle and yaw, in that order (letters X Y Z R U V). EdgeTX/OpenTX default (AETR): XYZR.\n"
        "The log names the radio when it connects.");
    s_cfgRadioInvert = Config.Bind("Radio", "Invert", "", "Axes to reverse, e.g. Y.");
    s_cfgRadioArm = Config.Bind("Radio", "Arm", "U",
        "With [Drone] Arming = Switch, the arm switch: an axis letter (armed above the middle; channel 5 is U on EdgeTX/OpenTX) or B<n> for\n"
        "button n. Empty: none (a gamepad's A still arms).");

    ResolveScriptHooks();  // the rest as their classes load (Tick)
    // Flipping the goggles: the game would power night vision; ours show the drone's camera instead.
    // (A delegate: it comes through ProcessEvent.)
    Hook("BP_NightVisionBase_C:BndEvt__BP_NightVisionBase_NVG_K2Node_ComponentBoundEvent_0_VRLeverStateChangedSignature__DelegateSignature",
         [](Object nvg, GUFunction* fn, void* params) {
             if (nvg.ptr != s_gog.actor) return false;
             s_gog.down = Param<bool>(fn, params, "LeverStatus");
             Log("goggles flipped {}", s_gog.down ? "down" : "up");
             return true;
         });
    // Multiplayer, on the host: a client's drone messages (see "multiplayer"), kept from the game. On a client
    // its own outgoing call passes through here too, and goes out.
    Hook("PlayerController:ServerSetSpectatorLocation", [](Object pc, GUFunction* fn, void* params) {
        V3 at = Param<V3>(fn, params, "NewLoc");
        int kind = (int)std::floor(at.z / kMsg + 0.5);
        if (kind < MsgFly || kind > MsgFresh || NetClient()) return false;  // the game's own, or ours going out
        at.z -= kMsg * kind;
        OnClientMessage(pc, kind, at, Param<Rot>(fn, params, "NewRot"));
        return true;
    });
    // A drone's grenade going off (the host's explosion, on every machine): the drone is gone at once.
    HookAfter("GrenadeActor_C:Explode_Multicast", [](Object g, GUFunction*, void*) {
        auto hide = [&](Drone& d) {
            if (d.actor != g.ptr) return;
            if (d.root) { Params h(d.root, "SetVisibility"); h.Set("bNewVisibility", false).Set("bPropagateToChildren", true).Call(); }
            if (d.sound) { Params v(d.sound, "SetVolumeMultiplier"); v.Set("VolumeMultiplier", 0.0f).Call(); }
        };
        hide(s_drone);
        for (Drone& d : s_parked) hide(d);
        for (Drone& d : s_remote) hide(d);
    });
    auto stick = [](bool down) {
        return [down](Object self, GUFunction*, void*) {
            if (self.ptr != PlayerPawn().ptr) return;
            s_rightStick = down;
            if (down) { s_rightStickHeld = 0; s_rightStickFired = false; }
        };
    };
    HookAfter("BP_PlayerCharacter_C:InpActEvt_RightThumbStick_K2Node_InputActionEvent_0", stick(true));
    HookAfter("BP_PlayerCharacter_C:InpActEvt_RightThumbStick_K2Node_InputActionEvent_1", stick(false));

    On(GML_EVENT_WORLD_BEGIN_PLAY, [](void*) {
        BuildAssets();
        s_gog = Goggles{};
        GUObject* rt = s_drone.rt;
        s_drone = Drone{};
        s_drone.rt = rt && API->IsValid(rt) ? rt : nullptr;
        if (Object gi = Object(API->WorldContext())) { AddGoggles(gi); AddDronePouch(gi); }  // if GetItems ran before we could see it
    });
    On(GML_EVENT_TICK, [](void* data) { Tick(data ? *(float*)data : 1.0 / 90); });
    Message("FPV Drone {} - DJI Goggles 2 in the Night Vision gear, hold the right stick in for the drone", FPVDRONE_VERSION);
    Log("{}", FPVDRONE_MODEL_CREDIT);
    return 0;
}
