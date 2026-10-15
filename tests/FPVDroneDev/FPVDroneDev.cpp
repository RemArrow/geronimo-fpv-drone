// FPVDroneDev - in-game test harness for the FPV Drone plugin. Not shipped.
// Loads TrainingGrounds and checks, in order: the goggles' gear entry, the game's GPNVG-18 for reference
// (where its parts sit), the goggles on the helmet up and down, the screen with no drone, a drone and a
// scripted flight (take off, climb, fly, turn, land), a crash, the right-stick hold, and an outfit saved
// and loaded with the goggles. "SNAP <name>" = a game-rendered screenshot (the runner names the files).
// The runner snapshots and restores SaveGames: the save test writes an outfit slot.
#include <GML/GML.hpp>
#include <windows.h>
#include <cmath>
#include <functional>

using namespace gml;

GML_PLUGIN("com.remarrow.fpvdronedev", "FPVDroneDev", "0.1.0", {{"com.remarrow.fpvdrone", "0.1.0", GML_DEPENDENCY_SOFT}});

struct V3 { double x = 0, y = 0, z = 0; };
struct Rot { double pitch = 0, yaw = 0, roll = 0; };
static const char* kMap = "/Game/Maps/Training/TrainingGrounds";
static const char* kGpnvg = "/Game/Characters/Data/DA_NightVision_GPNVG18_Green.DA_NightVision_GPNVG18_Green";

static void Dev(const char* cmd) {
    static auto fn = (void (*)(const char*))GetProcAddress(GetModuleHandleW(L"FPVDrone.dll"), "FPV_Dev");
    if (fn) fn(cmd);
    else Error("FPVDrone.dll's FPV_Dev not found");
}
static void Snap(const char* n) { Log("SNAP {}", n); API->ExecConsoleCommand("shot"); }
static Object Pawn() {
    Params p(Lib("GameplayStatics"), "GetPlayerPawn");
    p.SetObj("WorldContextObject", Object(API->WorldContext())).Set("PlayerIndex", (int32_t)0).Call();
    return p.Return<GUObject*>();
}
static Object PC() {
    Params p(Lib("GameplayStatics"), "GetPlayerController");
    p.SetObj("WorldContextObject", Object(API->WorldContext())).Set("PlayerIndex", (int32_t)0).Call();
    return p.Return<GUObject*>();
}
static Object Gear() {
    Object pawn = Pawn();
    if (!pawn) return {};
    Params p(pawn, "GetComponentByClass");
    p.Set("ComponentClass", FindClass("BP_GearManagerComponent_C")).Call();
    return p.Return<GUObject*>();
}
static V3 CompLoc(Object c) { Params p(c, "K2_GetComponentLocation"); p.Call(); return p.Return<V3>(); }
static Rot CompRot(Object c) { Params p(c, "K2_GetComponentRotation"); p.Call(); return p.Return<Rot>(); }

static void SetItem(GUObject* item) {
    Object gm = Gear();
    if (!gm || !item) { Error("no gear manager / item"); return; }
    uint8_t buf[16] = {};
    *(GUObject**)buf = item;
    Params p(gm, "SetOutfitItem");
    std::memcpy(p.Ptr("GearItem"), buf, 16);
    p.Call();
}
static uint8_t* MapValue(Object obj, const char* prop, uint8_t key) {
    GFProperty* mp = API->FindProperty(obj.Struct(), prop);
    void* map = obj.PropPtr(prop);
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
static std::string Text(Object o, const char* prop) {
    char b[128] = "";
    if (void* t = o.PropPtr(prop)) API->TextToString(t, b, sizeof b);
    return b;
}
static GUObject* Worn(uint8_t cat) {
    Object gm = Gear();
    uint8_t* v = gm ? MapValue(gm, "GearByCategory", cat) : nullptr;
    return v ? *(GUObject**)v : nullptr;
}

// Every item in a gear category, and which one is worn.
static GUObject* s_ours = nullptr;
static void ListCategory(uint8_t cat) {
    Object gi(API->WorldContext());
    uint8_t* v = MapValue(gi, "OutfitItems", cat);
    if (!v) { Log("category {}: none", cat); return; }
    auto* arr = (GML_TArray*)v;  // FOutfitItemList: Items is its only member
    for (int i = 0; i < arr->Num; i++) {
        Object it(((GUObject**)arr->Data)[i]);
        std::string name = Text(it, "DisplayName");
        Log("  cat {} [{}] {} '{}' allowsNVG {}", cat, i, it.Name(), name, it.IsA("OutfitItem_Helmet_C") ? (it.Get<bool>("bAllowsNVG") ? "yes" : "no") : "-");
        if (name.find("DJI") != std::string::npos) s_ours = it.ptr;
    }
    Log("  worn in {}: {}", cat, Worn(cat) ? Object(Worn(cat)).Name() : "none");
}

// Where the NVG actor's parts are, from the eye: for fitting the goggles against the game's own NVG.
static void NvgReport(const char* when) {
    Object gm = Gear();
    Object nvg = gm ? gm.GetObj("NVG") : Object();
    Object pawn = Pawn();
    Object cam = pawn ? pawn.GetObj("VRReplicatedCamera") : Object();
    if (!nvg || !cam) { Log("NVG [{}] none", when); return; }
    V3 c = CompLoc(cam);
    Rot cr = CompRot(cam);
    double t = cr.yaw * 3.14159265358979 / 180, cp = std::cos(t), sp = std::sin(t);
    auto local = [&](V3 w) { V3 d{w.x - c.x, w.y - c.y, w.z - c.z}; return V3{cp * d.x + sp * d.y, -sp * d.x + cp * d.y, d.z}; };
    Log("NVG [{}] {} class {} camera rot ({:.1f}, {:.1f}, {:.1f})", when, nvg.Name(), Object((GUObject*)nvg.Class()).Name(), cr.pitch, cr.yaw, cr.roll);
    Params gc(nvg, "K2_GetComponentsByClass");
    gc.Set("ComponentClass", FindClass("SceneComponent")).Call();
    auto* arr = (GML_TArray*)gc.Ptr("ReturnValue");
    for (int i = 0; arr && i < arr->Num; i++) {
        Object comp(((GUObject**)arr->Data)[i]);
        V3 l = local(CompLoc(comp));
        V3 rl = comp.Get<V3>("RelativeLocation");
        Rot rr = comp.Get<Rot>("RelativeRotation");
        Params par(comp, "GetAttachParent"); par.Call();
        Object parent = par.Return<GUObject*>();
        std::string mesh = "-", box;
        if (comp.IsA("StaticMeshComponent")) {
            Object m = comp.GetObj("StaticMesh");
            if (m) {
                mesh = m.Name();
                Params bb(m, "GetBoundingBox"); bb.Call();
                V3 mn = bb.Get<V3>("ReturnValue");
                V3 mx = *(V3*)((uint8_t*)bb.Ptr("ReturnValue") + 24);
                box = std::format(" box ({:.1f},{:.1f},{:.1f})..({:.1f},{:.1f},{:.1f})", mn.x, mn.y, mn.z, mx.x, mx.y, mx.z);
            }
        }
        Log("  {} [{}] parent {} rel ({:.2f},{:.2f},{:.2f}) rot ({:.1f},{:.1f},{:.1f}) eye->({:.2f},{:.2f},{:.2f}) visible {} mesh {}{}", comp.Name(),
            Object((GUObject*)comp.Class()).Name(), parent ? parent.Name() : "-", rl.x, rl.y, rl.z, rr.pitch, rr.yaw, rr.roll, l.x, l.y, l.z,
            comp.GetBool("bVisible"), mesh, box);
    }
    if (Object lever = nvg.GetObj("NVG"))
        Log("  lever angle {:.1f} limits +{:.1f} -{:.1f} toggle {:.2f} axis {}", lever.Get<float>("CurrentLeverAngle"), lever.Get<float>("LeverLimitPositive"),
            lever.Get<float>("LeverLimitNegative"), lever.Get<float>("LeverTogglePercentage"), (int)lever.Get<uint8_t>("LeverRotationAxis"));
}

// Owner-only components (every NVG part) don't render for a camera of our own: show them to it.
static void OwnerSeeOff() {
    Object gm = Gear();
    Object nvg = gm ? gm.GetObj("NVG") : Object();
    if (!nvg) return;
    Params gc(nvg, "K2_GetComponentsByClass");
    gc.Set("ComponentClass", FindClass("PrimitiveComponent")).Call();
    auto* arr = (GML_TArray*)gc.Ptr("ReturnValue");
    for (int i = 0; arr && i < arr->Num; i++) {
        Object c(((GUObject**)arr->Data)[i]);
        Params o(c, "SetOnlyOwnerSee"); o.Set("bNewOnlyOwnerSee", false).Call();
    }
}

static Object s_cam;
static void ViewFrom(V3 from, V3 at, float fov = 40) {
    if (!s_cam) {
        Params mk(Lib("KismetMathLibrary"), "MakeTransform");
        mk.Set("Location", from).Set("Rotation", Rot{}).Set("Scale", V3{1, 1, 1}).Call();
        GML_PropInfo xi;
        API->GetPropertyInfo(API->FindProperty((GUStruct*)API->FindFunction(Lib("KismetMathLibrary").Struct(), "MakeTransform"), "ReturnValue"), &xi);
        Params b(Lib("GameplayStatics"), "BeginDeferredActorSpawnFromClass");
        b.SetObj("WorldContextObject", Object(API->WorldContext())).Set("ActorClass", FindClass("CameraActor"));
        std::memcpy(b.Ptr("SpawnTransform"), mk.Ptr("ReturnValue"), xi.size);
        b.Set("CollisionHandlingOverride", (uint8_t)1).Call();
        s_cam = b.Return<GUObject*>();
        Params f(Lib("GameplayStatics"), "FinishSpawningActor");
        f.SetObj("Actor", s_cam);
        std::memcpy(f.Ptr("SpawnTransform"), mk.Ptr("ReturnValue"), xi.size);
        f.Call();
    }
    if (Object cc = s_cam.GetObj("CameraComponent")) cc.Set("FieldOfView", fov);
    double dx = at.x - from.x, dy = at.y - from.y, dz = at.z - from.z;
    Rot look{std::atan2(dz, std::sqrt(dx * dx + dy * dy)) * 180 / 3.14159265358979, std::atan2(dy, dx) * 180 / 3.14159265358979, 0};
    Params mv(s_cam, "K2_SetActorLocationAndRotation");
    mv.Set("NewLocation", from).Set("NewRotation", look).Set("bSweep", false).Set("bTeleport", true).Call();
    Params vt(PC(), "SetViewTargetWithBlend");
    vt.SetObj("NewViewTarget", s_cam).Set("BlendTime", 0.0f).Call();
}
static void ViewPlayer() {
    Params vt(PC(), "SetViewTargetWithBlend");
    vt.SetObj("NewViewTarget", Pawn()).Set("BlendTime", 0.0f).Call();
}
// Our camera `side` cm to the head's right (or front), looking at the NVG actor.
static void ViewHead(double fwd, double right, double up) {
    Object gm = Gear();
    Object nvg = gm ? gm.GetObj("NVG") : Object();
    Object cam = Pawn().GetObj("VRReplicatedCamera");
    if (!nvg || !cam) return;
    V3 c = CompLoc(cam);
    double t = CompRot(cam).yaw * 3.14159265358979 / 180;
    V3 at{c.x + std::cos(t) * 9, c.y + std::sin(t) * 9, c.z + 6};  // between the eye and the G24 hinge
    V3 from{at.x + std::cos(t) * fwd - std::sin(t) * right, at.y + std::sin(t) * fwd + std::cos(t) * right, at.z + up};
    ViewFrom(from, at, 35);
}
// The character's own meshes shown or hidden: with the headset lying on the floor its body is posed oddly
// and hides the head, so it is out of the way for the close-ups of the goggles only (BodyVisible(true) after).
static void BodyVisible(bool show) {
    Params gc(Pawn(), "K2_GetComponentsByClass");
    gc.Set("ComponentClass", FindClass("SkeletalMeshComponent")).Call();
    auto* arr = (GML_TArray*)gc.Ptr("ReturnValue");
    for (int i = 0; arr && i < arr->Num; i++) {
        Object c(((GUObject**)arr->Data)[i]);
        Params v(c, "SetVisibility"); v.Set("bNewVisibility", show).Set("bPropagateToChildren", false).Call();
    }
}
// A 2 cm ball on the eye (the VR camera), and the body hidden.
static void EyeMarker() {
    Object pawn = Pawn();
    Object cam = pawn.GetObj("VRReplicatedCamera");
    Object sphere = LoadObject("/Engine/BasicShapes/Sphere.Sphere");
    if (!cam || !sphere) return;
    BodyVisible(false);
    Params mk(Lib("KismetMathLibrary"), "MakeTransform");
    mk.Set("Location", V3{}).Set("Rotation", Rot{}).Set("Scale", V3{0.02, 0.02, 0.02}).Call();
    GML_PropInfo xi;
    API->GetPropertyInfo(API->FindProperty((GUStruct*)API->FindFunction(Lib("KismetMathLibrary").Struct(), "MakeTransform"), "ReturnValue"), &xi);
    Params add(pawn, "AddComponentByClass");
    add.Set("Class", FindClass("StaticMeshComponent")).Set("bManualAttachment", true).Set("bDeferredFinish", false);
    std::memcpy(add.Ptr("RelativeTransform"), mk.Ptr("ReturnValue"), xi.size);
    add.Call();
    Object ball = add.Return<GUObject*>();
    if (!ball) return;
    Params sm(ball, "SetStaticMesh"); sm.SetObj("NewMesh", sphere).Call();
    Params cc(ball, "SetCollisionEnabled"); cc.Set("NewType", (uint8_t)0).Call();
    Params at(ball, "K2_AttachToComponent");
    at.SetObj("Parent", cam).Set("SocketName", uint64_t(0)).Set("LocationRule", (uint8_t)2).Set("RotationRule", (uint8_t)2)
        .Set("ScaleRule", (uint8_t)1).Set("bWeldSimulatedBodies", false).Call();
}

// The gear menu's item tile, built the way the menu builds it, given the goggles: which icon it shows.
static void IconTest() {
    GUClass* cls = LoadClass("/Game/UI/Widgets/Customization/Outfit/WB_CustomizationItem.WB_CustomizationItem_C");
    if (!cls || !s_ours) { Error("no item widget class / goggles"); return; }
    Params c(Lib("WidgetBlueprintLibrary"), "Create");
    c.SetObj("WorldContextObject", Object(API->WorldContext())).Set("WidgetType", cls).SetObj("OwningPlayer", PC()).Call();
    Object w = c.Return<GUObject*>();
    if (!w) { Error("could not create the item widget"); return; }
    Params s(w, "SetOutfitItem"); s.SetObj("OutfitItem", Object(s_ours)).Call();
    Object image = w.GetObj("Image_30");
    GML_PropInfo bi, ri;
    API->GetPropertyInfo(API->FindProperty(image.Struct(), "Brush"), &bi);
    API->GetPropertyInfo(API->FindProperty(bi.structType, "ResourceObject"), &ri);
    GUObject* res = *(GUObject**)((uint8_t*)image.ptr + bi.offset + ri.offset);
    GUObject* icon = Object(s_ours).Get<GUObject*>("CustomIcon");
    Log("ICON tile shows {} - {}", res ? Object(res).FullName() : "nothing", res && res == icon ? "the goggles' icon (OK)" : "NOT the goggles' icon");
}

// Where FPV_Dev "station" puts the kit station's pouch: 60 cm ahead of the eye, level, 30 cm up (the headset lies on the floor).
static V3 StationLoc() {
    Object cam = Pawn().GetObj("VRReplicatedCamera");
    if (!cam) return {};
    V3 c = CompLoc(cam);
    double t = CompRot(cam).yaw * 3.14159265358979 / 180;
    return {c.x + std::cos(t) * 60, c.y + std::sin(t) * 60, c.z + 30};
}
static V3 DroneLoc() {
    static auto fn = (bool (*)(double*))GetProcAddress(GetModuleHandleW(L"FPVDrone.dll"), "FPV_DroneLoc");
    double l[3];
    if (fn && fn(l)) return {l[0], l[1], l[2]};
    return {};
}

// ------------------------------------------------------------------ the script

struct Step { double wait; std::function<void()> fn; };
static std::vector<Step> s_steps;
static size_t s_at = 0;
static double s_wait = -1;

static void Script() {
    auto add = [](double wait, std::function<void()> f) { s_steps.push_back({wait, std::move(f)}); };
    add(0, [] {
        Log("--- gear: helmets and night vision");
        ListCategory(5);
        ListCategory(19);
        Object helmet(Worn(5));
        if (!helmet || !helmet.Get<bool>("bAllowsNVG")) {
            Object gi(API->WorldContext());
            auto* arr = (GML_TArray*)MapValue(gi, "OutfitItems", 5);
            for (int i = 0; arr && i < arr->Num; i++) {
                Object h(((GUObject**)arr->Data)[i]);
                if (h.Get<bool>("bAllowsNVG")) { Log("putting on {} (it takes NVGs)", h.Name()); SetItem(h.ptr); break; }
            }
        }
    });
    add(1.5, [] { Log("--- the game's GPNVG-18, for reference"); SetItem(LoadObject(kGpnvg).ptr); EyeMarker(); });
    add(2.0, [] { NvgReport("GPNVG-18"); OwnerSeeOff(); ViewHead(0, 75, 8); });
    add(0.6, [] { Snap("gpnvg_side"); });
    add(0.4, [] { ViewHead(70, 25, 12); });
    add(0.6, [] { Snap("gpnvg_front"); });
    add(0.4, [] {
        Log("--- the FPV goggles");
        if (!s_ours) { Error("no goggles entry in the gear menu"); return; }
        SetItem(s_ours);
    });
    add(2.0, [] { NvgReport("goggles"); Dev("state"); OwnerSeeOff(); ViewHead(0, 75, 8); });
    add(0.6, [] { Snap("goggles_up_side"); });
    add(0.4, [] { ViewHead(70, 25, 12); });
    add(0.6, [] { Snap("goggles_up_front"); });
    add(0.4, [] { Dev("lever -65"); ViewHead(0, 75, 8); });
    add(0.8, [] { Dev("state"); Snap("goggles_half_side"); });
    // Nearly down: all the way down the folding half hides, and the screen's dark box is all there is to see
    add(0.4, [] { Dev("lever -120"); });
    add(0.8, [] { Dev("state"); Snap("goggles_neardown_side"); });
    add(0.2, [] { BodyVisible(true); });  // the close-ups are done: the body back
    add(0.4, [] { Dev("down"); });
    add(1.0, [] { NvgReport("goggles down"); Dev("state"); ViewPlayer(); });
    add(0.8, [] { Snap("screen_nosignal"); });
    add(0.2, [] {
        Log("--- the drone's pouch: among the kit station's explosives, then on the vest where a grenade pouch was");
        Dev("pouches");
        Dev("input 0 0 0 0");
        Dev("vestpouch");
    });
    add(1.5, [] { Dev("state"); Snap("osd_pouch"); });
    // From outside, the goggles up (down, their dark box around the eye is all a camera near the head sees)
    add(0.3, [] { Dev("up"); V3 d = DroneLoc(); ViewFrom({d.x - 70, d.y - 70, d.z + 40}, d, 55); });
    add(0.6, [] { Snap("drone_pouch"); });
    add(0.2, [] { V3 d = DroneLoc(); ViewFrom({d.x + 40, d.y - 25, d.z + 15}, d, 50); });
    add(0.6, [] { Snap("drone_pouch2"); });
    add(0.2, [] { Log("--- the kit station's pouch for it, as you take it to put on the vest"); Dev("station"); });
    add(0.6, [] { V3 s = StationLoc(); ViewFrom({s.x - 45, s.y - 45, s.z + 25}, s, 50); });
    add(0.6, [] { Snap("station_pouch"); });
    add(0.2, [] { ViewPlayer(); Log("--- the right hand takes it"); Dev("grab"); });
    add(0.8, [] { Dev("state"); V3 d = DroneLoc(); ViewFrom({d.x - 70, d.y - 70, d.z + 40}, d, 55); });
    add(0.6, [] { Snap("drone_hand"); });
    add(0.2, [] { ViewPlayer(); Dev("down"); });
    add(0.6, [] { Snap("osd_hand"); Log("--- let go at the vest: back into the pouch"); Dev("drop"); });
    add(0.8, [] { Dev("state"); Log("--- taken again"); Dev("grab"); });
    add(0.5, [] { Dev("state"); Log("--- let go away from the vest: it drops and flies"); Dev("drop away"); });
    add(2.0, [] { Dev("state"); Snap("osd_disarmed"); });
    add(0.3, [] { Dev("up"); V3 d = DroneLoc(); ViewFrom({d.x - 60, d.y - 60, d.z + 35}, d, 50); });
    add(0.6, [] { Snap("drone_ground"); });
    add(0.2, [] { ViewPlayer(); Dev("down"); Log("--- arm with the throttle up (refused), then down"); Dev("input 0.4 0 0 0"); });
    add(0.2, [] { Dev("arm"); });
    add(0.3, [] { Dev("state"); Dev("input 0 0 0 0"); });
    add(0.2, [] { Dev("arm"); });
    add(1.0, [] { Dev("state"); Log("--- punch up (75%)"); Dev("input 0.75 0 0 0"); });
    for (int i = 0; i < 4; i++) add(0.25, [] { Dev("state"); });
    add(0.1, [] { Log("--- hover throttle (35%)"); Dev("input 0.35 0 0 0"); });
    for (int i = 0; i < 4; i++) add(0.5, [] { Dev("state"); });
    add(0.1, [] { Snap("osd_hover"); });
    add(0.1, [] { Dev("zoom 1"); });
    add(0.3, [] { Snap("osd_zoom"); });
    add(0.1, [] { Dev("zoom 0"); });
    add(0.1, [] { Log("--- acro: tip forward (stick 0.4 for 0.25 s), then centred: it holds the angle"); Dev("input 0.6 0 0.4 0"); });
    add(0.25, [] { Dev("input 0.6 0 0 0"); Dev("state"); });
    for (int i = 0; i < 6; i++) add(0.3, [] { Dev("state"); });
    add(0.1, [] { Snap("osd_forward"); });
    add(0.1, [] { V3 d = DroneLoc(); ViewFrom({d.x - 150, d.y + 120, d.z + 60}, d, 45); });
    add(0.3, [] { Snap("drone_air"); });
    add(0.1, [] { ViewPlayer(); });
    add(0.1, [] { Log("--- level it (angle mode for a second), then a full roll at full stick"); Dev("angle"); Dev("input 0.55 0 0 0"); });
    add(1.2, [] { Dev("acro"); Dev("state"); Dev("input 0.6 0 0 1"); });
    add(1.0, [] { Dev("input 0.55 0 0 0"); Dev("state"); });  // a full roll at 360 deg/s
    add(0.3, [] { Dev("state"); });
    add(0.1, [] { Log("--- 25 m out, 3 m up (past the fuse's 10 m, clear of the blast), throttle to idle: it drops, and the payload goes off"); Dev("input 0.55 0 0 0"); Dev("away 25"); });
    add(0.5, [] { Dev("state"); Dev("input 0 0 0 0"); });
    for (int i = 0; i < 40; i++)
        add(0.3, [] {
            static V3 last;
            static int stage = 0;  // 0 flying, 1 camera on the blast, 2 shot taken
            Dev("state");
            V3 d = DroneLoc();
            if (d.x || d.y || d.z) last = d;
            else if (stage == 0) { stage = 1; ViewFrom({last.x - 900, last.y - 900, last.z + 500}, last, 60); }
            else if (stage == 1) { stage = 2; Snap("explosion"); }
        });
    add(0.2, [] { ViewPlayer(); });
    add(0.4, [] { Snap("osd_crash"); });
    add(0.2, [] { Log("--- respawn into the pouch, out by hand, on its back, then rolled back over with the roll stick (as FPV Kamikaze Drone does)"); Dev("hold"); });
    add(0.8, [] { Dev("release"); Dev("state"); Dev("grab"); });
    add(0.5, [] { Dev("drop away"); });
    add(1.5, [] { Dev("state"); Dev("up"); V3 d = DroneLoc(); ViewFrom({d.x - 60, d.y - 60, d.z + 35}, d, 50); });
    add(0.5, [] { Snap("drone_payload"); });
    add(0.2, [] { ViewPlayer(); Dev("down"); });
    add(0.1, [] { Dev("invert"); });
    add(0.6, [] { Dev("state"); Dev("input 0 0 0 1"); });  // full roll stick
    for (int i = 0; i < 4; i++) add(0.3, [] { Dev("state"); });  // about half a turn at full stick
    add(0.1, [] { Dev("input 0 0 0 0"); Dev("state"); });
    add(0.2, [] { Dev("input off"); Log("--- right stick held: called back, fresh pack"); Dev("hold"); });
    add(0.8, [] { Dev("release"); Dev("state"); });
    add(0.2, [] { Log("--- a second drone pouch: two drones on the vest, the one taken last flies"); Dev("vestpouch"); });
    add(1.0, [] { Dev("state"); Log("--- the waiting one taken"); Dev("grab waiting"); });
    add(0.5, [] { Dev("state"); Log("--- and put back"); Dev("drop"); });
    add(0.8, [] { Dev("state"); Dev("up"); V3 d = DroneLoc(); ViewFrom({d.x - 80, d.y - 60, d.z + 40}, d, 60); });
    add(0.6, [] { Snap("drone_two"); });
    add(0.2, [] { ViewPlayer(); Dev("down"); Log("--- a kit saved from the station's pouch, loaded and put on"); Dev("kitsave FPVTest"); });
    add(0.3, [] { Dev("kitload FPVTest"); });
    add(1.0, [] { Dev("state"); });
    add(0.2, [] { Log("--- flip up"); Dev("up"); });
    add(1.0, [] { Dev("state"); Snap("player_view_up"); });
    add(0.2, [] {
        Log("--- outfit save and load");
        Object gm = Gear();
        Params s(gm, "SaveOutfit"); s.Set("Preset", (int32_t)7).Call();
        Log("worn after save: {}", Worn(19) ? Object(Worn(19)).Name() : "none");
        SetItem(LoadObject(kGpnvg).ptr);
    });
    add(1.0, [] {
        Log("worn before load: {}", Worn(19) ? Object(Worn(19)).Name() : "none");
        Params l(Gear(), "LoadOutfit"); l.Set("Preset", (int32_t)7).Call();
    });
    add(1.5, [] {
        GUObject* w = Worn(19);
        Log("worn after load: {} - {}", w ? Object(w).Name() : "none", w && w == s_ours ? "the goggles (OK)" : "NOT the goggles");
        Dev("state");
        // The test's own outfit slot goes again (the runner also restores every save).
        Params del(Lib("GameplayStatics"), "DeleteGameInSlot");
        del.SetString("SlotName", L"SavedOutfit7").Set("UserIndex", (int32_t)0).Call();
        Log("test outfit slot deleted: {}", del.Return<bool>());
        Log("--- the gear menu tile");
        IconTest();
        Dev("hooks");
        Log("DEV TEST COMPLETE");
    });
}

GML_AWAKE() {
    On(GML_EVENT_WORLD_BEGIN_PLAY, [](void* gm) {
        std::string level = Object((GUObject*)gm).Path();
        if (level.find("/Game/Maps/MainMenu/") == 0) {
            Params p(Lib("GameplayStatics"), "OpenLevel");
            p.SetObj("WorldContextObject", Object(API->WorldContext())).Set("LevelName", API->MakeName(kMap)).Set("bAbsolute", true).Call();
        } else if (level.find(kMap) == 0 && s_steps.empty()) {
            Script();
            s_at = 0;
            s_wait = 7;
        }
    });
    On(GML_EVENT_TICK, [](void* data) {
        if (s_wait < 0 || s_at >= s_steps.size()) return;
        if ((s_wait -= data ? *(float*)data : 1.0 / 90) > 0) return;
        s_steps[s_at].fn();
        s_at++;
        s_wait = s_at < s_steps.size() ? s_steps[s_at].wait : -1;
    });
    return 0;
}
