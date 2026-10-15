// FPVDroneQuick - dev build: straight into the drone. Not shipped.
// Skips the main menu (once per launch) into TrainingGrounds, then in any level: puts on a helmet that
// takes NVGs if the worn one doesn't, puts on the FPV goggles, flips them down and spawns the drone in
// front of you, ready to fly. To remove it, delete GML\plugins\FPVDroneQuick.
#include <GML/GML.hpp>
#include <windows.h>
#include <cstring>

using namespace gml;

GML_PLUGIN("com.remarrow.fpvdronequick", "FPVDroneQuick", "0.1.0", {{"com.remarrow.fpvdrone", "0.2.0", GML_DEPENDENCY_SOFT}});

static const char* kMap = "/Game/Maps/Training/TrainingGrounds";

static void Dev(const char* cmd) {
    static auto fn = (void (*)(const char*))GetProcAddress(GetModuleHandleW(L"FPVDrone.dll"), "FPV_Dev");
    if (fn) fn(cmd);
    else Error("FPVDrone.dll's FPV_Dev not found");
}
static Object Pawn() {
    Params p(Lib("GameplayStatics"), "GetPlayerPawn");
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

// A helmet with an NVG mount (gear category 5), if the worn one has none.
static bool HelmetForNvg() {
    Object gm = Gear();
    uint8_t* worn = MapValue(gm, "GearByCategory", 5);
    Object helmet(worn ? *(GUObject**)worn : nullptr);
    if (helmet && helmet.Get<bool>("bAllowsNVG")) return false;
    auto* arr = (GML_TArray*)MapValue(Object(API->WorldContext()), "OutfitItems", 5);
    for (int i = 0; arr && i < arr->Num; i++) {
        Object h(((GUObject**)arr->Data)[i]);
        if (!h.Get<bool>("bAllowsNVG")) continue;
        Log("putting on {} (it takes NVGs)", h.Name());
        uint8_t item[16] = {};
        *(GUObject**)item = h.ptr;
        Params p(gm, "SetOutfitItem");
        std::memcpy(p.Ptr("GearItem"), item, 16);
        p.Call();
        return true;
    }
    return false;
}

static bool s_skippedMenu = false;
static int s_step = -1;
static double s_wait = 0;

GML_AWAKE() {
    On(GML_EVENT_WORLD_BEGIN_PLAY, [](void* gm) {
        std::string level = Object((GUObject*)gm).Path();
        if (level.find("/Game/Maps/MainMenu/") == 0) {
            s_step = -1;
            if (s_skippedMenu) return;  // back in the menu on purpose: stay there
            s_skippedMenu = true;
            Params p(Lib("GameplayStatics"), "OpenLevel");
            p.SetObj("WorldContextObject", Object(API->WorldContext())).Set("LevelName", API->MakeName(kMap)).Set("bAbsolute", true).Call();
        } else {
            s_skippedMenu = true;
            s_step = 0;
            s_wait = 4;
        }
    });
    On(GML_EVENT_TICK, [](void* data) {
        if (s_step < 0 || (s_wait -= data ? *(float*)data : 1.0 / 90) > 0) return;
        switch (s_step) {
        case 0:  // the player and their gear are there
            if (!Gear()) { s_wait = 1; return; }
            s_wait = HelmetForNvg() ? 1.5 : 0.2;
            break;
        case 1: Dev("equip"); s_wait = 2; break;
        case 2: Dev("down"); s_wait = 0.5; break;
        case 3:
            Dev("spawn");
            Log("QUICK FLY ready: push the throttle up from the middle to fly; Y or Start respawns the drone.");
            s_step = -1;
            return;
        }
        s_step++;
    });
    return 0;
}
