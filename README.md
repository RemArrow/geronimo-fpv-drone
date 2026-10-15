# FPV Drone for Geronimo

A [GML](https://github.com/RemArrow/geronimo-mod-loader) plugin: an FPV kamikaze drone that you fly
through DJI Goggles 2, with the flight model and controls of *FPV Kamikaze Drone* (HFM Games) and a
Betaflight OSD.

- **The goggles are gear.** *DJI Goggles 2 (FPV)* is a new entry in the gear menu's **Night Vision**
  list. Like any NVG it needs a helmet with an NVG mount. The goggles hang on a G24 mount: grab them
  and push them down, like NVGs, and their screen shows the drone's camera; push them up to fly line of
  sight.
- **The drone goes on your vest, like a grenade.** *DJI Mini 4K FPV* is a new pouch in the kit
  station, with the grenade, flashbang and slapcharge pouches. Take it and put it anywhere on your vest
  or belt: the drone hangs there, belly in. Take it in your hand, put it back on its spot, or let go of
  it anywhere else (set it down or throw it) and it flies. Put on several and you carry several drones;
  the one you took last is the one you fly.
- **A fresh drone.** Y, Start, or holding the right thumbstick in for half a second (a short press
  still kicks) puts a new drone, with a full battery and payload, in its pouch on the vest.
- **Fly it.** Use an **Xbox gamepad** or an **FPV radio in USB joystick mode**, mode 2, as in FPV
  Kamikaze Drone. There is no arming: push the throttle up and it flies.
- **See through it.** The goggles' screen shows the drone's FPV camera with a Betaflight OSD and the
  goggles' own readouts over it, also while you hold the drone.

Nothing in the game is replaced, and flipping the goggles down doesn't turn on night vision.

## Credits

**3D models and textures:** *DJI Mini 4K* and *DJI Goggles 2* (with G24 mount) by
**3DMA - 3D Military Assets** ([www.3dmilitaryassets.com](https://www.3dmilitaryassets.com)), used
under license. They are built into the plugin DLL encrypted, and may not be extracted from it or
redistributed.

Plugin by RemArrow. Not affiliated with or endorsed by 3DMA, DJI, Betaflight, HFM Games or Dark Matter
Studios.

## Flying

| | Gamepad (mode 2) | FPV radio |
|---|---|---|
| Throttle | left stick up from the middle | throttle stick |
| Yaw | left stick left and right | yaw stick |
| Pitch, roll | right stick | right stick |
| Respawn | Y, Start, or hold the right thumbstick | right thumbstick |
| Zoom | hold a trigger (up to 3×) | a gamepad's triggers |
| Camera tilt | LB down, RB up, 5° a press | `[Camera] Uptilt` |

It flies exactly as FPV Kamikaze Drone's kamikaze drone does: on the engine's physics, with that game's
drone body and the same per-frame code, read from its own Blueprints, physics asset and settings (see
[The flight model](#the-flight-model)). These are its numbers, not settings:

- **Turning:** the sticks set the turn rate directly, with nothing in between. It turns the moment you
  move a stick, at a rate in proportion to it, and stops dead when you let go. Full stick is 180°/s of
  roll and pitch and 250°/s of yaw, less the body's angular damping once a frame, so the rate depends on
  the frame rate, as in FPV Kamikaze Drone (120°/s of roll at 60 fps, 140 at 90). On its back, roll it
  over with the stick: the turn is set even on the ground.
- **Throttle:** the stick's middle is zero and all the way up is full; letting go cuts it. Thrust is
  linear in the stick, with no motors to spin up. It hovers at about 35%, and near the ground the
  ground effect lifts it more (up to twice the thrust 10 cm up).
- **Speed:** thrust fades as it rushes along its thrust axis, to nothing at 102 km/h, which is what
  caps its speed: about 76 km/h straight up and 130 km/h flat out at 50° nose-down.
- **Momentum:** the air barely slows it (damping 0.25/s), so it carries its speed.
- **Sticks:** the gamepad deadzone is 0.25, with the stick rescaled past it, as in FPV Kamikaze Drone.
- **The payload:** the DJI model's payload release and bomb hang under the drone, and it is a kamikaze
  charge. As with FPV Kamikaze Drone's warheads, it changes nothing about how the drone flies. It has
  that game's impact fuse: once the drone has been 10 m from you (safe at your feet, so a hop on
  take-off can't set it off), any hit while it is going faster than 5 km/h sets it off. The first blast
  is the drone's own grenade, the second another M67 beside it, and they are yours, so what they kill
  counts as yours. The drone is gone and the goggles show *No Signal*; respawn it for another. The OSD
  shows *2 x M67 SAFE*, then *ARMED* once the fuse is live. `[Payload] Enabled = false` flies it bare.
- **Impacts without a payload:** it bounces, slides and tumbles off things as FPV Kamikaze Drone's does
  (the same body on the same physics), and stays under control.
- **Battery:** it lasts 120 s, stepping down once a second; then control is lost, as there: the damping
  drops to almost nothing and it is kicked into a spin. *LOW BATTERY* comes up at 30% and *LAND NOW*
  at 10%.
- **Range:** the radio link weakens with distance and with walls between you and the drone. When link
  quality falls too low, *RXLOSS* comes up, and after a second the motors stop (failsafe). When the
  signal is very weak the picture freezes (*Weak signal*).

`[Drone] FlightModel = Sim` swaps this for a full quadcopter simulation instead: motors, a Betaflight
rate PID with airmode, a LiPo, Betaflight arming if `Arming = Switch`, and crashes that cut the
motors. See [The Sim model](#the-sim-model).

## On screen

The OSD is laid out like a DJI O3 air unit's Betaflight OSD, in a blocky font of its own.

- **Top left:** RSSI (dBm) and link quality.
- **Top right:** current (A) and throttle (%).
- **Left:** mAh used, average cell voltage and pack voltage, following the battery's charge.
- **Bottom right:** flight time.
- **Bottom centre:** the flight mode, ACRO.
- **Middle:** warnings, in Betaflight's order: *RXLOSS*, *LAND NOW*, *LOW BATTERY*, *BATTERY EMPTY*.

Around the edges are the goggles' own readouts:
- signal bars and channel
- latency and bitrate
- the drone's voltage
- time since spawn
- the card's free recording time

The screen says *No Signal* when there is no drone. The motors make a synthesised whine that follows
the throttle.

## The flight model

FPV Kamikaze Drone's drone is a Chaos rigid body that its Blueprint (`BP_Drone_FPV` on `BP_Drone_Base`)
pushes every frame; the engine does the rest. The plugin does the same, read from that game's assets:
the Blueprints' bytecode and defaults, its two curves, its physics asset (`PHYS_Drone_FPV`), its input
mappings and its project settings, which match Geronimo's where it counts (no substepping, no async
physics, a 33 ms step cap, default gravity and physical material).

- **The body.** When the drone is let go, the grenade under it becomes that body: a box 21.4 long, 22.2
  wide and 3.8 cm high, 2 cm under the drone's middle, and a capsule 5 cm round and 15 cm long along the
  nose, 4 cm over it; 0.1 kg shared between them by volume, as one body's mass is; CCD; the engine's
  default physical material; linear damping 0.25 and angular 20 (the Blueprint's BeginPlay). In a hand
  or a pouch it is the grenade again.
- **Each frame,** as the Blueprint:
  1. **Turning.** While any stick is off centre, the body's angular velocity is set to stick ×
     `RotationRate` (180, 180, 250°/s) about its own axes; when they come back to centre, to zero.
     Chaos then applies the angular damping once per step, which is why the turn rate depends on the
     frame rate.
  2. **Thrust.** A force of `Thrust` (350, i.e. 3.5 N) × throttle × `ThrustCurve`(speed along the up
     axis, km/h) × `GroundeffectCurve`(what a 10 m ray down finds, cm), along the body's up axis.
     Throttle is the stick clamped to 0..1.
     - **Thrust curve:** 0.8 when still or sinking, 1.0 at 30 km/h, 0.75 at 60 and 0 at 102.
     - **Ground-effect curve:** 1.0 with no ground within the ray; 2.0 at 10 cm, 1.25 at 60 cm, 1.0
       from 117 cm.
  3. **Drag.** A force along the nose of (world X speed / 1000)² × 0.5 × 1.225 × 0.063 × 0.03, which is
     next to nothing, kept as the Blueprint has it.
- **The battery** (`ConsumeBattery`) loses 1/120 once a second. At zero, `ControlLost`: no more thrust
  or turning, damping 0.01 and 0 (`SetDefaultDamping`) and a random kick of spin.
- **The fuse** (the warheads' `CheckHit`): a hit on something other than the drone while it is going
  faster than `DetonateSpeed`. The plugin finds the hit by sweeping the body's outline along each
  frame's way.

`src/kamikaze.h` holds the curves and numbers, and the same model integrated by hand, which
`tests/quadsim` checks offline (`quadsim kamikaze`):

| | |
|---|---|
| hover | 35% throttle |
| full throttle, straight up | 18 m/s after 1 s, 76 km/h at most |
| full throttle, 35° / 50° / 65° nose-down | 111 / 130 / 128 km/h |
| full stick at 60 fps | 120°/s roll, stops dead on release |
| zero-throttle fall | 39 m/s |
| battery | 120 s |

## The Sim model

With `FlightModel = Sim`, `src/quad.h` simulates a real quadcopter at 1 kHz:

- **The drone builder** (`[Build]`): propeller size and pitch, motor KV, cells, capacity and weight.
  From those come thrust (Abbott's propeller formula), a frame sized for the propellers, motor lag and
  power. The default is a 2.6 kg 7-inch on 6S.
- **Physics settings** (`[Physics]`): gravity, motor response, body drag, rotor drag ("air grip") and
  thrust lost to speed.
- **Motors:** they spin up and down with a lag. Thrust grows with speed squared, and propeller torque
  yaws the frame.
- **Flight controller:** Betaflight ACTUAL rates (`[Rates]`), RC smoothing, and a rate PID with
  feed-forward, D on measurement, I-term relax and airmode. Angle mode is on Y.
- **Battery:** a LiPo with an open-circuit voltage curve and internal resistance.
- **Air:** drag, rotor drag, thrust lost at speed, ground effect and prop wash.

## How it works

1. **Gear entry.** After the game builds its gear lists (`BP_GameInstance::GetItems`), a new
   `OutfitItem_NightVision` is added to Night Vision. It carries the GPNVG-18's settings (its NVG actor,
   flip lever and helmet mesh), a name, an icon and no night-vision post-process. The menu takes its
   icons from the item's mesh name, so the menu tile is given the goggles' own icon.
2. **Saving.** The game saves an outfit as its items' asset paths, and the goggles have none, because
   they are built at runtime. While an outfit is saved, the goggles' slot therefore holds the GPNVG-18
   entry with the colour name `FPVGoggles`. When an outfit is loaded, that pair becomes the goggles
   again. Without the plugin, such a save loads a GPNVG-18.
3. **On the helmet.** The game hangs its NVG actor on the VR camera. On that actor the GPNVG-18's meshes
   are hidden and its lever is kept, since its grip is what flips them. The G24's helmet half goes where
   the game's Wilcox mount sits. The folding half (rack, slider, DJI bracket, goggles) rides the lever
   itself: it is fitted at the lever's down angle, set there for a moment without its events, so what
   you see folds exactly as what your hand holds. The lever turns about its own origin, 4.7 cm behind
   the G24's hinge, so first that origin (its place and the `InitialRelativeTransform` its angle is
   applied to) is moved onto the hinge: the goggles fold about the hinge and stay on the mount.
4. **Flipping down.** The lever's event would power night vision. For the goggles it is caught, and the
   goggles' screen comes on instead.
5. **The drone on the vest.** The kit station lists `BP_GameInstance.Pouches` by `EPouchType`, which
   `GetItems` fills from every `PouchData` asset. After it, a new `PouchData` joins the explosives (type
   2, with the grenade, flashbang and slapcharge pouches): the name *DJI Mini 4K FPV*, the drone's icon,
   the grenade pouch's mesh (for its collision and attach socket) and, as its item (`Pouchable`), the
   grenade pouch's own: the M67 (`BP_M67_C`, a `GrenadeActor`). The drone is the grenade underneath:
   its grip, storing, physics and explosion are the game's grenade code, in Operations as anywhere.
   - **In the kit station** the pouch you take (`EditablePouch`) shows the drone where it will hang,
     with the grenade pouch hidden and a box the drone's size to take it by. You put it on like any
     pouch.
   - **On the vest** a pouch (`Pouch_C`) spawns its item and stores it (`CheckSpawnStorable`). The
     plugin hides the grenade pouch and dresses the grenade: its mesh, pin, pin ring and spoon hidden,
     the pin's physics kept (the grenade hangs it on a constraint when held) but no hand may grip it, so
     its fuse never burns. The drone hangs on it at its own scale with the grenade at the drone's bomb,
     so a hand holds the drone by it, with the grenade's own grip; a box the drone's size (query only,
     so it never pushes anything) lets a hand find it. Letting go of it at the pouch stores it again;
     `StoreToPouch` is hooked so that drones go only into drone pouches and nothing else does (not a
     real grenade either), and a drone always lands square on its pouch, belly in and nose up (the
     grenade's `PouchRelativeTransform`, from the pouch mesh's `AttachSocket`).
   - **In a hand** it stays as it was taken (the grenade snaps to the hand's grip, the drone turns on
     it so that it doesn't).
   - **Who moves it.** While the actor is held (`VRGripInterfaceSettings.bIsHeld`) or stored (attached
     to the pouch) the game moves it. Let go anywhere else and its physics is switched off and the
     flight model takes over from the speed it was let go at. It never hits or lands on you or what you
     wear.
   - **Several drones.** Each drone pouch holds its own. The one you fly is the one taken last; the
     others are built but quiet, and one set down lies where it lands (engine physics) until a hand
     takes it.
   - **Kits.** A kit is saved with its pouches' asset paths, and ours has none. While a kit is saved the
     drone's pouch is the grenade pouch with the colour name `FPVDrone`; when a kit loads,
     `GetKitSetup`'s kit (read from its cast, since the out parameter is written through the caller's
     frame) gets ours back. Without the plugin such a kit has a grenade pouch there.
   - **Spent.** When the payload goes off, the first blast is the drone's own grenade, moved to where
     it hit and set off with the grenade's `Explode` (its radial damage credited to you, effects, light,
     scorch, the AI's hearing and `OnExplode`); the rest of `[Payload] Grenades` are M67s spawned
     beside it. The grenade then removes itself as a thrown one does, and the pouch is empty until you
     call a fresh drone, which the pouch spawns again.
6. **The screen** is a 16:9 quad 30 cm ahead of your eyes, 52° across, like the real goggles' 51°,
   inside a black box that hides everything else. It shows a render target with the engine's unlit glTF
   material.
   - **Camera:** the FPV camera is a 120° scene capture fixed to the frame with a 10° up-tilt (FPV Kamikaze Drone's defaults). It renders
     into the screen's texture only while the goggles are down. It hands over the picture before tone
     mapping, so the game tone-maps it once like everything else you see.
   - **OSD:** drawn on top with a canvas. Its font is a 5 × 7 pixel font built into the plugin and turned
     into a texture at startup.
   - **Spectators:** the game's PC-monitor view shows the screen too, so whoever watches sees the FPV
     feed.
7. **Input.**
   - **The right-stick hold** comes from the player character's own input events.
   - **Gamepads** are read through XInput.
   - **Radios** are plain HID joysticks, which Unreal doesn't read, so they are read through winmm. Their
     names come from the registry, and Xbox and XInput pads in that list are skipped.
8. **The link.** Ten times a second, RSSI is worked out from distance plus a loss for each wall that a
   trace from you to the quad passes through. Link quality, latency and bitrate follow from it.
9. **Blueprint calls.** The gear menu saves, loads and fills its tiles with Blueprint functions that
   call each other directly, so they never pass through `ProcessEvent`, which is where GML hooks. The
   plugin therefore hooks the script VM's `ProcessLocalScriptFunction`, which every script call goes
   through. It finds the function by following the call to it in `UObject::ProcessInternal` (every
   Blueprint function's `ExecFunction`), and picks out the few functions it needs by their `UFunction`.

## Assets

`tools/prepare_assets.py` (Blender, headless) builds `Assets/` from the two 3DMA downloads. `Assets/`
and the models' files are never committed.

- **Goggles** are exported in camera axes around the G24's hinge, which sits 11.5 cm ahead of the eye
  and 8.6 cm above it. There are two parts: what stays on the helmet and what folds.
- **Drone.** The model's atlas has two problems for the game: both arms use the same texels, and its
  "MINI 4K" lettering is a floating decal with an opacity map, which the game's opaque shader can't
  show. So body, camera and propellers get one new layout where nothing overlaps.
  - The atlas is baked onto it with Cycles: colour, AO, roughness and metallic as emission, and the
    normal in tangent space.
  - The decals are baked from just above the surface and blended in.
  - The propellers and camera are exported around their own pivots.
- **Textures** are converted for the game's weapon shader (`M_DefaultShader`): Diffuse, Normal and ORM
  (R = AO, G = roughness, B = metallic). Normal and ORM are pre-encoded for the game's sRGB import, and
  OpenGL normals get their green channel flipped.
- **Icon:** a render of the goggles.
- `fpv.txt` holds the pivots the plugin needs.

`tools/pack` encrypts the assets (AES-256-GCM, a new key every build) into the payload that
`src/FPVDrone.rc` builds into the DLL (`src/payload.h`).

## Install

Needs [GML 2.2.0 or newer](https://github.com/RemArrow/geronimo-mod-loader/releases). Close the game,
or install it from GML Manager's Browse tab. By hand: download `FPVDrone-<version>.zip` from
[Releases](https://github.com/RemArrow/geronimo-fpv-drone/releases) and extract it into `...\GERONIMO\Geronimo\Binaries\Win64\`. That adds
`GML\plugins\FPVDrone\`. To remove the plugin, delete that folder.

## Build from source

The models aren't in this repo. Buy *DJI Mini 4K* and *DJI Goggles 2* from 3DMA and extract both.
Then:

```bat
set FPV_GOGGLES_DIR=...\Goggles_DJI
set FPV_DRONE_DIR=...\DJI_4k_v01
blender.exe --background --python tools\prepare_assets.py
build.bat install    rem refuses while the game runs
```

Needs Visual Studio 2022 (C++ desktop workload) and Blender. For the offline physics tests, run
`cl /std:c++20 /O2 /EHsc tests\quadsim\quadsim.cpp && quadsim` from a VS developer prompt.

## Settings: `GML\config\com.remarrow.fpvdrone.cfg`

| Key | Default | |
|---|---|---|
| `[Goggles] Enabled` | true | the goggles in the gear menu |
| `[Goggles] ScreenDistance` | 30 | how far ahead the screen seems to be, cm |
| `[Goggles] ScreenFOV` | 52 | how wide the screen looks, degrees |
| `[Drone] SpawnHold` | 0.5 | seconds to hold the right stick |
| `[Drone] FlightModel` | Kamikaze | Kamikaze: FPV Kamikaze Drone exactly (its numbers, no settings). Sim: a full quadcopter simulation |
| `[Drone] Volume` | 1 | motor sound |
| `[Pouch] Enabled` | true | the drone's pouch in the kit station (next launch) |
| `[Payload] Enabled` | true | the bomb under the drone, set off on impact |
| `[Payload] Grenades` | 2 | M67 detonations where it hits |
| `[Payload] DetonateSpeed` | 5 | km/h; a hit faster than this sets it off (FPV Kamikaze Drone's) |
| `[Payload] ArmDistance` | 10 | m from you before the fuse is live (0: live at once) |
| `[Drone] Mode` | Acro | Sim: flight mode at spawn, Acro or Angle |
| `[Drone] Arming` | Auto | Sim: Auto, live from spawn; Switch, Betaflight arming (A or the arm switch) |
| `[Drone] CrashSpeed` | 7 | Sim: m/s; harder hits cut the motors |
| `[Build] PropSize`, `PropPitch` | 7, 4 | Sim: propeller diameter and pitch, inches |
| `[Build] MotorKV` | 1300 | Sim: motor rpm per volt |
| `[Build] Cells`, `Capacity` | 6, 3000 | battery cells in series, mAh (Kamikaze: the OSD's voltage and mAh) |
| `[Build] Weight` | 2600 | Sim: all-up weight, g |
| `[Build] MinThrottle` | 0 | Sim: motor speed at zero throttle (0: they stop) |
| `[Physics] Gravity`, `InstantPower`, `AirFriction`, `AirGrip`, `RelativeAirSpeed` | 9.82, 1.2, 0.45, 0.8, 1 | Sim: gravity, motor response, body drag, rotor drag, thrust lost to speed |
| `[Rates] RollPitch`, `Yaw` | 30,220,0.3 / 30,160,0.2 | Sim: Betaflight ACTUAL rates (centre, max °/s, expo) |
| `[Rates] Smoothing` | 0.09 | Sim: seconds the sticks are smoothed over |
| `[Camera] FOV` | 120 | camera field of view |
| `[Camera] Uptilt` | 10 | camera up-tilt, degrees (near 40 you fly where you look) |
| `[Camera] Zoom` | 3 | zoom with a trigger fully in |
| `[Camera] Width`, `Height` | 1280, 720 | camera resolution |
| `[Camera] OSD` | true | the Betaflight OSD over the picture |
| `[Camera] OSDBrightness` | 1.4 | OSD white level (above 1 stays white over a bright sky) |
| `[Camera] HDR` | true | tone-mapped once, by the game |
| `[Controls] Device` | Auto | Auto (a radio if plugged in, else a gamepad), Gamepad or Radio |
| `[Controls] VRSticks` | false | also fly with the VR thumbsticks; they move you too |
| `[Controls] GamepadMode` | 2 | 1 puts the throttle on the right stick |
| `[Controls] GamepadThrottle` | Centre | Centre: the stick's middle is zero throttle. Full: the bottom is zero |
| `[Controls] Sensitivity` | 1 | stick curve: 1.5 is softer round the middle, 0.5 sharper |
| `[Controls] Deadzone` | 0.25 | gamepad stick dead zone, rescaled past it (FPV Kamikaze Drone's) |
| `[Radio] Axes` | XYZR | joystick axes carrying roll, pitch, throttle and yaw (EdgeTX/OpenTX AETR) |
| `[Radio] Invert` | | axes to reverse, e.g. `Y` |
| `[Radio] Arm` | U | with `Arming = Switch`, the arm switch: an axis letter (armed above the middle; channel 5 is U) or `B<n>` for button n |

The log names a radio when it connects. If a stick moves the wrong thing, change `[Radio] Axes`; the
letters are X, Y, Z, R, U and V.

## Testing

`tests/FPVDroneDev` is the in-game harness. It is built but never installed. Its run:

1. Puts on an NVG-capable helmet.
2. Fits the game's GPNVG-18 for reference, then the goggles, and logs where every part sits relative to
   the eye.
3. Flips the goggles up, halfway, nearly down and down, logging the gap between the folding half's
   hinge and the mount's.
4. Lists the kit station's explosives pouches and turns a grenade pouch on the vest into the drone's,
   as a kit with one loads. Spawns the kit station's pouch for it. The right hand takes the drone, lets
   go of it at the vest (back into the pouch), takes it again and lets go of it away from the vest.
5. Flies a scripted sequence:
   - throttle on the ground
   - a punch-out, then a hover, then a 3x zoom
   - a pitch forward, hands off
   - (Sim: angle mode)
   - a full roll
   - 25 m out and 3 m up, an idle drop: the payload goes off
   - a fresh drone from the pouch, on its back, then rolled over with the roll stick
   - calling it back
6. Checks the right-stick hold.
7. Puts on a second drone pouch, takes the waiting drone and puts it back.
8. Saves a kit from the kit station's pouch, loads it and puts it on.
9. Saves and loads an outfit with the goggles, then deletes the test slot.
10. Builds a gear menu tile to check its icon.

A camera of its own takes close-ups; the PC-monitor view shows the goggles' screen. For the goggles'
close-ups (the headset lies on the floor, so the body is posed oddly round the head) it hides the
character's body, and shows it again once they're done. Run it through a script that backs up and
restores `SaveGames` and the GML configs, and take it out of `plugins` afterwards.

Verified in game:

- **Gear menu:** the goggles are in the Night Vision list, 6 → 7 items, and the menu tile shows their
  own icon.
- **Goggles:**
  - They fit on the GPNVG-18's actor.
  - The lever reads 0° up and −130° down.
  - Flipping down shows the FPV feed with the OSD, or *No Signal* with no drone, and no night vision
    comes on.
  - The folding half stays on the mount's hinge (0.00 cm apart) up, halfway, nearly down and down.
- **On the vest:**
  - The kit station's explosives pouches go from 4 to 5, the drone's with its name and icon; its
    kit-station pouch shows the drone, with the grenade pouch under it hidden.
  - A drone pouch on the vest spawns its drone (an M67 underneath), belly in and nose up; in the hand
    it stays as it was taken and stays held.
  - Let go at the vest, it goes back in its pouch; let go away from it, it falls (0.1 m/s), arms and
    flies.
  - With two drone pouches there are two drones; taking the waiting one makes it the one you fly.
  - A kit saved from the kit station's drone pouch loads back with it and puts it on the vest, and a
    kit saved in the kit station by hand loads with its drone pouch at the start.
  - The payload arms 25 m out and goes off on impact (24 km/h): the drone's own grenade, then a second
    M67. A recall then puts a fresh drone in its pouch.
- **Spawn:** live straight away, FPV Kamikaze Drone model.
- **Flight (Kamikaze model, on the engine's physics):**
  - The flight body weighs 0.100 kg, the capsule's share included.
  - At 75% throttle it climbs to 15 m/s, with the payload as without.
  - At 35% it carries its climb, slowing only by the light damping, as in FPV Kamikaze Drone.
  - A nudge of pitch tips it 18° forward; the moment the stick centres it stops turning and holds the
    angle.
  - A full roll stick turns it over in the air, and stops dead on release.
  - On its back on the ground, the sticks still turn it (the yaw stick spun it there at 170°/s).
  - Trigger zoom narrows the picture 3×.
  - The payload's hit is found on the body's way: it went off at 42 km/h on impact.
- **Link:** −84 dBm and LQ 100 about 50 m away.
- **Recall:** gives a fresh pack.
- **Save:** an outfit saved with the goggles loads back with the goggles.

## Multiplayer

Everyone needs the plugin. Each player flies their own drone and sees everyone else's.

- **What travels.** A save and the network both carry only asset paths and names, and the goggles and the
  drone's pouch are built at runtime. So the outfit really holds the GPNVG-18 entry with the colour name
  `FPVGoggles`, and the vest the grenade pouch with `FPVDrone`. Every machine with the plugin reads those
  pairs as the goggles and the drone pouch, and both spawn the same NVG actor and M67 as ours. The game
  recolours pouches to the kit's camo, so each machine remembers a drone pouch once seen, and the host puts
  the name back where the game changed it.
- **Other players' drones and goggles.** Every machine dresses every player's drone pouches' grenades as
  drones (their look, their motors' sound while they move) and their NVG actors as the goggles (no screen:
  that is theirs).
- **Who moves what.** The host owns every actor, the drone's grenade too. The host's own drone flies as
  alone, and the game sends everyone its grenade's movement. A client flies its drone on its own machine and
  sends the host where it is 30 times a second; the host moves the grenade there for everyone. The client
  ignores the host's copy of that movement while it flies, so its flight isn't dragged back. The messages
  ride the PlayerController's `ServerSetSpectatorLocation` (unreliable), their location raised by 1000 km per
  kind, which nothing the game sends ever is; the host's plugin takes those and keeps them from the game.
- **The payload** goes off on the host, wherever the drone hit: its grenade's `Explode`, damage and kills
  credited to its pilot, and the rest of the grenades beside it. Every machine hides the drone when its
  grenade goes off (`Explode_Multicast`).
- **A fresh drone** (the right-stick hold, Y, Start) is spawned by the host into one of the client's drone
  pouches, and dressed on every machine when it arrives.

Verified in a two-player test (two copies of the game on one PC, `Utils\MPDev`, 2026-10-13): both players'
goggles and both vests' drones dressed on both machines; the client's drone, let go and flown 12 m out, at the
same place on the host (−1024, 33, 593 against 595); its hit set off by the host (two grenades at the spot); a
fresh drone in the client's pouch, dressed on both; the host's drone seen flying out on the client and its hit.
The single-player test (`tests/FPVDroneDev`) gives the same results as 0.6.0.

## Known limitations

- Multiplayer: a client's drone reaches the other machines at 30 updates a second (unreliable), so it
  can trail its pilot's view by a fraction of a second. Other players' propellers spin and their motors
  hum while their drone moves, from its movement, not its throttle.
- Multiplayer, not tested yet: a client picking the drone pouch in the kit station (its pouch goes to the
  vest as the grenade pouch marked `FPVDrone`, but how the kit station sends a client's pouches to the host
  wasn't exercised). Tested: a kit saved with drone pouches, loaded by a client.
- The motor sound is synthesised; the game has no drone sound to borrow.
- The model is a DJI Mini 4K carrying its own bomb; there is no FPV frame model. Its body for the
  physics is FPV Kamikaze Drone's, which is a little different in shape. The charge is the game's M67
  grenades, not FPV Kamikaze Drone's warheads.
- The turn rate depends on the frame rate, as in FPV Kamikaze Drone: in a headset at 90 fps it is that
  game's rate at 90 fps, so if you fly FPV Kamikaze Drone at another frame rate it turns a little
  differently.
- The fuse finds the hit with a sweep of the body's outline along each frame's way, not the engine's
  hit event (which the plugin can't receive), so a pass within a centimetre of something at speed also
  counts.
- In the hand the drone uses the M67's grip: the hand closes round the grenade under the drone. The
  in-game test takes it with a scripted grip, not a VR hand, and places the kit station's pouch by
  script, not by hand on the kit builder.
- While you fly, the VR controllers still move you (VRSticks off) and hands raised in front of your face
  show through the goggles' black.
