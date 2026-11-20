# Arm Simulator

A browser-based articulated arm simulator. Load any URDF, and the simulator reads its joints and limits, renders the robot, and lets you drive it in joint space or Cartesian space. You can teach waypoints and replay them as a simple MoveJ / MoveL program.

It is a static site with no build step, so it runs on GitHub Pages as-is. The default robot is a **Universal Robots UR10** built from the official UR description.

**Live:** https://d-langan.github.io/Robot_Arm_Simulator/ (after Pages is enabled, see [Hosting](#hosting-on-github-pages))

## Features

- **Any URDF**: revolute, continuous, prismatic, fixed and mimic joints. Limits and velocity limits come from the file. Meshes can be STL, COLLADA (DAE), OBJ, glTF/GLB or PLY; box, cylinder and sphere primitives also work. Materials and colours come from the URDF.
- **Joint control**: one slider and one numeric field per joint, in degrees or millimetres, with limit and velocity readouts and a near-limit highlight.
- **Cartesian control**: the following all solve inverse kinematics live:
  - an absolute **slider + numeric field for each axis** (X, Y, Z, Rx, Ry, Rz)
  - a draggable **3D gizmo** (move/rotate, tool or world axes)
  - a **go-to pose** command
  - **jog buttons** (step or hold-to-move) in the reference frame or the tool frame

  You can pick any link as the end-effector, any fixed link as the reference frame, and set a TCP offset.
- **Programs**: save the current pose as a point, then set MoveJ or MoveL, speed and dwell per point. Run, pause, step to a single point, loop, and adjust the speed override. Points show in 3D with a path preview. Programs are saved per robot in the browser and export/import as JSON.
- **Display**: visual and collision geometry, link frames, joint axes, a kinematic skeleton (handy when meshes are missing), and the TCP frame. Light and dark themes.

## Loading your own robot

| How | What to give it |
|---|---|
| **Open folder** | The robot's package folder (the one containing `urdf/` and `meshes/`). Recommended. |
| **Open files** | The `.urdf` plus its mesh files, or a single `.zip` of the package. |
| **Drag & drop** | A `.urdf`, a folder, or a `.zip`, dropped anywhere on the page. |
| **URL** | A URDF hosted online, for example on `raw.githubusercontent.com`. You can also link directly: `…/Robot_Arm_Simulator/?urdf=<url-to-urdf>` |

Mesh references are resolved like this:

- `package://pkg/meshes/x.stl`, relative paths, and `file://` paths are matched against the uploaded files by the longest matching path suffix, case-insensitively.
- For URL loads, `package://` assumes the usual `<pkg>/urdf/robot.urdf` layout.
- Missing meshes are listed in the robot card, and the robot still loads.

**xacro** files have to be expanded first, for example `xacro robot.urdf.xacro > robot.urdf` (ROS 2: `ros2 run xacro xacro …`; without ROS: `pip install xacro`). URDFs whose `lower == upper` (common in SolidWorks exports) get a default ±180° / ±0.5 m range and a "no limits" badge.

## Controls

| Key | Action |
|---|---|
| `J` / `C` | Joint / Cartesian mode |
| `W` / `E` | Gizmo move / rotate |
| `Q` | Gizmo tool / world axes |
| `P` | Save current pose as a point |
| `Space` | Run / pause program |
| `Esc` | Stop program |
| `F` | Fit view |

Any manual input (sliders, gizmo, jog) stops a running program.

**MoveJ** interpolates all joints together. The slowest joint, relative to its URDF velocity limit times the point's speed %, sets the duration.

**MoveL** moves the TCP in a straight line, with orientation slerped. The path is solved by IK every ~2 mm / 0.5° before the move starts. If any part of the line is unreachable or passes through a singularity, the move is refused with a message and the robot does not move. Its duration also respects joint velocity limits.

## Program file format

```json
{
  "format": "urdf-arm-sim-program",
  "version": 1,
  "robot": "ur10",
  "points": [
    { "name": "P1", "move": "J", "speed": 50,  "dwell": 0,   "joints": { "shoulder_pan_joint": 0.0, "…": 0 } },
    { "name": "P2", "move": "L", "speed": 250, "dwell": 0.5, "joints": { "…": 0 } }
  ]
}
```

Joint values are in radians (or metres for prismatic joints). MoveJ speed is a percentage of each joint's velocity limit, and MoveL speed is TCP mm/s. Each point also stores the TCP pose for reference, but motion always targets the stored joint values.

## Hosting on GitHub Pages

1. Push this repository to GitHub.
2. Go to **Settings → Pages → Build and deployment**, set **Source: Deploy from a branch**, and choose **Branch: `main`**, folder **`/ (root)`**.
3. After a minute the site is at `https://<user>.github.io/<repo>/`.

No build step is needed. `.nojekyll` stops Jekyll from processing the site. three.js is loaded from jsDelivr through the import map in `index.html`, pinned to `0.170.0`. GitHub Pages paths are case-sensitive, so keep file names in `robots/` exactly as the URDF references them.

### Adding built-in robots

Put the URDF and meshes under `robots/<name>/` with relative (or `package://`) mesh paths, then add an entry to `robots/index.json`:

```json
{ "id": "my_arm", "name": "My Arm", "urdf": "robots/my_arm/my_arm.urdf", "home": { "joint_2": -1.57 } }
```

`home` is optional. Link to it with `?robot=my_arm`.

## Local development

ES modules don't load from `file://`, so serve the folder:

```sh
npm install        # dev-only: three (for tests), xmldom, puppeteer-core
npm run serve      # http://localhost:8080
npm test           # kinematics tests (FK vs. UR10 DH table, Jacobian, IK)
npm run e2e -- --fanuc <path-to-another-urdf-package>   # headless Chrome/Edge smoke test
```

`npm run serve` needs only Node, with no dependencies. Any static server works too, e.g. `python -m http.server`.

## Project layout

```
index.html            page shell + import map
css/style.css         layout and themes
js/main.js            app wiring: loading, modes, view options, HUD, shortcuts
js/viewer.js          three.js scene, Z-up camera, lights, grid
js/urdf/              parser (URDF → model), asset resolvers, mesh loaders, robot scene graph
js/kinematics/        kinematic chain (FK, Jacobian) and damped-least-squares IK
js/cartesian.js       gizmo, Cartesian sliders, jog, go-to
js/program/           program format, MoveJ/MoveL planning and playback, 3D markers
js/ui/                joint, Cartesian and program panels
robots/               built-in robots (UR10)
tests/                unit tests and the browser smoke test
tools/serve.mjs       tiny static server for local use
```

## Credits and licenses

- Application code: MIT (see `LICENSE`).
- UR10 model: [Universal Robots ROS 2 Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description), BSD-3-Clause (`robots/ur10/LICENSE-ur_description`). Flattened from xacro with `ur_type:=ur10`.
- [three.js](https://threejs.org/) (MIT) and [fflate](https://github.com/101arrowz/fflate) (MIT), loaded from jsDelivr.
