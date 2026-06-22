# Robot Arm Digital Twin

Rust-first robotic arm simulator with inverse kinematics, a browser-based 3D view, and a path toward controlling a small physical servo arm.

I am building this as one of the first real `VineethMakes` projects: something visual enough to demo quickly, but technical enough to show robotics fundamentals. The core math lives in Rust so the same kinematics layer can eventually power a web demo, a desktop tool, or hardware commands.

## What It Does

- Models a 4-joint arm: base yaw, shoulder, elbow, and wrist pitch.
- Solves inverse kinematics for a target point in 3D.
- Enforces joint limits so poses stay plausible.
- Renders the arm in the browser with Three.js.
- Compiles the Rust solver to WebAssembly for the web UI.

## Why Rust

The interesting part of this project is the robot logic, not the web controls. Rust owns:

- Forward kinematics
- Inverse kinematics
- Joint limits
- Reachability and error reporting
- Unit tests around the solver

The JavaScript is intentionally thin: it loads the WASM module, renders the scene, and wires up the target sliders.

## Repo Layout

```text
.
|-- Cargo.toml
|-- crates
|   `-- arm-kinematics
|       `-- src/lib.rs
`-- web
    |-- index.html
    `-- src
        |-- app.js
        `-- styles.css
```

## Run It

Install Rust and the WASM build tool:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
cargo install wasm-pack
```

Build the Rust solver for the browser:

```bash
wasm-pack build crates/arm-kinematics --target web --out-dir ../../web/pkg --features wasm
```

Serve the web folder:

```bash
cd web
python3 -m http.server 8080
```

Open `http://localhost:8080`.

## Test The Rust Core

```bash
cargo test
```

## Next Steps

- Add mouse dragging for the target sphere.
- Add a ghost trail for planned motion.
- Export joint commands for an Arduino or ESP32 servo controller.
- Add collision hints for the base and table.
- Record a short demo video for the GitHub README and LinkedIn.

## About

Built by Vineeth Velmurugan through `VineethMakes` as a robotics, automation, 3D simulation, and hardware-control portfolio project.
