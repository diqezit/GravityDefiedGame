# Gravity Defied

Physics-based motorcycle trial game inspired by the classic J2ME mobile game from 2004.

Navigate challenging terrain, perform wheelies and stoppies, reach the finish without crashing.

---

## Preview

![Gameplay](screenshots/gameplay.gif)

---

<img src="https://github.com/user-attachments/assets/b5f6bd51-cd47-46ad-b99e-115479421b7b" alt="Gameplay screenshot" width="100%" />

<details>
<summary>More screenshots</summary>

<br/>

<img src="https://github.com/user-attachments/assets/7958aab3-c905-407b-84a8-1861343c1e79" alt="Menu" width="100%" />

---

<img src="https://github.com/user-attachments/assets/553abac0-ed55-4d8c-888a-1176422e645c" alt="Level Select" width="100%" />

---

<img src="https://github.com/user-attachments/assets/82378647-ebe8-4c8c-8af1-c682f4e154f7" alt="Themes" width="100%" />

</details>

---

## Controls

| Key | Action |
|-----|--------|
| W | Accelerate |
| S | Brake |
| A | Lean back (wheelie) |
| D | Lean forward (stoppie) |
| Scroll | Zoom in/out |
| Escape | Pause / Menu |
| F1 | Toggle render mode |

---

## Features

**Physics**
- Soft-body simulation with 6 mass points connected by springs
- Realistic wheel rotation with friction and slip
- Ground collision with proper normal response
- Crash detection based on bike angle and head impact

**Terrain**
- Procedural generation using layered noise
- Macro skeleton for overall track flow
- Medium bumps and micro surface roughness
- Rest zones for balanced difficulty
- Safety constraints to keep levels playable

**Visuals**
- Multiple color themes with custom sky palettes
- Stars and sun effects
- Pseudo-3D terrain perspective
- Vector graphics and sprite rendering modes

**Gameplay**
- 10 procedurally generated levels
- 3 bike types with unique handling
- Time tracking per level
- Settings saved between sessions

---

## Bikes

| Type | Top Speed | Acceleration | Lean Force | Best For |
|------|-----------|--------------|------------|----------|
| Standard | Medium | Medium | 0.4 | Learning the game |
| Sport | High | High | 0.6 | Speed runs |
| OffRoad | Medium | High | 1.0 | Technical terrain |

---

## Themes

The game includes multiple visual themes. Each theme defines:

- Background and terrain colors
- Sky gradient palette
- Optional stars and sun
- Bike color scheme

Themes are configured in `Content/Data/World.json` and can be easily extended.

---

## Build & Run

**Requirements**
- .NET 8.0 SDK
- MonoGame 3.8.1

**Steps**

```bash
git clone https://github.com/yourusername/GravityDefiedGame.git
cd GravityDefiedGame
dotnet build
dotnet run
```

---

## How It Works

**Physics Model**

The bike uses a soft-body approach with 6 connected points

Springs connect all points with configurable stiffness and damping. Wheels have angular velocity affected by ground contact, engine torque, and braking.

**Terrain Generation**

Levels are built in layers:
1. Control points create the macro track shape
2. Domain warping breaks obvious patterns
3. Medium noise adds bumps
4. Micro noise adds surface texture
5. Constraints ensure playability

---

## Roadmap

**Planned Features**

- [ ] **Level Editor** — Create and share custom tracks with drag-and-drop terrain tools, adjustable difficulty, and export/import functionality

- [ ] **Leaderboards** — Local and online best times for each level, personal records tracking, daily/weekly challenges

- [ ] **Sound & Music** — Engine sounds reactive to throttle, crash effects, ambient music, optional retro J2ME style beeps

- [ ] **Mobile Support** — Touch controls with on-screen buttons, tilt-to-lean option, responsive UI for different screen sizes

- [ ] **Ghost Replays** — Race against your best run, download ghosts from leaderboards, watch and learn from top players

- [ ] **Achievements** — Unlock badges for completing levels, performing stunts, reaching speed milestones, no-crash runs

- [ ] **More Bikes** — Additional bike types with unique physics (trials bike, chopper, BMX), visual customization options

- [ ] **Track Packs** — Themed level collections (desert, snow, moon), community-created track sharing

- [ ] **Multiplayer** — Split-screen local races, online time trials, weekly tournaments

- [ ] **Replay System** — Save and watch full runs, export as video, share highlights

---

## Credits

Inspired by **Gravity Defied** by Codeline Oy (2004)

---

## License

MIT
