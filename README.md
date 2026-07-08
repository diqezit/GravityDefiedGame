# Gravity Defied

Physics-based motorcycle trial game inspired by the classic J2ME mobile game from 2004

Navigate rough terrain, pull off wheelies and stoppies, reach the finish line without crashing

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
| RMB drag | Rotate camera (voxel mode) |
| G | Toggle ghost |
| F1 | Toggle sprites |
| F2 | Switch render mode |
| Escape | Pause / menu |

---

## Features

**Physics**
- Soft body simulation, 6 mass points connected by springs
- Wheel rotation with friction and slip
- Ground collision with proper normal response
- Crash detection based on bike angle and head impact
- Rider ragdoll on crash

**Terrain**
- Procedural generation using layered noise
- Macro shape for overall track flow, plus medium and micro detail
- Safe zones near start and finish for playability

**Visuals**
- Multiple color themes with custom sky palettes, stars and sun
- Vector graphics and sprite rendering
- Voxel style 3D render mode

**Gameplay**
- 10 procedurally generated levels
- 3 bike types with unique handling
- Ghost replay of your best run per level
- Time tracking per level
- Settings saved between sessions

---

## Themes

The game includes multiple visual themes. Each theme defines:

- Background and terrain colors
- Sky gradient palette
- Optional stars and sun
- Bike color scheme

Themes are configured in `Content/Data/World.json` and can be easily extended

---

## Build & Run

**Requirements**
- .NET 8.0 SDK

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

Springs connect all points with configurable stiffness and damping. Wheels have angular velocity affected by ground contact, engine torque, and braking

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

- [ ] **Level Editor** Create and share custom tracks with drag-and-drop terrain tools, adjustable difficulty, and export/import functionality
- [ ] **Leaderboards** Local and online best times for each level, personal records tracking, daily/weekly challenges
- [ ] **Sound & Music** Engine sounds reactive to throttle, crash effects, ambient music
- [ ] **Mobile Support** Touch controls with on-screen buttons, tilt-to-lean option
- [ ] **Achievements** Unlock badges for completing levels, performing stunts, reaching speed milestones
- [ ] **More Bikes** Additional bike types with unique physics, visual customization options
- [ ] **Track Packs** Themed level collections, community-created track sharing
- [ ] **Multiplayer** Split-screen local races, online time trials
- [ ] **Replay System** Save and watch full runs, export as video

---

## Credits

Inspired by the original **Gravity Defied**, released for J2ME phones in 2004

---

## License

MIT
