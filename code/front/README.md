# Interface

Live view of the anchor network: who is connected, how far apart the anchors
are, and where the drones are.

```sh
bun install
bun run dev
```

It connects to the server's websocket, `ws://<same host>:1227` by default.
Override with `VITE_SERVER_WS`.

With no hardware running there is nothing to look at, so `?demo=1` feeds the
view a generated installation, including a drone flying laps and an anchor that
loses line of sight now and then.

## What is on screen

| | |
| --- | --- |
| Amber sphere | the main anchor, the one tags blink at |
| Cyan spheres | the other anchors |
| Thick cyan links | a measured distance between two anchors, labelled in metres |
| Red link | the same, but the anchors could not see each other cleanly |
| Pink polyhedron | a drone, at the position solved from its distances |
| Thin pink lines | an anchor's distance to that drone, dark red when obstructed |

Anchor coordinates are not configured anywhere: they come from the anchors
measuring each other. The frame is therefore arbitrary but stable, with the
first anchor at the origin and the second on the X axis. The solver works Z up
and the scene is Y up, so points are mapped on the way in.

## Layout

| Path | |
| --- | --- |
| `src/stores/state.ts` | websocket connection, reconnects on its own |
| `src/stores/demo.ts` | the generated installation behind `?demo=1` |
| `src/components/NetworkScene.vue` | the 3D view |
| `src/components/StatusPanel.vue` | the side panel |
| `src/three/labels.ts` | distance labels, canvas sprites that face the camera |

The server sends a full snapshot four times a second, so there is no state to
merge here: the last message is the truth.
