/*
 * A generated installation, so the interface can be looked at before any
 * hardware exists. Reached with ?demo=1.
 */

import type { Snapshot } from "../types";

const ANCHORS = [
  { address: 1, x: 0, y: 0, z: 0, main: true },
  { address: 2, x: 6.2, y: 0.4, z: 0.1 },
  { address: 3, x: 5.4, y: 5.8, z: 0 },
  { address: 4, x: 0.3, y: 5.2, z: 2.4 },
];

const distance = (a: (typeof ANCHORS)[number], b: (typeof ANCHORS)[number]) =>
  Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);

/** Ranging noise, so the numbers move the way real ones do. */
const jitter = (seed: number) => Math.sin(seed) * 0.02;

export const demoSnapshot = (): Snapshot => {
  const now = Date.now();
  const t = now / 1000;

  /* a drone doing slow laps at head height */
  const drone = {
    x: 3 + 2 * Math.cos(t / 3),
    y: 3 + 2 * Math.sin(t / 3),
    z: 1.6 + 0.3 * Math.sin(t / 2),
  };

  const links = [];
  for (let i = 0; i < ANCHORS.length; i++) {
    for (let j = i + 1; j < ANCHORS.length; j++) {
      links.push({
        a: ANCHORS[i]!.address,
        b: ANCHORS[j]!.address,
        distance: distance(ANCHORS[i]!, ANCHORS[j]!) + jitter(t + i * 7 + j),
        los: true,
        at: now,
      });
    }
  }

  return {
    type: "state",
    at: now,
    anchors: ANCHORS.map((a) => ({
      id: `demo${a.address}`,
      address: a.address,
      main: a.main === true,
      ip: `192.168.1.${10 + a.address}`,
      lastSeen: now,
    })),
    links,
    layout: ANCHORS.map(({ address, x, y, z }) => ({ address, x, y, z })),
    tags: [
      {
        eui: "aabbccddeeff0001",
        ranges: ANCHORS.map((a, i) => ({
          anchor: a.address,
          distance: Math.hypot(a.x - drone.x, a.y - drone.y, a.z - drone.z) + jitter(t + i),
          /* one anchor loses sight of it now and then */
          los: !(i === 2 && Math.sin(t / 4) > 0.6),
        })),
        position: { ...drone, used: 4, zAssumed: false },
      },
    ],
  };
};
