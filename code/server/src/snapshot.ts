/*
 * The picture the web interface renders, assembled from the stores.
 */

import { solveLayout, type AnchorPoint } from "./layout";
import { getAnchors, getPeerDistance, getPeerLinks } from "./network";
import { solvePosition, type Position } from "./position";
import { getKnownTags, getRanges } from "./ranging";

export interface SnapshotAnchor {
  id: string;
  address: number;
  main: boolean;
  ip: string;
  lastSeen: number;
}

export interface SnapshotLink {
  a: number;
  b: number;
  distance: number;
  los: boolean;
  at: number;
}

export interface SnapshotTagRange {
  anchor: number;
  distance: number;
  los: boolean;
}

export interface SnapshotTag {
  eui: string;
  ranges: SnapshotTagRange[];
  /** null until three anchors with known coordinates have a clear distance */
  position: Position | null;
}

export interface Snapshot {
  type: "state";
  at: number;
  anchors: SnapshotAnchor[];
  links: SnapshotLink[];
  layout: AnchorPoint[] | null;
  tags: SnapshotTag[];
}

export const buildSnapshot = (now = Date.now()): Snapshot => {
  const anchors = getAnchors(now);
  const byAddress = new Map(anchors.map((a) => [a.id, a.address]));

  const layout = solveLayout(
    anchors.map((a) => a.address),
    (a, b) => getPeerDistance(a, b, now),
  );

  const tags: SnapshotTag[] = [];
  for (const eui of getKnownTags()) {
    const ranges = getRanges(eui, now)
      .map((sample) => ({
        anchor: byAddress.get(sample.anchor) ?? 0,
        distance: sample.distance,
        los: sample.los,
      }))
      .sort((l, r) => l.anchor - r.anchor);
    if (ranges.length === 0) continue;

    /* Obstructed measurements read long, so they are left out of the fix. But
       dropping one can cost the fourth distance, and without a fourth the two
       mirrored solutions are indistinguishable and the height is a coin toss.
       A biased distance settles that better than a convention does. */
    const clear = ranges.filter((r) => r.los);
    const position = layout
      ? solvePosition(layout, clear.length >= 4 ? clear : ranges)
      : null;
    tags.push({ eui, ranges, position });
  }

  return {
    type: "state",
    at: now,
    anchors: anchors
      .map(({ id, address, main, ip, lastSeen }) => ({ id, address, main, ip, lastSeen }))
      .sort((l, r) => l.address - r.address),
    links: getPeerLinks(now).sort((l, r) => l.a - r.a || l.b - r.b),
    layout,
    tags,
  };
};
