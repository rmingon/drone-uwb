/*
 * Turns the survey distances into coordinates.
 *
 * Nobody measures the room: the anchors measure each other, and the shape
 * follows. The frame is arbitrary but deterministic, which is all a view needs.
 *
 *   first anchor  -> origin
 *   second        -> on the +X axis
 *   third         -> in the XY plane, +Y side
 *   the rest      -> trilaterated from those three, +Z side
 *
 * Mirror images are therefore not distinguished. Distances alone cannot tell
 * them apart, so the convention is part of the answer.
 */

export interface AnchorPoint {
  address: number;
  x: number;
  y: number;
  z: number;
}

/** Distance between two anchors, or undefined when that leg is not measured. */
export type DistanceLookup = (a: number, b: number) => number | undefined;

const sq = (v: number) => v * v;

/** Never let floating point noise turn a near zero into a NaN. */
const safeSqrt = (v: number) => Math.sqrt(Math.max(0, v));

/**
 * Returns a point per anchor, or null when the first three anchors are not
 * fully measured, which is the minimum for a frame to exist at all.
 */
export const solveLayout = (addresses: number[], distance: DistanceLookup): AnchorPoint[] | null => {
  const ordered = [...addresses].sort((a, b) => a - b);

  if (ordered.length === 0) return null;
  if (ordered.length === 1) return [{ address: ordered[0]!, x: 0, y: 0, z: 0 }];

  const [a0, a1, a2] = ordered;
  const d01 = distance(a0!, a1!);
  if (d01 === undefined || d01 <= 0) return null;

  const points: AnchorPoint[] = [
    { address: a0!, x: 0, y: 0, z: 0 },
    { address: a1!, x: d01, y: 0, z: 0 },
  ];

  if (ordered.length === 2) return points;

  const d02 = distance(a0!, a2!);
  const d12 = distance(a1!, a2!);
  if (d02 === undefined || d12 === undefined) return null;

  const x2 = (sq(d02) - sq(d12) + sq(d01)) / (2 * d01);
  const y2 = safeSqrt(sq(d02) - sq(x2));
  if (y2 === 0) return null; // the three are in line, no plane to build on

  points.push({ address: a2!, x: x2, y: y2, z: 0 });

  for (const address of ordered.slice(3)) {
    const d0 = distance(a0!, address);
    const d1 = distance(a1!, address);
    const d2 = distance(a2!, address);
    if (d0 === undefined || d1 === undefined || d2 === undefined) continue;

    const x = (sq(d0) - sq(d1) + sq(d01)) / (2 * d01);
    const y = (sq(d0) - sq(d2) + sq(x2) + sq(y2) - 2 * x2 * x) / (2 * y2);
    const z = safeSqrt(sq(d0) - sq(x) - sq(y));
    points.push({ address, x, y, z });
  }

  return points;
};
