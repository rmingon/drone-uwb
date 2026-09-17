/*
 * Where a drone is, from its distances to anchors whose coordinates the survey
 * already gave us.
 *
 * Four spheres meet in one point, three in two points mirrored about the plane
 * through their centres. With measurement noise they do not meet at all, so
 * this solves the overdetermined system in the least squares sense rather than
 * intersecting anything.
 */

import type { AnchorPoint } from "./layout";

export interface Position {
  x: number;
  y: number;
  z: number;
  /** anchors that contributed */
  used: number;
  /**
   * The anchors used were all in one plane, so the drone could be on either
   * side of it. The side above the plane was chosen.
   */
  zAssumed: boolean;
}

export interface Observation {
  anchor: number;
  distance: number;
}

/** How far out of the common plane an anchor may sit and still count as in it. */
const COPLANAR_EPSILON = 0.05;

type Vec = [number, number, number];

const sub = (a: Vec, b: Vec): Vec => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a: Vec, b: Vec): Vec => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (a: Vec, k: number): Vec => [a[0] * k, a[1] * k, a[2] * k];
const dot = (a: Vec, b: Vec) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a: Vec, b: Vec): Vec => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const norm = (a: Vec) => Math.sqrt(dot(a, a));
const unit = (a: Vec): Vec => scale(a, 1 / norm(a));

/** Gaussian elimination with partial pivoting. Returns null on a singular system. */
const solve = (a: number[][], b: number[]): number[] | null => {
  const n = b.length;
  const m = a.map((row, i) => [...row, b[i]!]);

  for (let col = 0; col < n; col++) {
    let pivot = col;
    for (let row = col + 1; row < n; row++) {
      if (Math.abs(m[row]![col]!) > Math.abs(m[pivot]![col]!)) pivot = row;
    }
    if (Math.abs(m[pivot]![col]!) < 1e-12) return null;
    [m[col], m[pivot]] = [m[pivot]!, m[col]!];

    for (let row = 0; row < n; row++) {
      if (row === col) continue;
      const factor = m[row]![col]! / m[col]![col]!;
      for (let k = col; k <= n; k++) m[row]![k]! -= factor * m[col]![k]!;
    }
  }

  /* full Gauss-Jordan leaves one non zero per row, on the diagonal */
  return m.map((row, i) => row[n]! / row[i]!);
};

/** Normal equations of A x = b. */
const leastSquares = (a: number[][], b: number[]): number[] | null => {
  const width = a[0]?.length ?? 0;
  if (width === 0 || a.length < width) return null;

  const ata = Array.from({ length: width }, () => new Array(width).fill(0));
  const atb = new Array(width).fill(0);

  for (let row = 0; row < a.length; row++) {
    for (let i = 0; i < width; i++) {
      atb[i] += a[row]![i]! * b[row]!;
      for (let j = 0; j < width; j++) ata[i]![j] += a[row]![i]! * a[row]![j]!;
    }
  }

  return solve(ata, atb);
};

/**
 * Linearises |x - p_i| = d_i by subtracting the first equation from the others,
 * in whatever coordinates the callers hands over.
 */
const multilaterate = (centres: number[][], distances: number[]): number[] | null => {
  const base = centres[0]!;
  const baseDistance = distances[0]!;
  const squared = (p: number[]) => p.reduce((sum, v) => sum + v * v, 0);

  const rows: number[][] = [];
  const rhs: number[] = [];

  for (let i = 1; i < centres.length; i++) {
    rows.push(centres[i]!.map((v, axis) => 2 * (v - base[axis]!)));
    rhs.push(
      squared(centres[i]!) - squared(base) -
        distances[i]! * distances[i]! + baseDistance * baseDistance,
    );
  }

  const solution = leastSquares(rows, rhs);
  if (!solution || solution.some((v) => !Number.isFinite(v))) return null;
  return solution;
};

export const solvePosition = (
  layout: AnchorPoint[],
  observations: Observation[],
): Position | null => {
  const byAddress = new Map(layout.map((p) => [p.address, p]));

  const fixes = observations
    .map(({ anchor, distance }) => ({ point: byAddress.get(anchor), distance }))
    .filter((f): f is { point: AnchorPoint; distance: number } => f.point !== undefined);

  if (fixes.length < 3) return null;

  const centres: Vec[] = fixes.map((f) => [f.point.x, f.point.y, f.point.z]);
  const distances = fixes.map((f) => f.distance);

  /* Build the plane through the first three anchors. Everything is measured in
     that frame, so a tilted or wall mounted installation works the same as a
     floor mounted one. */
  const origin = centres[0]!;
  const alongFirst = sub(centres[1]!, origin);
  if (norm(alongFirst) < 1e-9) return null;
  const e1 = unit(alongFirst);

  const towardsThird = sub(centres[2]!, origin);
  const inPlane = sub(towardsThird, scale(e1, dot(towardsThird, e1)));
  if (norm(inPlane) < 1e-9) return null; // the three are in line
  const e2 = unit(inPlane);
  const normal = cross(e1, e2);

  const local = centres.map((c) => {
    const d = sub(c, origin);
    return [dot(d, e1), dot(d, e2), dot(d, normal)];
  });

  const spread = local.map((p) => p[2]!);
  const coplanar = Math.max(...spread) - Math.min(...spread) < COPLANAR_EPSILON;

  /* Out of plane anchors are the only thing that can tell the two mirrored
     solutions apart, and three anchors never can. */
  if (!coplanar && fixes.length >= 4) {
    const solution = multilaterate(local, distances);
    if (!solution) return null;

    const world = add(
      origin,
      add(add(scale(e1, solution[0]!), scale(e2, solution[1]!)), scale(normal, solution[2]!)),
    );
    return { x: world[0], y: world[1], z: world[2], used: fixes.length, zAssumed: false };
  }

  const flat = local.map((p) => [p[0]!, p[1]!]);
  const solution = multilaterate(flat, distances);
  if (!solution) return null;

  const u = solution[0]!;
  const v = solution[1]!;

  /* Lift perpendicular to the anchor plane with one sphere. Both signs fit, so
     take the one on the far side from the origin of the world frame, which for
     a floor mounted set is up. */
  const offset = Math.sqrt(
    Math.max(0, distances[0]! * distances[0]! - (u - local[0]![0]!) ** 2 - (v - local[0]![1]!) ** 2),
  );

  const inPlanePoint = add(origin, add(scale(e1, u), scale(e2, v)));
  const above = add(inPlanePoint, scale(normal, offset));
  const below = add(inPlanePoint, scale(normal, -offset));
  const world = above[2] >= below[2] ? above : below;

  return { x: world[0], y: world[1], z: world[2], used: fixes.length, zAssumed: true };
};
