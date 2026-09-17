import { expect, test, describe } from "bun:test";
import { solveLayout } from "../src/layout";

/** Pairwise distances of a known set of points, as the survey would report them. */
const distancesOf = (points: Record<number, [number, number, number]>) => {
  return (a: number, b: number) => {
    const p = points[a];
    const q = points[b];
    if (!p || !q) return undefined;
    return Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
  };
};

const between = (points: { x: number; y: number; z: number }[], i: number, j: number) =>
  Math.hypot(
    points[i]!.x - points[j]!.x,
    points[i]!.y - points[j]!.y,
    points[i]!.z - points[j]!.z,
  );

describe("anchor layout", () => {
  test("rebuilds a square room from its diagonals", () => {
    const truth = {
      1: [0, 0, 0],
      2: [4, 0, 0],
      3: [4, 3, 0],
      4: [0, 3, 0],
    } as Record<number, [number, number, number]>;

    const points = solveLayout([1, 2, 3, 4], distancesOf(truth));
    expect(points).not.toBeNull();
    expect(points).toHaveLength(4);

    // the frame is arbitrary, the distances are not
    expect(between(points!, 0, 1)).toBeCloseTo(4, 6);
    expect(between(points!, 1, 2)).toBeCloseTo(3, 6);
    expect(between(points!, 0, 2)).toBeCloseTo(5, 6);
    expect(between(points!, 0, 3)).toBeCloseTo(3, 6);
  });

  test("puts the network in the frame it promises", () => {
    const truth = {
      1: [7, -2, 5],
      2: [7, 1, 5],
      3: [11, 1, 5],
    } as Record<number, [number, number, number]>;

    const points = solveLayout([1, 2, 3], distancesOf(truth))!;

    expect(points[0]).toEqual({ address: 1, x: 0, y: 0, z: 0 });
    expect(points[1]!.y).toBeCloseTo(0, 6);
    expect(points[1]!.z).toBe(0);
    expect(points[1]!.x).toBeCloseTo(3, 6);
    expect(points[2]!.y).toBeGreaterThan(0);
  });

  test("lifts a fourth anchor out of the plane", () => {
    const truth = {
      1: [0, 0, 0],
      2: [4, 0, 0],
      3: [0, 4, 0],
      4: [1, 1, 2.5],
    } as Record<number, [number, number, number]>;

    const points = solveLayout([1, 2, 3, 4], distancesOf(truth))!;
    expect(points[3]!.z).toBeCloseTo(2.5, 6);
    expect(between(points, 0, 3)).toBeCloseTo(Math.hypot(1, 1, 2.5), 6);
  });

  test("refuses to invent a frame it cannot measure", () => {
    const partial = (a: number, b: number) => (a === 1 && b === 2 ? 3 : undefined);
    expect(solveLayout([1, 2, 3], partial)).toBeNull();
  });

  test("refuses three anchors in a straight line", () => {
    const inLine = {
      1: [0, 0, 0],
      2: [2, 0, 0],
      3: [5, 0, 0],
    } as Record<number, [number, number, number]>;
    expect(solveLayout([1, 2, 3], distancesOf(inLine))).toBeNull();
  });

  test("skips an anchor whose survey is incomplete rather than dropping the rest", () => {
    const truth = {
      1: [0, 0, 0],
      2: [4, 0, 0],
      3: [0, 4, 0],
    } as Record<number, [number, number, number]>;
    const lookup = distancesOf(truth);

    const points = solveLayout([1, 2, 3, 9], lookup)!;
    expect(points.map((p) => p.address)).toEqual([1, 2, 3]);
  });
});
