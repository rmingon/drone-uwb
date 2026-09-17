import { expect, test, describe } from "bun:test";
import { solvePosition } from "../src/position";
import type { AnchorPoint } from "../src/layout";

const square: AnchorPoint[] = [
  { address: 1, x: 0, y: 0, z: 0 },
  { address: 2, x: 6, y: 0, z: 0 },
  { address: 3, x: 6, y: 6, z: 0 },
  { address: 4, x: 0, y: 6, z: 0 },
];

const tetra: AnchorPoint[] = [
  { address: 1, x: 0, y: 0, z: 0 },
  { address: 2, x: 6, y: 0, z: 0 },
  { address: 3, x: 3, y: 6, z: 0 },
  { address: 4, x: 3, y: 3, z: 4 },
];

const observe = (anchors: AnchorPoint[], at: [number, number, number], noise = 0) =>
  anchors.map((a, i) => ({
    anchor: a.address,
    distance:
      Math.hypot(a.x - at[0], a.y - at[1], a.z - at[2]) + (i % 2 === 0 ? noise : -noise),
  }));

describe("drone position", () => {
  test("finds a drone inside a square of anchors on the floor", () => {
    const truth: [number, number, number] = [2, 3, 1.5];
    const fix = solvePosition(square, observe(square, truth))!;

    expect(fix.x).toBeCloseTo(truth[0], 6);
    expect(fix.y).toBeCloseTo(truth[1], 6);
    expect(fix.z).toBeCloseTo(truth[2], 6);
    expect(fix.used).toBe(4);
    expect(fix.zAssumed).toBe(true);
  });

  test("solves height properly once the anchors are not all on one plane", () => {
    const truth: [number, number, number] = [2.5, 2, 1];
    const fix = solvePosition(tetra, observe(tetra, truth))!;

    expect(fix.x).toBeCloseTo(truth[0], 6);
    expect(fix.y).toBeCloseTo(truth[1], 6);
    expect(fix.z).toBeCloseTo(truth[2], 6);
    expect(fix.zAssumed).toBe(false);
  });

  test("stays close with ten centimetres of ranging noise", () => {
    const truth: [number, number, number] = [2, 3, 1.5];
    const fix = solvePosition(square, observe(square, truth, 0.1))!;

    expect(Math.hypot(fix.x - truth[0], fix.y - truth[1])).toBeLessThan(0.25);
  });

  test("works with the three anchors that are the bare minimum", () => {
    const three = square.slice(0, 3);
    const truth: [number, number, number] = [3, 2, 1];
    const fix = solvePosition(three, observe(three, truth))!;

    expect(fix.x).toBeCloseTo(truth[0], 6);
    expect(fix.y).toBeCloseTo(truth[1], 6);
    expect(fix.used).toBe(3);
  });

  test("lands on one of the two mirrored solutions when a tilted plane is all it has", () => {
    /* Three anchors define a plane and the drone can be on either side of it.
       Nothing in the distances says which, so the contract is that the fix is
       one of the two, exactly, not that it is the true one. */
    const spread: AnchorPoint[] = [
      { address: 1, x: 0, y: 0, z: 0 },
      { address: 2, x: 5.5, y: 0.2, z: 0 },
      { address: 4, x: 0.2, y: 4.6, z: 2.2 },
    ];
    const truth: [number, number, number] = [2.6, 2.4, 1.5];
    const measured = observe(spread, truth);
    const fix = solvePosition(spread, measured)!;

    expect(fix.zAssumed).toBe(true);
    for (const anchor of spread) {
      const reported = measured.find((m) => m.anchor === anchor.address)!.distance;
      expect(Math.hypot(fix.x - anchor.x, fix.y - anchor.y, fix.z - anchor.z))
        .toBeCloseTo(reported, 6);
    }
  });

  test("uses an obstructed distance rather than losing the fourth anchor", () => {
    /* The height would otherwise be a coin toss, see the test above. */
    const spread: AnchorPoint[] = [
      { address: 1, x: 0, y: 0, z: 0 },
      { address: 2, x: 5.5, y: 0.2, z: 0 },
      { address: 3, x: 5.1, y: 4.8, z: 0 },
      { address: 4, x: 0.2, y: 4.6, z: 2.2 },
    ];
    const truth: [number, number, number] = [2.6, 2.4, 1.5];
    const fix = solvePosition(spread, observe(spread, truth))!;

    expect(fix.zAssumed).toBe(false);
    expect(fix.z).toBeCloseTo(truth[2], 6);
  });

  test("handles anchors mounted on a wall rather than a floor", () => {
    /* all four in the x = 0 plane, which the world XY frame cannot describe */
    const wall: AnchorPoint[] = [
      { address: 1, x: 0, y: 0, z: 0 },
      { address: 2, x: 0, y: 6, z: 0 },
      { address: 3, x: 0, y: 6, z: 4 },
      { address: 4, x: 0, y: 0, z: 4 },
    ];
    const truth: [number, number, number] = [2.5, 3, 2];
    const fix = solvePosition(wall, observe(wall, truth))!;

    expect(Math.abs(fix.x)).toBeCloseTo(2.5, 6);
    expect(fix.y).toBeCloseTo(truth[1], 6);
    expect(fix.z).toBeCloseTo(truth[2], 6);
    expect(fix.zAssumed).toBe(true);
  });

  test("gives up rather than guessing from two distances", () => {
    expect(solvePosition(square, observe(square.slice(0, 2), [1, 1, 1]))).toBeNull();
  });

  test("ignores a distance to an anchor that has no coordinates yet", () => {
    const truth: [number, number, number] = [2, 3, 1.5];
    const withStranger = [...observe(square, truth), { anchor: 99, distance: 12 }];
    const fix = solvePosition(square, withStranger)!;

    expect(fix.used).toBe(4);
    expect(fix.x).toBeCloseTo(truth[0], 6);
  });
});
