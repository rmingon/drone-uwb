import { expect, test, describe, beforeEach } from "bun:test";
import { Anchor } from "../src/anchor";
import {
  ANCHOR_TIMEOUT_MS,
  getAnchors,
  getPeerDistance,
  getPeerLinks,
  registerAnchor,
  reset,
} from "../src/network";

const peerMeasurement = (address: number, range: number, los = true) => ({
  address,
  range,
  raw_range: range + 0.05,
  rx_power: -80,
  fp_power: -83,
  los,
});

describe("anchor network", () => {
  beforeEach(() => reset());

  test("records a leg of the survey under both anchors", () => {
    registerAnchor({ id: "a1", ip: "10.0.0.1", address: 1, main: true });
    new Anchor("a1", "10.0.0.1", 1, true).setPeerRange(peerMeasurement(2, 4.2));

    expect(getPeerDistance(1, 2)).toBe(4.2);
    expect(getPeerDistance(2, 1)).toBe(4.2);
    expect(getPeerLinks()).toEqual([
      expect.objectContaining({ a: 1, b: 2, distance: 4.2, los: true }),
    ]);
  });

  test("ignores a measurement from an anchor that never announced itself", () => {
    new Anchor("ghost", "10.0.0.9", 3).setPeerRange(peerMeasurement(1, 2));
    expect(getPeerLinks()).toHaveLength(0);
  });

  test("keeps one entry per pair whichever side measured it", () => {
    registerAnchor({ id: "a1", ip: "10.0.0.1", address: 1, main: true });
    registerAnchor({ id: "a2", ip: "10.0.0.2", address: 2, main: false });
    new Anchor("a1", "10.0.0.1", 1, true).setPeerRange(peerMeasurement(2, 4.2));
    new Anchor("a2", "10.0.0.2", 2).setPeerRange(peerMeasurement(1, 4.25));

    expect(getPeerLinks()).toHaveLength(1);
    expect(getPeerDistance(1, 2)).toBe(4.25);
  });

  test("drops an anchor that stopped reporting", () => {
    registerAnchor({ id: "a1", ip: "10.0.0.1", address: 1, main: true });
    expect(getAnchors()).toHaveLength(1);
    expect(getAnchors(Date.now() + ANCHOR_TIMEOUT_MS + 1)).toHaveLength(0);
  });
});
