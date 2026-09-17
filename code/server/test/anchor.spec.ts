import { expect, test, describe, beforeEach } from "bun:test";
import { Anchor } from "../src/anchor";
import { getRange, getRanges, getUsableRanges, reset, RANGE_MAX_AGE_MS } from "../src/ranging";

const ip = "192.168.1.1";
const eui = "aabbccddeeff0001";

const measurement = (over: Partial<Parameters<Anchor["setRange"]>[0]> = {}) => ({
  address: 5,
  range: 3.42,
  raw_range: 3.51,
  rx_power: -82.3,
  fp_power: -85.1,
  los: true,
  ...over,
});

describe("Anchor", () => {
  beforeEach(() => reset());

  test("keeps the identity it was announced with", () => {
    const anchor = new Anchor("anchor1", ip, 1, true);
    expect(anchor.id).toBe("anchor1");
    expect(anchor.ip).toBe(ip);
    expect(anchor.address).toBe(1);
    expect(anchor.main).toBe(true);
  });

  test("drops a measurement for a short address no main anchor announced", () => {
    const anchor = new Anchor("anchor1", ip);
    anchor.setRange(measurement());
    expect(getRanges(eui)).toHaveLength(0);
  });

  test("stores a measurement once the address is known", () => {
    const anchor = new Anchor("anchor1", ip, 1, true);
    anchor.registerTag({ eui, address: 5 });
    anchor.setRange(measurement());

    const sample = getRange("anchor1", eui);
    expect(sample?.distance).toBe(3.42);
    expect(sample?.rawDistance).toBe(3.51);
    expect(sample?.los).toBe(true);
  });

  test("collects one distance per anchor for the same tag", () => {
    const main = new Anchor("anchor1", ip, 1, true);
    const second = new Anchor("anchor2", "192.168.1.2", 2);
    main.registerTag({ eui, address: 5 });
    main.setRange(measurement({ range: 3.4 }));
    second.setRange(measurement({ range: 5.1 }));

    expect(getRanges(eui).map((s) => s.distance).sort()).toEqual([3.4, 5.1]);
  });

  test("keeps obstructed measurements out of the trilateration set", () => {
    const main = new Anchor("anchor1", ip, 1, true);
    const second = new Anchor("anchor2", "192.168.1.2", 2);
    main.registerTag({ eui, address: 5 });
    main.setRange(measurement({ range: 3.4 }));
    second.setRange(measurement({ range: 9.9, los: false }));

    expect(getRanges(eui)).toHaveLength(2);
    expect(getUsableRanges(eui).map((s) => s.distance)).toEqual([3.4]);
  });

  test("forgets a measurement once it is too old to describe a moving drone", () => {
    const anchor = new Anchor("anchor1", ip, 1, true);
    anchor.registerTag({ eui, address: 5 });
    anchor.setRange(measurement());

    const later = Date.now() + RANGE_MAX_AGE_MS + 1;
    expect(getRanges(eui, later)).toHaveLength(0);
  });
});
