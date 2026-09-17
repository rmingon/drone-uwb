/*
 * Store of the distances the anchors measure.
 *
 * Anchors report a tag by the short address the main anchor handed it, because
 * that is all a secondary anchor sees during the exchange. The main anchor
 * publishes the address to EUI mapping, which is what turns a measurement into
 * something attributable to a drone.
 */

/** A drone moves, so a distance older than this tells us nothing useful. */
export const RANGE_MAX_AGE_MS = 2000;

export interface RangeSample {
  anchor: string;      // anchor uniq
  tag: string;         // tag EUI
  distance: number;    // meters, median filtered by the anchor
  rawDistance: number; // meters, this exchange alone
  rxPower: number;     // dBm
  fpPower: number;     // dBm, first path
  los: boolean;        // false when the direct path looks obstructed
  at: number;          // Date.now() of reception
}

const tagAddresses = new Map<number, string>();
const samples = new Map<string, RangeSample>();

const key = (anchor: string, tag: string) => `${anchor}|${tag}`;

export const registerTagAddress = (address: number, eui: string) => {
  tagAddresses.set(address, eui);
};

export const resolveTagAddress = (address: number) => tagAddresses.get(address);

/** Every tag an anchor has announced, which is who the ranging data is about. */
export const getKnownTags = () => [...new Set(tagAddresses.values())];

export const setRange = (sample: RangeSample) => {
  samples.set(key(sample.anchor, sample.tag), sample);
};

export const getRange = (anchor: string, tag: string, now = Date.now()) => {
  const sample = samples.get(key(anchor, tag));
  if (!sample || now - sample.at >= RANGE_MAX_AGE_MS) return undefined;
  return sample;
};

/** Every anchor that currently has a fresh distance to this tag. */
export const getRanges = (tag: string, now = Date.now()) =>
  [...samples.values()].filter((s) => s.tag === tag && now - s.at < RANGE_MAX_AGE_MS);

export const getAllRanges = (now = Date.now()) =>
  [...samples.values()].filter((s) => now - s.at < RANGE_MAX_AGE_MS);

/** Only the measurements worth trilaterating with. */
export const getUsableRanges = (tag: string, now = Date.now()) =>
  getRanges(tag, now).filter((s) => s.los);

export const reset = () => {
  samples.clear();
  tagAddresses.clear();
};
