/*
 * The anchor network: who is connected, and how far apart they are.
 *
 * Anchors measure each other with the same two way ranging they use on the
 * drones, so the shape of the installation is discovered rather than entered by
 * hand. See base/code/src/ranging.cpp, the self survey.
 */

/** An anchor that has not been heard from in this long is treated as gone. */
export const ANCHOR_TIMEOUT_MS = 15000;

/** Anchors do not move, so a leg of the survey stays valid much longer than a
 *  distance to a drone. */
export const LINK_MAX_AGE_MS = 60000;

export interface AnchorInfo {
  id: string;      // anchor uniq, its MAC
  ip: string;
  address: number; // short address in the ranging chain
  main: boolean;
  lastSeen: number;
}

export interface PeerLink {
  a: number; // lower address
  b: number; // higher address
  distance: number;
  los: boolean;
  at: number;
}

const anchors = new Map<string, AnchorInfo>();
const links = new Map<string, PeerLink>();

const linkKey = (a: number, b: number) => (a < b ? `${a}|${b}` : `${b}|${a}`);

export const registerAnchor = (info: Omit<AnchorInfo, "lastSeen">, now = Date.now()) => {
  anchors.set(info.id, { ...info, lastSeen: now });
};

export const touchAnchor = (id: string, now = Date.now()) => {
  const anchor = anchors.get(id);
  if (anchor) anchor.lastSeen = now;
};

export const getAnchor = (id: string) => anchors.get(id);

export const getAnchors = (now = Date.now()) =>
  [...anchors.values()].filter((a) => now - a.lastSeen < ANCHOR_TIMEOUT_MS);

export const getAllAnchors = () => [...anchors.values()];

/**
 * One leg of the survey. Only the anchor that ran the exchange knows the
 * distance, so a leg is reported once, by whichever side computed it.
 */
export const setPeerLink = (
  from: number,
  to: number,
  distance: number,
  los: boolean,
  now = Date.now(),
) => {
  if (from === to) return;
  links.set(linkKey(from, to), {
    a: Math.min(from, to),
    b: Math.max(from, to),
    distance,
    los,
    at: now,
  });
};

export const getPeerLinks = (now = Date.now()) =>
  [...links.values()].filter((l) => now - l.at < LINK_MAX_AGE_MS);

export const getPeerDistance = (a: number, b: number, now = Date.now()) => {
  const link = links.get(linkKey(a, b));
  if (!link || now - link.at >= LINK_MAX_AGE_MS) return undefined;
  return link.distance;
};

export const reset = () => {
  anchors.clear();
  links.clear();
};
