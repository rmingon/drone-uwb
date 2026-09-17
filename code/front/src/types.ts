/* Mirror of the snapshot code/server/src/snapshot.ts publishes. */

export interface AnchorPoint {
  address: number;
  x: number;
  y: number;
  z: number;
}

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

export interface TagRange {
  anchor: number;
  distance: number;
  los: boolean;
}

export interface Position {
  x: number;
  y: number;
  z: number;
  used: number;
  zAssumed: boolean;
}

export interface SnapshotTag {
  eui: string;
  ranges: TagRange[];
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
