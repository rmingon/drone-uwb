/*
 * Text in the 3D scene.
 *
 * Sprites drawn on a canvas rather than real geometry: no font file to ship, and
 * they always face the camera, which is what a measurement readout wants.
 */

import { CanvasTexture, Sprite, SpriteMaterial } from "three";

const WIDTH = 256;
const HEIGHT = 64;
/** World height of a label, independent of how far the camera is. */
const SCALE = 0.42;

interface Label {
  sprite: Sprite;
  canvas: HTMLCanvasElement;
  texture: CanvasTexture;
  text: string;
}

const labels = new Map<string, Label>();

const paint = (label: Label, text: string, color: string) => {
  const ctx = label.canvas.getContext("2d");
  if (!ctx) return;

  ctx.clearRect(0, 0, WIDTH, HEIGHT);
  ctx.font = "600 34px ui-monospace, SFMono-Regular, Menlo, monospace";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";

  const width = ctx.measureText(text).width + 28;
  ctx.fillStyle = "rgba(2, 6, 23, 0.82)";
  ctx.beginPath();
  ctx.roundRect((WIDTH - width) / 2, 8, width, HEIGHT - 16, 10);
  ctx.fill();

  ctx.fillStyle = color;
  ctx.fillText(text, WIDTH / 2, HEIGHT / 2);

  label.texture.needsUpdate = true;
  label.text = text;
};

/**
 * One sprite per key, repainted in place when the text changes. Rebuilding them
 * every frame would churn GPU memory for no reason.
 */
export const getLabel = (key: string, text: string, color = "#e2e8f0"): Sprite => {
  let label = labels.get(key);

  if (!label) {
    const canvas = document.createElement("canvas");
    canvas.width = WIDTH;
    canvas.height = HEIGHT;
    const texture = new CanvasTexture(canvas);
    const sprite = new Sprite(new SpriteMaterial({ map: texture, depthTest: false, transparent: true }));
    sprite.scale.set((SCALE * WIDTH) / HEIGHT, SCALE, 1);
    sprite.renderOrder = 10;
    label = { sprite, canvas, texture, text: "" };
    labels.set(key, label);
  }

  if (label.text !== text) paint(label, text, color);
  return label.sprite;
};

/** Frees the sprites whose key is gone, for instance an anchor that dropped. */
export const pruneLabels = (keep: Set<string>) => {
  for (const [key, label] of labels) {
    if (keep.has(key)) continue;
    label.texture.dispose();
    label.sprite.material.dispose();
    labels.delete(key);
  }
};
