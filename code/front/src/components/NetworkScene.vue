<script setup lang="ts">
/*
 * The installation in three dimensions: anchors where the survey puts them,
 * a labelled link for every measured distance, and the drones on top.
 *
 * The solver works in a Z up frame, three.js is Y up, so every point goes
 * through toScene(). Positions, rotations and colours are passed as three.js
 * objects rather than arrays because that is what the Tres prop types expect.
 */
import { computed } from 'vue'
import { Color, Euler, Quaternion, Vector3 } from 'three'
import { OrbitControls } from '@tresjs/cientos'
import { getLabel, pruneLabels } from '../three/labels'
import type { AnchorPoint, Snapshot } from '../types'

const props = defineProps<{ snapshot: Snapshot | null }>()

const UP = new Vector3(0, 1, 0)

const ANCHOR_COLOUR = new Color('#22d3ee')
const ANCHOR_GLOW = new Color('#0e7490')
const MAIN_COLOUR = new Color('#fbbf24')
const MAIN_GLOW = new Color('#78350f')
const LINK_COLOUR = new Color('#0ea5e9')
const LINK_BLOCKED = new Color('#ef4444')
const DRONE_COLOUR = new Color('#f472b6')
const DRONE_GLOW = new Color('#9d174d')
const SIGHT_BLOCKED = new Color('#7f1d1d')

const toScene = (p: { x: number; y: number; z: number }) => new Vector3(p.x, p.z, p.y)

/** Position, orientation and length of a cylinder spanning two points. */
const segment = (from: Vector3, to: Vector3) => {
  const direction = to.clone().sub(from)
  const length = direction.length()
  if (length === 0) return null

  return {
    length,
    position: from.clone().add(to).multiplyScalar(0.5),
    rotation: new Euler().setFromQuaternion(
      new Quaternion().setFromUnitVectors(UP, direction.clone().normalize()),
    ),
  }
}

const layout = computed<AnchorPoint[]>(() => props.snapshot?.layout ?? [])
const mainAddress = computed(() => props.snapshot?.anchors.find((a) => a.main)?.address ?? -1)
const points = computed(() => new Map(layout.value.map((p) => [p.address, toScene(p)])))

const anchors = computed(() =>
  layout.value.map((point) => ({
    address: point.address,
    main: point.address === mainAddress.value,
    position: toScene(point),
  })),
)

const links = computed(() =>
  (props.snapshot?.links ?? []).flatMap((link) => {
    const from = points.value.get(link.a)
    const to = points.value.get(link.b)
    if (!from || !to) return []

    const span = segment(from, to)
    if (!span) return []

    return [{ key: `${link.a}-${link.b}`, distance: link.distance, los: link.los, ...span }]
  }),
)

const drones = computed(() =>
  (props.snapshot?.tags ?? [])
    .filter((tag) => tag.position !== null)
    .map((tag) => {
      const position = toScene(tag.position!)
      /* the drop line makes the height readable against the grid */
      const height = Math.max(0.001, position.y)
      return {
        eui: tag.eui,
        position,
        dropHeight: height,
        dropPosition: new Vector3(position.x, height / 2, position.z),
      }
    }),
)

/** Distance from each anchor that still sees a drone. */
const sightLines = computed(() =>
  (props.snapshot?.tags ?? []).flatMap((tag) => {
    if (!tag.position) return []
    const target = toScene(tag.position)

    return tag.ranges.flatMap((range) => {
      const from = points.value.get(range.anchor)
      if (!from) return []

      const span = segment(from, target)
      if (!span) return []

      return [{ key: `${tag.eui}-${range.anchor}`, los: range.los, ...span }]
    })
  }),
)

/* Sprites are cached by key; drop the ones whose anchor or drone is gone. */
const sprites = computed(() => {
  const onLinks = links.value.map((link) => {
    const key = `link:${link.key}`
    const sprite = getLabel(
      key,
      `${link.distance.toFixed(2)} m`,
      link.los ? '#7dd3fc' : '#fca5a5',
    )
    sprite.position.copy(link.position)
    return { key, sprite }
  })

  const onAnchors = anchors.value.map((anchor) => {
    const key = `anchor:${anchor.address}`
    const sprite = getLabel(
      key,
      anchor.main ? `A${anchor.address} main` : `A${anchor.address}`,
      anchor.main ? '#fcd34d' : '#e2e8f0',
    )
    sprite.position.copy(anchor.position).setY(anchor.position.y + 0.45)
    return { key, sprite }
  })

  const all = [...onLinks, ...onAnchors]
  pruneLabels(new Set(all.map((s) => s.key)))
  return all
})

/** Frames the whole installation without fighting the user's orbiting. */
const cameraPosition = computed(() => {
  const all = [...points.value.values()]
  if (all.length === 0) return new Vector3(7, 5, 7)

  const centre = all
    .reduce((sum, p) => sum.add(p), new Vector3())
    .multiplyScalar(1 / all.length)
  const radius = Math.max(2, ...all.map((p) => p.distanceTo(centre)))

  return new Vector3(
    centre.x + radius * 1.7,
    centre.y + radius * 1.3,
    centre.z + radius * 1.7,
  )
})
</script>

<template>
  <TresCanvas clear-color="#020617">
    <TresPerspectiveCamera :position="cameraPosition" :fov="45" />
    <OrbitControls :enable-damping="true" />

    <TresAmbientLight :intensity="1.4" />
    <TresDirectionalLight :position="new Vector3(6, 10, 6)" :intensity="1.6" />
    <TresGridHelper :args="[40, 40, '#1e3a5f', '#132033']" />

    <!-- anchors -->
    <TresMesh v-for="anchor in anchors" :key="anchor.address" :position="anchor.position">
      <TresSphereGeometry :args="[0.17, 32, 24]" />
      <TresMeshStandardMaterial
        :color="anchor.main ? MAIN_COLOUR : ANCHOR_COLOUR"
        :emissive="anchor.main ? MAIN_GLOW : ANCHOR_GLOW"
        :emissive-intensity="0.6"
        :roughness="0.35"
      />
    </TresMesh>

    <!-- measured distance between two anchors -->
    <TresMesh
      v-for="link in links"
      :key="link.key"
      :position="link.position"
      :rotation="link.rotation"
    >
      <TresCylinderGeometry :args="[0.022, 0.022, link.length, 8]" />
      <TresMeshBasicMaterial
        :color="link.los ? LINK_COLOUR : LINK_BLOCKED"
        :transparent="true"
        :opacity="0.75"
      />
    </TresMesh>

    <!-- distance from an anchor to a drone -->
    <TresMesh
      v-for="line in sightLines"
      :key="line.key"
      :position="line.position"
      :rotation="line.rotation"
    >
      <TresCylinderGeometry :args="[0.008, 0.008, line.length, 6]" />
      <TresMeshBasicMaterial
        :color="line.los ? DRONE_COLOUR : SIGHT_BLOCKED"
        :transparent="true"
        :opacity="line.los ? 0.5 : 0.25"
      />
    </TresMesh>

    <!-- drones -->
    <template v-for="drone in drones" :key="drone.eui">
      <TresMesh :position="drone.position">
        <TresIcosahedronGeometry :args="[0.2, 0]" />
        <TresMeshStandardMaterial
          :color="DRONE_COLOUR"
          :emissive="DRONE_GLOW"
          :emissive-intensity="0.8"
        />
      </TresMesh>
      <TresMesh :position="drone.dropPosition">
        <TresCylinderGeometry :args="[0.004, 0.004, drone.dropHeight, 4]" />
        <TresMeshBasicMaterial :color="DRONE_COLOUR" :transparent="true" :opacity="0.35" />
      </TresMesh>
    </template>

    <primitive v-for="sprite in sprites" :key="sprite.key" :object="sprite.sprite" />
  </TresCanvas>
</template>
