<script setup lang="ts">
import { computed } from 'vue'
import type { Snapshot } from '../types'

const props = defineProps<{
  snapshot: Snapshot | null
  connected: boolean
  demo: boolean
  serverUrl: string
}>()

const anchors = computed(() => props.snapshot?.anchors ?? [])
const links = computed(() => props.snapshot?.links ?? [])
const tags = computed(() => props.snapshot?.tags ?? [])

const anchorName = (address: number) => `A${address}`

/** Legs still missing from the survey, which is why a layout can be absent. */
const expectedLinks = computed(() => {
  const n = anchors.value.length
  return (n * (n - 1)) / 2
})

const age = (at: number) => {
  const seconds = Math.max(0, Math.round((Date.now() - at) / 1000))
  return seconds < 1 ? 'now' : `${seconds}s ago`
}
</script>

<template>
  <aside class="flex h-full w-[24rem] shrink-0 flex-col gap-4 overflow-y-auto border-l border-slate-800 bg-slate-950/80 p-5 text-slate-200 backdrop-blur">
    <header>
      <h1 class="text-lg font-semibold tracking-tight text-white">UWB network</h1>
      <div class="mt-1.5 flex items-center gap-2 text-xs">
        <span
          class="h-2 w-2 rounded-full"
          :class="connected ? 'bg-emerald-400' : 'bg-rose-500'"
        />
        <span class="text-slate-400">
          <template v-if="demo">generated data, no hardware</template>
          <template v-else-if="connected">{{ serverUrl }}</template>
          <template v-else>reaching {{ serverUrl }}</template>
        </span>
      </div>
    </header>

    <!-- anchors -->
    <section>
      <h2 class="mb-2 text-xs font-semibold uppercase tracking-widest text-slate-500">
        Anchors ({{ anchors.length }})
      </h2>

      <p v-if="anchors.length === 0" class="rounded-lg border border-slate-800 bg-slate-900/50 p-3 text-sm text-slate-500">
        None connected. An anchor announces itself over UDP 7051 at boot.
      </p>

      <ul v-else class="flex flex-col gap-1.5">
        <li
          v-for="anchor in anchors"
          :key="anchor.id"
          class="flex items-center gap-3 rounded-lg border border-slate-800 bg-slate-900/50 px-3 py-2"
        >
          <span
            class="flex h-7 w-7 shrink-0 items-center justify-center rounded-md text-xs font-bold"
            :class="anchor.main ? 'bg-amber-400/15 text-amber-300' : 'bg-cyan-400/15 text-cyan-300'"
          >{{ anchor.address }}</span>
          <div class="min-w-0 flex-1">
            <p class="truncate font-mono text-xs text-slate-300">{{ anchor.id }}</p>
            <p class="truncate text-[11px] text-slate-500">{{ anchor.ip }} · {{ age(anchor.lastSeen) }}</p>
          </div>
          <span v-if="anchor.main" class="shrink-0 rounded bg-amber-400/10 px-1.5 py-0.5 text-[10px] font-medium uppercase tracking-wide text-amber-300">
            main
          </span>
        </li>
      </ul>
    </section>

    <!-- distances between anchors -->
    <section>
      <h2 class="mb-2 text-xs font-semibold uppercase tracking-widest text-slate-500">
        Distances between anchors
      </h2>

      <p v-if="links.length === 0" class="rounded-lg border border-slate-800 bg-slate-900/50 p-3 text-sm text-slate-500">
        Nothing measured yet. Anchors survey each other every few seconds.
      </p>

      <template v-else>
        <ul class="flex flex-col gap-1">
          <li
            v-for="link in links"
            :key="`${link.a}-${link.b}`"
            class="flex items-baseline gap-2 rounded-md bg-slate-900/50 px-3 py-1.5 font-mono text-sm"
          >
            <span class="text-slate-400">{{ anchorName(link.a) }}</span>
            <span class="text-slate-600">&rarr;</span>
            <span class="text-slate-400">{{ anchorName(link.b) }}</span>
            <span class="flex-1" />
            <span :class="link.los ? 'text-sky-300' : 'text-rose-400'">
              {{ link.distance.toFixed(2) }} m
            </span>
          </li>
        </ul>
        <p v-if="links.length < expectedLinks" class="mt-2 text-[11px] text-slate-500">
          {{ links.length }} of {{ expectedLinks }} legs measured. A layout needs the
          first three anchors fully surveyed.
        </p>
      </template>
    </section>

    <!-- drones -->
    <section>
      <h2 class="mb-2 text-xs font-semibold uppercase tracking-widest text-slate-500">
        Drones ({{ tags.length }})
      </h2>

      <p v-if="tags.length === 0" class="rounded-lg border border-slate-800 bg-slate-900/50 p-3 text-sm text-slate-500">
        No tag is ranging.
      </p>

      <div
        v-for="tag in tags"
        :key="tag.eui"
        class="rounded-lg border border-slate-800 bg-slate-900/50 p-3"
      >
        <p class="font-mono text-xs text-pink-300">{{ tag.eui }}</p>

        <p v-if="tag.position" class="mt-1 font-mono text-sm text-slate-200">
          x {{ tag.position.x.toFixed(2) }} ·
          y {{ tag.position.y.toFixed(2) }} ·
          z {{ tag.position.z.toFixed(2) }}
          <span v-if="tag.position.zAssumed" class="text-[10px] text-slate-500">
            (height from one sphere)
          </span>
        </p>
        <p v-else class="mt-1 text-xs text-slate-500">
          No fix: needs three clear distances and a solved layout.
        </p>

        <ul class="mt-2 flex flex-col gap-0.5">
          <li
            v-for="range in tag.ranges"
            :key="range.anchor"
            class="flex items-baseline gap-2 font-mono text-xs"
          >
            <span class="text-slate-500">{{ anchorName(range.anchor) }}</span>
            <span class="flex-1" />
            <span :class="range.los ? 'text-slate-300' : 'text-rose-400'">
              {{ range.distance.toFixed(2) }} m
            </span>
            <span v-if="!range.los" class="text-[10px] uppercase text-rose-400">nlos</span>
          </li>
        </ul>
      </div>
    </section>

    <footer class="mt-auto text-[11px] leading-relaxed text-slate-600">
      Distances come from two way ranging, not signal strength. Anchor coordinates
      are derived from the legs they measure between themselves, so the frame is
      arbitrary: the first anchor sits at the origin and the second on the X axis.
    </footer>
  </aside>
</template>
