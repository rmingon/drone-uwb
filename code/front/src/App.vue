<script setup lang="ts">
import { onMounted } from 'vue'
import NetworkScene from './components/NetworkScene.vue'
import StatusPanel from './components/StatusPanel.vue'
import { networkState, serverAddress, startNetworkState } from './stores/state'

onMounted(startNetworkState)
</script>

<template>
  <div class="flex h-screen w-screen overflow-hidden bg-slate-950">
    <main class="relative h-full min-w-0 flex-1">
      <NetworkScene :snapshot="networkState.snapshot" />

      <div
        v-if="!networkState.snapshot"
        class="pointer-events-none absolute inset-0 flex items-center justify-center"
      >
        <div class="pointer-events-auto rounded-xl border border-slate-800 bg-slate-900/90 px-6 py-5 text-center">
          <p class="text-sm text-slate-300">Waiting for the server</p>
          <p class="mt-1 font-mono text-xs text-slate-500">{{ serverAddress() }}</p>
          <a
            href="?demo=1"
            class="mt-3 inline-block rounded-md bg-cyan-500/15 px-3 py-1.5 text-xs font-medium text-cyan-300 hover:bg-cyan-500/25"
          >
            look at it with generated data
          </a>
        </div>
      </div>
    </main>

    <StatusPanel
      :snapshot="networkState.snapshot"
      :connected="networkState.connected"
      :demo="networkState.demo"
      :server-url="serverAddress()"
    />
  </div>
</template>
