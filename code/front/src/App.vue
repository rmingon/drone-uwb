<script setup>
import { ref, onMounted, toRef, shallowRef } from 'vue'
import { TresCanvas, useLoop } from '@tresjs/core'
import { useGlobalState } from './stores/data';

import Drone from './Drone.vue'

const data = useGlobalState()

const drones = toRef(data, 'drones')
const dronesPosition = toRef(data, 'dronesPosition')

</script>

<template>
  <div v-for="drone in drones" class="border-2 border-radius w-1/2 p-4 m-2 flex flex-col">
    <p class="w-full text-center">{{ drone.id }}</p>
    <p class="w-full text-center">{{ dronesPosition[drone.id] }}</p>
    <div class="w-full h-52">
      <TresCanvas render-mode="manual"> 
        <Drone :position="dronesPosition[drone.id]"/>
      </TresCanvas>
    </div>

  </div>
</template>

