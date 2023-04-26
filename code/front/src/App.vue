<script setup lang="ts">
import Drone3d from './components/Drone3d.vue'
import { useWebSocket } from '@vueuse/core'
import {computed, reactive, ref, watch} from "vue";
interface Drone {
  x: number
  y: number
  altitude: number
  id: number
}

const { status, data, send, open, close } = useWebSocket<Drone>('ws://127.0.0.1:8081/user')

const pwm = ref(0)

let drone = reactive({
  x: 0,
  y: 0,
  altitude: 0,
  id: 0
})

if (data)  {
  console.log(data)
  // drone = data.value as Drone
}

const sendToServer = (cmd: object) => {
  console.log(cmd)
  send(JSON.stringify(cmd))
}
</script>

<template>

  {{data}}<br>
  {{status}}<br>
  <div class="flex-col">
    <button class="border p-4" @click="sendToServer({'cmd': 'FLY'})">FLY</button>
    <button class="border p-4" @click="sendToServer({'cmd': 'STOP'})">STOP</button>
    <input type="number" name="" id="" v-model="pwm">
    <button class="border p-4" @click="sendToServer({'cmd': 'SPEED', params: pwm})">SET SPEED</button>
  </div>
</template>
