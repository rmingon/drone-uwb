<script setup>
import { useWebSocket } from '@vueuse/core'

const { status, data, send, open, close } = useWebSocket('ws://127.0.0.1:1227')
</script>

<template>
  {{ status }}
</template>

