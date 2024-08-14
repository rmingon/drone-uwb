<script setup>
import { shallowRef } from 'vue'
import { useLoop } from '@tresjs/core'

const boxRef = shallowRef()

const { onBeforeRender } = useLoop()

const props = defineProps({
    position: {
        type: Object,
        default(rawProps) {
            return { pitch: 1, roll: 1 }
        }
    }
})

onBeforeRender(({ delta, renderer, scene, camera, elapsed }) => {
    if (boxRef.value) {
        boxRef.value.rotation.x = props.position.pitch / 10
        boxRef.value.rotation.z = props.position.roll / 10
    }
  renderer.render(scene, camera)
})
</script>

<template>
    <TresPerspectiveCamera :position="[3, 3, 3]" :look-at="[0, 0, 0]" />
    <TresMesh ref="boxRef">
        <TresBoxGeometry :args="[1, 0.5, 1, 1]" />
        <TresMeshBasicMaterial color="black" />
    </TresMesh>
</template>