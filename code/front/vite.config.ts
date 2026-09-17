import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'
import { templateCompilerOptions } from '@tresjs/core'

// https://vitejs.dev/config/
export default defineConfig({
  // templateCompilerOptions tells the Vue compiler that Tres* tags are three.js
  // objects rather than components it should try to resolve
  plugins: [vue({ ...templateCompilerOptions })],
})
