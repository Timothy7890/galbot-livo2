<script setup>
// 主相机实时画面：录制中每 0.4 s 取一帧
import { onUnmounted, ref, watch } from 'vue'
import { cameraUrl } from '../api.js'

const props = defineProps({ recording: Boolean })

const src = ref('')
let timer = null

function next() {
  // 上一帧加载完再取下一帧，网络慢时不会越积越多
  const url = `${cameraUrl}?t=${Date.now()}`
  const img = new Image()
  img.onload = () => {
    src.value = url
    timer = setTimeout(next, 400)
  }
  img.onerror = () => (timer = setTimeout(next, 2000))
  img.src = url
}

watch(() => props.recording, rec => {
  clearTimeout(timer)
  if (rec) next()
  else src.value = ''
}, { immediate: true })
onUnmounted(() => clearTimeout(timer))
</script>

<template>
  <div class="card cam">
    <div class="title">主相机</div>
    <div class="frame">
      <img v-if="src" :src="src" alt="主相机画面" />
      <div v-else class="hint muted">录制中显示</div>
    </div>
  </div>
</template>

<style scoped>
.cam { padding: 12px; }
.title { font-weight: 600; margin-bottom: 8px; }
.frame { aspect-ratio: 4 / 3; border-radius: 8px; overflow: hidden; background: var(--map-bg); display: grid; place-items: center; }
img { width: 100%; height: 100%; object-fit: contain; display: block; }
.hint { font-size: 13px; }
</style>
