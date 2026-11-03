<script setup>
// 4 路环视相机：录制中显示 2x2 预览（320x240，每 0.5 s 一帧），点击某一路放大看原图
import { onUnmounted, reactive, ref, watch } from 'vue'
import { cameraUrl } from '../api.js'

const props = defineProps({
  recording: Boolean,
  mainCamera: { type: String, default: 'left_front' },
})

const CAMERAS = [
  ['left_front', '左前'], ['right_front', '右前'],
  ['left_rear', '左后'], ['right_rear', '右后'],
]

const src = reactive({})          // 名称 -> 已加载完成的图片地址
const zoomed = ref('')            // 放大显示的相机名
const timers = {}

function poll(name) {
  // 上一帧加载完再取下一帧，网络慢时不会越积越多；放大的那一路取原图
  const full = zoomed.value === name
  const url = `${cameraUrl(name)}?${full ? 'full=1&' : ''}t=${Date.now()}`
  const img = new Image()
  img.onload = () => {
    src[name] = url
    timers[name] = setTimeout(() => poll(name), full ? 400 : 500)
  }
  img.onerror = () => (timers[name] = setTimeout(() => poll(name), 2000))
  img.src = url
}

function stopAll() {
  for (const name in timers) clearTimeout(timers[name])
}

watch(() => props.recording, rec => {
  stopAll()
  if (rec) CAMERAS.forEach(([name]) => poll(name))
  else {
    for (const name in src) delete src[name]
    zoomed.value = ''
  }
}, { immediate: true })
onUnmounted(stopAll)

function toggle(name) {
  zoomed.value = zoomed.value === name ? '' : name
}
</script>

<template>
  <div class="card cams">
    <div class="title">
      环视相机
      <span class="muted hint-text">{{ zoomed ? '点击画面返回四路' : '点击某一路放大' }}</span>
    </div>
    <div v-if="!recording" class="frame empty muted">录制中显示</div>
    <div v-else-if="zoomed" class="frame" @click="toggle(zoomed)">
      <img v-if="src[zoomed]" :src="src[zoomed]" alt="" />
      <span class="label">{{ CAMERAS.find(c => c[0] === zoomed)[1] }}<b v-if="zoomed === mainCamera"> · 主相机</b></span>
    </div>
    <div v-else class="grid">
      <div v-for="[name, label] in CAMERAS" :key="name" class="frame" :class="{ main: name === mainCamera }"
           @click="toggle(name)">
        <img v-if="src[name]" :src="src[name]" alt="" />
        <span class="label">{{ label }}<b v-if="name === mainCamera"> · 主相机</b></span>
      </div>
    </div>
  </div>
</template>

<style scoped>
.cams { padding: 12px; }
.title { font-weight: 600; margin-bottom: 8px; display: flex; align-items: baseline; gap: 8px; }
.hint-text { font-size: 12px; font-weight: 400; }
.grid { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; }
.frame {
  position: relative; aspect-ratio: 4 / 3; border-radius: 6px; overflow: hidden;
  background: var(--map-bg); cursor: pointer; border: 2px solid transparent;
}
.frame.main { border-color: var(--go); }
.frame.empty { display: grid; place-items: center; font-size: 13px; cursor: default; }
img { width: 100%; height: 100%; object-fit: contain; display: block; }
.label {
  position: absolute; left: 6px; top: 4px; font-size: 12px; color: #fff;
  text-shadow: 0 0 3px #000, 0 0 3px #000;
}
.label b { color: #7ee2a0; font-weight: 600; }
</style>
