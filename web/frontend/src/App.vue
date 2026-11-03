<script setup>
import { onMounted, onUnmounted, ref, computed } from 'vue'
import { getStatus, startRecording, stopRecording } from './api.js'
import RecordPanel from './components/RecordPanel.vue'
import LiveMap from './components/LiveMap.vue'
import SessionList from './components/SessionList.vue'
import CameraView from './components/CameraView.vue'

const status = ref({ status: 'connecting', sessions: [] })
const pending = ref('')       // 本地操作进行中的提示
const actionError = ref('')
let timer = null

const recording = computed(() => status.value.status === 'recording')

async function refresh() {
  try {
    status.value = await getStatus()
  } catch {
    status.value = { ...status.value, status: 'offline' }
  }
}

async function onStart() {
  pending.value = 'starting'
  actionError.value = ''
  try {
    const r = await startRecording()
    if (!r.ok) actionError.value = r.msg
  } catch (e) {
    actionError.value = String(e)
  }
  pending.value = ''
  refresh()
}

async function onStop() {
  if (!confirm('停止录制并保存？')) return
  pending.value = 'stopping'
  actionError.value = ''
  try {
    const r = await stopRecording()
    if (!r.ok) actionError.value = r.msg
  } catch (e) {
    actionError.value = String(e)
  }
  pending.value = ''
  refresh()
}

onMounted(() => {
  refresh()
  timer = setInterval(refresh, 2000)
})
onUnmounted(() => clearInterval(timer))
</script>

<template>
  <main class="layout">
    <header>
      <h1>Galbot G1 建图录制</h1>
      <span class="muted">Mid-360 + 环视相机原始数据 · FAST-LIVO2 实时建图</span>
    </header>

    <RecordPanel
      class="panel"
      :status="status"
      :pending="pending"
      :action-error="actionError"
      @start="onStart"
      @stop="onStop"
    />
    <CameraView class="cam" :recording="recording" />
    <SessionList class="list" :sessions="status.sessions || []" :data-root="status.data_root" />
    <section class="map">
      <LiveMap :recording="recording" />
    </section>
  </main>
</template>

<style scoped>
.layout {
  display: grid;
  grid-template-columns: 340px 1fr;
  grid-template-rows: auto auto auto 1fr;
  grid-template-areas: "head head" "panel map" "cam map" "list map";
  gap: 16px;
  padding: 16px;
  height: 100vh;
}
header { grid-area: head; display: flex; align-items: baseline; gap: 12px; flex-wrap: wrap; }
h1 { font-size: 20px; margin: 0; }
.panel { grid-area: panel; }
.cam { grid-area: cam; }
.list { grid-area: list; min-height: 0; overflow: auto; align-self: start; max-height: 100%; }
.map { grid-area: map; min-height: 0; }

/* 手机：录制按钮 → 实时地图 → 主相机 → 历史记录 */
@media (max-width: 860px) {
  .layout {
    grid-template-columns: 1fr;
    grid-template-rows: auto auto 70vh auto auto;
    grid-template-areas: "head" "panel" "map" "cam" "list";
    height: auto;
  }
  .list { overflow: visible; max-height: none; }
}
</style>
