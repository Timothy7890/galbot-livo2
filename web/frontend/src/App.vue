<script setup>
import { onMounted, onUnmounted, ref, computed } from 'vue'
import { getExplore, getStatus, startRecording, stopRecording } from './api.js'
import RecordPanel from './components/RecordPanel.vue'
import LiveMap from './components/LiveMap.vue'
import SessionList from './components/SessionList.vue'
import CameraView from './components/CameraView.vue'
import ExplorePanel from './components/ExplorePanel.vue'

const status = ref({ status: 'connecting', sessions: [] })
const pending = ref('')       // 本地操作进行中的提示
const actionError = ref('')
let timer = null
let exploreTimer = null

const recording = computed(() => status.value.status === 'recording')
const exploring = computed(() => recording.value && status.value.mode === 'explore')
const explore = ref(null)     // 自主探索状态（explorer.py）

async function refresh() {
  try {
    status.value = await getStatus()
  } catch {
    status.value = { ...status.value, status: 'offline' }
  }
}

// 自主探索状态单独每秒刷新一次（录制状态 2 s 一次即可）
async function refreshExplore() {
  if (!exploring.value) return
  try { explore.value = await getExplore() } catch { /* 探索进程刚启动或已结束 */ }
}

async function onExplore() {
  if (!confirm('自主建图：机器人会自己移动，走遍整个场地后回到起点并自动停止录制。\n\n' +
    '· 请确认周围没有人和易碎物品，场地出口已关闭\n· 随时可在 7011 遥控页按急停（暂停探索）或在此停止录制\n\n开始？')) return
  onStart(true)
}

async function onStart(withExplore = false) {
  pending.value = 'starting'
  actionError.value = ''
  explore.value = null
  try {
    const r = await startRecording(withExplore === true)
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
  exploreTimer = setInterval(refreshExplore, 1000)
})
onUnmounted(() => { clearInterval(timer); clearInterval(exploreTimer) })
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
      @explore="onExplore"
      @stop="onStop"
    />
    <ExplorePanel v-if="exploring || explore" class="exp" :explore="explore" :active="exploring" />
    <CameraView class="cam" :active="recording" :main-camera="status.main_camera" />
    <SessionList class="list" :sessions="status.sessions || []" :data-root="status.data_root" />
    <section class="map">
      <LiveMap :recording="recording" :explore="exploring ? explore : null" />
    </section>
  </main>
</template>

<style scoped>
.layout {
  display: grid;
  grid-template-columns: 340px 1fr;
  grid-template-rows: auto auto auto auto 1fr;
  grid-template-areas: "head head" "panel map" "exp map" "cam map" "list map";
  gap: 16px;
  padding: 16px;
  height: 100vh;
}
header { grid-area: head; display: flex; align-items: baseline; gap: 12px; flex-wrap: wrap; }
h1 { font-size: 20px; margin: 0; }
.panel { grid-area: panel; }
.cam { grid-area: cam; }
.exp { grid-area: exp; }
.list { grid-area: list; min-height: 0; overflow: auto; align-self: start; max-height: 100%; }
.map { grid-area: map; min-height: 0; }

/* 手机：录制按钮 → 实时地图 → 主相机 → 历史记录 */
@media (max-width: 860px) {
  .layout {
    grid-template-columns: 1fr;
    grid-template-rows: auto auto auto 70vh auto auto;
    grid-template-areas: "head" "panel" "exp" "map" "cam" "list";
    height: auto;
  }
  .list { overflow: visible; max-height: none; }
}
</style>
