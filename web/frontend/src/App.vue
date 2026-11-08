<script setup>
import { onMounted, onUnmounted, ref, computed, watch } from 'vue'
import { estop, exploreAction, getExplore, getStatus, releaseEstop, startRecording, stopRecording } from './api.js'
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
const explore = ref(null)     // 录制中 explorer.py 的状态（自主建图）
// 探索策略：记住上次的选择（只是本浏览器的偏好，读写失败就用默认）
const strategy = ref((() => { try { return localStorage.getItem('exploreStrategy') || 'region' } catch { return 'region' } })())
watch(strategy, v => { try { localStorage.setItem('exploreStrategy', v) } catch { /* 无痕模式等 */ } })

async function refresh() {
  try {
    status.value = await getStatus()
  } catch {
    status.value = { ...status.value, status: 'offline' }
  }
}

// 自主建图状态单独每秒刷新一次（录制状态 2 s 一次即可）
async function refreshExplore() {
  if (!recording.value) { explore.value = null; return }
  try { explore.value = await getExplore() } catch { /* 刚开始录制，程序还没就绪 */ }
}

async function onToggleExplore() {
  const enabled = explore.value?.enabled
  if (!enabled && !confirm('进入自主建图：机器人会自己移动，走遍场地后回到录制起点；' +
    '之后 2 分钟内没人取消就自动停止录制。\n\n' +
    '· 请确认周围没有人和易碎物品，场地出口已关闭\n' +
    '· 随时可点「退出自主建图」，或按本页 / 7011 遥控页的急停（暂停）/ 按方向键（人工接管，自动退出）\n\n进入？')) return
  actionError.value = ''
  try {
    const r = await exploreAction(enabled ? 'disable' : 'enable', enabled ? null : { strategy: strategy.value })
    if (!r.ok) actionError.value = r.msg
  } catch (e) {
    actionError.value = String(e)
  }
  refreshExplore()
}

async function onCancelStop() {
  const r = await exploreAction('cancel')
  if (!r.ok) actionError.value = r.msg
  refreshExplore()
}

// 急停不确认、立即发；解除需确认
const estopError = ref('')
async function onEstop() {
  estopError.value = ''
  status.value = { ...status.value, estop: true }
  try {
    const r = await estop()
    if (!r.ok) estopError.value = r.msg || '急停失败'
  } catch (e) {
    estopError.value = `急停请求失败：${e}`
  }
  refresh()
}
async function onReleaseEstop() {
  if (!confirm('解除急停？\n\n解除后自主建图会继续移动底盘。')) return
  estopError.value = ''
  try {
    const r = await releaseEstop()
    if (!r.ok) estopError.value = r.msg || '解除失败'
  } catch (e) {
    estopError.value = String(e)
  }
  refresh()
}
function onKeyDown(e) {
  if (e.key === ' ' && e.target.tagName !== 'INPUT') { e.preventDefault(); onEstop() }
}

async function onStart() {
  pending.value = 'starting'
  actionError.value = ''
  explore.value = null
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
  exploreTimer = setInterval(refreshExplore, 1000)
  window.addEventListener('keydown', onKeyDown)
})
onUnmounted(() => {
  clearInterval(timer); clearInterval(exploreTimer)
  window.removeEventListener('keydown', onKeyDown)
})
</script>

<template>
  <main class="layout">
    <header>
      <h1>Galbot G1 建图录制</h1>
      <span class="muted">Mid-360 + 环视 / 头部相机原始数据 · FAST-LIVO2 实时建图 · 空格急停</span>
    </header>

    <section class="estop-box">
      <button v-if="!status.estop" class="estop" @click="onEstop">急停</button>
      <button v-else class="release" @click="onReleaseEstop">急停中 · 点击解除</button>
      <div v-if="status.estop === null && status.status !== 'connecting'" class="estop-msg">
        7011 遥控服务未就绪，急停可能无效
      </div>
      <div v-if="estopError" class="estop-msg">{{ estopError }}</div>
    </section>

    <RecordPanel
      class="panel"
      :status="status"
      :pending="pending"
      :action-error="actionError"
      @start="onStart"
      :explore="explore"
      v-model:strategy="strategy"
      @toggle-explore="onToggleExplore"
      @stop="onStop"
    />
    <ExplorePanel v-if="recording" class="exp" :explore="explore" @cancel-stop="onCancelStop" />
    <CameraView class="cam" :active="recording" :main-camera="status.main_camera" />
    <SessionList class="list" :sessions="status.sessions || []" :data-root="status.data_root" />
    <section class="map">
      <LiveMap :recording="recording" :explore="recording ? explore : null" />
    </section>
  </main>
</template>

<style scoped>
.layout {
  display: grid;
  grid-template-columns: 340px 1fr;
  grid-template-rows: auto auto auto auto auto 1fr;
  grid-template-areas: "head head" "estop map" "panel map" "exp map" "cam map" "list map";
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
.estop-box { grid-area: estop; display: flex; flex-direction: column; gap: 6px; }
.estop, .release {
  border: 0; border-radius: 12px; padding: 14px; font-size: 20px; font-weight: 700; color: #fff; cursor: pointer;
}
.estop { background: var(--stop); }
.release { background: var(--warn); }
.estop-msg { color: var(--stop); font-size: 12px; }

/* 手机：急停 → 录制按钮 → 实时地图 → 相机 → 历史记录；急停固定在顶部 */
@media (max-width: 860px) {
  .estop-box { position: sticky; top: 0; z-index: 10; background: var(--bg); padding: 4px 0; }
  .layout {
    grid-template-columns: 1fr;
    grid-template-rows: auto auto auto auto 70vh auto auto;
    grid-template-areas: "head" "estop" "panel" "exp" "map" "cam" "list";
    height: auto;
  }
  .list { overflow: visible; max-height: none; }
}
</style>
