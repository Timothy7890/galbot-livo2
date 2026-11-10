<script setup>
// 7010 导航页：选导航地图 → 启动导航 → 自动重定位（失败时在地图上点选大概位置）→ 点选目标 → 前往
import { computed, onMounted, onUnmounted, ref, watch } from 'vue'
import { estop, getNav, getStatus, navAction, navGridUrl, releaseEstop } from './api.js'
import NavMap from './components/NavMap.vue'

const nav = ref({ status: 'connecting', maps: [] })
const rec = ref({ sessions: [] })
const selected = ref('')
const mode = ref('')            // '' | 'goal' | 'reloc'
const busy = ref('')
const msg = ref('')
const msgOk = ref(true)
const gridTick = ref(0)
let timer = null
let recTimer = null

const running = computed(() => nav.value.status === 'running')
const st = computed(() => nav.value.state || null)
const loc = computed(() => st.value?.loc || null)
const localized = computed(() => loc.value?.state === 'localized')
const confirmed = computed(() => localized.value && !!st.value?.confirmed)
const navigating = computed(() => !!st.value?.enabled)
const recording = computed(() => ['recording', 'starting', 'stopping'].includes(rec.value.status))
const runningSession = computed(() => (nav.value.map || '').split('/').slice(-2, -1)[0] || '')
const mapMeta = computed(() => running.value ? st.value?.map
  : nav.value.maps.find(m => m.session === selected.value) || null)
const gridUrl = computed(() => {
  gridTick.value      // 依赖：运行中每次刷新状态都重载实时栅格
  if (running.value) return st.value ? `${navGridUrl()}?t=${gridTick.value}` : ''
  return selected.value ? navGridUrl(selected.value) : ''
})
const grid = computed(() => running.value ? (st.value?.grid || mapMeta.value?.grid) : mapMeta.value?.grid)
const buildable = computed(() => (rec.value.sessions || []).filter(s => s.status === 'done'))

function note(text, ok = true) { msg.value = text; msgOk.value = ok }

async function refresh() {
  try {
    nav.value = await getNav()
    if (running.value) gridTick.value++
    if (!selected.value && nav.value.maps.length) selected.value = nav.value.maps[0].session
    if (running.value && runningSession.value) selected.value = runningSession.value
  } catch {
    nav.value = { ...nav.value, status: 'offline' }
  }
}
async function refreshRec() {
  try { rec.value = await getStatus() } catch { /* 下次再取 */ }
}

async function act(action, body, confirmText) {
  if (confirmText && !confirm(confirmText)) return null
  busy.value = action
  try {
    const r = await navAction(action, body)
    note(r.msg ? String(r.msg).split('\n').slice(-1)[0] : (r.ok ? '完成' : '失败'), r.ok)
    return r
  } catch (e) {
    note(String(e), false)
    return null
  } finally {
    busy.value = ''
    refresh()
  }
}

const start = () => act('start', { session: selected.value })
const stop = () => act('stop', null, '停止导航？')
const cancel = () => act('cancel')
const autoReloc = () => act('reloc', { mode: 'auto' })

async function onPick(p) {
  if (mode.value === 'goal') {
    mode.value = ''
    await act('goto', { x: p.x, y: p.y }, `前往 (${p.x.toFixed(1)}, ${p.y.toFixed(1)})？\n\n机器人会自己开过去；随时可点「取消导航」或按急停。`)
  } else if (mode.value === 'reloc') {
    mode.value = ''
    await act('reloc', { mode: 'manual', x: p.x, y: p.y, yaw: p.yaw })
  }
}

async function build(s) {
  const name = prompt(`给这张导航地图起个名字（录制 ${s.name}）`, s.nav?.name || s.name)
  if (name === null) return
  if (s.nav && !confirm('这次录制已有导航地图，重新生成？（原地图改名留底，不删除）')) return
  await act('build', { session: s.name, name })
  refreshRec()
}

// 急停（与 7010 录制页、7011 遥控页同一个锁定状态）
async function onEstop() {
  mode.value = ''
  const r = await estop().catch(e => ({ ok: false, msg: String(e) }))
  if (!r.ok) note(r.msg || '急停失败', false)
  refresh()
}
async function onRelease() {
  if (!confirm('解除急停？\n\n正在导航的话，机器人会继续前往目标。')) return
  const r = await releaseEstop().catch(e => ({ ok: false, msg: String(e) }))
  if (!r.ok) note(r.msg || '解除失败', false)
  refresh()
}
function onKey(e) {
  if (e.key === ' ' && e.target.tagName !== 'INPUT') { e.preventDefault(); onEstop() }
  if (e.key === 'Escape') mode.value = ''
}

watch(running, r => { if (!r) mode.value = '' })

const locText = { unlocalized: '未定位', relocalizing: '定位中…', localized: '已定位', lost: '定位丢失' }
const stateText = { idle: '待命', home: '前往目标', arrived: '已到达', error: '出错停车' }
function fmtTime(s) { return s == null ? '' : `${Math.floor(s / 60)} 分 ${s % 60} 秒` }

onMounted(() => {
  refresh(); refreshRec()
  timer = setInterval(refresh, 1000)
  recTimer = setInterval(refreshRec, 3000)
  window.addEventListener('keydown', onKey)
})
onUnmounted(() => { clearInterval(timer); clearInterval(recTimer); window.removeEventListener('keydown', onKey) })
</script>

<template>
  <main class="layout">
    <header>
      <h1>Galbot G1 导航</h1>
      <a href="./">← 录制</a>
      <span class="muted">在录制生成的地图里定位、点选目标前往 · 空格急停 · Esc 取消点选</span>
    </header>

    <section class="estop-box">
      <button v-if="!nav.estop" class="estop" @click="onEstop">急停</button>
      <button v-else class="release" @click="onRelease">急停中 · 点击解除</button>
      <div v-if="nav.estop === null && nav.status !== 'connecting'" class="bad small">7011 遥控服务未就绪，急停可能无效</div>
    </section>

    <section class="card panel">
      <div class="row">
        <span class="dot" :class="running ? 'on' : ''" />
        <b>{{ running ? '导航运行中' : nav.status === 'starting' ? '启动中…' : '导航未运行' }}</b>
        <span v-if="st?.dry_run" class="tag">只算不动</span>
      </div>

      <template v-if="!running">
        <label class="field">导航地图
          <select v-model="selected" :disabled="!!busy">
            <option v-if="!nav.maps.length" value="">（还没有，先在下面由录制生成）</option>
            <option v-for="m in nav.maps" :key="m.session" :value="m.session">{{ m.name }}（{{ m.session }}）</option>
          </select>
        </label>
        <div v-if="mapMeta" class="muted small">
          {{ mapMeta.grid.w.toFixed(0) }} × {{ mapMeta.grid.h.toFixed(0) }} m · 建图路程 {{ mapMeta.path_length_m }} m ·
          生成于 {{ mapMeta.created }}
        </div>
        <button class="primary" :disabled="!selected || !!busy || recording" @click="start">
          {{ busy === 'start' ? '启动中（约 30 秒）…' : '启动导航' }}
        </button>
        <div v-if="recording" class="muted small">正在录制，导航与录制不能同时进行</div>
      </template>

      <template v-else>
        <div class="kv">
          <span>地图</span><b>{{ st?.map?.name || runningSession }}</b>
          <span>定位</span>
          <b :class="{ ok: localized, bad: loc?.state === 'lost' || loc?.state === 'unlocalized' }">
            {{ locText[loc?.state] || '—' }}
          </b>
          <span />
          <span class="muted small">
            {{ loc?.msg }}
            <template v-if="loc?.fitness != null"> · 匹配 {{ (loc.fitness * 100).toFixed(0) }}% / {{ (loc.rmse * 100).toFixed(1) }} cm</template>
            <template v-if="loc?.time_s != null"> · 用时 {{ loc.time_s }} s</template>
          </span>
          <span>导航</span><b>{{ stateText[st?.state] || st?.state || '—' }}</b>
          <span /><span class="muted small">{{ st?.msg }}<template v-if="st?.elapsed_s != null"> · {{ fmtTime(st.elapsed_s) }}</template></span>
        </div>
        <div v-if="localized && !confirmed" class="check">
          <b>请核对定位</b>
          <div class="small">地图上的白色箭头是否就是机器人现在的位置和车头朝向？自动定位在相似的走廊、展区可能认错地方。</div>
          <div class="btns">
            <button class="primary" :disabled="!!busy" @click="act('confirm')">位置正确</button>
            <button @click="mode = 'reloc'">不对，手动定位</button>
          </div>
        </div>
        <div v-if="st?.limit_reason" class="warn small">避障：{{ st.limit_reason }}</div>
        <div v-if="st?.teleop_msg" class="bad small">{{ st.teleop_msg }}</div>

        <div class="btns">
          <button :class="{ active: mode === 'goal' }" :disabled="!confirmed" @click="mode = mode === 'goal' ? '' : 'goal'">
            {{ mode === 'goal' ? '取消点选' : '点选目标' }}
          </button>
          <button :disabled="!navigating" @click="cancel">取消导航</button>
        </div>
        <div class="btns">
          <button :disabled="loc?.state === 'relocalizing' || !!busy" @click="autoReloc">自动重定位</button>
          <button :class="{ active: mode === 'reloc' }" :disabled="loc?.state === 'relocalizing'"
                  @click="mode = mode === 'reloc' ? '' : 'reloc'">
            {{ mode === 'reloc' ? '取消点选' : '手动定位' }}
          </button>
        </div>
        <button class="danger" :disabled="!!busy" @click="stop">{{ busy === 'stop' ? '停止中…' : '停止导航' }}</button>
      </template>

      <div v-if="msg" class="small" :class="msgOk ? 'muted' : 'bad'">{{ msg }}</div>
      <div v-if="nav.error" class="bad small">{{ nav.error }}</div>
    </section>

    <section class="card builds">
      <div class="row"><b>由录制生成导航地图</b></div>
      <div class="muted small">用录制时的雷达栅格与点云生成（新录制约 1 分钟；旧录制需重读 raw.bag，较慢）。生成时可以继续录制或导航。</div>
      <div v-for="s in buildable.slice(0, 12)" :key="s.name" class="build-row">
        <span class="num">{{ s.name }}</span>
        <span class="muted small grow">
          <template v-if="s.nav_progress?.state === 'running'">{{ s.nav_progress.msg }}</template>
          <template v-else-if="s.nav_progress?.state === 'failed'"><span class="bad">{{ s.nav_progress.msg }}</span></template>
          <template v-else-if="s.nav">已有：{{ s.nav.name }}</template>
        </span>
        <button :disabled="s.nav_progress?.state === 'running' || !!busy" @click="build(s)">
          {{ s.nav ? '重新生成' : '生成' }}
        </button>
      </div>
    </section>

    <section class="map">
      <NavMap :grid-url="gridUrl" :grid="grid" :pose="st?.pose" :target="navigating ? st?.target : null"
              :path="st?.path" :candidates="loc?.state === 'localized' ? null : loc?.candidates"
              :mode="mode" @pick="onPick" />
    </section>
  </main>
</template>

<style scoped>
.layout {
  display: grid; grid-template-columns: 360px 1fr; grid-template-rows: auto auto auto 1fr;
  grid-template-areas: "head head" "estop map" "panel map" "builds map"; gap: 16px; padding: 16px; height: 100vh;
}
header { grid-area: head; display: flex; align-items: baseline; gap: 12px; flex-wrap: wrap; }
h1 { font-size: 20px; margin: 0; }
a { color: #3b82f6; }
.estop-box { grid-area: estop; display: flex; flex-direction: column; gap: 6px; }
.estop, .release { border: 0; border-radius: 12px; padding: 14px; font-size: 20px; font-weight: 700; color: #fff; cursor: pointer; }
.estop { background: var(--stop); }
.release { background: var(--warn); }
.panel { grid-area: panel; display: flex; flex-direction: column; gap: 10px; }
.builds { grid-area: builds; display: flex; flex-direction: column; gap: 6px; overflow: auto; min-height: 0; align-self: start; max-height: 100%; }
.map { grid-area: map; min-height: 0; }
.row { display: flex; align-items: center; gap: 8px; }
.dot { width: 10px; height: 10px; border-radius: 50%; background: var(--muted); }
.dot.on { background: var(--go); }
.tag { font-size: 12px; background: var(--warn); color: #fff; border-radius: 10px; padding: 0 8px; }
.field { display: flex; flex-direction: column; gap: 4px; font-size: 13px; }
select { padding: 6px; border-radius: 6px; border: 1px solid var(--line); background: var(--card); color: var(--text); }
.kv { display: grid; grid-template-columns: 40px 1fr; gap: 2px 10px; font-size: 14px; }
.btns { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
button {
  border: 1px solid var(--line); background: transparent; color: var(--text); border-radius: 8px;
  padding: 8px 10px; cursor: pointer; font-size: 14px;
}
button:disabled { opacity: 0.45; cursor: not-allowed; }
button.primary { background: var(--go); color: #fff; border: 0; font-weight: 600; }
button.danger { color: var(--stop); border-color: var(--stop); }
button.active { background: #2563eb; color: #fff; border-color: #2563eb; }
.build-row { display: flex; align-items: center; gap: 8px; font-size: 13px; }
.check { border: 2px solid var(--warn); border-radius: 10px; padding: 10px; display: flex; flex-direction: column; gap: 6px; }
.build-row button { padding: 3px 10px; font-size: 13px; }
.grow { flex: 1; min-width: 0; }
.small { font-size: 12px; }
.ok { color: var(--go); }
.bad { color: var(--stop); }
.warn { color: var(--warn); }

@media (max-width: 860px) {
  .estop-box { position: sticky; top: 0; z-index: 10; background: var(--bg); padding: 4px 0; }
  .layout {
    grid-template-columns: 1fr; grid-template-rows: auto auto auto 70vh auto;
    grid-template-areas: "head" "estop" "panel" "map" "builds"; height: auto;
  }
}
</style>
