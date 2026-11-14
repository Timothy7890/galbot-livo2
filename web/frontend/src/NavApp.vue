<script setup>
// 7010 导航页：选导航地图 → 启动导航（可选是否开视觉 / 只算不动，显示分步进度）→ 自动重定位
// （在上次停止的位置附近对上了就直接可用，否则要人核对；失败时在地图上点选大概位置）→ 选目标 → 前往。
// 运行中的状态由 SSE 推送（每 0.2 s），启停状态轮询 /api/nav；页面每秒发心跳，页面关了正在走的机器人会停。
import { computed, onMounted, onUnmounted, ref, watch } from 'vue'
import { estop, getNav, getNavTraj, navAction, navGridUrl, navStreamUrl, releaseEstop } from './api.js'
import NavMap from './components/NavMap.vue'
import StartProgress from './components/StartProgress.vue'

const load = (k, d) => { try { const v = localStorage.getItem(k); return v === null ? d : v === '1' } catch { return d } }
const save = (k, v) => { try { localStorage.setItem(k, v ? '1' : '0') } catch { /* 无痕模式等 */ } }

const nav = ref({ status: 'connecting', maps: [] })
const state = ref(null)         // navigator 推送
const selected = ref('')
const traj = ref([])
const mode = ref('')            // '' | 'goal' | 'reloc'
const vision = ref(load('navVision', true))
const dryRun = ref(load('navDryRun', false))
const speed = ref(0.3)
const alertMsg = ref(null)      // { text, ok }
const pendingOp = ref('')       // 刚点了启动 / 停止、服务端状态还没跟上
const relocSent = ref(false)
let es = null, timer = null, hbTimer = null, alertTimer = null, speedSynced = false
let lastStart = null            // 启动中最后一次看到的进度（失败后标出停在哪一步）

watch(vision, v => save('navVision', v))
watch(dryRun, v => save('navDryRun', v))

const running = computed(() => nav.value.status === 'running')
const starting = computed(() => nav.value.status === 'starting' || pendingOp.value === 'starting')
const stopping = computed(() => nav.value.status === 'stopping' || pendingOp.value === 'stopping')
const busy = computed(() => starting.value || stopping.value)
const loc = computed(() => state.value?.loc || {})
const localized = computed(() => loc.value.state === 'localized')
const confirmed = computed(() => localized.value && !!state.value?.confirmed)
const navigating = computed(() => !!state.value?.enabled)
const relocBusy = computed(() => relocSent.value || loc.value.state === 'relocalizing' || !!state.value?.reloc_pending)
const runningSession = computed(() => (nav.value.map || '').split('/').slice(-2, -1)[0] || '')
const mapMeta = computed(() => (running.value && state.value?.map) || nav.value.maps.find(m => m.session === selected.value) || null)
const gridUrl = computed(() => {
  if (running.value && state.value) return `${navGridUrl()}?v=${state.value.grid_v ?? 0}`
  return selected.value ? navGridUrl(selected.value) : ''
})
const grid = computed(() => (running.value && state.value?.grid) || mapMeta.value?.grid || null)
watch(() => loc.value.state, s => { if (s === 'relocalizing') relocSent.value = false })

// 启动步骤，与 nav.sh cmd_start 的 step 1–4 对应；typ 为大致耗时（s），只用来估计进度条
const startVision = ref(true)   // 本次启动选的是哪种（进度步骤不同）
const STEPS = computed(() => startVision.value ? [
  { title: '读取标定、启动 ROS 与传感器', hint: '从 Galbot SDK 现读相机外参；同时启动 ROS 和传感器桥接', typ: 7 },
  { title: '等待雷达、IMU、相机数据', hint: '三路都出数据', typ: 2 },
  { title: '启动 FAST-LIVO2', hint: 'IMU 初始化，等第一个位姿', typ: 2 },
  { title: '确认导航程序就绪', hint: '载入地图', typ: 1 },
] : [
  { title: '启动 ROS 与传感器（不开相机）', hint: '沿用建图时的标定，不读 SDK', typ: 2 },
  { title: '等待雷达、IMU 数据', hint: '连接 Galbot SDK', typ: 6 },
  { title: '启动 FAST-LIVO2（雷达 + IMU）', hint: 'IMU 初始化，等第一个位姿', typ: 2 },
  { title: '确认导航程序就绪', hint: '载入地图', typ: 1 },
])

function note(text, ok = true) {
  clearTimeout(alertTimer)
  if (!text) { alertMsg.value = null; return }
  alertMsg.value = { text, ok }
  if (ok) alertTimer = setTimeout(() => (alertMsg.value = null), 5000)
}

async function refresh() {
  try {
    const d = await getNav()
    if (!d || typeof d.status !== 'string') throw new Error('bad')
    nav.value = d
    if (d.status === 'starting') {
      startVision.value = d.vision !== false
      lastStart = { step: d.step || lastStart?.step || '', elapsed: Number(d.elapsed_s ?? 0), stepElapsed: Number(d.step_elapsed_s ?? 0) }
    }
    if (pendingOp.value === 'starting' && d.status !== 'idle') pendingOp.value = ''
    if (pendingOp.value === 'stopping' && d.status !== 'running') pendingOp.value = ''
    if (!selected.value && d.maps.length) selected.value = d.maps[0].session
    if (d.status !== 'idle' && runningSession.value) selected.value = runningSession.value
  } catch {
    nav.value = { ...nav.value, status: 'offline' }
  }
}
function schedule() {
  timer = setTimeout(async () => { await refresh(); schedule() }, busy.value ? 500 : 2000)
}

watch(selected, async s => {
  traj.value = []
  if (s) traj.value = await getNavTraj(s).catch(() => [])
}, { immediate: true })

// 运行中：订阅实时状态 + 每秒心跳
watch(running, on => {
  es?.close(); es = null
  clearInterval(hbTimer)
  if (!on) { state.value = null; mode.value = ''; speedSynced = false; return }
  const connect = () => {
    es = new EventSource(navStreamUrl)
    es.addEventListener('state', e => {
      state.value = JSON.parse(e.data)
      if (!speedSynced && state.value.v_max) { speed.value = state.value.v_max; speedSynced = true }
    })
    es.onerror = () => { es?.close(); es = null; if (running.value) setTimeout(() => { if (running.value && !es) connect() }, 1500) }
  }
  connect()
  navAction('heartbeat')
  hbTimer = setInterval(() => navAction('heartbeat'), 1000)
}, { immediate: true })

async function act(action, body) {
  const r = await navAction(action, body)
  note(r.msg ? String(r.msg).split('\n').slice(-1)[0] : (r.ok ? '' : '操作失败'), !!r.ok)
  return r
}

async function start() {
  startVision.value = vision.value
  lastStart = null
  pendingOp.value = 'starting'
  note('')
  const r = await navAction('start', { session: selected.value, vision: vision.value, dry_run: dryRun.value })
  if (!r.ok) { pendingOp.value = ''; note(r.msg || '启动失败', false) }
  refresh()
}
async function stop() {
  if (!confirm('停止导航？')) return
  pendingOp.value = 'stopping'
  const r = await navAction('stop')
  if (!r.ok) { pendingOp.value = ''; note(r.msg || '停止失败', false) }
  refresh()
}
// 兜底：点了之后 8 s 服务端状态还没变，就不再显示"进行中"
watch(pendingOp, v => { if (v) setTimeout(() => { if (pendingOp.value === v) pendingOp.value = '' }, 8000) })

const startFail = computed(() => {
  const f = nav.value.start_fail
  if (!f || nav.value.status !== 'idle' || pendingOp.value) return null
  return { step: f.step || lastStart?.step || '', elapsed: lastStart?.elapsed ?? 0, stepElapsed: lastStart?.stepElapsed ?? 0, msg: f.msg }
})

async function reloc(body) {
  if (relocBusy.value) return
  relocSent.value = true
  const r = await act('reloc', body)
  // 请求被拒绝，或 3 s 内定位模块没有开始，则恢复按钮
  if (!r.ok) relocSent.value = false
  else setTimeout(() => (relocSent.value = false), 3000)
}
const autoReloc = () => reloc({ mode: 'auto' })
const cancel = () => act('cancel')
const pending = computed(() => state.value?.pending || null)
const go = () => act('go')
const setSpeed = () => act('speed', { v: speed.value })

async function onPick(p) {
  const m = mode.value
  mode.value = ''
  if (m === 'goal') await act('plan', p)     // 只规划：路线画出来，等人确认后才出发
  else if (m === 'reloc') await reloc({ mode: 'manual', ...p })
}

// 急停（与 7010 录制页、7011 遥控页同一个锁定状态）；急停会取消本次导航
async function onEstop() {
  mode.value = ''
  nav.value = { ...nav.value, estop: true }
  const r = await estop().catch(e => ({ ok: false, msg: `急停请求失败：${e}` }))
  if (!r.ok) note(r.msg || '急停失败', false)
  refresh()
}
async function onRelease() {
  if (!confirm('解除急停？\n\n本次导航已取消，解除后需要重新选目标。')) return
  const r = await releaseEstop().catch(e => ({ ok: false, msg: String(e) }))
  if (!r.ok) note(r.msg || '解除失败', false)
  refresh()
}
function onKey(e) {
  if (e.key === ' ' && !['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) { e.preventDefault(); onEstop() }
  if (e.key === 'Escape') mode.value = ''
}

const LOC_TEXT = { unlocalized: '未定位', relocalizing: '定位中…', localized: '已定位', lost: '定位丢失' }
const STATE_TEXT = { idle: '待命', home: '前往目标', arrived: '已到达', error: '出错停车' }
const fmtPose = p => (p ? `(${p[0].toFixed(2)}, ${p[1].toFixed(2)}) ${(p[2] * 180 / Math.PI).toFixed(0)}°` : '—')
const curSpeed = computed(() => { const c = state.value?.cmd; return c ? Math.hypot(c[0], c[1]) : 0 })

onMounted(async () => {
  await refresh()
  schedule()
  window.addEventListener('keydown', onKey)
})
onUnmounted(() => {
  clearTimeout(timer); clearInterval(hbTimer); clearTimeout(alertTimer); es?.close()
  window.removeEventListener('keydown', onKey)
})
</script>

<template>
  <main class="layout">
    <header>
      <h1>Galbot G1 导航</h1>
      <a href="./">← 录制</a>
      <span class="muted">在录制生成的地图里定位、选目标前往 · 空格急停 · Esc 取消点选</span>
      <div v-if="nav.stale?.length" class="banner bad">
        服务端代码已更新但没有重启（{{ nav.stale.join('、') }}），页面功能可能失败。在机器人上运行 <code>web/web.sh restart</code>
      </div>
      <div v-if="nav.status === 'offline'" class="banner bad">连不上 7010 服务，正在重试…</div>
    </header>

    <section class="side">
      <div class="estop-box">
        <button v-if="!nav.estop" class="estop" @click="onEstop">急 停（空格）</button>
        <button v-else class="release" @click="onRelease">急停中 · 点击解除</button>
        <div v-if="nav.estop === null && !['connecting', 'offline'].includes(nav.status)" class="bad small">
          7011 遥控服务未就绪，急停可能无效
        </div>
      </div>

      <div v-if="alertMsg" class="banner" :class="alertMsg.ok ? 'info' : 'bad'" @click="note('')">
        {{ alertMsg.text }}<span class="x">×</span>
      </div>

      <!-- 地图与启停 -->
      <div class="card">
        <div class="title">
          <span class="dot" :class="{ on: running, busy }" />
          {{ running ? '导航运行中' : starting ? '正在启动' : stopping ? '正在停止' : '导航未运行' }}
          <template v-if="running || starting">
            <span class="tag" :class="nav.dry_run ? 'warn' : 'go'">{{ nav.dry_run ? '只算不动' : '实车' }}</span>
            <span class="tag">{{ nav.vision === false ? '雷达 + IMU' : '视觉 + 雷达' }}</span>
          </template>
        </div>
        <select v-model="selected" :disabled="running || busy">
          <option v-if="!nav.maps.length" value="">（还没有导航地图）</option>
          <option v-for="m in nav.maps" :key="m.session" :value="m.session">{{ m.name }}（{{ m.session }}）</option>
        </select>
        <div v-if="!nav.maps.length && nav.status === 'idle'" class="muted small">
          到 <a href="./">录制页</a> 的「录制记录」里，给一次完成的录制点「生成」。
        </div>
        <div v-else-if="mapMeta" class="muted small">
          {{ mapMeta.grid.w.toFixed(0) }} × {{ mapMeta.grid.h.toFixed(0) }} m · 建图路程 {{ mapMeta.path_length_m }} m · 生成于 {{ mapMeta.created }}
        </div>

        <template v-if="!running && !busy">
          <label class="chk"><input v-model="vision" type="checkbox" />
            <span>开视觉<span class="muted">（相机参与里程计，与建图时一致）</span></span></label>
          <div class="muted small opt">不勾：只用雷达 + IMU，启动更快、占用更低；空旷或少特征处可能不如开视觉稳</div>
          <label class="chk"><input v-model="dryRun" type="checkbox" />
            <span>只算不动<span class="muted">（照常定位、规划，不向底盘发速度）</span></span></label>
        </template>

        <StartProgress v-if="starting" :key="'s' + startVision" :steps="STEPS" what="导航" :step="nav.step || ''"
                       :step-elapsed="nav.step_elapsed_s != null ? Number(nav.step_elapsed_s) : null"
                       :elapsed="nav.elapsed_s != null ? Number(nav.elapsed_s) : null" />
        <StartProgress v-else-if="startFail" :key="'f' + startVision" :steps="STEPS" what="导航" :step="startFail.step"
                       :elapsed="startFail.elapsed" :step-elapsed="startFail.stepElapsed" failed :failed-msg="startFail.msg" />

        <div class="btns">
          <button class="go" :disabled="running || busy || !selected || nav.recording" @click="start">启动导航</button>
          <button class="stop" :disabled="!running || busy" @click="stop">停止导航</button>
        </div>
        <div v-if="nav.error && !startFail" class="bad small">{{ nav.error }}</div>
      </div>

      <template v-if="running">
        <!-- 定位 -->
        <div class="card">
          <div class="title">定位 <span class="tag" :class="loc.state">{{ LOC_TEXT[loc.state] || '等待中' }}</span></div>
          <div class="muted small">{{ loc.msg || '等待 FAST-LIVO2 位姿' }}</div>
          <div v-if="loc.fitness != null" class="muted small num">
            匹配 {{ (loc.fitness * 100).toFixed(0) }}% · 误差 {{ (loc.rmse * 100).toFixed(1) }} cm
            <template v-if="loc.time_s != null"> · 用时 {{ loc.time_s }} s</template>
          </div>
          <div class="muted small num">位置 {{ fmtPose(state?.pose) }}</div>
          <div v-if="localized && !confirmed" class="check">
            <b>请核对定位</b>
            <div class="small">地图上机器人的位置、车头朝向对吗？青色的实时障碍点应当压在红色的墙和障碍上。自动定位在相似的走廊、展区可能认错地方。</div>
            <div class="btns">
              <button class="go" @click="act('confirm')">位置正确</button>
              <button :disabled="relocBusy" @click="mode = 'reloc'">不对，手动定位</button>
            </div>
          </div>
          <div class="btns">
            <button :disabled="relocBusy" @click="autoReloc">{{ relocBusy ? '定位中…' : '自动重定位' }}</button>
            <button :disabled="relocBusy" :class="{ active: mode === 'reloc' }" @click="mode = mode === 'reloc' ? '' : 'reloc'">
              {{ mode === 'reloc' ? '取消点选' : '手动定位' }}
            </button>
          </div>
        </div>

        <!-- 行驶 -->
        <div class="card">
          <div class="title">行驶 <span class="tag" :class="state?.state">{{ STATE_TEXT[state?.state] || '—' }}</span></div>
          <div class="muted small">{{ state?.msg }}</div>
          <div v-if="state?.limit_reason" class="warn small">避障：{{ state.limit_reason }}</div>
          <div v-if="state?.teleop_msg" class="bad small">{{ state.teleop_msg }}</div>
          <div class="muted small num">当前速度 {{ curSpeed.toFixed(2) }} m/s</div>
          <div v-if="pending" class="check go-check">
            <b>确认路线</b>
            <div class="small">地图上绿色虚线是规划的路线，{{ pending.msg }}。确认后机器人才会出发。</div>
            <div class="btns">
              <button class="go" :disabled="!!nav.estop" @click="go">确认出发</button>
              <button @click="cancel">取消</button>
            </div>
          </div>
          <div class="btns">
            <button :disabled="!confirmed || !!nav.estop" :class="{ active: mode === 'goal', go: !pending }"
                    @click="mode = mode === 'goal' ? '' : 'goal'">
              {{ mode === 'goal' ? '取消点选' : (navigating || pending) ? '换个目标' : '选目标' }}
            </button>
            <button :disabled="!navigating" @click="cancel">取消导航</button>
          </div>
          <div v-if="!confirmed" class="muted small">定位并核对后才能选目标</div>
          <div class="speed">
            <span class="num">限速 {{ speed.toFixed(2) }} m/s</span>
            <input v-model.number="speed" type="range" :min="state?.v_range?.[0] ?? 0.1" :max="state?.v_range?.[1] ?? 0.5"
                   step="0.05" @change="setSpeed" />
          </div>
          <div class="muted small">本页关闭或手机息屏约 5 秒后，正在走的机器人会自动停下。</div>
        </div>
      </template>
    </section>

    <section class="map">
      <NavMap :grid-url="gridUrl" :grid="grid" :state="state" :traj="traj" :mode="mode" @pick="onPick" />
      <div v-if="pending && !mode" class="go-bar">
        <span>已规划路线，{{ pending.msg }}</span>
        <button class="go" :disabled="!!nav.estop" @click="go">确认出发</button>
        <button @click="cancel">取消</button>
      </div>
    </section>
  </main>
</template>

<style scoped>
.layout {
  display: grid; grid-template-columns: 360px 1fr; grid-template-rows: auto 1fr;
  grid-template-areas: "head head" "side map"; gap: 16px; padding: 16px; height: 100vh;
}
header { grid-area: head; display: flex; align-items: baseline; gap: 12px; flex-wrap: wrap; }
h1 { font-size: 20px; margin: 0; }
a { color: #3b82f6; }
.side { grid-area: side; display: flex; flex-direction: column; gap: 12px; overflow: auto; min-height: 0; }
.map { grid-area: map; min-height: 0; position: relative; }
.go-bar {
  position: absolute; left: 50%; top: 58px; transform: translateX(-50%); z-index: 5; max-width: calc(100% - 40px);
  display: flex; align-items: center; gap: 10px; flex-wrap: wrap; justify-content: center;
  background: var(--card); border: 2px solid var(--go); border-radius: 12px; padding: 8px 12px; font-size: 13px;
  box-shadow: 0 4px 16px rgba(0, 0, 0, 0.35);
}
.go-bar button { padding: 7px 16px; }
.check.go-check { border-color: var(--go); }
.card { display: flex; flex-direction: column; gap: 8px; }
.title { font-weight: 600; display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.estop-box { display: flex; flex-direction: column; gap: 6px; }
.estop, .release { border: 0; border-radius: 12px; padding: 16px; font-size: 20px; font-weight: 700; color: #fff; cursor: pointer; }
.estop { background: var(--stop); box-shadow: 0 2px 0 #991b1b; }
.release { background: var(--warn); animation: blink 1s infinite; }
@keyframes blink { 50% { opacity: 0.75; } }
.banner { border-radius: 8px; padding: 8px 12px; font-size: 13px; position: relative; white-space: pre-wrap; overflow-wrap: anywhere; }
header .banner { flex-basis: 100%; }
.banner.bad { background: var(--stop); color: #fff; }
.banner.info { background: color-mix(in srgb, #2563eb 14%, var(--card)); border: 1px solid color-mix(in srgb, #2563eb 40%, transparent); }
.side .banner { cursor: pointer; padding-right: 28px; }
.x { position: absolute; right: 10px; top: 6px; opacity: 0.7; }
.dot { width: 10px; height: 10px; border-radius: 50%; background: var(--muted); }
.dot.on { background: var(--go); }
.dot.busy { background: var(--warn); animation: blink 1s infinite; }
.tag { font-size: 12px; font-weight: 500; padding: 1px 8px; border-radius: 10px; background: var(--line); }
.tag.go, .tag.localized, .tag.home, .tag.arrived { background: var(--go); color: #fff; }
.tag.warn, .tag.relocalizing { background: var(--warn); color: #fff; }
.tag.lost, .tag.error, .tag.unlocalized { background: var(--stop); color: #fff; }
select { width: 100%; padding: 8px; border-radius: 8px; border: 1px solid var(--line); background: var(--card); color: var(--text); }
.chk { display: flex; gap: 6px; align-items: baseline; cursor: pointer; font-size: 13px; user-select: none; }
.opt { margin: -6px 0 0 22px; }
.btns { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
button {
  border: 1px solid var(--line); background: transparent; color: var(--text); border-radius: 8px;
  padding: 9px 10px; cursor: pointer; font-size: 14px;
}
button:disabled { opacity: 0.4; cursor: not-allowed; }
button.go { background: var(--go); color: #fff; border: 0; font-weight: 600; }
button.stop { background: var(--stop); color: #fff; border: 0; font-weight: 600; }
button.active { background: #2563eb; color: #fff; border-color: #2563eb; }
.check { border: 2px solid var(--warn); border-radius: 10px; padding: 10px; display: flex; flex-direction: column; gap: 6px; }
.speed { display: flex; align-items: center; gap: 10px; font-size: 13px; }
.speed input { flex: 1; }
.small { font-size: 12px; }
.bad { color: var(--stop); }
.banner.bad { color: #fff; }
.warn { color: var(--warn); }
code { background: rgba(0, 0, 0, 0.2); padding: 0 4px; border-radius: 4px; }

@media (max-width: 860px) {
  .layout {
    grid-template-columns: 1fr; grid-template-rows: auto auto 72vh;
    grid-template-areas: "head" "side" "map"; height: auto;
  }
  .side { overflow: visible; }
  .estop-box { position: sticky; top: 0; z-index: 10; background: var(--bg); padding: 4px 0; }
}
</style>
