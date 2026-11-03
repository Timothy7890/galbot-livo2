<script setup>
// 底盘遥控：按住按钮或键盘（W/S 前后、A/D 横移、Q/E 转向、空格急停）期间每 0.1 s 发一次速度指令，
// 松手、切走页面、断线都会让服务端在 0.35 s 内停车
import { computed, onMounted, onUnmounted, reactive, ref } from 'vue'
import { estop, getStatus, release, sendCmd } from './teleopApi.js'
import CameraView from './components/CameraView.vue'

const status = ref({ sdk_ok: false })
const offline = ref(false)
const cmdError = ref('')
const speed = ref(0.2)          // m/s
const turn = ref(0.5)           // rad/s
const safe = ref(true)          // Galbot 碰撞检测

// 按键 -> [vx, vy, wz] 方向
const DIRS = {
  w: [1, 0, 0], s: [-1, 0, 0], a: [0, 1, 0], d: [0, -1, 0], q: [0, 0, 1], e: [0, 0, -1],
}
const PAD = [
  [['q', '⟲ 左转'], ['w', '▲ 前进'], ['e', '右转 ⟳']],
  [['a', '◀ 左移'], null, ['d', '右移 ▶']],
  [null, ['s', '▼ 后退'], null],
]
const held = reactive(new Set())

const command = computed(() => {
  let vx = 0, vy = 0, wz = 0
  for (const k of held) {
    vx += DIRS[k][0]; vy += DIRS[k][1]; wz += DIRS[k][2]
  }
  return [vx * speed.value, vy * speed.value, wz * turn.value]
})

let sendTimer = null
let wasMoving = false

async function tick() {
  const [vx, vy, wz] = command.value
  const moving = vx || vy || wz
  if (!moving && !wasMoving) return
  wasMoving = moving
  try {
    const r = await sendCmd(vx, vy, wz, safe.value)
    cmdError.value = r.ok ? '' : r.msg
  } catch (e) {
    cmdError.value = String(e)
  }
}

function press(k) {
  if (status.value.estop) return
  held.add(k)
}
function releaseKey(k) { held.delete(k) }
function releaseAll() { held.clear() }

async function onEstop() {
  releaseAll()
  await estop()
  refresh()
}

async function onRelease() {
  await release()
  refresh()
}

function onSafeChange(e) {
  if (!e.target.checked && !confirm('关闭碰撞检测后底盘不会避障，只能靠你手动停车。确定关闭？')) {
    e.target.checked = true
    return
  }
  safe.value = e.target.checked
}

function onKeyDown(e) {
  if (e.target.tagName === 'INPUT') return
  const k = e.key.toLowerCase()
  if (k === ' ') { e.preventDefault(); onEstop(); return }
  if (DIRS[k] && !e.repeat) press(k)
}
function onKeyUp(e) {
  const k = e.key.toLowerCase()
  if (DIRS[k]) releaseKey(k)
}
function onVisibility() { if (document.hidden) releaseAll() }

let statusTimer = null
async function refresh() {
  try {
    status.value = await getStatus()
    offline.value = false
  } catch {
    offline.value = true
    releaseAll()
  }
}

onMounted(() => {
  refresh()
  statusTimer = setInterval(refresh, 500)
  sendTimer = setInterval(tick, 100)
  window.addEventListener('keydown', onKeyDown)
  window.addEventListener('keyup', onKeyUp)
  window.addEventListener('blur', releaseAll)
  document.addEventListener('visibilitychange', onVisibility)
})
onUnmounted(() => {
  clearInterval(statusTimer)
  clearInterval(sendTimer)
  window.removeEventListener('keydown', onKeyDown)
  window.removeEventListener('keyup', onKeyUp)
  window.removeEventListener('blur', releaseAll)
  document.removeEventListener('visibilitychange', onVisibility)
})

const fmt = (v, d = 2) => (v == null ? '-' : Number(v).toFixed(d))
</script>

<template>
  <main class="layout">
    <header>
      <h1>Galbot G1 底盘遥控</h1>
      <span class="muted">W/S 前后 · A/D 横移 · Q/E 转向 · 空格急停</span>
    </header>

    <section class="card ctrl">
      <div class="state">
        <span class="dot" :class="{ ok: status.sdk_ok && !status.estop, bad: offline || status.estop }" />
        <b>{{ offline ? '无法连接到机器人上的服务' : !status.sdk_ok ? '正在连接 Galbot SDK…' : status.estop ? '急停中' : status.moving ? '运动中' : '待命' }}</b>
      </div>
      <div v-if="status.sdk_error" class="err">{{ status.sdk_error }}</div>

      <button v-if="!status.estop" class="estop" @click="onEstop">急停</button>
      <button v-else class="release" @click="onRelease">解除急停</button>

      <div class="pad" :class="{ disabled: !status.sdk_ok || status.estop }">
        <template v-for="(row, i) in PAD" :key="i">
          <template v-for="(b, j) in row" :key="j">
            <button v-if="b" class="key" :class="{ on: held.has(b[0]) }"
                    @pointerdown.prevent="press(b[0])" @pointerup="releaseKey(b[0])"
                    @pointerleave="releaseKey(b[0])" @pointercancel="releaseKey(b[0])"
                    @contextmenu.prevent>
              {{ b[1] }}<small>{{ b[0].toUpperCase() }}</small>
            </button>
            <span v-else />
          </template>
        </template>
      </div>

      <label class="slider">线速度 <b class="num">{{ speed.toFixed(2) }} m/s</b>
        <input v-model.number="speed" type="range" min="0.05" :max="status.max_v || 0.5" step="0.05" />
      </label>
      <label class="slider">角速度 <b class="num">{{ turn.toFixed(2) }} rad/s</b>
        <input v-model.number="turn" type="range" min="0.1" :max="status.max_w || 1.0" step="0.05" />
      </label>
      <label class="check">
        <input type="checkbox" :checked="safe" @change="onSafeChange" />
        碰撞检测（Galbot 融合全部传感器避障）
      </label>
    </section>

    <section class="card info">
      <div class="title">状态</div>
      <table class="num">
        <tr><td>位置</td><td>x {{ fmt(status.odom?.x) }} m · y {{ fmt(status.odom?.y) }} m · 朝向 {{ fmt(status.odom?.yaw, 1) }}°</td></tr>
        <tr><td>实际速度</td><td>vx {{ fmt(status.odom?.vx) }} · vy {{ fmt(status.odom?.vy) }} m/s · wz {{ fmt(status.odom?.wz) }} rad/s</td></tr>
        <tr><td>发送指令</td><td>vx {{ fmt(command[0]) }} · vy {{ fmt(command[1]) }} m/s · wz {{ fmt(command[2]) }} rad/s</td></tr>
        <tr><td>指令结果</td><td>{{ cmdError || status.last_result || '-' }}</td></tr>
        <tr><td>导航状态</td><td>{{ status.nav_status || '-' }}</td></tr>
      </table>
    </section>

    <CameraView class="cams" :active="status.sdk_ok" idle-text="连接 SDK 后显示" />
  </main>
</template>

<style scoped>
.layout {
  display: grid;
  grid-template-columns: 360px 1fr;
  grid-template-rows: auto auto 1fr;
  grid-template-areas: "head head" "ctrl cams" "info cams";
  gap: 16px;
  padding: 16px;
  min-height: 100vh;
  align-items: start;
}
header { grid-area: head; display: flex; align-items: baseline; gap: 12px; flex-wrap: wrap; }
h1 { font-size: 20px; margin: 0; }
.ctrl { grid-area: ctrl; display: flex; flex-direction: column; gap: 12px; }
.info { grid-area: info; }
.cams { grid-area: cams; }
.state { display: flex; align-items: center; gap: 10px; font-size: 16px; }
.dot { width: 12px; height: 12px; border-radius: 50%; background: var(--warn); }
.dot.ok { background: var(--go); }
.dot.bad { background: var(--stop); }
.err { color: var(--stop); font-size: 12px; white-space: pre-wrap; }
.estop, .release {
  border: 0; border-radius: 12px; padding: 16px; font-size: 20px; font-weight: 700; color: #fff; cursor: pointer;
}
.estop { background: var(--stop); }
.release { background: var(--warn); }
.pad { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; user-select: none; }
.pad.disabled { opacity: 0.35; pointer-events: none; }
.key {
  border: 1px solid var(--line); background: var(--bg); color: var(--text); border-radius: 10px;
  padding: 16px 4px; font-size: 15px; cursor: pointer; touch-action: none;
  display: flex; flex-direction: column; align-items: center; gap: 2px;
}
.key small { color: var(--muted); font-size: 11px; }
.key.on { background: var(--go); color: #fff; border-color: var(--go); }
.key.on small { color: #fff; }
.slider { display: flex; flex-direction: column; gap: 4px; font-size: 13px; }
.check { display: flex; align-items: center; gap: 6px; font-size: 13px; cursor: pointer; }
.title { font-weight: 600; margin-bottom: 6px; }
table { width: 100%; font-size: 12px; border-collapse: collapse; }
td { padding: 4px 2px; border-bottom: 1px solid var(--line); vertical-align: top; }
td:first-child { color: var(--muted); white-space: nowrap; width: 64px; }

@media (max-width: 860px) {
  .layout { grid-template-columns: 1fr; grid-template-areas: "head" "ctrl" "cams" "info"; }
}
</style>
