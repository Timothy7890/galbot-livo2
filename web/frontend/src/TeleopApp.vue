<script setup>
// 底盘遥控：按住按钮或键盘（W/S 前后、A/D 横移、Q/E 转向、空格急停）期间每 0.1 s 发一次速度指令，
// 松手立即平顺减速停车；切走页面、断线时服务端 0.5 s 内开始减速、1 s 内强制停车
import { computed, onMounted, onUnmounted, reactive, ref } from 'vue'
import { calibrate, estop, getStatus, release, sendCmd, setLeg } from './teleopApi.js'
import CameraView from './components/CameraView.vue'
import ObstacleView from './components/ObstacleView.vue'

const status = ref({ sdk_ok: false })
const offline = ref(false)
const cmdError = ref('')
const speed = ref(0.4)          // m/s
const turn = ref(0.8)           // rad/s
const safe = ref(true)          // 我们的避障（Mid-360 + 超声波）

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

async function onCalibrate() {
  if (!confirm('标定车身：机器人静止约 2 s，记录 Mid-360 看到的机器人自身（手臂等）。\n\n' +
    '请先确认机器人四周 0.8 m 内没有任何障碍物和人，否则障碍物会被当成机器人的一部分。开始？')) return
  const r = await calibrate()
  cmdError.value = r.ok ? '' : r.msg
}

async function onLeg(pose, label) {
  if (!confirm(`躯干升降到「${label}」：关节速度 0.1 rad/s，约 10–20 s。\n\n` +
    '请确认机器人周围（尤其上方和手臂附近）没有人和障碍物，并随时准备点急停（急停会让躯干停在当前位置）。\n' +
    '升降后请重新标定车身轮廓（相机外参在下次开始录制时自动更新）。开始？')) return
  releaseAll()
  const r = await setLeg(pose)
  cmdError.value = r.ok ? '' : r.msg
  refresh()
}

function onSafeChange(e) {
  if (!e.target.checked && !confirm('关闭避障后底盘不会因障碍减速或停车，只能靠你手动停车。确定关闭？')) {
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
        避障（Mid-360 + 超声波，按运动方向减速 / 停车）
      </label>
      <button class="calib" :disabled="!status.sdk_ok || status.moving || status.calibrating" @click="onCalibrate">
        {{ status.calibrating ? '标定中…' : '标定车身轮廓' }}
      </button>
      <div v-if="status.calib_msg" class="muted calib-msg">{{ status.calib_msg }}</div>

      <div class="leg">
        <div class="leg-head">躯干高度
          <span class="muted">{{ status.leg_target ? '升降中…' : status.leg_pose ? status.leg_poses?.[status.leg_pose] : status.leg ? '自定义' : '-' }}</span>
        </div>
        <div class="leg-btns">
          <button v-for="(label, k) in status.leg_poses || {}" :key="k" class="calib"
                  :class="{ cur: status.leg_pose === k }"
                  :disabled="!status.sdk_ok || status.estop || status.moving || status.calibrating || !!status.leg_target || status.recording"
                  @click="onLeg(k, label)">{{ label }}</button>
        </div>
        <div class="muted calib-msg num">关节 {{ status.leg ? status.leg.map(v => v.toFixed(2)).join(' / ') : '-' }}</div>
        <div v-if="status.recording" class="muted calib-msg">录制中不能升降</div>
        <div v-if="status.leg_msg" class="muted calib-msg">{{ status.leg_msg }}</div>
      </div>
    </section>

    <section class="card info">
      <div class="title">状态</div>
      <table class="num">
        <tr><td>位置</td><td>x {{ fmt(status.odom?.x) }} m · y {{ fmt(status.odom?.y) }} m · 朝向 {{ fmt(status.odom?.yaw, 1) }}°</td></tr>
        <tr><td>实际速度</td><td>vx {{ fmt(status.odom?.vx) }} · vy {{ fmt(status.odom?.vy) }} m/s · wz {{ fmt(status.odom?.wz) }} rad/s</td></tr>
        <tr><td>网页指令</td><td>vx {{ fmt(command[0]) }} · vy {{ fmt(command[1]) }} m/s · wz {{ fmt(command[2]) }} rad/s</td></tr>
        <tr><td>实际下发</td><td>vx {{ fmt(status.applied?.[0]) }} · vy {{ fmt(status.applied?.[1]) }} m/s · wz {{ fmt(status.applied?.[2]) }} rad/s</td></tr>
        <tr><td>底盘调用</td><td>{{ status.sdk_ms_p90 != null ? `${status.sdk_ms_p90} ms（最近 100 次的 90% 分位）` : '-' }}</td></tr>
        <tr><td>结果</td><td :class="{ warn: /停止|禁止|减速|减慢|中断/.test(cmdError || status.last_result) }">{{ cmdError || status.last_result || '-' }}</td></tr>
      </table>
    </section>

    <ObstacleView class="obst" :active="status.sdk_ok" :command="command" />
    <CameraView class="cams" :active="status.sdk_ok" idle-text="连接 SDK 后显示" />
  </main>
</template>

<style scoped>
.layout {
  display: grid;
  grid-template-columns: 340px minmax(320px, 1fr) minmax(320px, 1fr);
  grid-template-rows: auto auto 1fr;
  grid-template-areas: "head head head" "ctrl obst cams" "info obst cams";
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
.obst { grid-area: obst; }
.calib {
  border: 1px solid var(--line); background: transparent; color: var(--text); border-radius: 8px;
  padding: 8px; cursor: pointer;
}
.calib:disabled { opacity: 0.4; cursor: default; }
.calib-msg { font-size: 12px; }
.leg { display: flex; flex-direction: column; gap: 6px; border-top: 1px solid var(--line); padding-top: 10px; }
.leg-head { display: flex; justify-content: space-between; font-size: 13px; font-weight: 600; }
.leg-btns { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }
.calib.cur { border-color: var(--go); color: var(--go); font-weight: 600; }
td.warn { color: var(--warn); font-weight: 600; }
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
  .layout { grid-template-columns: 1fr; grid-template-areas: "head" "ctrl" "obst" "cams" "info"; }
}
</style>
