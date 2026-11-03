<script setup>
// 实时俯视地图：累积栅格（按高度着色）+ 当前扫描 + 机器人位置与轨迹；自主探索时叠加 2D 栅格、边界、目标与路径
import { onMounted, onUnmounted, ref, watch } from 'vue'
import { exploreGridUrl, liveUrl } from '../api.js'
import { colorOf, legendGradient, Z_MIN, Z_MAX } from '../colormap.js'

const props = defineProps({ recording: Boolean, explore: { type: Object, default: null } })

const TILE = 256
const EMPTY = -32768

const canvasEl = ref(null)
const wrapEl = ref(null)
const follow = ref(true)
const showScan = ref(true)
const showExplore = ref(true)
const conn = ref('idle')          // idle | connecting | live | ended
const stats = ref({ cells: 0, area: 0, path: 0, fps: 0 })

// 地图数据不放进 Vue 响应式系统，避免大量对象被代理
let cell = 0.05
let tiles = new Map()             // "tx,ty" -> { canvas, ctx, img, z, dirty }
let traj = []
let pose = null
let scan = new Int16Array(0)
let cellCount = 0
let pathLen = 0

// 视图：屏幕中心对应的世界坐标 (vx, vy)，scale 为像素/米
const view = { vx: 0, vy: 0, scale: 40 }
let W = 0, H = 0, dpr = 1
let needDraw = true
let frames = 0, lastFpsT = performance.now()

// 自主探索叠加层：栅格图随状态每秒刷新，加载完成再替换，避免闪烁
let gridImg = null
let gridMeta = null
watch(() => props.explore, ex => {
  needDraw = true
  if (!ex?.grid) return
  const img = new Image()
  img.onload = () => { gridImg = img; gridMeta = ex.grid; needDraw = true }
  img.src = `${exploreGridUrl}?t=${Date.now()}`
})

let es = null
let retryTimer = null
let raf = 0

function decode(b64) {
  const bin = atob(b64)
  const bytes = new Uint8Array(bin.length)
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i)
  return new Int16Array(bytes.buffer)
}

function resetMap() {
  tiles = new Map()
  traj = []
  pose = null
  scan = new Int16Array(0)
  cellCount = 0
  pathLen = 0
}

function tileOf(tx, ty) {
  const key = tx + ',' + ty
  let t = tiles.get(key)
  if (!t) {
    const canvas = document.createElement('canvas')
    canvas.width = canvas.height = TILE
    const ctx = canvas.getContext('2d')
    t = { tx, ty, canvas, ctx, img: ctx.createImageData(TILE, TILE), z: new Int16Array(TILE * TILE).fill(EMPTY), dirty: false }
    tiles.set(key, t)
  }
  return t
}

function setCells(arr) {
  for (let i = 0; i < arr.length; i += 3) {
    const ix = arr[i], iy = arr[i + 1], z = arr[i + 2]
    const t = tileOf(ix >> 8, iy >> 8)
    const lx = ix & 255, ly = iy & 255
    const k = ly * TILE + lx
    if (t.z[k] === EMPTY) cellCount++
    t.z[k] = z
    // 图像第 0 行对应 tile 的最大 y（屏幕上方）
    const p = ((255 - ly) * TILE + lx) * 4
    const [r, g, b] = colorOf(z)
    t.img.data[p] = r
    t.img.data[p + 1] = g
    t.img.data[p + 2] = b
    t.img.data[p + 3] = 255
    t.dirty = true
  }
}

function pushPose(p) {
  if (!p) return
  pose = p
  const last = traj[traj.length - 1]
  if (!last || Math.hypot(p[0] - last[0], p[1] - last[1]) > 0.05) {
    if (last) pathLen += Math.hypot(p[0] - last[0], p[1] - last[1])
    traj.push([p[0], p[1]])
  }
}

function onSnapshot(e) {
  const d = JSON.parse(e.data)
  resetMap()
  cell = d.cell
  setCells(decode(d.cells))
  traj = d.traj || []
  for (let i = 1; i < traj.length; i++) pathLen += Math.hypot(traj[i][0] - traj[i - 1][0], traj[i][1] - traj[i - 1][1])
  pose = d.pose
  conn.value = 'live'
  needDraw = true
}

function onFrame(e) {
  const d = JSON.parse(e.data)
  setCells(decode(d.d))
  scan = decode(d.s)
  pushPose(d.p)
  frames++
  needDraw = true
}

function connect() {
  disconnect()
  conn.value = 'connecting'
  es = new EventSource(liveUrl)
  es.addEventListener('snapshot', onSnapshot)
  es.addEventListener('frame', onFrame)
  es.onerror = () => {
    // 录制刚开始时推送节点可能还没就绪，稍后重连
    disconnect()
    if (props.recording) retryTimer = setTimeout(connect, 2000)
  }
}

function disconnect() {
  clearTimeout(retryTimer)
  if (es) { es.close(); es = null }
}

watch(() => props.recording, rec => {
  if (rec) connect()
  else {
    disconnect()
    if (conn.value === 'live') conn.value = 'ended'
  }
}, { immediate: true })

// ------------------------------------------------------------------ 绘制
function worldToScreen(x, y) {
  return [W / 2 + (x - view.vx) * view.scale, H / 2 - (y - view.vy) * view.scale]
}

function niceStep(px) {
  const m = px / view.scale
  const p = Math.pow(10, Math.floor(Math.log10(m)))
  return [1, 2, 5, 10].map(k => k * p).find(s => s >= m)
}

function draw() {
  const ctx = canvasEl.value.getContext('2d')
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  ctx.fillStyle = '#0e1013'
  ctx.fillRect(0, 0, W, H)

  if (follow.value && pose) { view.vx = pose[0]; view.vy = pose[1] }

  // 网格
  const step = niceStep(80)
  ctx.strokeStyle = 'rgba(255,255,255,0.06)'
  ctx.lineWidth = 1
  ctx.beginPath()
  const x0 = Math.floor((view.vx - W / 2 / view.scale) / step) * step
  const y0 = Math.floor((view.vy - H / 2 / view.scale) / step) * step
  for (let x = x0; x < view.vx + W / 2 / view.scale; x += step) {
    const [sx] = worldToScreen(x, 0); ctx.moveTo(sx, 0); ctx.lineTo(sx, H)
  }
  for (let y = y0; y < view.vy + H / 2 / view.scale; y += step) {
    const [, sy] = worldToScreen(0, y); ctx.moveTo(0, sy); ctx.lineTo(W, sy)
  }
  ctx.stroke()

  // 地图 tiles
  ctx.imageSmoothingEnabled = false
  const size = TILE * cell * view.scale
  for (const t of tiles.values()) {
    if (t.dirty) { t.ctx.putImageData(t.img, 0, 0); t.dirty = false }
    const [sx, sy] = worldToScreen(t.tx * TILE * cell, (t.ty + 1) * TILE * cell)
    if (sx > W || sy > H || sx + size < 0 || sy + size < 0) continue
    ctx.drawImage(t.canvas, sx, sy, size, size)
  }

  // 自主探索：2D 栅格（白 可通行 / 红 障碍）、拉黑点、边界、路径、目标、起点
  const ex = props.explore
  if (ex && showExplore.value) {
    if (gridImg && gridMeta) {
      const [gx, gy] = worldToScreen(gridMeta.x0, gridMeta.y0 + gridMeta.h)
      ctx.drawImage(gridImg, gx, gy, gridMeta.w * view.scale, gridMeta.h * view.scale)
    }
    ctx.fillStyle = '#facc15'
    for (const [[x, y], n] of ex.frontiers || []) {
      const [sx, sy] = worldToScreen(x, y)
      ctx.beginPath(); ctx.arc(sx, sy, Math.min(9, 3 + n / 10), 0, Math.PI * 2); ctx.fill()
    }
    ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 2
    for (const [x, y] of ex.blacklist || []) {
      const [sx, sy] = worldToScreen(x, y)
      ctx.beginPath(); ctx.moveTo(sx - 6, sy - 6); ctx.lineTo(sx + 6, sy + 6)
      ctx.moveTo(sx + 6, sy - 6); ctx.lineTo(sx - 6, sy + 6); ctx.stroke()
    }
    if (ex.path?.length > 1) {
      ctx.strokeStyle = '#22c55e'; ctx.lineWidth = 3
      ctx.beginPath()
      ex.path.forEach(([x, y], i) => { const [sx, sy] = worldToScreen(x, y); i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy) })
      ctx.stroke()
    }
    if (ex.goal) {
      const [sx, sy] = worldToScreen(ex.goal[0], ex.goal[1])
      ctx.strokeStyle = '#22c55e'; ctx.lineWidth = 3
      ctx.beginPath(); ctx.arc(sx, sy, 8, 0, Math.PI * 2); ctx.stroke()
    }
    if (ex.home) {
      const [sx, sy] = worldToScreen(ex.home[0], ex.home[1])
      ctx.fillStyle = '#3b82f6'; ctx.fillRect(sx - 6, sy - 6, 12, 12)
    }
    ctx.lineWidth = 1
  }

  // 当前扫描
  if (showScan.value && scan.length) {
    ctx.fillStyle = 'rgba(255,255,255,0.75)'
    const r = Math.max(1.5, cell * view.scale)
    for (let i = 0; i < scan.length; i += 2) {
      const [sx, sy] = worldToScreen((scan[i] + 0.5) * cell, (scan[i + 1] + 0.5) * cell)
      ctx.fillRect(sx - r / 2, sy - r / 2, r, r)
    }
  }

  // 轨迹
  if (traj.length > 1) {
    ctx.strokeStyle = '#ff4fd8'
    ctx.lineWidth = 2
    ctx.beginPath()
    traj.forEach(([x, y], i) => {
      const [sx, sy] = worldToScreen(x, y)
      i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy)
    })
    ctx.stroke()
  }

  // 机器人
  if (pose) {
    const [sx, sy] = worldToScreen(pose[0], pose[1])
    ctx.save()
    ctx.translate(sx, sy)
    ctx.rotate(-pose[2])
    ctx.fillStyle = '#ffffff'
    ctx.strokeStyle = '#000'
    ctx.lineWidth = 1.5
    ctx.beginPath()
    ctx.moveTo(14, 0); ctx.lineTo(-9, 8); ctx.lineTo(-4, 0); ctx.lineTo(-9, -8); ctx.closePath()
    ctx.fill(); ctx.stroke()
    ctx.restore()
  }

  // 比例尺
  const barM = niceStep(100)
  const barPx = barM * view.scale
  ctx.fillStyle = ctx.strokeStyle = 'rgba(255,255,255,0.85)'
  ctx.lineWidth = 2
  ctx.beginPath()
  ctx.moveTo(16, H - 16); ctx.lineTo(16 + barPx, H - 16)
  ctx.moveTo(16, H - 21); ctx.lineTo(16, H - 11)
  ctx.moveTo(16 + barPx, H - 21); ctx.lineTo(16 + barPx, H - 11)
  ctx.stroke()
  ctx.font = '12px sans-serif'
  ctx.fillText(barM >= 1 ? `${barM} m` : `${barM * 100} cm`, 16, H - 26)
}

function loop(now) {
  if (now - lastFpsT > 1000) {
    stats.value = { cells: cellCount, area: cellCount * cell * cell, path: pathLen, fps: frames * 1000 / (now - lastFpsT) }
    frames = 0
    lastFpsT = now
  }
  if (needDraw && canvasEl.value) { draw(); needDraw = false }
  raf = requestAnimationFrame(loop)
}

function resize() {
  const r = wrapEl.value.getBoundingClientRect()
  dpr = window.devicePixelRatio || 1
  W = r.width; H = r.height
  canvasEl.value.width = Math.round(W * dpr)
  canvasEl.value.height = Math.round(H * dpr)
  needDraw = true
}

// ------------------------------------------------------------------ 交互：拖动平移、滚轮/双指缩放
const pointers = new Map()
let pinchDist = 0

function zoomAt(factor, sx, sy) {
  const s = Math.min(800, Math.max(2, view.scale * factor))
  const wx = view.vx + (sx - W / 2) / view.scale
  const wy = view.vy - (sy - H / 2) / view.scale
  view.scale = s
  if (!follow.value) {
    view.vx = wx - (sx - W / 2) / s
    view.vy = wy + (sy - H / 2) / s
  }
  needDraw = true
}

function onWheel(e) {
  e.preventDefault()
  const r = canvasEl.value.getBoundingClientRect()
  zoomAt(Math.exp(-e.deltaY * 0.0015), e.clientX - r.left, e.clientY - r.top)
}

function onPointerDown(e) {
  canvasEl.value.setPointerCapture(e.pointerId)
  pointers.set(e.pointerId, [e.clientX, e.clientY])
  if (pointers.size === 2) {
    const [a, b] = [...pointers.values()]
    pinchDist = Math.hypot(a[0] - b[0], a[1] - b[1])
  }
}

function onPointerMove(e) {
  const prev = pointers.get(e.pointerId)
  if (!prev) return
  pointers.set(e.pointerId, [e.clientX, e.clientY])
  if (pointers.size === 1) {
    follow.value = false
    view.vx -= (e.clientX - prev[0]) / view.scale
    view.vy += (e.clientY - prev[1]) / view.scale
    needDraw = true
  } else if (pointers.size === 2) {
    const [a, b] = [...pointers.values()]
    const d = Math.hypot(a[0] - b[0], a[1] - b[1])
    const r = canvasEl.value.getBoundingClientRect()
    if (pinchDist) zoomAt(d / pinchDist, (a[0] + b[0]) / 2 - r.left, (a[1] + b[1]) / 2 - r.top)
    pinchDist = d
  }
}

function onPointerUp(e) {
  pointers.delete(e.pointerId)
  pinchDist = 0
}

function fitAll() {
  if (!tiles.size) return
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity
  for (const t of tiles.values()) {
    for (let k = 0; k < t.z.length; k++) {
      if (t.z[k] === EMPTY) continue
      const x = (t.tx * TILE + (k & 255)) * cell, y = (t.ty * TILE + (k >> 8)) * cell
      if (x < minX) minX = x; if (x > maxX) maxX = x
      if (y < minY) minY = y; if (y > maxY) maxY = y
    }
  }
  follow.value = false
  view.vx = (minX + maxX) / 2
  view.vy = (minY + maxY) / 2
  view.scale = Math.min(800, Math.max(2, 0.9 * Math.min(W / (maxX - minX + 1), H / (maxY - minY + 1))))
  needDraw = true
}

watch([follow, showScan, showExplore], () => (needDraw = true))

let ro = null
onMounted(() => {
  ro = new ResizeObserver(resize)
  ro.observe(wrapEl.value)
  resize()
  raf = requestAnimationFrame(loop)
})
onUnmounted(() => {
  disconnect()
  cancelAnimationFrame(raf)
  ro?.disconnect()
})
</script>

<template>
  <div class="card live">
    <div class="bar">
      <span class="title">实时地图</span>
      <span class="badge" :class="conn">
        {{ { idle: '未在录制', connecting: '连接中…', live: '实时', ended: '录制已结束' }[conn] }}
      </span>
      <span class="stats muted num">
        {{ stats.area.toFixed(1) }} m² · 轨迹 {{ stats.path.toFixed(1) }} m · {{ stats.fps.toFixed(1) }} 帧/秒
      </span>
      <span class="grow" />
      <label><input v-model="showScan" type="checkbox" /> 当前扫描</label>
      <label v-if="explore"><input v-model="showExplore" type="checkbox" /> 探索栅格</label>
      <label><input v-model="follow" type="checkbox" /> 跟随机器人</label>
      <button @click="fitAll">全图</button>
    </div>
    <div ref="wrapEl" class="canvas-wrap">
      <canvas
        ref="canvasEl"
        @wheel="onWheel"
        @pointerdown="onPointerDown"
        @pointermove="onPointerMove"
        @pointerup="onPointerUp"
        @pointercancel="onPointerUp"
      />
      <div v-if="conn === 'idle'" class="hint">开始录制后，这里会实时显示正在建的地图</div>
      <div class="legend">
        <div class="ramp" :style="{ background: legendGradient }" />
        <div class="ticks num"><span>{{ Z_MIN }} m</span><span>高度</span><span>{{ Z_MAX }} m</span></div>
      </div>
    </div>
  </div>
</template>

<style scoped>
.live { display: flex; flex-direction: column; height: 100%; padding: 12px; gap: 10px; }
.bar { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; font-size: 13px; }
.title { font-weight: 600; }
.grow { flex: 1; }
.badge { font-size: 12px; padding: 1px 8px; border-radius: 10px; background: var(--line); }
.badge.live { background: var(--go); color: #fff; }
.badge.connecting { background: var(--warn); color: #fff; }
.stats { font-size: 12px; }
label { display: flex; align-items: center; gap: 4px; cursor: pointer; user-select: none; }
button {
  border: 1px solid var(--line); background: transparent; color: var(--text);
  border-radius: 6px; padding: 3px 10px; cursor: pointer; font-size: 13px;
}
.canvas-wrap {
  position: relative; flex: 1; min-height: 300px; border-radius: 8px; overflow: hidden;
  background: var(--map-bg);
}
canvas { position: absolute; inset: 0; width: 100%; height: 100%; touch-action: none; cursor: grab; }
canvas:active { cursor: grabbing; }
.hint {
  position: absolute; inset: 0; display: grid; place-items: center;
  color: rgba(255, 255, 255, 0.5); pointer-events: none; font-size: 14px;
}
.legend { position: absolute; right: 12px; bottom: 12px; width: 160px; color: rgba(255, 255, 255, 0.8); font-size: 11px; pointer-events: none; }
.ramp { height: 8px; border-radius: 4px; }
.ticks { display: flex; justify-content: space-between; margin-top: 2px; }
</style>
