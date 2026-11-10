<script setup>
// 导航地图：成熟地图的 2D 栅格（运行中为实时更新的栅格）+ 机器人、目标、路线；
// mode = 'goal'：点一下选目标；mode = 'reloc'：按下定位置、拖动定车头朝向；其余时候拖动平移、滚轮 / 双指缩放
import { onMounted, onUnmounted, ref, watch } from 'vue'

const props = defineProps({
  gridUrl: { type: String, default: '' },     // 栅格图片地址（变化即重新加载）
  grid: { type: Object, default: null },      // { x0, y0, w, h } 世界范围
  pose: { type: Array, default: null },       // [x, y, yaw]
  target: { type: Array, default: null },
  path: { type: Array, default: null },
  candidates: { type: Array, default: null }, // 重定位候选 [{x, y, yaw}]
  mode: { type: String, default: '' },
})
const emit = defineEmits(['pick'])

const canvasEl = ref(null)
const wrapEl = ref(null)
const follow = ref(false)
const view = { vx: 0, vy: 0, scale: 20 }
let W = 0, H = 0, dpr = 1
let needDraw = true
let raf = 0
let img = null
let fitted = false
let drag = null       // reloc 模式下按下的位置与当前指针（世界坐标）

watch(() => props.gridUrl, url => {
  if (!url) { img = null; needDraw = true; return }
  const im = new Image()
  im.onload = () => { img = im; needDraw = true; if (!fitted) fitAll() }
  im.src = url
}, { immediate: true })
watch(() => [props.grid, props.pose, props.target, props.path, props.candidates, props.mode, follow.value], () => (needDraw = true))

function w2s(x, y) { return [W / 2 + (x - view.vx) * view.scale, H / 2 - (y - view.vy) * view.scale] }
function s2w(sx, sy) { return [view.vx + (sx - W / 2) / view.scale, view.vy - (sy - H / 2) / view.scale] }

function arrow(ctx, x, y, yaw, color, size = 1) {
  const [sx, sy] = w2s(x, y)
  ctx.save()
  ctx.translate(sx, sy)
  ctx.rotate(-yaw)
  ctx.scale(size, size)
  ctx.fillStyle = color
  ctx.strokeStyle = '#000'
  ctx.lineWidth = 1.5
  ctx.beginPath()
  ctx.moveTo(14, 0); ctx.lineTo(-9, 8); ctx.lineTo(-4, 0); ctx.lineTo(-9, -8); ctx.closePath()
  ctx.fill(); ctx.stroke()
  ctx.restore()
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
  if (follow.value && props.pose) { view.vx = props.pose[0]; view.vy = props.pose[1] }
  const g = props.grid
  if (img && g) {
    ctx.imageSmoothingEnabled = false
    const [gx, gy] = w2s(g.x0, g.y0 + g.h)
    ctx.drawImage(img, gx, gy, g.w * view.scale, g.h * view.scale)
  }
  for (const c of props.candidates || []) arrow(ctx, c.x, c.y, c.yaw, 'rgba(250,204,21,0.6)', 0.8)
  if (props.path?.length > 1) {
    ctx.strokeStyle = '#22c55e'; ctx.lineWidth = 3
    ctx.beginPath()
    props.path.forEach(([x, y], i) => { const [sx, sy] = w2s(x, y); i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy) })
    ctx.stroke()
  }
  if (props.target) {
    const [sx, sy] = w2s(props.target[0], props.target[1])
    ctx.strokeStyle = '#22c55e'; ctx.lineWidth = 3
    ctx.beginPath(); ctx.arc(sx, sy, 9, 0, Math.PI * 2); ctx.stroke()
    ctx.beginPath(); ctx.moveTo(sx - 13, sy); ctx.lineTo(sx + 13, sy); ctx.moveTo(sx, sy - 13); ctx.lineTo(sx, sy + 13); ctx.stroke()
  }
  if (props.pose) arrow(ctx, props.pose[0], props.pose[1], props.pose[2], '#ffffff', 1.1)
  if (drag) {
    const yaw = Math.atan2(drag.cur[1] - drag.start[1], drag.cur[0] - drag.start[0])
    arrow(ctx, drag.start[0], drag.start[1], yaw, '#60a5fa', 1.3)
  }
  // 比例尺
  const barM = niceStep(100), barPx = barM * view.scale
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

function loop() {
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

function fitAll() {
  const g = props.grid
  if (!g || !W) return
  fitted = true
  follow.value = false
  view.vx = g.x0 + g.w / 2
  view.vy = g.y0 + g.h / 2
  view.scale = Math.min(400, Math.max(2, 0.92 * Math.min(W / g.w, H / g.h)))
  needDraw = true
}

// ------------------------------------------------------------------ 交互
const pointers = new Map()
let pinchDist = 0
let downAt = null

function local(e) {
  const r = canvasEl.value.getBoundingClientRect()
  return [e.clientX - r.left, e.clientY - r.top]
}

function zoomAt(f, sx, sy) {
  const s = Math.min(400, Math.max(2, view.scale * f))
  const [wx, wy] = s2w(sx, sy)
  view.scale = s
  if (!follow.value) { view.vx = wx - (sx - W / 2) / s; view.vy = wy + (sy - H / 2) / s }
  needDraw = true
}

function onWheel(e) {
  e.preventDefault()
  const [sx, sy] = local(e)
  zoomAt(Math.exp(-e.deltaY * 0.0015), sx, sy)
}

function onDown(e) {
  canvasEl.value.setPointerCapture(e.pointerId)
  pointers.set(e.pointerId, [e.clientX, e.clientY])
  downAt = { x: e.clientX, y: e.clientY, t: performance.now() }
  if (pointers.size === 2) {
    drag = null
    const [a, b] = [...pointers.values()]
    pinchDist = Math.hypot(a[0] - b[0], a[1] - b[1])
  } else if (props.mode === 'reloc') {
    const w = s2w(...local(e))
    drag = { start: w, cur: w }
    needDraw = true
  }
}

function onMove(e) {
  const prev = pointers.get(e.pointerId)
  if (!prev) return
  pointers.set(e.pointerId, [e.clientX, e.clientY])
  if (pointers.size === 2) {
    const [a, b] = [...pointers.values()]
    const d = Math.hypot(a[0] - b[0], a[1] - b[1])
    const r = canvasEl.value.getBoundingClientRect()
    if (pinchDist) zoomAt(d / pinchDist, (a[0] + b[0]) / 2 - r.left, (a[1] + b[1]) / 2 - r.top)
    pinchDist = d
  } else if (drag) {
    drag.cur = s2w(...local(e))
    needDraw = true
  } else {
    follow.value = false
    view.vx -= (e.clientX - prev[0]) / view.scale
    view.vy += (e.clientY - prev[1]) / view.scale
    needDraw = true
  }
}

function onUp(e) {
  const wasSingle = pointers.size === 1
  pointers.delete(e.pointerId)
  pinchDist = 0
  if (!wasSingle) return
  if (drag) {
    const d = drag
    drag = null
    needDraw = true
    const len = Math.hypot(d.cur[0] - d.start[0], d.cur[1] - d.start[1])
    if (len * view.scale < 15) return     // 没拖出朝向：不算
    emit('pick', { x: d.start[0], y: d.start[1], yaw: Math.atan2(d.cur[1] - d.start[1], d.cur[0] - d.start[0]) })
    return
  }
  // 点一下（几乎没移动）选目标
  if (props.mode === 'goal' && downAt && Math.hypot(e.clientX - downAt.x, e.clientY - downAt.y) < 6) {
    const [x, y] = s2w(...local(e))
    emit('pick', { x, y })
  }
}

let ro = null
onMounted(() => {
  ro = new ResizeObserver(() => { resize(); if (!fitted) fitAll() })
  ro.observe(wrapEl.value)
  resize()
  raf = requestAnimationFrame(loop)
})
onUnmounted(() => { cancelAnimationFrame(raf); ro?.disconnect() })
defineExpose({ fitAll })
</script>

<template>
  <div class="card navmap">
    <div class="bar">
      <span class="title">导航地图</span>
      <span class="muted legend">白 可通行 · 红 障碍 · 白箭头 机器人 · 绿 目标与路线</span>
      <span class="grow" />
      <label><input v-model="follow" type="checkbox" /> 跟随机器人</label>
      <button @click="fitAll">全图</button>
    </div>
    <div ref="wrapEl" class="canvas-wrap" :class="mode">
      <canvas ref="canvasEl" @wheel="onWheel" @pointerdown="onDown" @pointermove="onMove" @pointerup="onUp"
              @pointercancel="onUp" />
      <div v-if="!gridUrl" class="hint">选一张导航地图</div>
      <div v-if="mode === 'goal'" class="tip">点一下地图上的白色区域，机器人会开过去</div>
      <div v-if="mode === 'reloc'" class="tip">在机器人实际所在处按下，朝车头方向拖动后松开</div>
    </div>
  </div>
</template>

<style scoped>
.navmap { display: flex; flex-direction: column; height: 100%; padding: 12px; gap: 10px; }
.bar { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; font-size: 13px; }
.title { font-weight: 600; }
.legend { font-size: 12px; }
.grow { flex: 1; }
label { display: flex; align-items: center; gap: 4px; cursor: pointer; user-select: none; }
button { border: 1px solid var(--line); background: transparent; color: var(--text); border-radius: 6px; padding: 3px 10px; cursor: pointer; font-size: 13px; }
.canvas-wrap { position: relative; flex: 1; min-height: 320px; border-radius: 8px; overflow: hidden; background: var(--map-bg); }
.canvas-wrap.goal canvas, .canvas-wrap.reloc canvas { cursor: crosshair; }
canvas { position: absolute; inset: 0; width: 100%; height: 100%; touch-action: none; cursor: grab; }
.hint { position: absolute; inset: 0; display: grid; place-items: center; color: rgba(255, 255, 255, 0.5); pointer-events: none; }
.tip {
  position: absolute; left: 50%; top: 10px; transform: translateX(-50%); pointer-events: none;
  background: rgba(37, 99, 235, 0.9); color: #fff; font-size: 13px; padding: 4px 12px; border-radius: 6px; white-space: nowrap;
}
</style>
