<script setup>
// 导航地图：成熟地图的 2D 栅格（运行中为实时更新的栅格）+ 建图轨迹 + 实时障碍点 + 机器人（车身圆 + 车头箭头）+ 目标与路线。
// mode = 'goal'：点一下选目标；按住拖动再松开 = 同时指定到点后的车头朝向
// mode = 'reloc'：在机器人实际位置按下，朝车头方向拖动后松开
// 其余时候单指拖动平移；任何时候滚轮 / 双指都可缩放和平移
import { onMounted, onUnmounted, ref, watch } from 'vue'

const props = defineProps({
  gridUrl: { type: String, default: '' },     // 栅格图片地址（变化即重新加载）
  grid: { type: Object, default: null },      // { x0, y0, w, h } 世界范围
  state: { type: Object, default: null },     // navigator 推送的状态（pose / path / target / scan / loc ...）
  traj: { type: Array, default: () => [] },   // 建图轨迹 [[x, y], ...]
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
let userMoved = false   // 人拖过 / 缩放过视图之后，窗口大小变化不再自动回到全图
let drag = null       // 点选中：{ x, y, sx, sy, yaw（拖出方向后才有） }
const DRAG_PX = 10    // 拖过这么多像素才算在指定朝向

watch(() => props.gridUrl, url => {
  if (!url) { img = null; needDraw = true; return }
  const im = new Image()
  im.onload = () => { img = im; needDraw = true; if (!fitted) fitAll() }
  im.src = url
}, { immediate: true })
watch(() => [props.grid, props.state, props.traj, props.mode, follow.value], () => (needDraw = true))
watch(follow, v => { if (v) userMoved = true })
watch(() => props.mode, () => { drag = null })

function w2s(x, y) { return [W / 2 + (x - view.vx) * view.scale, H / 2 - (y - view.vy) * view.scale] }
function s2w(sx, sy) { return [view.vx + (sx - W / 2) / view.scale, view.vy - (sy - H / 2) / view.scale] }

let scanSrc = '', scanPts = new Int16Array(0)
function decodeScan(b64) {
  if (b64 === scanSrc) return scanPts
  scanSrc = b64
  if (!b64) return (scanPts = new Int16Array(0))
  const bin = atob(b64)
  const bytes = new Uint8Array(bin.length)
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i)
  return (scanPts = new Int16Array(bytes.buffer))
}

function arrow(ctx, x, y, yaw, fill, size = 1, stroke = '#000') {
  const [sx, sy] = w2s(x, y)
  ctx.save()
  ctx.translate(sx, sy)
  ctx.rotate(-yaw)
  ctx.scale(size, size)
  ctx.fillStyle = fill
  ctx.strokeStyle = stroke
  ctx.lineWidth = 1.5 / size
  ctx.beginPath()
  ctx.moveTo(14, 0); ctx.lineTo(-9, 8); ctx.lineTo(-4, 0); ctx.lineTo(-9, -8); ctx.closePath()
  ctx.fill(); ctx.stroke()
  ctx.restore()
}

function polyline(ctx, pts, color, width, dash = []) {
  if (!pts || pts.length < 2) return
  ctx.strokeStyle = color; ctx.lineWidth = width
  ctx.setLineDash(dash)
  ctx.beginPath()
  pts.forEach(([x, y], i) => { const [sx, sy] = w2s(x, y); i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy) })
  ctx.stroke()
  ctx.setLineDash([])
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
  const st = props.state || {}
  const pose = st.pose
  if (follow.value && pose) { view.vx = pose[0]; view.vy = pose[1] }
  const g = props.grid
  if (img && g) {
    ctx.imageSmoothingEnabled = false
    const [gx, gy] = w2s(g.x0, g.y0 + g.h)
    ctx.drawImage(img, gx, gy, g.w * view.scale, g.h * view.scale)
  }
  polyline(ctx, props.traj, 'rgba(147,170,255,0.55)', 2, [6, 5])

  // 实时障碍点：机器人此刻看到的东西。和红色栅格对得上 = 定位是对的
  const scan = decodeScan(st.scan || '')
  if (scan.length) {
    const u = st.scan_unit || 0.02
    const sz = Math.max(2, Math.min(4, view.scale * 0.05))
    ctx.fillStyle = '#22d3ee'
    for (let i = 0; i < scan.length; i += 2) {
      const [sx, sy] = w2s(scan[i] * u, scan[i + 1] * u)
      ctx.fillRect(sx - sz / 2, sy - sz / 2, sz, sz)
    }
  }

  if (st.loc?.state !== 'localized') {
    for (const c of st.loc?.candidates || []) arrow(ctx, c.x, c.y, c.yaw, 'rgba(250,204,21,0.6)', 0.8)
  }
  if (st.enabled) {
    polyline(ctx, st.path, '#22c55e', 3)
    const t = st.target
    if (t) {
      const [sx, sy] = w2s(t[0], t[1])
      ctx.strokeStyle = '#22c55e'; ctx.lineWidth = 3
      ctx.beginPath(); ctx.arc(sx, sy, 9, 0, Math.PI * 2); ctx.stroke()
      if (t.length > 2) arrow(ctx, t[0], t[1], t[2], '#22c55e', 1.0)
      else { ctx.beginPath(); ctx.moveTo(sx - 13, sy); ctx.lineTo(sx + 13, sy); ctx.moveTo(sx, sy - 13); ctx.lineTo(sx, sy + 13); ctx.stroke() }
    }
  }
  if (pose) {
    const [sx, sy] = w2s(pose[0], pose[1])
    const r = (st.robot_r || 0.31) * view.scale
    ctx.fillStyle = 'rgba(59,130,246,0.28)'; ctx.strokeStyle = '#60a5fa'; ctx.lineWidth = 1.5
    ctx.beginPath(); ctx.arc(sx, sy, r, 0, Math.PI * 2); ctx.fill(); ctx.stroke()
    arrow(ctx, pose[0], pose[1], pose[2], '#ffffff', Math.max(0.8, Math.min(1.3, r / 14)))
  }
  if (drag) {
    const color = props.mode === 'goal' ? '#22c55e' : '#a78bfa'
    if (drag.yaw != null) arrow(ctx, drag.x, drag.y, drag.yaw, color, 1.4, '#fff')
    else {
      const [sx, sy] = w2s(drag.x, drag.y)
      ctx.strokeStyle = color; ctx.lineWidth = 3
      ctx.beginPath(); ctx.arc(sx, sy, 9, 0, Math.PI * 2); ctx.stroke()
    }
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
  if (needDraw && canvasEl.value && W) { draw(); needDraw = false }
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
  userMoved = false
  follow.value = false
  view.vx = g.x0 + g.w / 2
  view.vy = g.y0 + g.h / 2
  view.scale = Math.min(400, Math.max(2, 0.92 * Math.min(W / g.w, H / g.h)))
  needDraw = true
}
// 换了一张地图：重新适配
watch(() => props.grid && [props.grid.x0, props.grid.y0, props.grid.w, props.grid.h].join(), (a, b) => {
  if (a && (!fitted || (b && a !== b && !props.state))) fitAll()
})

// ------------------------------------------------------------------ 交互
const pointers = new Map()
let pinch = null      // 双指：{ dist, mx, my }

function local(e) {
  const r = canvasEl.value.getBoundingClientRect()
  return [e.clientX - r.left, e.clientY - r.top]
}

function zoomAt(f, sx, sy) {
  userMoved = true
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

function pinchInfo() {
  const [a, b] = [...pointers.values()]
  const r = canvasEl.value.getBoundingClientRect()
  return { dist: Math.hypot(a[0] - b[0], a[1] - b[1]), mx: (a[0] + b[0]) / 2 - r.left, my: (a[1] + b[1]) / 2 - r.top }
}

function onDown(e) {
  canvasEl.value.setPointerCapture(e.pointerId)
  pointers.set(e.pointerId, [e.clientX, e.clientY])
  if (pointers.size === 2) {
    drag = null             // 第二根手指落下：这次是缩放 / 平移，不是点选
    pinch = pinchInfo()
  } else if (props.mode === 'goal' || props.mode === 'reloc') {
    const [sx, sy] = local(e)
    const [x, y] = s2w(sx, sy)
    drag = { x, y, sx, sy, yaw: null }
  }
  needDraw = true
}

function onMove(e) {
  const prev = pointers.get(e.pointerId)
  if (!prev) return
  pointers.set(e.pointerId, [e.clientX, e.clientY])
  if (pointers.size === 2) {
    const p = pinchInfo()
    if (pinch) {
      if (!follow.value) { view.vx -= (p.mx - pinch.mx) / view.scale; view.vy += (p.my - pinch.my) / view.scale }
      zoomAt(p.dist / (pinch.dist || p.dist), p.mx, p.my)
    }
    pinch = p
  } else if (drag) {
    const [sx, sy] = local(e)
    drag.yaw = Math.hypot(sx - drag.sx, sy - drag.sy) > DRAG_PX ? Math.atan2(-(sy - drag.sy), sx - drag.sx) : null
  } else {
    follow.value = false
    userMoved = true
    view.vx -= (e.clientX - prev[0]) / view.scale
    view.vy += (e.clientY - prev[1]) / view.scale
  }
  needDraw = true
}

function onUp(e) {
  const wasSingle = pointers.size === 1
  pointers.delete(e.pointerId)
  pinch = null
  needDraw = true
  if (!wasSingle || !drag) { if (!pointers.size) drag = null; return }
  const d = drag
  drag = null
  if (e.type === 'pointercancel') return
  if (props.mode === 'reloc') {
    if (d.yaw == null) return       // 没拖出朝向：不算（手动定位必须给车头方向）
    emit('pick', { x: d.x, y: d.y, yaw: d.yaw })
  } else if (props.mode === 'goal') {
    emit('pick', d.yaw == null ? { x: d.x, y: d.y } : { x: d.x, y: d.y, yaw: d.yaw })
  }
}

let ro = null
onMounted(() => {
  ro = new ResizeObserver(() => { resize(); if (!fitted || !userMoved) fitAll() })
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
      <span class="grow" />
      <label><input v-model="follow" type="checkbox" /> 跟随机器人</label>
      <button @click="fitAll">全图</button>
    </div>
    <div ref="wrapEl" class="canvas-wrap" :class="mode">
      <canvas ref="canvasEl" @wheel="onWheel" @pointerdown="onDown" @pointermove="onMove" @pointerup="onUp"
              @pointercancel="onUp" />
      <div v-if="!gridUrl" class="hint">选一张导航地图</div>
      <div v-if="mode === 'goal'" class="tip goal">点一下白色区域选目标；按住拖动可指定到点后的车头朝向 · Esc 取消</div>
      <div v-if="mode === 'reloc'" class="tip reloc">在机器人实际所在处按下，朝车头方向拖动后松开 · Esc 取消</div>
      <div class="legend">
        <span><i class="sw" style="background:rgba(255,255,255,0.55)" />可通行</span>
        <span><i class="sw" style="background:#ff5050" />障碍</span>
        <span><i class="sw" style="background:#22d3ee" />实时障碍点</span>
        <span><i class="sw" style="background:#60a5fa" />机器人</span>
        <span><i class="sw" style="background:#22c55e" />目标与路线</span>
        <span><i class="sw dash" />建图轨迹</span>
      </div>
    </div>
  </div>
</template>

<style scoped>
.navmap { display: flex; flex-direction: column; height: 100%; padding: 12px; gap: 10px; }
.bar { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; font-size: 13px; }
.title { font-weight: 600; }
.grow { flex: 1; }
label { display: flex; align-items: center; gap: 4px; cursor: pointer; user-select: none; }
button { border: 1px solid var(--line); background: transparent; color: var(--text); border-radius: 6px; padding: 3px 10px; cursor: pointer; font-size: 13px; }
.canvas-wrap { position: relative; flex: 1; min-height: 320px; border-radius: 8px; overflow: hidden; background: var(--map-bg); }
.canvas-wrap.goal { outline: 3px solid #16a34a; }
.canvas-wrap.reloc { outline: 3px solid #7c3aed; }
.canvas-wrap.goal canvas, .canvas-wrap.reloc canvas { cursor: crosshair; }
canvas { position: absolute; inset: 0; width: 100%; height: 100%; touch-action: none; cursor: grab; }
.hint { position: absolute; inset: 0; display: grid; place-items: center; color: rgba(255, 255, 255, 0.5); pointer-events: none; }
.tip {
  position: absolute; left: 50%; top: 10px; transform: translateX(-50%); pointer-events: none; max-width: calc(100% - 20px);
  color: #fff; font-size: 13px; padding: 4px 12px; border-radius: 6px; text-align: center;
}
.tip.goal { background: rgba(22, 163, 74, 0.92); }
.tip.reloc { background: rgba(124, 58, 237, 0.92); }
.legend {
  position: absolute; right: 10px; bottom: 10px; display: flex; gap: 10px; flex-wrap: wrap; justify-content: flex-end;
  max-width: calc(100% - 210px); pointer-events: none;
  font-size: 11px; color: #e5e7eb; background: rgba(0, 0, 0, 0.55); padding: 3px 8px; border-radius: 6px;
}
.sw { display: inline-block; width: 10px; height: 10px; border-radius: 2px; margin-right: 4px; vertical-align: -1px; }
.sw.dash { background: repeating-linear-gradient(90deg, #93aaff 0 4px, transparent 4px 7px); }
</style>
