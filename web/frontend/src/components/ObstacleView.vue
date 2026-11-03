<script setup>
// 避障俯视图：机器人坐标系（车头朝上），Mid-360 障碍点、车身安全轮廓、原地转向扫过的圆、8 个超声波读数
import { onMounted, onUnmounted, ref } from 'vue'

const props = defineProps({
  active: Boolean,
  command: { type: Array, default: () => [0, 0, 0] },      // 网页发出的 [vx, vy, wz]
})

const RANGE = 2.5           // 显示半径 (m)
const US_STOP = 0.3, US_SLOW = 0.5
const BASE = 0.3            // 底盘半边长 (m)，超声波装在底盘边上
// 超声波在底盘边上的位置（按"面朝外时左/右"推定，待逐个用手遮挡确认）：[哪条边, 沿边位置 -1..1，+ 为 x 前 / y 左]
const US_POS = {
  FRONT_LEFT: ['front', 0.5], FRONT_RIGHT: ['front', -0.5],
  LEFT_LEFT: ['left', -0.5], LEFT_RIGHT: ['left', 0.5],
  BACK_LEFT: ['back', -0.5], BACK_RIGHT: ['back', 0.5],
  RIGHT_LEFT: ['right', 0.5], RIGHT_RIGHT: ['right', -0.5],
}

const canvasEl = ref(null)
const scan = ref(null)
let timer = null

async function poll() {
  if (props.active) {
    try {
      scan.value = await (await fetch('api/scan')).json()
      draw()
    } catch { /* 下次重试 */ }
  }
  timer = setTimeout(poll, 200)
}

function usColor(d) {
  if (d == null) return 'rgba(150,150,150,0.35)'
  if (d < US_STOP) return 'rgba(220,38,38,0.75)'
  if (d < US_SLOW) return 'rgba(217,119,6,0.65)'
  return 'rgba(22,163,74,0.35)'
}

function draw() {
  const c = canvasEl.value
  const s = scan.value
  if (!c || !s) return
  const dpr = window.devicePixelRatio || 1
  const W = c.clientWidth, H = c.clientHeight
  c.width = W * dpr; c.height = H * dpr
  const ctx = c.getContext('2d')
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  ctx.fillStyle = '#0e1013'
  ctx.fillRect(0, 0, W, H)
  const k = Math.min(W, H) / 2 / RANGE
  // 机器人坐标 (x 前, y 左) -> 屏幕：车头朝上
  const P = (x, y) => [W / 2 - y * k, H / 2 - x * k]

  // 1 m 刻度圈
  ctx.strokeStyle = 'rgba(255,255,255,0.08)'
  for (let r = 1; r <= RANGE; r++) {
    ctx.beginPath(); ctx.arc(W / 2, H / 2, r * k, 0, Math.PI * 2); ctx.stroke()
  }

  const fp = s.footprint
  // 超声波：从车身边上的探头位置画一个 ±30° 的扇形，半径为读数
  for (const [name, d] of Object.entries(s.us || {})) {
    const [side, t] = US_POS[name]
    const [ex, ey] = { front: [BASE, BASE * t], back: [-BASE, BASE * t],
      left: [BASE * t, BASE], right: [BASE * t, -BASE] }[side]
    const a = (s.us_dir[name] * Math.PI) / 180
    const r = Math.min(d ?? RANGE, RANGE)
    const [sx, sy] = P(ex, ey)
    // 屏幕上车头朝上：机器人朝向角 a 对应屏幕角度 -(a + 90°)
    const sa = -(a + Math.PI / 2)
    ctx.fillStyle = usColor(d)
    ctx.beginPath(); ctx.moveTo(sx, sy)
    ctx.arc(sx, sy, r * k, sa - Math.PI / 6, sa + Math.PI / 6); ctx.closePath(); ctx.fill()
    ctx.fillStyle = '#fff'
    ctx.font = '11px sans-serif'
    const [lx, ly] = [sx + Math.cos(sa) * 26, sy + Math.sin(sa) * 26]
    ctx.fillText(d == null ? '—' : d.toFixed(2), lx - 12, ly + 4)
  }

  // Mid-360 障碍点
  ctx.fillStyle = '#ff5a5a'
  for (const [x, y] of s.pts || []) {
    const [sx, sy] = P(x, y)
    ctx.fillRect(sx - 1.5, sy - 1.5, 3, 3)
  }

  // 原地转向扫过的圆、安全轮廓
  ctx.setLineDash([4, 4]); ctx.strokeStyle = 'rgba(255,255,255,0.35)'
  ctx.beginPath(); ctx.arc(W / 2, H / 2, s.sweep_r * k, 0, Math.PI * 2); ctx.stroke()
  ctx.setLineDash([])
  const [x0, y0] = P(fp.front, fp.left), [x1, y1] = P(fp.back, fp.right)
  ctx.strokeStyle = '#fff'; ctx.lineWidth = 2
  ctx.strokeRect(x0, y0, x1 - x0, y1 - y0)
  ctx.lineWidth = 1

  // 指令方向（白）与限速后（绿）
  const [vx, vy] = props.command
  const v = Math.hypot(vx, vy)
  if (v > 1e-3) {
    const scale = s.limit?.lin_scale ?? 1
    for (const [len, color] of [[1, 'rgba(255,255,255,0.6)'], [scale, '#22c55e']]) {
      const [ex, ey] = P((vx / v) * 0.9 * len, (vy / v) * 0.9 * len)
      ctx.strokeStyle = color; ctx.lineWidth = 3
      ctx.beginPath(); ctx.moveTo(W / 2, H / 2); ctx.lineTo(ex, ey); ctx.stroke()
    }
    ctx.lineWidth = 1
  }
  // 车头标记
  ctx.fillStyle = '#fff'
  const [hx, hy] = P(fp.front, 0)
  ctx.beginPath(); ctx.moveTo(hx, hy - 8); ctx.lineTo(hx - 6, hy + 2); ctx.lineTo(hx + 6, hy + 2); ctx.fill()
}

onMounted(poll)
onUnmounted(() => clearTimeout(timer))
defineExpose({ scan })
</script>

<template>
  <div class="card obst">
    <div class="title">
      避障视图
      <span class="muted hint">车头朝上 · 红点 Mid-360 障碍 · 扇形超声波 · 圈间距 1 m</span>
    </div>
    <div class="wrap"><canvas ref="canvasEl" /></div>
    <div v-if="scan" class="meta muted">
      <span v-if="scan.age != null && scan.age > 0.5" class="err">传感器数据中断 {{ scan.age }} s</span>
      车身轮廓 前 {{ scan.footprint.front }} 后 {{ scan.footprint.back }} 左 {{ scan.footprint.left }}
      右 {{ scan.footprint.right }} m
      · {{ scan.calibrated ? `已标定（${scan.calibrated}，${scan.self_cells} 格）` : '未标定车身' }}
    </div>
    <table v-if="scan" class="us num">
      <tr>
        <td v-for="(d, name) in scan.us" :key="name" :class="{ near: d != null && d < US_SLOW }">
          <small>{{ scan.us_label[name] }}</small><br />{{ d == null ? '—' : d.toFixed(2) }}
        </td>
      </tr>
    </table>
  </div>
</template>

<style scoped>
.obst { padding: 12px; }
.title { font-weight: 600; margin-bottom: 8px; display: flex; gap: 8px; align-items: baseline; flex-wrap: wrap; }
.hint { font-size: 12px; font-weight: 400; }
.wrap { aspect-ratio: 1; border-radius: 8px; overflow: hidden; background: var(--map-bg); }
canvas { width: 100%; height: 100%; display: block; }
.meta { font-size: 12px; margin-top: 6px; }
.err { color: var(--stop); margin-right: 6px; }
.us { width: 100%; border-collapse: collapse; margin-top: 6px; font-size: 12px; text-align: center; }
.us td { border: 1px solid var(--line); padding: 3px 2px; }
.us td.near { background: var(--warn); color: #fff; }
.us small { color: var(--muted); }
.us td.near small { color: #fff; }
</style>
