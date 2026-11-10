<script setup>
// 开始录制的启动进度：record.sh 每进入一步写 state 的 step（"3/6 文字"），status 接口带出来；
// 这里显示全部步骤（完成 / 进行中 / 未开始）、当前步已用时间与总进度。失败时标出停在哪一步。
import { computed, onUnmounted, ref, watch } from 'vue'

const props = defineProps({
  step: { type: String, default: '' },          // status.step
  stepElapsed: { type: Number, default: null }, // status.step_elapsed_s
  elapsed: { type: Number, default: null },     // status.elapsed_s（从点开始算）
  failed: { type: Boolean, default: false },
  failedMsg: { type: String, default: '' },
})

// 与 record.sh cmd_start 的 step 1–6 对应；typ 为大致耗时（s），只用来估计进度条
const STEPS = [
  { title: '读取标定、启动 ROS', hint: '从 Galbot SDK 现读相机外参与关节角', typ: 6 },
  { title: '启动传感器桥接', hint: '连接 Galbot SDK，打开雷达、IMU 与 6 路相机', typ: 7 },
  { title: '等待传感器数据', hint: '雷达、IMU、主相机都出数据', typ: 3 },
  { title: '启动 FAST-LIVO2', hint: 'IMU 初始化，等第一个位姿', typ: 4 },
  { title: '开始录包与实时地图', hint: '原始数据、轨迹、网页实时地图', typ: 1 },
  { title: '启动自主建图程序', hint: '平时只建 2D 栅格，不动底盘', typ: 1 },
]
const TOTAL = STEPS.reduce((a, s) => a + s.typ, 0)

const idx = computed(() => {
  const m = /^(\d+)\//.exec(props.step || '')
  return m ? Number(m[1]) - 1 : 0          // 还没写 step 时（刚点开始）算第 1 步
})

// 本地每 0.2 s 走一下，服务端 0.5 s 一次的数据之间也连续
const now = ref(Date.now())
const tick = setInterval(() => (now.value = Date.now()), 200)
onUnmounted(() => clearInterval(tick))
let base = { t: Date.now(), step: 0, total: 0 }
watch(() => [props.stepElapsed, props.elapsed, idx.value], () => {
  base = { t: Date.now(), step: props.stepElapsed ?? 0, total: props.elapsed ?? 0 }
}, { immediate: true })
const stepSec = computed(() => props.failed ? base.step : base.step + (now.value - base.t) / 1000)
const totalSec = computed(() => props.failed ? base.total : base.total + (now.value - base.t) / 1000)

// 进度：已完成步骤的典型耗时 + 当前步（按典型耗时折算，超时就停在 95%）
const pct = computed(() => {
  const done = STEPS.slice(0, idx.value).reduce((a, s) => a + s.typ, 0)
  const cur = STEPS[idx.value]?.typ ?? 1
  const frac = Math.min(0.95, stepSec.value / cur)
  return Math.min(99, ((done + frac * cur) / TOTAL) * 100)
})
const slow = computed(() => !props.failed && stepSec.value > (STEPS[idx.value]?.typ ?? 5) * 2.5 + 5)
</script>

<template>
  <div class="sp" :class="{ failed }">
    <div class="head">
      <span>{{ failed ? '启动失败' : '正在启动录制' }}</span>
      <span class="num muted">{{ Math.floor(totalSec) }} s<template v-if="!failed"> / 约 {{ TOTAL }} s</template></span>
    </div>
    <div class="bar"><div class="fill" :style="{ width: (failed ? 100 : pct) + '%' }" /></div>
    <ol>
      <li v-for="(s, i) in STEPS" :key="i"
          :class="i < idx ? 'done' : i === idx ? (failed ? 'bad' : 'cur') : 'todo'">
        <span class="ico">
          <svg v-if="i < idx" viewBox="0 0 16 16"><path d="M3.5 8.5l3 3 6-7" /></svg>
          <svg v-else-if="i === idx && failed" viewBox="0 0 16 16"><path d="M4.5 4.5l7 7M11.5 4.5l-7 7" /></svg>
          <span v-else-if="i === idx" class="spin" />
          <span v-else class="n">{{ i + 1 }}</span>
        </span>
        <span class="txt">
          <span class="t">{{ s.title }}</span>
          <span v-if="i === idx" class="h">
            <template v-if="failed">{{ failedMsg || '失败' }}</template>
            <template v-else>{{ s.hint }} · <span class="num">{{ stepSec.toFixed(0) }} s</span></template>
          </span>
        </span>
      </li>
    </ol>
    <div v-if="slow" class="slow">这一步比平时慢，可能 SDK 或传感器还没就绪；超过 30 s 会自动失败并提示原因</div>
  </div>
</template>

<style scoped>
.sp { margin: 10px 0 12px; display: flex; flex-direction: column; gap: 8px; }
.head { display: flex; justify-content: space-between; font-size: 13px; font-weight: 600; }
.bar { height: 6px; border-radius: 3px; background: var(--line); overflow: hidden; }
.fill {
  height: 100%; border-radius: 3px; transition: width 0.4s ease;
  background: linear-gradient(90deg, #2563eb, #22c55e);
  background-size: 200% 100%; animation: flow 2s linear infinite;
}
.failed .fill { background: var(--stop); animation: none; }
@keyframes flow { to { background-position: -200% 0; } }
ol { list-style: none; margin: 2px 0 0; padding: 0; display: flex; flex-direction: column; gap: 2px; }
li { display: flex; gap: 10px; align-items: flex-start; padding: 5px 6px; border-radius: 8px; font-size: 13px; }
li.cur { background: color-mix(in srgb, #2563eb 10%, transparent); }
li.bad { background: color-mix(in srgb, var(--stop) 12%, transparent); }
li.todo { color: var(--muted); }
.ico { width: 20px; height: 20px; flex: none; display: grid; place-items: center; border-radius: 50%; }
.done .ico { background: var(--go); }
.bad .ico { background: var(--stop); }
.todo .ico { border: 1.5px solid var(--line); }
.ico svg { width: 14px; height: 14px; fill: none; stroke: #fff; stroke-width: 2.2; stroke-linecap: round; stroke-linejoin: round; }
.n { font-size: 11px; }
.spin {
  width: 16px; height: 16px; border-radius: 50%; border: 2.5px solid color-mix(in srgb, #2563eb 25%, transparent);
  border-top-color: #2563eb; animation: rot 0.8s linear infinite;
}
@keyframes rot { to { transform: rotate(360deg); } }
.txt { display: flex; flex-direction: column; min-width: 0; padding-top: 1px; }
.t { font-weight: 500; }
.cur .t { font-weight: 600; }
.h { font-size: 12px; color: var(--muted); }
.bad .h { color: var(--stop); white-space: pre-wrap; overflow-wrap: anywhere; }
.slow { font-size: 12px; color: var(--warn); }
</style>
