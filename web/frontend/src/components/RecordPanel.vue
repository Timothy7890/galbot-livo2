<script setup>
import { computed, onUnmounted, ref, watch } from 'vue'
import StartProgress from './StartProgress.vue'

const props = defineProps({
  status: { type: Object, required: true },
  pending: { type: String, default: '' },
  actionError: { type: String, default: '' },
  explore: { type: Object, default: null },     // 录制中 explorer.py 的状态
  startFail: { type: Object, default: null },   // 上次启动失败停在哪一步 { step, elapsed, msg }
})
defineEmits(['start', 'toggle-explore', 'stop'])
const strategy = defineModel('strategy', { type: String, default: 'region' })
// 选目标策略，与 scripts/explore_grid.py 的 STRATEGIES 对应
const STRATEGIES = [
  ['region', '分区清扫', '采完一块（约 10 m × 10 m）再去下一块，来回最少，推荐采集用'],
  ['tour', '顺序清扫', '把所有缺口排成一条最短路线依次去，少来回'],
  ['nearest', '就近清扫', '总去最近的缺口、少掉头'],
  ['balanced', '均衡', '原策略：近且大的边界优先，前期覆盖最快，但常在两片区域间来回'],
]

const LABEL = {
  connecting: '连接中…',
  offline: '无法连接到机器人上的服务',
  idle: '空闲',
  starting: '正在启动…',
  recording: '录制中',
  stopping: '正在停止并保存…',
  error: '异常：录制进程已退出',
}

const state = computed(() => props.pending || props.status.status || 'idle')
const canStart = computed(() => state.value === 'idle')
const canStop = computed(() => ['recording', 'error'].includes(state.value))

// 本地计时，每 2 s 用服务端 elapsed_s 校正
const elapsed = ref(0)
let startedAt = 0
let tick = null
watch(
  () => [props.status.status, props.status.elapsed_s],
  ([st, e]) => {
    clearInterval(tick)
    if (st === 'recording' && e != null) {
      startedAt = Date.now() / 1000 - Number(e)
      const upd = () => (elapsed.value = Date.now() / 1000 - startedAt)
      upd()
      tick = setInterval(upd, 1000)
    } else if (st === 'idle') {
      elapsed.value = 0
    }
  },
  { immediate: true },
)
onUnmounted(() => clearInterval(tick))

const hms = s => [s / 3600, (s / 60) % 60, s % 60].map(v => String(Math.floor(v)).padStart(2, '0')).join(':')
</script>

<template>
  <div class="card panel" :class="state">
    <div class="status"><span class="dot" />{{ LABEL[state] || state }}</div>
    <StartProgress v-if="state === 'starting'" :step="status.step || ''"
                   :step-elapsed="status.step_elapsed_s != null ? Number(status.step_elapsed_s) : null"
                   :elapsed="status.elapsed_s != null ? Number(status.elapsed_s) : null" />
    <StartProgress v-else-if="startFail && state === 'idle'" :step="startFail.step" :elapsed="startFail.elapsed"
                   :step-elapsed="startFail.stepElapsed" failed :failed-msg="startFail.msg" />
    <div v-else class="timer num">{{ hms(elapsed) }}</div>
    <div class="btns">
      <button class="go" :disabled="!canStart" @click="$emit('start')">开始录制</button>
      <button class="stop" :disabled="!canStop" @click="$emit('stop')">停止录制</button>
    </div>
    <label class="strategy">
      探索策略
      <select v-model="strategy" :disabled="explore?.enabled">
        <option v-for="[k, label] in STRATEGIES" :key="k" :value="k">{{ label }}</option>
      </select>
    </label>
    <div class="strategy-hint muted">{{ STRATEGIES.find(s => s[0] === strategy)?.[2] }}</div>
    <button class="explore" :class="{ on: explore?.enabled }" :disabled="state !== 'recording' || !explore"
            @click="$emit('toggle-explore')">
      {{ explore?.enabled ? '退出自主建图' : '进入自主建图（机器人自己走）' }}
    </button>
    <div v-if="status.session" class="meta muted">
      保存到 {{ status.session }}<template v-if="status.size">（{{ status.size }}）</template>
    </div>
    <div v-if="(actionError || status.error) && !startFail" class="err">{{ actionError || status.error }}</div>
  </div>
</template>

<style scoped>
.status { display: flex; align-items: center; gap: 10px; font-size: 17px; font-weight: 600; }
.dot { width: 12px; height: 12px; border-radius: 50%; background: var(--muted); }
.recording .dot { background: var(--stop); animation: blink 1s infinite; }
.starting .dot, .stopping .dot { background: var(--warn); }
.error .dot, .offline .dot { background: var(--stop); }
@keyframes blink { 50% { opacity: 0.3; } }
.timer { font-size: 40px; font-weight: 600; margin: 8px 0 12px; }
.btns { display: flex; gap: 10px; }
button {
  flex: 1; border: 0; border-radius: 10px; padding: 13px; font-size: 16px; font-weight: 600;
  color: #fff; cursor: pointer;
}
button:disabled { opacity: 0.35; cursor: default; }
.go { background: var(--go); }
.stop { background: var(--stop); }
.explore { background: #2563eb; width: 100%; margin-top: 6px; flex: none; }
.strategy { display: flex; align-items: center; gap: 8px; margin-top: 12px; font-size: 13px; }
.strategy select {
  flex: 1; padding: 6px; border-radius: 8px; border: 1px solid var(--line); background: var(--bg); color: var(--text);
}
.strategy-hint { font-size: 12px; margin-top: 4px; }
.explore.on { background: var(--warn); }
.meta { font-size: 12px; margin-top: 10px; word-break: break-all; }
.err { color: var(--stop); font-size: 12px; margin-top: 8px; white-space: pre-wrap; }
</style>
