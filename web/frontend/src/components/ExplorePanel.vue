<script setup>
// 自主探索状态：阶段、说明、已到达的边界数、避障限速原因
defineProps({
  explore: { type: Object, default: null },
  active: Boolean,
})

const STATE = {
  starting: '启动中', spin: '原地环视', explore: '探索中', home: '返回起点', done: '已完成', error: '已停止（异常）',
}
const mmss = s => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`
</script>

<template>
  <div class="card exp">
    <div class="title">
      自主探索
      <span v-if="explore" class="tag" :class="explore.state">{{ STATE[explore.state] || explore.state }}</span>
      <span v-else class="muted">{{ active ? '连接中…' : '' }}</span>
    </div>
    <template v-if="explore">
      <div class="msg">{{ explore.msg }}</div>
      <div v-if="explore.teleop_msg" class="err">{{ explore.teleop_msg }}</div>
      <div v-else-if="explore.limit_reason" class="warn">避障：{{ explore.limit_reason }}</div>
      <div class="meta muted num">
        用时 {{ mmss(explore.elapsed_s) }} / {{ explore.time_limit_min }} 分 · 已到达边界 {{ explore.goals_done }} 处 ·
        剩余 {{ explore.frontiers.length }} 处 · 拉黑 {{ explore.blacklist.length }} 处 · 限速 {{ explore.v_max }} m/s
      </div>
    </template>
  </div>
</template>

<style scoped>
.exp { padding: 12px; }
.title { font-weight: 600; display: flex; gap: 8px; align-items: center; margin-bottom: 6px; }
.tag { font-size: 12px; font-weight: 500; padding: 1px 8px; border-radius: 10px; background: #2563eb; color: #fff; }
.tag.done { background: var(--go); }
.tag.error { background: var(--stop); }
.tag.home { background: var(--warn); }
.msg { font-size: 13px; }
.err { color: var(--stop); font-size: 12px; margin-top: 4px; }
.warn { color: var(--warn); font-size: 12px; margin-top: 4px; }
.meta { font-size: 12px; margin-top: 6px; }
</style>
