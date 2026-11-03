<script setup>
// 自主建图状态：阶段、说明、用时与进度；完成后显示停止录制倒计时与取消按钮
defineProps({ explore: { type: Object, default: null } })
defineEmits(['cancel-stop'])

const STATE = {
  idle: '未进入', spin: '原地环视', explore: '探索中', home: '返回起点', countdown: '已完成', error: '已退出（异常）',
}
const mmss = s => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`
</script>

<template>
  <div class="card exp">
    <div class="title">
      自主建图
      <span v-if="explore" class="tag" :class="explore.state">{{ STATE[explore.state] || explore.state }}</span>
      <span v-if="explore?.dry_run" class="tag dry">只算不动</span>
      <span v-if="!explore" class="muted">连接中…</span>
    </div>
    <template v-if="explore">
      <div class="msg">{{ explore.msg }}</div>
      <div v-if="explore.teleop_msg" class="err">{{ explore.teleop_msg }}</div>
      <div v-else-if="explore.limit_reason" class="warn">避障：{{ explore.limit_reason }}</div>
      <button v-if="explore.stop_in != null" class="cancel" @click="$emit('cancel-stop')">
        取消自动停止（{{ mmss(explore.stop_in) }} 后停止录制）
      </button>
      <div class="meta muted num">
        <template v-if="explore.enabled">用时 {{ mmss(explore.elapsed_s) }} / {{ explore.time_limit_min }} 分 · </template>
        已到达边界 {{ explore.goals_done }} 处 · 剩余 {{ explore.frontiers.length }} 处
        <template v-if="explore.blacklist.length"> · 拉黑 {{ explore.blacklist.length }} 处</template>
        · 限速 {{ explore.v_max }} m/s
      </div>
    </template>
  </div>
</template>

<style scoped>
.exp { padding: 12px; }
.title { font-weight: 600; display: flex; gap: 8px; align-items: center; margin-bottom: 6px; }
.tag { font-size: 12px; font-weight: 500; padding: 1px 8px; border-radius: 10px; background: #2563eb; color: #fff; }
.tag.idle { background: var(--line); color: var(--text); }
.tag.countdown { background: var(--go); }
.tag.error { background: var(--stop); }
.tag.home { background: var(--warn); }
.tag.dry { background: #7c3aed; }
.msg { font-size: 13px; }
.err { color: var(--stop); font-size: 12px; margin-top: 4px; }
.warn { color: var(--warn); font-size: 12px; margin-top: 4px; }
.cancel {
  width: 100%; margin-top: 10px; border: 0; border-radius: 10px; padding: 12px; font-size: 15px; font-weight: 600;
  color: #fff; background: var(--warn); cursor: pointer;
}
.meta { font-size: 12px; margin-top: 6px; }
</style>
