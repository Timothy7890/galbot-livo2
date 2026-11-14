<script setup>
// 录制记录；每条完成的录制可以在这里生成 / 重新生成导航地图（进度在 nav_progress，由 7010 状态带出来）
import { ref } from 'vue'
import { navAction } from '../api.js'

defineProps({
  sessions: { type: Array, required: true },
  dataRoot: { type: String, default: '' },
})

const STATUS = { done: '完成', failed: '失败', recording: '进行中' }
const hms = s => [s / 3600, (s / 60) % 60, s % 60].map(v => String(Math.floor(v)).padStart(2, '0')).join(':')
const toTime = n => n.replace(/^(\d{4})(\d\d)(\d\d)_(\d\d)(\d\d)(\d\d)$/, '$2-$3 $4:$5')
const points = n => (n > 1e6 ? `${(n / 1e6).toFixed(1)} M` : n > 1e3 ? `${Math.round(n / 1e3)} K` : n)
const msg = ref('')
const msgOk = ref(true)

async function build(s) {
  if (s.nav && !confirm(`重新生成导航地图「${s.nav.name}」？\n\n原地图改名留底，不删除；生成完成前照常可用。`)) return
  const name = prompt(s.nav ? '地图名称：' : `用 ${toTime(s.name)} 的录制生成导航地图。\n地图名称：`, s.nav?.name || '')
  if (name === null) return
  const r = await navAction('build', { session: s.name, name: name.trim() || s.name })
  msg.value = r.msg || (r.ok ? '开始生成' : '生成失败')
  msgOk.value = !!r.ok
}
</script>

<template>
  <div class="card">
    <div class="title">录制记录</div>
    <div class="muted root">{{ dataRoot }}</div>
    <table>
      <thead>
        <tr><th>开始</th><th>时长</th><th>原始包</th><th>点数</th><th /><th>导航地图</th></tr>
      </thead>
      <tbody>
        <tr v-for="s in sessions" :key="s.name">
          <td class="num" :title="s.name">{{ toTime(s.name) }}</td>
          <td class="num">{{ s.duration_s ? hms(Number(s.duration_s)) : '-' }}</td>
          <td class="num">{{ s.bag_size || '-' }}</td>
          <td class="num">{{ Number(s.map_points) ? points(Number(s.map_points)) : '-' }}</td>
          <td><span class="tag" :class="s.status" :title="s.error || ''">{{ STATUS[s.status] || s.status }}</span></td>
          <td><div class="nav-cell">
            <span v-if="s.nav" class="tag nav" :title="`生成于 ${s.nav.created}\n建图路程 ${s.nav.path_length_m} m`">{{ s.nav.name }}</span>
            <span v-if="s.nav_progress?.state === 'running'" class="tag running" :title="s.nav_progress.msg">
              {{ s.nav_progress.msg || '生成中…' }}</span>
            <template v-else-if="s.status === 'done'">
              <span v-if="s.nav_progress?.state === 'failed'" class="tag failed" :title="s.nav_progress.msg">失败</span>
              <button class="link" @click="build(s)">
                {{ s.nav ? '重新生成' : s.nav_progress?.state === 'failed' ? '重试' : '生成' }}
              </button>
            </template>
          </div></td>
        </tr>
        <tr v-if="!sessions.length"><td colspan="6" class="muted">暂无记录</td></tr>
      </tbody>
    </table>
    <div v-if="msg" class="msg" :class="msgOk ? 'muted' : 'bad'">{{ msg }}</div>
  </div>
</template>

<style scoped>
.title { font-weight: 600; }
.root { font-size: 12px; margin-bottom: 6px; word-break: break-all; }
table { width: 100%; border-collapse: collapse; font-size: 12px; }
th, td { text-align: left; padding: 6px 4px; border-bottom: 1px solid var(--line); }
th { color: var(--muted); font-weight: 500; }
.tag { padding: 1px 6px; border-radius: 4px; background: var(--line); white-space: nowrap; }
.tag.failed { background: var(--stop); color: #fff; }
.tag.nav { background: var(--go); color: #fff; }
.tag.running { background: var(--warn); color: #fff; max-width: 150px; overflow: hidden; text-overflow: ellipsis; }
.nav-cell { display: flex; flex-wrap: wrap; gap: 4px; align-items: center; }
.link {
  border: 1px solid var(--line); background: transparent; color: var(--text); border-radius: 4px;
  padding: 1px 6px; font-size: 12px; cursor: pointer; white-space: nowrap;
}
.msg { font-size: 12px; margin-top: 8px; }
.bad { color: var(--stop); }
</style>
