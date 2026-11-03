<script setup>
defineProps({
  sessions: { type: Array, required: true },
  dataRoot: { type: String, default: '' },
})

const STATUS = { done: '完成', failed: '失败', recording: '进行中' }
const hms = s => [s / 3600, (s / 60) % 60, s % 60].map(v => String(Math.floor(v)).padStart(2, '0')).join(':')
const toTime = n => n.replace(/^(\d{4})(\d\d)(\d\d)_(\d\d)(\d\d)(\d\d)$/, '$2-$3 $4:$5')
const points = n => (n > 1e6 ? `${(n / 1e6).toFixed(1)} M` : n > 1e3 ? `${Math.round(n / 1e3)} K` : n)
</script>

<template>
  <div class="card">
    <div class="title">录制记录</div>
    <div class="muted root">{{ dataRoot }}</div>
    <table>
      <thead>
        <tr><th>开始</th><th>时长</th><th>原始包</th><th>地图点数</th><th /></tr>
      </thead>
      <tbody>
        <tr v-for="s in sessions" :key="s.name">
          <td class="num" :title="s.name">{{ toTime(s.name) }}</td>
          <td class="num">{{ s.duration_s ? hms(Number(s.duration_s)) : '-' }}</td>
          <td class="num">{{ s.bag_size || '-' }}</td>
          <td class="num">{{ Number(s.map_points) ? points(Number(s.map_points)) : '-' }}</td>
          <td><span class="tag" :class="s.status" :title="s.error || ''">{{ STATUS[s.status] || s.status }}</span></td>
        </tr>
        <tr v-if="!sessions.length"><td colspan="5" class="muted">暂无记录</td></tr>
      </tbody>
    </table>
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
</style>
