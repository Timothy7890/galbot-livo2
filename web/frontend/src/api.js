export async function getStatus() {
  const r = await fetch('api/status')
  return r.json()
}

export async function startRecording() {
  const r = await fetch('api/start', { method: 'POST' })
  return r.json()
}

export async function getExplore() {
  const r = await fetch('api/explore')
  if (!r.ok) throw new Error('未在录制')
  return r.json()
}

// action：enable 进入自主建图 / disable 退出 / cancel 取消完成后的自动停止录制
// enable 时 body 为 { strategy }
export async function exploreAction(action, body) {
  const r = await fetch(`api/explore/${action}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: body ? JSON.stringify(body) : undefined,
  })
  return r.json()
}
export const exploreGridUrl = 'api/explore/grid.png'

export async function stopRecording() {
  const r = await fetch('api/stop', { method: 'POST' })
  return r.json()
}

// 急停 / 解除：经 7010 转发给 7011 遥控服务
export async function estop() {
  const r = await fetch('api/estop', { method: 'POST' })
  return r.json()
}
export async function releaseEstop() {
  const r = await fetch('api/estop/release', { method: 'POST' })
  return r.json()
}

export const liveUrl = 'api/live'
export const cameraUrl = name => `api/camera/${name}.jpg`

// 导航（成熟地图）
export async function getNav() {
  const r = await fetch('api/nav')
  return r.json()
}
// action：start {session} / stop / goto {x, y} / cancel / reloc {mode, x, y, yaw} / build {session, name}
export async function navAction(action, body) {
  const r = await fetch(`api/nav/${action}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body || {}),
  })
  try { return await r.json() } catch { return { ok: false, msg: `HTTP ${r.status}` } }
}
export const navGridUrl = session => (session ? `api/nav/grid.png?session=${session}` : 'api/nav/grid.png')
