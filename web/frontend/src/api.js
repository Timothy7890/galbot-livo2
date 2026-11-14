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
// action：start {session, vision, dry_run} / stop / goto {x, y, yaw?} / cancel / reloc {mode, x, y, yaw} / confirm /
//         speed {v} / heartbeat / build {session, name}
export async function navAction(action, body) {
  let r
  try {
    r = await fetch(`api/nav/${action}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {}),
    })
  } catch (e) {
    return { ok: false, msg: `连不上 7010 服务：${e.message || e}` }
  }
  try {
    const d = await r.json()
    if (d.ok === undefined) d.ok = r.ok
    if (!d.ok && !d.msg) d.msg = d.error || `HTTP ${r.status}`
    return d
  } catch {
    return { ok: false, msg: `服务端返回异常（HTTP ${r.status}），可能代码更新后没重启：web/web.sh restart` }
  }
}
export const navGridUrl = session => (session ? `api/nav/grid.png?session=${session}` : 'api/nav/grid.png')
export const navStreamUrl = 'api/nav/stream'
export async function getNavTraj(session) {
  const r = await fetch(`api/nav/traj?session=${session}`)
  return r.ok ? r.json() : []
}
