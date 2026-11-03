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
export async function exploreAction(action) {
  const r = await fetch(`api/explore/${action}`, { method: 'POST' })
  return r.json()
}
export const exploreGridUrl = 'api/explore/grid.png'

export async function stopRecording() {
  const r = await fetch('api/stop', { method: 'POST' })
  return r.json()
}

export const liveUrl = 'api/live'
export const cameraUrl = name => `api/camera/${name}.jpg`
