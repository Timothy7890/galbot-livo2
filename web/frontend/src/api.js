export async function getStatus() {
  const r = await fetch('api/status')
  return r.json()
}

export async function startRecording(explore = false) {
  const r = await fetch('api/start', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ explore }),
  })
  return r.json()
}

export async function getExplore() {
  const r = await fetch('api/explore')
  if (!r.ok) throw new Error('未在自主探索')
  return r.json()
}
export const exploreGridUrl = 'api/explore/grid.png'

export async function stopRecording() {
  const r = await fetch('api/stop', { method: 'POST' })
  return r.json()
}

export const liveUrl = 'api/live'
export const cameraUrl = name => `api/camera/${name}.jpg`
