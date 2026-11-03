export async function getStatus() {
  const r = await fetch('api/status')
  return r.json()
}

export async function startRecording() {
  const r = await fetch('api/start', { method: 'POST' })
  return r.json()
}

export async function stopRecording() {
  const r = await fetch('api/stop', { method: 'POST' })
  return r.json()
}

export const liveUrl = 'api/live'
export const cameraUrl = 'api/camera.jpg'
