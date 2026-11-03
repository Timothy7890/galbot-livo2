async function post(path, body) {
  const r = await fetch(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: body ? JSON.stringify(body) : undefined,
  })
  return r.json()
}

export async function getStatus() {
  const r = await fetch('api/status')
  return r.json()
}

export const sendCmd = (vx, vy, wz, safe) => post('api/cmd', { vx, vy, wz, safe })
export const estop = () => post('api/stop')
export const release = () => post('api/release')
export const calibrate = () => post('api/calibrate')
