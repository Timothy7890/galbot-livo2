// 高度着色：Turbo 色带的多项式近似，生成 256 级查找表
function turbo(t) {
  const r = 34.61 + t * (1172.33 - t * (10793.56 - t * (33300.12 - t * (38394.49 - t * 14825.05))))
  const g = 23.31 + t * (557.33 + t * (1225.33 - t * (3574.96 - t * (1073.77 + t * 707.56))))
  const b = 27.2 + t * (3211.1 - t * (15327.97 - t * (27814.0 - t * (22569.18 - t * 6838.66))))
  const c = v => Math.max(0, Math.min(255, Math.round(v)))
  return [c(r), c(g), c(b)]
}

export const LUT = Array.from({ length: 256 }, (_, i) => turbo(i / 255))

// 高度范围（m，FAST-LIVO2 世界坐标系，原点为起始时 IMU 位置，约在地面以上 0.19 m）
export const Z_MIN = -1.0
export const Z_MAX = 1.5

export function colorOf(zcm) {
  const t = (zcm / 100 - Z_MIN) / (Z_MAX - Z_MIN)
  return LUT[Math.max(0, Math.min(255, Math.round(t * 255)))]
}

export const legendGradient = `linear-gradient(to right, ${[0, 0.25, 0.5, 0.75, 1]
  .map(t => `rgb(${LUT[Math.round(t * 255)].join(',')})`).join(', ')})`
