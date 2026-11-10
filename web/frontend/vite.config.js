import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'
import { resolve } from 'node:path'

// 构建产物输出到 web/dist：index.html（7010 录制）、nav.html（7010 导航）、teleop.html（7011 底盘遥控）
// 本地开发：VITE_API=http://localhost:7010 npm run dev（遥控页面改为 7011）
export default defineConfig({
  plugins: [vue()],
  base: './',
  build: {
    outDir: '../dist',
    emptyOutDir: true,
    rollupOptions: {
      input: {
        index: resolve(__dirname, 'index.html'),
        teleop: resolve(__dirname, 'teleop.html'),
        nav: resolve(__dirname, 'nav.html'),
      },
    },
  },
  server: {
    host: true,
    proxy: { '/api': process.env.VITE_API || 'http://localhost:7010' },
  },
})
