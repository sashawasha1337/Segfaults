// npm run test for testing with Vitest

import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import mkcert from 'vite-plugin-mkcert'

export default defineConfig({
  plugins: [
    react(),
    mkcert({
      // specify any hosts you need; default covers localhost
      hosts: ['localhost', '0.0.0.0', '192.168.1.100'],
    })
  ],
  test: {
    globals: true,
    environment: 'jsdom',
    setupFiles: './tests/setupTests.js',
    include: ['tests/**/*.{test,spec}.{js,jsx}', 'src/**/*.{test,spec}.{js,jsx}'], 
  },
  server: {
    host: true,    // bind on 0.0.0.0 so LAN IP works
    port: 3000,
    https: false    // plugin will hook this up
  }
})