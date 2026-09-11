// Service worker: cache-first for the app shell so the iPad runs offline.
// Bump VERSION whenever any file changes so clients pick up the new build.
const VERSION = 'sailsim-v3';
const SHELL = ['./', './index.html', './src/app.js', './src/physics.js', './src/wind.js',
               './src/collision.js', './src/shelter.js', './manifest.webmanifest', './icon.svg', './data/index.json'];
self.addEventListener('install', e => {
  e.waitUntil(caches.open(VERSION).then(c => c.addAll(SHELL)).then(() => self.skipWaiting()));
});
self.addEventListener('activate', e => {
  e.waitUntil(caches.keys().then(ks => Promise.all(ks.filter(k => k !== VERSION).map(k => caches.delete(k))))
    .then(() => self.clients.claim()));
});
self.addEventListener('fetch', e => {
  if (e.request.method !== 'GET') return;
  // data/ is network-first so a scenario edited on the server shows up without
  // bumping VERSION; the cached copy is the offline fallback.
  if (new URL(e.request.url).pathname.includes('/data/')) {
    e.respondWith(fetch(e.request).then(res => {
      const copy = res.clone(); caches.open(VERSION).then(c => c.put(e.request, copy)); return res;
    }).catch(() => caches.match(e.request, { ignoreSearch: true })));
    return;
  }
  e.respondWith(caches.match(e.request, { ignoreSearch: true }).then(hit => hit ||
    fetch(e.request).then(res => { const copy = res.clone(); caches.open(VERSION).then(c => c.put(e.request, copy)); return res; })
  ));
});
