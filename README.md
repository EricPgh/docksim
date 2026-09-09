# sailsim — docking a keel yacht under power

Plan-view simulator: 3-DOF equations of motion with physically based hull,
keel, rudder (with prop wash), propeller and windage models, RK4 integration,
stochastic wind, and fender-style dock contact. Physics is a pure ES module
with no DOM dependence; the same file runs in Node (tests) and the browser.

    npm test              # 18 unit tests: every force model, wind, contact, RK4
    npm run serve         # then open http://localhost:8000

Files

    src/physics.js     parameters, force models, dynamics, RK4     (pure)
    src/wind.js        OU gusts + Poisson puffs, seeded             (pure)
    src/collision.js   dock polygons, hull sampling, contact forces (pure)
    src/app.js         canvas renderer, keys/touch, scenario editor
    index.html         page, HUD, side panel, on-screen buttons
    test/              node --test
    MODEL.md           derivation of every equation and its calibration

Controls: Z / X wheel to port / starboard (hold to spin faster; rudder stays
where you leave it, C centres). Up / Down step the throttle through
neutral, 800, 1000, 1500, 2000, 2500 rpm ahead and the same astern. Space
pauses, R resets, N draws a new random wind, F fits the view, B toggles
boat-up, + / - zoom in / out on the boat.

Your own marina: "Import map" takes either an image (new layout) or a
scenario JSON exported earlier (scale, polygons and image all come back).
For a new image: "Set map scale" and click two points a known distance apart,
then "Draw polygon" and click corners. Finish it either as land (a dock, quay
or shore — contact when the hull is inside) or as water (a navigable basin —
contact when the hull is outside it). Both kinds can be mixed. "Place boat":
click the position, then a point ahead of the bow. "Export scenario JSON"
saves everything for record-keeping and re-import.
Boat sprite: any image with the bow pointing up; you are asked its length.

Everything tunable is in `JEANNEAU_36` at the top of physics.js. The two
least-grounded numbers are `hull.cdCross` and `hull.T0` (set the pivot rate);
see MODEL.md §3.

## On an iPad, without the Mac

The app is a PWA: `manifest.webmanifest` + `sw.js` cache the whole thing on
first load. Safari only installs the service worker over HTTPS, so serve it
from GitHub Pages once:

1. Create a repo (e.g. `sailsim`), push this folder to it.
2. Repo Settings -> Pages -> Source: "Deploy from a branch", branch `main`,
   folder `/ (root)`. Save. After a minute the site is at
   `https://<user>.github.io/sailsim/`.
3. On the iPad open that URL in Safari, wait for it to load once, then
   Share -> Add to Home Screen. From then on it launches fullscreen and works
   with no network.

Updating: push, then bump `VERSION` in `sw.js` (or the iPad keeps the cached
build). The new version installs on the next launch with network.
