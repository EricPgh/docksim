# Docking planner (Python)

Plans a manoeuvre under power for a scenario exported from the app and
renders it as a movie.  Needs numpy, matplotlib, and ffmpeg (gif fallback).

    python3 dock_planner.py ../data/default-marina.json \
        --wind 12 200 --start 65 45 90 --goal 54.5 9 180 --out docking.mp4

    --wind KN FROM_DEG        steady wind
    --start E N HDG           metres in the scenario frame, heading deg
    --goal  E N HDG           where the boat should end up, at rest
    --pop / --iters           search budget (default 160 / 30)
    --replay plan.json        skip planning; render this plan (hand-editable)
    --no-movie                plan only

Outputs `<out>.mp4`, `<out>.plan.json` (the segment schedule and arrival
numbers) and a plain-language narrative on stdout.

Files

    sailsim_physics.py   batched numpy port of src/physics.js + collision.js
    dock_planner.py      rollout, cost, cross-entropy search, movie
    test_port.py         checks the port against a JS-generated trajectory
    make_fixture.mjs     regenerates that fixture:  node py/make_fixture.mjs
    examples/            a plan from the default marina to replay

Verify the port after any change to physics.js:

    node py/make_fixture.mjs && python3 py/test_port.py

How it works, and what to tune, is in the docstring at the top of
dock_planner.py.  The cost weights (top of Rollout.run) encode seamanship
priors: gentle rpm, few gear changes, arrive slowly.  A full-budget run is a
few minutes on one core.
