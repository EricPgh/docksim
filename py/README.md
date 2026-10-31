# Docking planner (Python)

Plans a manoeuvre under power for a scenario exported from the app and
renders it as a movie.  Needs numpy, matplotlib, and ffmpeg (gif fallback).

    python3 dock_planner.py ../data/default-marina.json \
        --wind 12 200 --start 65 45 90 --goal 54.5 9 180 --out docking.mp4

    --wind KN FROM_DEG        steady wind
    --start E N HDG           metres in the scenario frame, heading deg
    --goal  E N HDG           where the boat should end up, at rest
    --pop / --iters           search budget (default 160 / 30)
    --vessel union32|jeanneau36   default: the scenario's vessel, else union32
    --init plan.json          seed the search from an existing plan (any vessel)
    --replay plan.json        skip planning; render this plan (hand-editable)
    --no-movie                plan only

Outputs `<out>.mp4`, `<out>.plan.json` (the segment schedule and arrival
numbers) and a plain-language narrative on stdout.

Files

    sailsim_physics.py   batched numpy port of src/physics.js + collision.js
    dock_planner.py      rollout, cost, cross-entropy search, movie
    test_port.py         checks the port against a JS-generated trajectory
    make_fixture.mjs     regenerates that fixture:  node py/make_fixture.mjs
    examples/            plans for the same slip, one per vessel, to replay or --init from

Verify the port after any change to physics.js:

    node py/make_fixture.mjs && python3 py/test_port.py

How it works, and what to tune, is in the docstring at the top of
dock_planner.py.  The cost weights (top of Rollout.run) encode seamanship
priors: gentle rpm, few gear changes, arrive slowly.  A full-budget run is a
few minutes on one core.

Seeding matters.  With a small budget the Union 32 search from scratch fell into
a local minimum (all-astern, 103 deg off heading); seeded with --init from the
Jeanneau's plan for the same slip it converged to 0.04 m, 0.5 deg, 0.2 kn.  A
solved plan for a nearby slip, or another boat's plan for the same one, is a
good prior.

Small sandbox budgets are marginal for the Union 32.  After the measured-hull
update the saved slip plan caught the anchor on a finger pier (the bow now
reaches 0.24 m further forward), and three seeded passes at pop 100 / 10
iterations did not get back inside tolerance; examples/union32-slip4.plan.json
is the best candidate (0.67 m, 6.3 deg, 0.1 kn, a fender touch).  Run the
default budget for a plan that meets tolerance.
