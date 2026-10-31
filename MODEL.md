# Physical model

Plan-view (3-DOF) manoeuvring model of a keel yacht under power at docking speeds.
Everything below is implemented in `src/physics.js`; each numbered section has a
matching unit test in `test/test_physics.mjs`.

## 0. Frames and conventions

Body frame: x forward, y to starboard, z down (SNAME). World frame: E east, N north,
heading ψ clockwise from north. State

    s = [x_E, y_N, ψ, u, v, r, δ, n]

u, v surge/sway of the CG through the water, r yaw rate, δ rudder angle
(+ = trailing edge to starboard), n propeller shaft speed (rev/s, + ahead).
Kinematics:

    ẋ_E = u sin ψ + v cos ψ,   ẏ_N = u cos ψ − v sin ψ,   ψ̇ = r

## 1. Rigid body + added mass

With diagonal added mass (m_ax, m_ay, J_a), CG at the body origin, Fossen's
M ν̇ + (C_RB + C_A) ν = τ reduces to

    m_x u̇ − m_y v r = X
    m_y v̇ + m_x u r = Y
    I_z ṙ + (m_y − m_x) u v = N

with m_x = m + m_ax, m_y = m + m_ay, I_z = I_zz + J_a. The uv term is the Munk
moment (potential-flow, destabilising: it tries to swing the hull broadside).
Its coefficient `addedMass.munk` is exposed because the keel-foil model below
already captures part of the same physics in viscous form; 1.0 is the
strict potential-flow value.

## 2. Lifting surfaces (keel, rudder)

Foil velocity through local water (u_in, v_in). Angle of attack

    α = δ + atan2(v_in, u_in)

(sign: with MMG's drift angle β = −atan(v/u) this is the usual α = δ − β).
Pre-stall thin-foil lift with Helmbold's finite-AR slope and induced drag,

    C_L = C_Lα α,  C_Lα = 2π AR / (2 + √(AR² + 4)),  C_D = C_D0 + C_L² / (π e AR)

post-stall flat plate with normal-force coefficient C_n,

    C_L = C_n sin α cos α,  C_D = C_n sin² α

blended with a smoothstep over [α_s, 1.5 α_s]. α is first wrapped to
(−π/2, π/2] so the model is π-periodic: reversed flow (sternway) is handled by
the same function. Force in body axes with q = ½ ρ A V²,
d = −(u_in, v_in)/V the flow direction past the foil, n = d rotated +90°:

    F = q (C_D d + C_L n)

Check: u>0, δ>0 → α>0, d = (−1,0), n = (0,−1): side force to port at the
stern → N = x_r Y > 0 → bow to starboard. u<0, δ>0 → d = (1,0), n = (0,1):
force to starboard → stern follows the wheel when backing. Both are tests.

## 3. Canoe-body hull

Surge: R = k_Q u|u| + k_L u. k_Q is calibrated so R(3.1 m/s) ≈ 1.25 kN, i.e.
resistance/displacement ≈ 2 % at 6 kn, which with the propeller below gives
~5 kn at 2000 rpm and ~6.4 kn at 2500 rpm (tests).

Wave-making (optional, `kWave`): a pure quadratic calibrated at top speed
over-predicts resistance at docking speeds, because real resistance turns up
steeply near hull speed. The added term

    R_w = k_W u|u| (u / u_hull)^4,   u_hull = 1.34 √LWL[ft]  kn

is negligible below ~4 kn and dominant near hull speed (unit test: R/u² rises
with speed). Used for the Union 32 (u_hull = 7.0 kn); zero for the Jeanneau.

Sway/yaw: strip theory with cross-flow drag. Sectional draft T(x) = T₀(1 − (2x/L)²);
local sway velocity v_l(x) = v + x r;

    dY = −½ ρ C_d T(x) |v_l| v_l dx,   dN = x dY

Integrating gives sway drag, yaw damping (∝ r|r| ∫x²|x|T dx) and the coupling
terms from a single model. C_d = 1.0, T₀ = 0.6 m were set by demanding a
hard-over pivot at 1500 rpm of roughly 10°/s with a ~6 m radius; this is the
least-grounded pair of numbers in the model and the first thing to tune against
a real boat.

Keel depth profile (optional, `keel.profile`): stations (x, d) with d the
depth BELOW the canoe body, so canoe(x) + d(x) is the keel bottom and the
canoe body (hull.T0) and keel never double-count lateral area. Strip area is
d(x_i) Δx, so a Brewer bite is simply a stretch of reduced d. Stations must be
dense enough to follow the canoe curvature, or canoe + d overshoots the draft
between them; a unit test checks the maximum of canoe + d against 1.47 m.

Keel: foil of §2 with δ = 0; contributes restoring side force, induced drag,
and post-stall cross-flow drag automatically. A fin keel is concentrated at
x_k. A long keel (`nSeg` > 0) is distributed over n strips of area A/n from
x_fwd to x_aft, each seeing its own inflow:

    dY_i = foil(u, v + x_i r),   N_keel = Σ x_i dY_i

Lumping a 5.8 m keel at one point resists sideslip but gives almost no yaw
damping (moment arm ~0.6 m); with the aperture wash on the rudder that made the
Union 32 pivot at 14 °/s on a 1.9 m radius. Distributing it is what restores
the long keel's yaw damping (unit test: >10x the lumped value).

## 4. Propeller

Momentum/K_T form with the advance ratio measured in the direction of thrust,

    J_a = sign(n) u_A / (|n| D),  u_A = u (1 − w)
    K_T = K_T0 (1 − J_a / J_0)   clipped to [−0.5 K_T0, 1.5 K_T0]
    T = sign(n) ρ n² D⁴ K_T × (astern factor if n < 0)

So: T ∝ n² at bollard; ahead thrust falls with speed and reverses above the
zero-thrust speed J₀ n D; astern thrust while still moving ahead is a stronger
brake (J_a < 0). Engine rpm → shaft rps through the KM2P mechanical gearbox,
which has a taller reduction astern (3.06:1) than ahead (2.21:1): at equal
engine rpm the shaft turns 28 % slower in reverse and thrust, ∝ n², is
roughly halved before any propeller loss. The feathering prop's reverse
efficiency is a separate factor of 0.85, so astern bollard thrust is ~45 % of
ahead (test). First-order rpm lag τ = 0.8 s.
No prop walk (as requested — it would be one added term Y_pw ∝ n|n| at x_p).

## 4a. Prop walk (vessels that have it)

    Y_pw = hand · k · T,   k = k_ahead (T > 0),  k_astern (T < 0),   N_pw = x_p Y_pw

hand = +1 for a right-hand prop (clockwise from astern when going ahead): ahead
the stern walks weakly to starboard; astern it walks strongly to port, bow to
starboard. The Jeanneau has none (feathering prop, observed on the boat). The
Union 32 uses k_astern = 0.25, which is FITTED to give ~15° of swing in 12 s
backing from rest at 1800 rpm. That is above the 5–15 % side-force fractions
usually quoted for open props; the point force at the propeller stands in for
wash striking the deadwood, which this model cannot represent directly. It is
the first coefficient to calibrate on the real boat: back off at fixed rpm,
wheel centred, and time the heading change.

## 5. Rudder in the slipstream

Actuator-disc far-wake velocity for ahead thrust

    u_W = √(u_A² + 2T / (ρ A_p)),  u_R = u_A + k_R (u_W − u_A)

applied to the fraction `slipFrac` of the rudder area in the wash, free-stream
u on the rest; both panels see v_in = v + x_r r. With astern thrust the wash
goes forward and the rudder sees only u — hence no steering from rest in
reverse until sternway builds, and a "kick ahead" steers even with sternway.
Both behaviours are tests.

Actuator-disc theory gives the axial velocity along the slipstream as
u(x) = u_A + ½(u_W − u_A)(1 + x/√(x² + R²)). For the Union 32 (rudder ~0.45 m
behind a 0.19 m-radius prop) the bracket is ~0.92; for the Jeanneau ~0.99.
Yet the Jeanneau's k_R was FITTED to 0.5 against pivot-rate targets, so the
fit is absorbing losses the theory ignores (hull wake, blockage, slipstream
contraction past the blade). The Union 32 therefore carries the same fitted
0.5 rather than the theoretical value; its closer coupling shows up through
the larger slipstream fraction instead (0.65 of the blade in the wash).

A keel-hung or skeg-hung rudder is a flap behind a fixed foil, so it generates less lift
than an all-moving rudder of the same area: C_L is scaled by `rudder.eff`
(0.7 for the Union 32). The skeg itself is a fixed foil (`skeg`) at its own x,
the same model as the keel, adding directional stability and yaw damping.

## 6. Windage

Apparent wind = true wind air velocity − boat velocity, rotated into the body
frame (w_x, w_y), |W| its magnitude:

    X = ½ ρ_a C_x A_F |W| w_x,  Y = ½ ρ_a C_y A_L |W| w_y,  N = x_CE Y

x_CE > 0 puts the aerodynamic centre ahead of the CG, so a beam wind makes the
bow fall off downwind at rest (test). The |W| w form keeps the quadratic scaling
and is exact head-on and beam-on.

## 7. Wheel

Rudder rate = wheel rate × (2 δ_max)/(2π × lock-to-lock turns). Wheel rate ramps
from 90°/s to 360°/s over 1 s while a key is held (rateMin/rateMax/rampTime).
The rudder holds where it is released (no self-centring).

## 8. Wind field

Speed = V̄ (1 + g) + Σ puffs, direction = θ̄ + d. g and d are Ornstein–Uhlenbeck
processes advanced with the exact discretisation

    x_{k+1} = x_k e^{−h/τ} + σ √(1 − e^{−2h/τ}) ξ,  ξ ~ N(0,1)

(test: stationary std → σ). Puffs arrive as a Poisson process (rate λ), each a
sin² bump of 6–16 s with amplitude up to `puffAmp` V̄ and a direction veer of
±15°. Seeded PRNG (mulberry32) so scenarios are reproducible.

## 9. Local wind modification (sheltering)

Optional, pre-computed once per scenario and stored in the JSON. Obstacle
polygons carry a `height`; the field is built on a grid (default 2 m) for
`nDirs` wind directions (default 16) in two stages, in this order:

(a) **Inviscid flow-around.** The uniform free stream is projected so it does
not penetrate the obstacles: **u** ← **u** + ∇λ with ∇²λ = −∇·**u**. This is a
MAC (staggered) discretisation — velocities on cell faces, λ at cell centres —
because on a collocated grid with centred differences the divergence and
gradient operators are not adjoint, the odd and even points decouple
(checkerboard), and the divergence barely falls. Walls get Neumann conditions
on λ with the face velocity forced to zero; the domain edge gets Dirichlet
λ = 0, so flow may enter and leave. An all-Neumann problem would be
incompatible (its source does not integrate to zero) and SOR would drift.
This stage alone produces the windward stagnation slow-down and the
acceleration past the corners, and it conserves mass, because a solid body
really is a mass barrier.

(b) **Empirical wake.** A lee cavity of length Lr and a far wake to 3 Lr are
multiplied in, tapering elliptically to the wake edge, using the Wilson/Röckle
cavity length

    Lr/H = 1.8 (W/H) / [ (L/H)^0.3 (1 + 0.24 W/H) ]

with W the crosswind width, L the along-wind length, H the height. Turbulence
intensity is raised in the same region.

**The order is essential.** Projecting a field that already contains the wake
makes the solver read the velocity deficit as a mass *sink* and draw flow
inward from upwind, which is unphysical — a wake is a momentum deficit, not a
sink. Getting this backwards produced a field that accelerated toward the
centreline 20 m ahead of the building.

At run time the boat's position and the current wind direction give
`{mult, deflDeg, turb}` by bilinear interpolation in space and linear
interpolation between direction bins. The free stream is split into mean and
fluctuation so the multipliers act on the right parts: `mult` scales the mean
and `turb` amplifies the gusts, so a lee is calmer on average yet gustier —
which is what wind-tunnel data shows and what catches people out.

**Limitations.** This is a Röckle-type diagnostic model (the family behind
QUIC-URB), not CFD. The zone parameterisations are empirical wind-tunnel fits,
so the coefficients are borrowed rather than derived. The flow is
depth-averaged, so building downwash — a genuinely three-dimensional effect,
and the dominant one at low levels — is only mimicked. No thermal effects, no
vertical profile, no separation modelling beyond the stamped cavity.

## 10. Contact

Hull outline sampled every 0.3 m. Polygons are either land (contact when a sample
is inside) or water (contact when a sample is outside every water polygon). A
contacting sample at depth d from the nearest edge gets F_n = max(0, k d − c v_n) along the outward direction plus
Coulomb friction μ F_n (viscously regularised), applied at the point (so it also
yaws the boat). k = 25 kN/m, c = 6 kN·s/m ≈ fender-like. Limitation: a sharp
dock corner between two hull samples is missed; sample spacing is a parameter.

## 11. Integration

Classical RK4, h = 0.01 s, on the full state including δ and n (their rate
limits are inside the derivative; δ is clamped after the step). The wind
process is stochastic and is stepped separately with its exact update and held
constant over the RK4 stages. Test: harmonic oscillator error drops ~16× when h
is halved.

## Parameters (UNION_32, default)

Class data: LOA 31.75 ft, LWL 27.25 ft, beam 10.83 ft, 11 500 lb, ballast
4 400 lb, draft 4.83 ft. Volvo Penta MD11D, 2-cylinder, 25 hp. Per the owner:
partial keel with a raked forefoot and a subtle Brewer bite, keel-hung rudder
on the aft deadwood, shaft exiting into an aperture just forward of the rudder,
Kiwiprop feathering 3-blade, bow pulpit with the anchor on a roller (in the
collision outline, ~0.45 m past the stem), no bowsprit.

Frame: x = 0 at mid-waterline. The overhangs are unequal, so mid-LOA is not
the origin. Owner's measurements, stem to mast step: 13.62 ft at deck level and
10.6 ft at the waterline. Their difference is the bow overhang, 3.02 ft =
0.92 m; the stern overhang is the rest of LOA − LWL, 0.45 m. Both measurements
then place the mast step at x = 0.924 m, and a unit test checks they agree.
(An earlier version split the overhangs equally, 0.685 m each, which put the
whole outline 0.235 m too far aft.)

Keel geometry: the forefoot rake starts at the vee-berth / head bulkhead,
2.72 ft forward of the mast step (owner), at x = 1.753 m; about a 45° rake to full draft;
bottom flat then rising aft in a straight line to the rudder heel; the bite is
cut 12 % of total draft (0.18 m) above that line (owner). Canoe body 0.52 m
at midships + keel 0.95 m = 1.47 m draft. Integrated keel area 4.51 m².

Calibrated: 5.4 kn at 2400 rpm, 6.0 kn at 2800, bollard ~2.6 kN. Hard over
under power the radius is ~6.7 m against the 36i's 6.0 m; coasting in neutral
at 3 kn, hard over, she turns 33° in 10 s against the 36i's 56°. Backing from
rest at 1800 rpm with the wheel centred, walk swings the bow ~13° in 12 s.

Unverified, to check on the boat: rake angle (~45° assumed), rated rpm and redline
(2800 assumed), MS2 gear ratio (2.4:1 assumed, equal both ways), Kiwiprop
diameter and set pitch (15″ assumed), prop handedness (right-hand assumed: in
reverse from rest a right-hand prop kicks the stern to port), rudder size
(0.6 m chord × 1.03 m). The walk coefficient (0.15) is fitted: back off at
fixed rpm, wheel centred, and time the heading change.

## Parameters (JEANNEAU_36, backup)

Sun Odyssey 36i figures: LOA 10.94 m, LWL 9.84 m, beam 3.59 m, draft 1.94 m,
5700 kg light (6200 loaded), Yanmar 3YM30 29 hp on a conventional shaft, KM2P
gearbox 2.21:1 ahead / 3.06:1 astern, 0.42 m 3-blade feathering prop about
0.6 m ahead of the rudder stock, single spade rudder ~0.75 m². No prop walk
(feathering prop; none observed on the boat). Appendage areas,
windage areas and C_x/C_y are estimates, not manufacturer data.
