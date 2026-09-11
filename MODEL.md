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

Sway/yaw: strip theory with cross-flow drag. Sectional draft T(x) = T₀(1 − (2x/L)²);
local sway velocity v_l(x) = v + x r;

    dY = −½ ρ C_d T(x) |v_l| v_l dx,   dN = x dY

Integrating gives sway drag, yaw damping (∝ r|r| ∫x²|x|T dx) and the coupling
terms from a single model. C_d = 1.0, T₀ = 0.6 m were set by demanding a
hard-over pivot at 1500 rpm of roughly 10°/s with a ~6 m radius; this is the
least-grounded pair of numbers in the model and the first thing to tune against
a real boat.

Keel: foil of §2 at x_k with δ = 0; contributes restoring side force,
induced drag, and post-stall cross-flow drag automatically.

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

## 5. Rudder in the slipstream

Actuator-disc far-wake velocity for ahead thrust

    u_W = √(u_A² + 2T / (ρ A_p)),  u_R = u_A + k_R (u_W − u_A)

applied to the fraction `slipFrac` of the rudder area in the wash, free-stream
u on the rest; both panels see v_in = v + x_r r. With astern thrust the wash
goes forward and the rudder sees only u — hence no steering from rest in
reverse until sternway builds, and a "kick ahead" steers even with sternway.
Both behaviours are tests.

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

## Parameters (JEANNEAU_36)

Sun Odyssey 36i figures: LOA 10.94 m, LWL 9.84 m, beam 3.59 m, draft 1.94 m,
5700 kg light (6200 loaded), Yanmar 3YM30 29 hp on a conventional shaft, KM2P
gearbox 2.21:1 ahead / 3.06:1 astern, 0.42 m 3-blade feathering prop about
0.6 m ahead of the rudder stock, single spade rudder ~0.75 m². No prop walk
(feathering prop; none observed on the boat). Appendage areas,
windage areas and C_x/C_y are estimates, not manufacturer data.
