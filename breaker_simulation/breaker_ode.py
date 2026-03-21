'''
breaker_param_fit.py

Fits thermal circuit breaker model parameters (C, R_thermal) from
trip time vs. overcurrent data read off the CB285-120 datasheet curve.

Thermal model:
    C * dT/dt = P_in - (T - T_ambient) / R_thermal

At trip:  T reaches T_trip
At rated current (steady state): T_steady = T_ambient + P_rated * R_thermal < T_trip

We solve the ODE analytically for a step overcurrent I (constant):
    T(t) = T_ambient + P_in*R_thermal * (1 - exp(-t/tau))
    where P_in = I^2 * R_breaker  (or directly in watts if known)
    and   tau  = C * R_thermal

Trip occurs when T(t_trip) = T_trip, so:
    T_trip = T_ambient + P_in*R_thermal * (1 - exp(-t_trip/tau))

Rearranging:
    exp(-t_trip/tau) = 1 - (T_trip - T_ambient) / (P_in * R_thermal)

This gives us a nonlinear system: two trip-time data points -> two equations
-> solve for (C, R_thermal) numerically.

-----------------------------------------------------------------------
HOW TO USE:
1. Pull up the CB285-120 datasheet from Eaton

2. Read two (% rated current, trip time) points off the trip curve.
   Example values below are ESTIMATES for the CB285 hi-amp series —
   replace them with values you read directly from your datasheet.

3. You also need the breaker's internal resistance (R_breaker).
   For the CB285-120, this is typically in the range of 0.5–2 mΩ.
   If you don't have it, use the power-based mode (see P_IN_WATTS_MODE).

4. Run: python breaker_ode.py
-----------------------------------------------------------------------
'''

import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt

# ============================================================
# USER INPUTS — Edit these to match your datasheet readings
# ============================================================

# Rated current of the breaker (A)
I_RATED = 120.0  # CB285-120

# Ambient temperature (°C)
T_AMBIENT = 25.0

# Trip temperature (°C) — bimetallic strip deflection threshold.
# Not published directly; 100–130°C is typical for automotive breakers.
# This is a tuning parameter — adjust until simulated trip times match datasheet.
T_TRIP_RANGE = (100, 240)

# Internal resistance of the breaker (Ohms).
# Used to convert I^2*R -> heat power into the strip.
# Typical CB285-series value: ~1–2 mΩ. Adjust or measure with a milliohmmeter.
R_BREAKER = 0.003  # 1 mΩ — measure this

# ---------------------------------------------------------------
# Trip curve data points (read from CB285 datasheet graph).
# Each entry: (overcurrent_multiplier, trip_time_seconds)
# e.g. (2.0, 45.0) means: at 2x rated current, trips in ~45 seconds.
#
# IMPORTANT: These are PLACEHOLDER estimates for the CB285 hi-amp series.
# Read the actual values off your datasheet before using for real design work.
# ---------------------------------------------------------------
TRIP_DATA = [
    # (1.35, 100.0),
    # (1.5, 40.0),
    (1.75, 15.0),
    (2.0, 10.0),
    (2.5, 5.0),
    (2.75, 3.0),
    (3.9, 2.0),  # 350% rated current -> ~5s trip   (estimate)
    (5.0, 1.0),  # 500% rated current -> ~1s trip    (estimate)
]

# sort by the trip times
TRIP_DATA.sort(key=lambda x: x[0])

# ============================================================
# THERMAL MODEL
# ============================================================

def power_in(multiplier):
    '''Heat dissipated in breaker strip at given current multiplier (W).'''
    I = multiplier * I_RATED
    return I ** 2 * R_BREAKER

# ============================================================
# MAIN
# ============================================================

if __name__ == '__main__':
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.semilogy(
        [m * I_RATED for m, _ in TRIP_DATA],
        [t for _, t in TRIP_DATA],
        'ro', markersize=8, label='Datasheet points (estimated)'
    )

    print('=' * 60)
    print('CB285-120 Thermal Parameter Fitting')
    print('=' * 60)
    print(f'\nBreaker: {I_RATED}A  |  R_breaker: {R_BREAKER * 1000:.2f} mΩ')
    print(f'T_ambient: {T_AMBIENT}°C  |  T_trip: {T_TRIP_RANGE[0]}°C-{T_TRIP_RANGE[1]}°C\n')

    # --- Fit from first two data points ---
    p1, p2 = TRIP_DATA[0], TRIP_DATA[1]
    print(f'Fitting from data points:')
    print(f'  Point 1: {p1[0] * 100:.0f}% rated ({p1[0] * I_RATED:.0f}A) -> {p1[1]:.1f}s')
    print(f'  Point 2: {p2[0] * 100:.0f}% rated ({p2[0] * I_RATED:.0f}A) -> {p2[1]:.1f}s\n')

    line_colors = ['b', 'g', 'C1', 'C4', 'C6', 'C8', 'C9']
    T_TRIP_INCREMENT = (T_TRIP_RANGE[1] - T_TRIP_RANGE[0]) / float(len(line_colors) - 1)

    for i in range(len(line_colors)):
        LINE_COLOR = line_colors[i]
        T_TRIP = T_TRIP_RANGE[0] + (T_TRIP_INCREMENT * i)

        def trip_time_model(multiplier, C, R_thermal):
            '''
            Analytically predicted trip time for a step overcurrent.

            From ODE solution:
                T(t) = T_ambient + P*R_thermal*(1 - exp(-t/tau))
            At trip: T(t_trip) = T_trip
                => t_trip = -tau * ln(1 - delta_T / (P * R_thermal))
            where tau = C * R_thermal
            '''
            P = power_in(multiplier)
            tau = C * R_thermal
            delta_T = T_TRIP - T_AMBIENT
            steady_state_rise = P * R_thermal

            if steady_state_rise <= delta_T:
                # Current too low to ever trip at this R_thermal
                return float('inf')

            t_trip = -tau * np.log(1.0 - delta_T / steady_state_rise)
            return t_trip

        def residuals(params, data_points):
            '''Residuals between model predictions and datasheet observations.'''
            C, R_thermal = params
            if C <= 0 or R_thermal <= 0:
                return [1e9, 1e9]
            res = []
            for mult, t_measured in data_points:
                t_pred = trip_time_model(mult, C, R_thermal)
                res.append(t_pred - t_measured)
            return res


        # ============================================================
        # FIT: use the first two data points to solve for C, R_thermal
        # ============================================================

        def fit_two_points(p1, p2):
            '''Fit C and R_thermal from exactly two trip-time data points.'''

            def equations(params):
                C, R_thermal = params
                return residuals(params, [p1, p2])

            # Initial guess — reasonable starting values for a small automotive breaker
            C0 = 5.0  # J/°C
            R0 = 2.0  # °C/W
            solution, info, ier, msg = fsolve(equations, [C0, R0], full_output=True)
            if ier != 1:
                print(f'  Warning: solver did not fully converge: {msg}')
            return solution


        # ============================================================
        # VALIDATE: simulate the ODE numerically and compare to all data
        # ============================================================

        def simulate_trip_time(C, R_thermal, multiplier, dt=0.01, max_t=600.0):
            '''Numerically integrates the thermal ODE to find trip time.'''
            T = T_AMBIENT
            P = power_in(multiplier)
            t = 0.0
            while t < max_t:
                cooling = (T - T_AMBIENT) / R_thermal
                dTdt = (P - cooling) / C
                T += dTdt * dt
                t += dt
                if T >= T_TRIP:
                    return t
            return float('inf')  # Did not trip within max_t

        C_fit, R_fit = fit_two_points(p1, p2)
        tau_fit = C_fit * R_fit

        print(f'Fitted Parameters:')
        print(f'  Thermal Capacitance (C):    {C_fit:.4f} J/°C')
        print(f'  Thermal Resistance (R):     {R_fit:.4f} °C/W')
        print(f'  Thermal Time Constant (τ):  {tau_fit:.2f} s')

        # --- Validate against all data points ---
        print(f'\nValidation against all datasheet points for T_TRIP={T_TRIP:.2f}:')
        print(f'  {'Multiplier':>10}  {'Current(A)':>10}  {'Measured(s)':>12}  {'Simulated(s)':>13}  {'Error%':>8}')
        print('  ' + '-' * 60)
        for mult, t_meas in TRIP_DATA:
            t_sim = simulate_trip_time(C_fit, R_fit, mult)
            err = (t_sim - t_meas) / t_meas * 100 if t_meas != float('inf') else float('nan')
            t_sim_str = f'{t_sim:.2f}' if t_sim != float('inf') else 'no trip'
            print(f'  {mult * 100:>9.0f}%  {mult * I_RATED:>10.0f}  {t_meas:>12.1f}  {t_sim_str:>13}  {err:>7.1f}%')

        # --- Plot: simulated trip curve vs datasheet points ---
        multipliers = np.linspace(TRIP_DATA[0][0], TRIP_DATA[-1][0], 200)
        trip_times = [simulate_trip_time(C_fit, R_fit, m, dt=0.05) for m in multipliers]
        trip_times = [t if t < 600 else None for t in trip_times]
        ax.semilogy(
            [m * I_RATED for m, t in zip(multipliers, trip_times) if t is not None],
            [t for t in trip_times if t is not None],
            LINE_COLOR + '-', linewidth=2, label=f'T_TRIP={T_TRIP}°C'
        )

    ax.set_xlabel('Current (A)')
    ax.set_ylabel('Trip Time (s)')
    ax.set_title(f'CB285-120 Trip Curve\nC={C_fit:.3f} J/°C, R={R_fit:.3f} °C/W, τ={tau_fit:.1f}s')
    ax.legend()
    ax.grid(True, which='both', alpha=0.3)
    plt.tight_layout()
    plt.savefig('cb285_trip_curve.png', dpi=150)
    print(f'\nTrip curve plot saved to: cb285_trip_curve.png')
