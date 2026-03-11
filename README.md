# 🚌 Columbia Evening Shuttle — Hybrid Fleet Sizing

> **IEOR 4418: Transportation Analytics & Logistics — Final Project**
> Columbia University | J. Ling, T. Flamand, S. Sheng, A. Huang, M. Zhang | October 2025

---

## 📋 Overview

This project combines an **M/M/s queueing model** with an **agent-based simulation** (STARS) to determine optimal driver staffing levels for the Columbia Evening Shuttle, using historical ride-request data from October 2025.

---

## 📂 Repository Structure

```
Columbia_Shuttle/
├── stars/
│   ├── simulation.py       # Master controller & main loop
│   ├── driver.py           # Driver agent (location, capacity, routing)
│   ├── request.py          # Request state machine (PENDING → COMPLETED)
│   ├── route.py            # Gurobi TSP solver + shortest-path geometry
│   └── logger.py           # CSV event logging (SimulationLogger)
├── mms/
│   ├── mms_model.py        # Erlang-C metrics per weekday–hour slot
│   └── threshold.py        # Exponential fit + closed-form x_β formula
├── analysis/
│   ├── figures.py          # Fleet size and shift duration figures
│   └── comparison.py       # M/M/s vs. STARS side-by-side comparison
├── data/                   # ← NOT included (confidential, see Data Availability)
│   └── .gitkeep
├── logs/                   # Simulation CSV outputs (auto-generated at runtime)
├── requirements.txt
└── README.md
```

---

## 🗄️ Data Availability

> **The raw trip data is not included in this repository.**

Historical ride-request records were provided by the Columbia Shuttle service (operated via [Via](https://ridewithvia.com)) under a data-sharing agreement for academic research only. Redistribution is prohibited by Columbia University's data governance policy.

---

## 🧮 Models

### Part 1 — M/M/s Queueing Model

Each weekday–hour slot *(d, h)* is modelled as an independent **M/M/s queue** with:

- $\lambda_{d,h}$: Poisson arrival rate, estimated from normalised October 2025 request counts.
- $\mu_{d,h}$: Exponential service rate $= 1 /$ mean trip duration in the same slot.
- **Offered load** $a = \lambda / \mu$, **utilisation per server** $\rho = a / s$ (stability: $\rho < 1$).

The **Erlang-C formula** yields $P_{\text{wait}}$ and $L_q$; Little's Law gives $W_q$ (queue wait) and $W$ (total time in system).

**Staffing threshold.** Because $W_q(s)$ has no closed-form inverse, an exponential is fitted to the discrete curve:

$$W(x) = a \cdot e^{-bx} + c$$

The smallest fleet size at which each additional driver reduces wait by less than $\beta$ is then:

$$x_\beta = \left\lceil \frac{1}{b} \cdot \ln\!\left( \frac{a\left(e^{b(1-\beta)} - 1\right)}{\beta \cdot c} \right) \right\rceil$$

At $\beta = 5\%$, thresholds range from **22–33 drivers** depending on day and hour.

---

### Part 2 — STARS: Agent-Based Simulation

STARS embeds driver and passenger agents on the real **Manhattan road network** (OpenStreetMap via NetworkX/osmnx), relaxing the spatial-homogeneity assumptions of the queueing model.

**Key assumptions**

| Dimension | Assumption |
|---|---|
| Network | Directed OSM graph; stops snapped to nearest node |
| Travel times | Deterministic, piecewise-constant hourly speed schedule |
| Demand | Historical requests replayed; no cancellations |
| Fleet | Homogeneous 4-seat vehicles; fixed starting location |
| Dispatch | Full information; greedy earliest-pickup assignment |

**Routing — open TSP with precedence & capacity constraints**

For each candidate assignment, the simulator solves a MILP via Gurobi:

$$\min \sum_{i,j} d_{ij} \cdot x_{ij}$$

subject to:
- Flow conservation (open path from driver location)
- Pickup precedes dropoff for every request $k$
- Vehicle load $\leq$ capacity $C$ at all nodes
- Existing passengers' in-system time not increased by more than 20%

A route is accepted only if all constraints are satisfied. The feasible driver offering the earliest pickup time wins the greedy dispatch.

**Simulation loop** (1-minute increments)

```
for each minute t:
    1. update_fleet()       # advance positions, execute stops, log completions
    2. manage_fleet()       # hourly: scale drivers up/down vs. staffing profile
    3. ingest_requests()    # add new requests with timestamp ≤ t
    4. dispatch()           # assign pending requests → earliest feasible driver
```

**Calibration gap.** Validated against 2,025 real trips (Oct 6–12, 18:00–22:59):

| Metric | Real | Simulated | Factor |
|---|---|---|---|
| Mean wait time | 20.39 min | 3.79 min | ×5.39 |
| Mean in-system time | 28.96 min | 8.53 min | ×3.39 |
| Mean service time | 8.57 min | 4.75 min | ×1.81 |

The disproportionate gap in wait (vs. service) time indicates that the simulator underestimates congestion and driver availability friction more than road speeds. STARS is nonetheless appropriate for **comparative staffing analysis** since it preserves the structural demand–fleet relationships.

---

## 📊 Results

### Fleet size vs. average wait time

Fitted on the busiest hour of the week (Oct 9, 20:00–20:59, 218 requests):

$$W(x) = 3.792 \cdot e^{-0.0949\,x} + 0.838 \qquad R^2 = 0.977$$

$x_{\beta=5\%}$ — diminishing-returns threshold → **15 drivers**
$x_{\beta=1\%}$ — near-zero marginal benefit → **39 drivers**

Wait time falls sharply between 15 and 25 drivers, then flattens near 30.

### Shift duration vs. average wait time

Fitted on Oct 6, 18:00–22:59 (power-law model):

$$W(x) = -0.499 \cdot x^{-2.721} + 3.831 \qquad R^2 = 0.995$$

Wait time rises sharply from 1-hour to 2-hour shifts, then plateaus. Shifts beyond 2 hours primarily add route carryover without throughput gains.

### M/M/s vs. STARS comparison

The queueing model consistently **underestimates required staffing** — it assumes instantaneous pickup and ignores routing delays, geographic imbalance, and batching overhead. It provides an optimistic lower bound; STARS captures the higher fleet levels needed in practice.

---

## ⚙️ Setup & Requirements

```bash
pip install networkx osmnx gurobipy pandas numpy
```

A valid **Gurobi license** is required. Academic licenses are available free at [gurobi.com](https://www.gurobi.com/academia/academic-program-and-licenses/).

---

## 🚀 Usage

```bash
python stars/simulation.py       # Run agent-based simulation
python mms/mms_model.py          # Compute Erlang-C metrics
python analysis/comparison.py    # Generate M/M/s vs. STARS comparison
```

---

## 🔬 Technical Highlights

- **M/M/s queueing** with Erlang-C formula and closed-form staffing threshold $x_\beta$
- **MILP routing** via Gurobi with precedence and capacity constraints
- **Agent-based simulation** on real OSM road network (NetworkX/osmnx)
- **Exponential and power-law curve fitting** for fleet-size and shift-duration analyses
- **Calibration against 2,025 real trips** for comparative validity assessment

---

## 📄 Citation

```bibtex
@techreport{ling2025shuttle,
  title   = {A Hybrid Approach to Rideshare Fleet Sizing:
             Integrating M/M/s Queueing Models with Agent-Based Simulation},
  author  = {Jiahe Ling, Thomas Flamand, Sarah Sheng, Alina Huang, Michael Zhang},
  year    = {2025},
  institution = {Department of Industrial Engineering and Operations Research,
                 Columbia University},
  note    = {IEOR 4418 -- Transportation Analytics & Logistics}
}
```
