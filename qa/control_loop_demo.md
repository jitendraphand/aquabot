# QA control-loop demonstration

`python -m scripts.control_loop_demo` starts the crawl gait from a deliberately aggressive setting (period 1.6 s, commanded 0.06 m/s) and lets the QA control loop correct it. Each iteration runs the MuJoCo walk-and-turn scenario, evaluates checks S-SIM-01…05, and applies the registered fixer of every failing check. The project's own tuned parameters are restored afterwards.

| Iter | Crawl period (s) | v_cmd (m/s) | speed (m/s) | tilt (°) | 50 ms peak τ | p99 τ | Failing | Action |
|---:|---:|---:|---:|---:|---:|---:|---|---|
| 1 | 1.6 | 0.06 | 0.0646 | 3.1817 | 0.9397 | 0.6556 | S-SIM-03 (0.94 vs 0.9) | crawl period -> 1.728 s, v_cmd -> 0.051 m/s |
| 2 | 1.728 | 0.051 | 0.057 | 3.4364 | 0.9013 | 0.6321 | S-SIM-03 (0.901 vs 0.9) | crawl period -> 1.866 s, v_cmd -> 0.0433 m/s |
| 3 | 1.866 | 0.0433 | 0.0519 | 3.6962 | 0.8639 | 0.6099 | ✅ none | — |
