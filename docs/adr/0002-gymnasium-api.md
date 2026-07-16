# ADR 0002: use Gymnasium reset and step semantics

- Status: accepted

The environment returns `(observation, info)` from reset and the five-element
`step` result. Goal/collision terminate the MDP; timeout/sensor/backend faults truncate
the episode. DQN bootstraps through truncation but not termination. Small local space
objects keep Core installable without Gymnasium while preserving its calling contract.
