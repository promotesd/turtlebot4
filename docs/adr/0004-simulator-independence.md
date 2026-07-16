# ADR 0004: keep simulator control out of task and algorithm code

- Status: accepted

World names, entities, pause/resume, goal markers, and pose reset belong only to a
world adapter. Core requests a reset and reads a goal. New simulators can replace this
adapter without changing environment or DQN code. Simulator integration tests remain
separate because unit/Mock success is not simulator evidence.
