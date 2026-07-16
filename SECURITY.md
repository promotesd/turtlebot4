# Security policy

The RL framework is pre-1.0 and currently maintained on `jazzy`. Security updates are
made on that branch; older development snapshots are unsupported.

Do not open a public issue for a vulnerability. Email `1412822254@qq.com` with the
affected revision, impact, reproduction steps, and any suggested mitigation. Expect
acknowledgement within seven days. Please allow maintainers time to investigate and
coordinate a release before public disclosure.

Robot motion is safety-sensitive. Treat uncommanded velocity, watchdog bypass,
unsafe reset behavior, command injection, and sensor freshness errors as security and
safety issues. Never include secrets, private maps, or personally identifying sensor
recordings in a report.
