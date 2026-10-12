# Triage labels

Default vocabulary, one label per canonical role:

- `needs-triage` — raw issue, not yet assessed
- `needs-info` — waiting on information from the user
- `ready-for-agent` — agent-grabbable as-is
- `ready-for-human` — needs a human decision or hands-on step
- `wontfix` — explicitly rejected

Label strings equal the role names. Tickets published by `to-spec`/`to-tickets` carry `Status: ready-for-agent` by default.
