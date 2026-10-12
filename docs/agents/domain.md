# Domain docs

Single-context layout:

- `GLOSSARY.md` at the repo root — the project's canonical vocabulary
- `docs/adr/` — architecture decision records, numbered `NNNN-slug.md`

## Consumer rules

- Use `GLOSSARY.md` terms when writing tickets, specs, and commit messages; if a term conflicts with how the code names things, surface the conflict instead of silently picking one
- Before proposing a design that contradicts an ADR, read the ADR's reasoning first — the decision may be a deliberate deviation; if circumstances changed, supersede the ADR rather than ignoring it
- Record new hard-to-reverse decisions as ADRs when made, not after the fact
