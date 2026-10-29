# Contributing

Keep each commit focused on one change. Separate unrelated documentation,
bug fixes and features so they can be reviewed independently.

## Commit messages

Use [Conventional Commits 1.0.0](https://www.conventionalcommits.org/en/v1.0.0/):

```text
type(scope): short description
```

The scope is optional. Use lowercase types:

- `feat`: new functionality.
- `fix`: a bug fix.
- `docs`: documentation changes.
- `refactor`, `test`, `build`, `ci`, `chore`: restructuring, tests, build configuration, automation or maintenance.

Examples:

```text
docs(rc): clarify ESC startup
fix(localization): correct odometry topic
```

Mark breaking changes with `!` before the colon or a `BREAKING CHANGE:` footer,
and explain the compatibility impact.

## Pull requests

Use the same title format. Briefly describe what changed, how you checked it,
and any remaining limitations.
