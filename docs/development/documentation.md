# Documentation

The site uses [Zensical](https://zensical.org/), configured in `zensical.toml`
at the repository root. Markdown pages and media live under `docs/`.

## Preview and build

Install the documentation tool in a dedicated environment:

```bash
python3 -m venv /tmp/svea-docs-venv
source /tmp/svea-docs-venv/bin/activate
python3 -m pip install zensical
zensical serve
```

To validate the site:

```bash
zensical build --clean --strict
```

The generated site is written to `site/`; do not edit that output. If serving
from a container, explicitly expose port 8000; `util/run` does not publish it.

## Editing pages

Add or update Markdown files in `docs/`. The `[project]` section of
`zensical.toml` defines `nav` as nested TOML tables mapping titles to paths
relative to `docs/`. Keep navigation and relative Markdown links valid.
Use fenced code blocks with a language, and verify commands against the
actual launch filenames, package metadata, and scripts.

The current configuration does not enable `mkdocstrings` API generation.
Use explanatory text and links to source files rather than literal `:::`
autodoc directives. The reference pages identify the actual import paths.

## Deployment

The `.github/workflows/docs.yml` workflow builds Zensical and deploys the
result through GitHub Pages on pushes to `main` or `master`. A repository guard
restricts deployment to `KTH-SML/svea`. Submit documentation through a pull
request.
