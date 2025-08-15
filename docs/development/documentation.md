# Documentation

This project is using [MkDocs](https://www.mkdocs.org/) for documentation.
MkDocs is easy to use and the goal of this guide is to quickly get you
started with writing documentation.

## Installation

```text
pip install -r docs/requirements.txt
```

!!! note

    For the plugin `mkdocstrings-python` it is important that you can import
    the relevant packages. Therefore it is required that we have (catkin) built
    the SVEA project and have sourced the workspace.

## Usage

MkDocs is so easy because it uses a `yml` file for configuration and markdown
for the actual documents.

```text
project/
    mkdocs.yml
    docs/
        document1.md
        document2.md
    ...
```

There is a lot of configuration options in `mkdocs.yml`. The most important
part of this file is the navigation structure. It is possible to add pages,
sections and order them in any way you like. The content of `nav` is a list
or list of lists, where the elements are key-value pairs for each page. The key
corresponds to the page name and the value is a path to the document relative
to `project/docs`.

```yml
# mkdocs.yml

nav:
  - Home: index.md
  - Tutorial:
    - 0. Introduction: tutorials/0_intro.md
    - 1. Floor 2 and Pure-Pursuit: tutorials/1_floor2.md
    - 2. Lidar Simulation: tutorials/2_lidar_sim.md
    - 3. The SVEA: tutorials/3_svea_operation.md
    - 4. Docker: tutorials/4_docker.md
  - Instructions:
    - Documentation: instructions/documentation.md
```

When you write documentation it is useful to see the result in real-time as
you type. Built in to MkDocs is a local "development server". Essentially,
by calling `mkdocs serve` MkDocs will host a local site with hot reloading.

!!! note

    If you are doing this from within a docker container, remember to open
    `port `8000` so you can access the site.

It is equally easy to build the documentation. Simply call `mkdocs build` and
MkDocs will generate the site inside `project/site`.

When you are ready to deploy, you can do so to GitHub Pages very easily. By
calling `mkdocs gh-deploy` everything will be setup and uploaded
automatically.

Use *docstrings* to document code. The plugin `mkdocstrings` will auto-generate
content with the syntax shown below. Follow Google's
[style guide](https://google.github.io/styleguide/pyguide.html) to write the
documentation.

```text
# ref/interfaces.md

::: svea.ActuationInterface

::: svea.LocalizationInterface
```

## Future

#### docstrings

After going through `svea_core` we should add auto-generated docs from
docstrings using `mkdocstrings`. This is what should be done:

1. Adding reference section containing one page per module.
```text
svea/docs/
    instructions/
    tutorials/
    reference/
        actuation.md
        ...
```

2. Each page should contain:
```text
::: svea.actuation
    <options>
```

See more [here](https://mkdocstrings.github.io/).

#### List

Here is a list of interesting things for the future:

- [Popular Material theme](https://squidfunk.github.io/mkdocs-material/)
- [Generate PDF](https://github.com/mkdocs/mkdocs/wiki/MkDocs-Plugins#pdf--site-conversion)
- [MkDocs plugin list](https://github.com/mkdocs/mkdocs/wiki/MkDocs-Plugins)

