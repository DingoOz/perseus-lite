# CI/CD

## What is CI/CD?

CI/CD (which stands for Continuous Integration, Continuous Delivery/Deployment) is, in theory, exactly what it says on the tin.

### Continuous Integration

Continuous Integration refers to frequently (and automatically) uploading and merging code to the main repository.
This keeps code merges small (reducing the chances and sizes of merge conflicts), and therefore more manageable.
However, that's only one part of code _integration_ - the other is continuous and _automated_ testing.
CI/CD is only made possible by continuous and automated builds and tests which run every time there is code pushed to the main repository.

#### CI testing frameworks

As this is a ROS2 project with C++ and Python nodes, the GoogleTest (gtest) and pytest frameworks are used respectively to create tests that are automatically run.

### Continuous Delivery

Continuous Delivery refers to the project always being _delivered_ in a functional, ready-to-go state, and handles any final stages needed to package the project and get it into a _deployment_ ready state.
Since this project is built with Nix, all dependencies fixed and known ahead-of-time, so there's nothing to do here.

### Continuous Deployment

Continuous Deployment is exactly what it sounds like - automatically _deploying_ a project to production after the continuous _delivery_ process of the pipeline finishes its build.
For this project, we aren't employing continuous deployment for anything but the docs website - see the [documentation system](project:/systems/documentation.md) for details on that.

## Execution

The CI/CD pipeline for this project is run entirely using [GitHub Actions](https://docs.github.com/en/actions).
The typical workflow looks something like this:

```{graphviz}
:caption: Per-push CI pipeline, end-to-end
:align: center

digraph cicd {
    graph [rankdir=LR, bgcolor="transparent", fontname="Roboto",
           nodesep=0.3, ranksep=0.45];
    node  [fontname="Roboto", fontsize=10, style="filled,rounded",
           shape=box, penwidth=1.1, margin="0.18,0.10"];
    edge  [fontname="Roboto", fontsize=9, color="#7a6cad"];

    trigger [label="git push\n/ pull_request", shape=cds, fillcolor="#ec407a", fontcolor="white"];

    subgraph cluster_run {
        label=<<b>GitHub Actions runner</b>>; labeljust="l";
        style="rounded,filled"; color="#3949ab"; fillcolor="#1a237e"; fontcolor="#d6c8ff";

        checkout [label="actions/checkout", fillcolor="#311b92", fontcolor="white"];
        installnix [label="nix-quick-install-action", fillcolor="#311b92", fontcolor="white"];
        cache [label="magic-nix-cache-action\n(read cache)", shape=cylinder, fillcolor="#0277bd", fontcolor="white"];
        update [label="optional\nnix run …\nauto-commit", fillcolor="#ad1457", fontcolor="white", style="rounded,filled,dashed"];
        build [label="nix build / check -L", fillcolor="#5e35b1", fontcolor="white", penwidth=2.0];
        push_cache [label="cachix push\n(scripts.cachix.*)", shape=cylinder, fillcolor="#00838f", fontcolor="white"];
    }

    pass [label="✓ green build", shape=oval, fillcolor="#2e7d32", fontcolor="white"];
    fail [label="✗ failure", shape=oval, fillcolor="#b71c1c", fontcolor="white"];

    trigger    -> checkout;
    checkout   -> installnix -> cache -> update -> build;
    cache      -> build [style=dashed, label="cache hit"];
    build      -> push_cache [label="on success"];
    push_cache -> pass;
    build      -> fail [label="non-zero exit", color="#b71c1c"];
}
```

1. Check out the repo with [`actions/checkout`](https://github.com/actions/checkout)
2. Install Nix with [`nixbuild/nix-quick-install-action`](https://github.com/nixbuild/nix-quick-install-action)
3. Set up Nix output caching with [`DeterminateSystems/magic-nix-cache-action`](https://github.com/DeterminateSystems/magic-nix-cache-action)
4. (Optionally) Run some kind of update with `nix run` and commit it back to the repo
5. Run `nix build -L` (or `check`) on the output - adding the `-L` flag enables logging into the shell and makes it easier to debug when things go wrong
6. If that succeeds, the builds are passing!
7. (Optionally) Upload to cachix using one of the scripts in `software/scripts` (wrapped for execution with `nix run`)

If you're curious about any specific workflow, they're all well commented.
