# Isaac ROS from-source build report

A LaTeX summary of the twelve-stage native Isaac ROS build documented in
`docs/source/systems/software/isaac-ros-nitros-source-build.md` and this
directory's parent `README.md`. Covers all twelve stages, the root cause
of each blocked stage, and the full Isaac ROS GEM-repository map.

`isaac_ros_report.pdf` is the built output and is committed directly —
treat it as a snapshot as of the date on its title page, not something
that auto-updates. Regenerate it after future stages land:

```console
cd software/docker/isaac-ros/nitros-source/report
python3 make_charts.py                              # regenerates the 3 chart PDFs
pdflatex -interaction=nonstopmode isaac_ros_report.tex
pdflatex -interaction=nonstopmode isaac_ros_report.tex   # twice, to settle cross-references
```

Requires `texlive-latex-base`, `texlive-latex-extra`, `texlive-pictures`,
and `texlive-fonts-recommended` (for `pgfplots`/`tikz`/`booktabs`), plus
Python's `matplotlib`. The three intermediate chart PDFs
(`stage_status.pdf`, `map_breakdown.pdf`, `engine_build_times.pdf`) and
LaTeX build artifacts (`.aux`/`.log`/`.out`/`.toc`) are gitignored —
only the `.tex` source, the chart-generation script, and the final
`isaac_ros_report.pdf` are tracked.
