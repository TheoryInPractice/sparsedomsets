## How to create paper figures

### Requirements

- [easy-plotly](https://github.com/mogproject/easy-plotly)
- rSVG
  - Mac: `brew install librsvg` using [HomeBrew](https://brew.sh/)

### Instructions

- Run `paper_figures.py`
  - SVG files will be created under the `out` directory.
- Run the following commands.
  - PNG files will be created under the `out` directory.

```bash
rsvg-convert -w 1200 -h 1200 out/paper_figure_1.svg -o out/domset_runtime_scatter.png
rsvg-convert -w 1200 -h 1200 out/paper_figure_2.svg -o out/domset_quality_scatter.png
rsvg-convert -w 940 -h 800 out/paper_figure_3.svg -o out/nbrprt_quality_fixed_alg.png
rsvg-convert -w 1200 -h 800 out/paper_figure_4.svg -o out/nbrprt_quality_fixed_doms.png
rsvg-convert -w 2000 -h 600 out/paper_figure_5.svg -o out/hu_s1_hu_r1_result.png
rsvg-convert -w 2000 -h 600 out/paper_figure_6.svg -o out/hu_s1_hu_r2_result.png
rsvg-convert -w 940 -h 800 out/paper_figure_7.svg -o out/hu_s1_hu_similarity.png
```
