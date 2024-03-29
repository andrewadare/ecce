#!/usr/bin/env python
"""Do `pip install plotly kaleido` to get plotly packages"""

from time import sleep
from typing import Optional

from plotly.offline import plot
import plotly.graph_objects as go


def draw_factor_errors(
    d0: dict[str, float],
    df: dict[str, float],
    title: str,
    filename: Optional[str] = None,
    display: bool = True,  # in browser tab
):
    data = [
        go.Bar(
            x=list(d0.keys()),
            y=list(d0.values()),
            name="Initial estimate",
            marker=dict(
                color="rgba(58, 71, 80, 0.6)",
                line=dict(color="rgba(58, 71, 80, 1.0)", width=3),
            ),
        ),
        go.Bar(
            x=list(df.keys()),
            y=list(df.values()),
            name="After fit",
            marker=dict(
                color="rgba(255, 0, 0, 0.6)",
                line=dict(color="rgba(255, 0, 0, 1.0)", width=3),
            ),
        ),
    ]
    layout = go.Layout(
        barmode="overlay",
        title=title,
        yaxis=dict(title="Projection factor error"),
    )

    fig = go.Figure(data, layout)

    if filename:
        plot(fig, filename=filename, auto_open=display)

        # Work around a bug
        # https://github.com/plotly/plotly.py/issues/3469
        fig.write_image(filename.replace(".html", ".pdf"))
        sleep(1)
        fig.write_image(filename.replace(".html", ".pdf"))
    else:
        plot(fig)


if __name__ == "__main__":
    initial_errors = dict()
    final_errors = dict()
    with open("initial-errors.txt") as f:
        for line in f.readlines():
            cam, tag, err = line.split()
            initial_errors[f"{cam}->{tag}"] = float(err)
    with open("final-errors.txt") as f:
        for line in f.readlines():
            cam, tag, err = line.split()
            final_errors[f"{cam}->{tag}"] = float(err)

    draw_factor_errors(
        initial_errors,
        final_errors,
        "Factor errors",
        "factor-errors.html",
        display=False,
    )
