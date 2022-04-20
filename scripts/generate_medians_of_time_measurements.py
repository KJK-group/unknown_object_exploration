import pandas as pd

median_length = 10

df = pd.read_csv("../measurements/perf.csv")


if len(df) % median_length != 0:
    df = df[:median_length * len(df) % median_length]

pd.da
