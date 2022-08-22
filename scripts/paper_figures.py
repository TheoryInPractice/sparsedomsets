#!/usr/bin/env python3

import os
import pandas as pd
from collections import defaultdict
from math import inf
import easy_plotly as ep

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
DATA_DIR = f'{PROJECT_DIR}/data'
OUTPUT_DIR = f'{SCRIPT_DIR}/out'

# DATA_DIR = ''  # edit if necessary


def load_instance_names(path):
    with open(path) as f:
        return [tuple(line.strip().split('/', 1)) for line in f]


def get_runtime(runtime_df, dataset, radius, algorithm):
    rows = runtime_df[(runtime_df['dataset'] == dataset) & (runtime_df['radius'] == radius) & (runtime_df['algorithm'] == algorithm)]['running time']
    if len(rows) == 1:
        return rows.iloc[0]
    else:
        return 0


def is_timeout(runtime_df, small_instances, dataset, radius, algorithm):
    timeout = 10800
    if algorithm in ['mds', 'mac'] and (radius > 1 or dataset not in [d for d, _ in small_instances]):
        timeout = 3600

    return get_runtime(runtime_df, dataset, radius, algorithm) > timeout


def average(xs):
    return sum(xs) / len(xs) if xs else 0


def create_runtime_df(all_instances):
    df = pd.DataFrame()

    for dataset, _ in all_instances:
        for alg in ['qp', 'sgc', 'weight', 'layer', 'branch']:
            path = f'{DATA_DIR}/stage6/{dataset}/nbrprt_{alg}_runtime.csv'
            if os.path.exists(path):
                df = pd.concat([df, pd.read_csv(path)])

    # take average
    raw_data = []
    d = defaultdict(list)
    for _, row in df.iterrows():
        tokens = row['domset'].split('.', 1)[0].split('_')
        r = int(tokens[4][1:])
        dalg = tokens[5][:4] if tokens[5].startswith('g') else tokens[5]
        dseed = int(tokens[5][5:]) if tokens[5].startswith('g') else 0
        palg = row['algorithm']
        pseed = row['seed']
        dataset = row['dataset']
        runtime = row['running time']
        key = (dataset, r, dalg, palg)
        raw_data += [(dataset, r, dalg, dseed, palg, pseed, runtime)]
        d[key] += [runtime]

    # create record
    rec = []
    for k, xs in d.items():
        rec += [(k[0], k[1], k[2], k[3], average(xs), sum(1 if x > 3600 else 0 for x in xs), len(xs))]

    raw = pd.DataFrame.from_records(raw_data, columns=['dataset', 'radius', 'dalg', 'dseed', 'palg', 'pseed', 'running time'])
    agg = pd.DataFrame.from_records(rec, columns=['dataset', 'radius', 'dalg', 'palg', 'running time', 'timeouts', 'samples'])
    return raw, agg


def load_domset_quality(runtime_df, all_instances, small_instances):
    records = []

    for dataset, _ in all_instances:
        d = defaultdict(list)
        df = pd.read_csv(f'{DATA_DIR}/stage5/{dataset}/domset_stats.csv')
        n = 0

        for index, row in df.iterrows():
            if row['solsize'] == 0:
                # timeout (could not find a solution)
                continue

            n = row['n']
            tokens = row['domset'].split('.', 1)[0].split('_')
            r = int(tokens[4][1:])
            alg = tokens[5][:4] if tokens[5].startswith('g') else tokens[5]

            if alg not in ['sgc', 'g10n', 'g12n', 'g20n', 'g21n', 'mds', 'mac']:
                continue

            if is_timeout(runtime_df, small_instances, dataset, r, alg):
                continue

            d[(r, alg)] += [(row['solsize'], row['congsum'] / row['n'])]

        # take average
        for (r, alg), vs in d.items():
            avg_solsize = sum(v[0] for v in vs) / len(vs)
            avg_avgcong = sum(v[1] for v in vs) / len(vs)
            records += [(dataset, n, r, alg, avg_solsize, avg_avgcong)]

    return pd.DataFrame.from_records(records, columns=['dataset', 'n', 'radius', 'algorithm', 'solsize', 'avgcong'])


def get_dalg_name(dalg):
    return {'sgc': 'sgc-d', 'g10n': 'deg', 'g12n': 'deg+', 'g20n': 'ratio', 'g21n': 'ratio+'}.get(dalg, dalg)


def get_alg_name2(alg):
    return alg.replace('-', ', ').replace(', sgc', ', sgc-p').replace('sgc,', 'sgc-d,').replace('degree', 'deg').replace('weight', 'wt')


def create_figure_1(runtime_df, small_instances, closure_size, output_path):
    fig = ep.PlotlyFigure(f'{SCRIPT_DIR}/paper_figure_1.yml')

    # traces
    for i, alg in enumerate(['sgc', 'g12n', 'g21n', 'mds', 'mac']):
        xs = []
        ys = []

        for _, row in runtime_df[runtime_df['algorithm'] == alg].iterrows():
            dataset = row['dataset']
            radius = row['radius']
            runtime = row['running time']
            runtime = min(runtime, 3600 if alg in ['mds', 'mac'] and (radius > 1 or dataset not in [d for d, _ in small_instances]) else 10800)
            xs += [closure_size[(dataset, radius)]]
            ys += [runtime]

        fig.add_scatter(
            x=xs,
            y=ys,
            name=get_dalg_name(alg),
            template=i
        )

    # horizontal lines
    fig.fig.add_hline(y=3600, line_width=2, line_dash='dot', line_color='#333333', layer='below')
    fig.fig.add_hline(y=10800, line_width=2, line_dash='dot', line_color='#333333', layer='below')

    fig.save(output_path)
    print(f'Created: {output_path}')


def create_figure_2(runtime_df, small_instances, quality_df, output_path):
    best_known_solsize = defaultdict(lambda: inf)
    best_known_avgcong = defaultdict(lambda: inf)

    for _, row in quality_df.iterrows():
        dataset = row['dataset']
        r = row['radius']
        solsize = row['solsize']
        avgcong = row['avgcong']
        best_known_solsize[(dataset, r)] = min(best_known_solsize[(dataset, r)], solsize)
        best_known_avgcong[(dataset, r)] = min(best_known_avgcong[(dataset, r)], avgcong)

    fig = ep.PlotlyFigure(f'{SCRIPT_DIR}/paper_figure_2.yml')

    # traces
    for i, alg in enumerate(['sgc', 'g10n', 'g12n', 'g20n', 'g21n', 'mds', 'mac']):
        xs = []
        ys = []

        for _, row in quality_df[(quality_df['algorithm'] == alg)].iterrows():
            dataset = row['dataset']
            r = row['radius']

            if is_timeout(runtime_df, small_instances, dataset, r, alg) or dataset.startswith('soc-'):
                continue

            solsize = row['solsize']
            avgcong = row['avgcong']

            xs += [solsize / best_known_solsize[(dataset, r)]]
            ys += [avgcong / best_known_avgcong[(dataset, r)]]

        fig.add_scatter(
            x=xs,
            y=ys,
            name=get_dalg_name(alg),
            template=i
        )

    fig.save(output_path)
    print(f'Created: {output_path}')


def create_figure_3(part_runtime_df, part_stats_df, large_instances, output_path):
    fig = ep.PlotlyFigure(f'{SCRIPT_DIR}/paper_figure_3.yml')

    # traces
    palg = 'weight'  # fix part alg

    for r in [1, 2, 3, 5, 10]:
        for j, dalg in enumerate(['sgc', 'g12n', 'g21n']):
            ys = []
            ds = []

            for _, row in part_stats_df[(part_stats_df['partitioning algorithm'] == palg) & (part_stats_df['domset algorithm'].isin([dalg, dalg + 's1', dalg + 's2', dalg + 's3']))].iterrows():
                dataset = row['dataset']
                radius = row['radius']
                if radius != r:
                    continue

                if dataset in [xx[0] for xx in large_instances]:
                    # use only small and medium
                    continue

                dseed = 0 if dalg == 'sgc' else int(row['domset algorithm'][-1])

                runtime = part_runtime_df[
                    (part_runtime_df['dalg'] == dalg) &
                    (part_runtime_df['dseed'] == dseed) &
                    (part_runtime_df['palg'] == palg) &
                    (part_runtime_df['dataset'] == dataset) &
                    (part_runtime_df['radius'] == radius)
                ]['running time'].iloc[0]
                if runtime > 3600:
                    continue

                # size_mean = row['size: mean']
                size_dev = row['size: std dev']

                ys += [size_dev]
                ds += [dataset]

            fig.add_box(
                x=[[r'$r=' + str(r) + r'$' for _ in ys], [get_dalg_name(dalg) for _ in ys]],
                y=ys,
                name=f'{get_dalg_name(dalg)}, r={r}',
                text=ds,
                template=j,
            )

    fig.save(output_path)
    print(f'Created: {output_path}')


def create_figure_4(part_runtime_df, part_stats_df, large_instances, output_path):
    fig = ep.PlotlyFigure(f'{SCRIPT_DIR}/paper_figure_4.yml')

    # traces
    dalg = 'g21n'  # fix domset

    for r in [1, 2, 3, 5, 10]:
        for j, palg in enumerate(['sgc', 'weight', 'layer', 'qp']):
            ys = []
            ds = []

            for _, row in part_stats_df[(part_stats_df['partitioning algorithm'] == palg) & (part_stats_df['domset algorithm'].isin([dalg, dalg + 's1', dalg + 's2', dalg + 's3']))].iterrows():
                dataset = row['dataset']
                radius = row['radius']
                if radius != r:
                    continue

                if dataset in [xx[0] for xx in large_instances]:
                    # use only small and medium
                    continue

                dseed = int(row['domset algorithm'][-1])

                runtime = part_runtime_df[
                    (part_runtime_df['dalg'] == dalg) &
                    (part_runtime_df['dseed'] == dseed) &
                    (part_runtime_df['palg'] == palg) &
                    (part_runtime_df['dataset'] == dataset) &
                    (part_runtime_df['radius'] == radius)
                ]['running time'].iloc[0]
                if runtime > 3600:
                    continue

                # size_mean = row['size: mean']
                size_dev = row['size: std dev']

                ys += [size_dev]  # / (100 if best_dev[(dataset, radius)] == 0 else best_dev[(dataset, radius)])]
                ds += [dataset]

            fig.add_box(
                x=[[r'$r=' + str(r) + r'$' for _ in ys], [{'sgc': 'sgc-p', 'weight': 'wt', 'qp': 'exact'}.get(palg, palg) for _ in ys]],
                y=ys,
                name=f'{"sgc-p" if palg == "sgc" else palg}, r={r}',
                text=ds,
                template=j,
            )

    fig.save(output_path)
    print(f'Created: {output_path}')


def create_figure_5(dfs, algs, output_path):
    fig = ep.PlotlyFigure(f'{SCRIPT_DIR}/paper_figure_5.yml')

    for i, alg in enumerate(algs):
        if alg not in dfs:
            continue

        fig.add_scatter(
            x=dfs[alg]['size'],
            y=dfs[alg]['frequency'],
            name=get_alg_name2(alg),
            template=i,
        )

    fig.save(output_path)
    print(f'Created: {output_path}')


def create_figure_6(dfs, algs, output_path):
    fig = ep.PlotlyFigure(f'{SCRIPT_DIR}/paper_figure_6.yml')

    for i, alg in enumerate(algs):
        if alg not in dfs:
            continue

        fig.add_scatter(
            x=dfs[alg]['size'],
            y=dfs[alg]['frequency'],
            name=get_alg_name2(alg),
            template=i,
        )

    fig.save(output_path)
    print(f'Created: {output_path}')


def create_figure_7(dfss, algs, output_path):
    fig = ep.PlotlyFigure(f'{SCRIPT_DIR}/paper_figure_7.yml')

    for r in [1, 2]:
        for i, alg in enumerate(algs):
            ys = dfss[r - 1][alg]['similarity']

            fig.add_box(
                x=[[r'$r=' + str(r) + r'$' for _ in ys], [get_alg_name2(alg) for _ in ys]],
                y=ys,
                name=f'{alg}, r={r}',
                text=f'{alg}, r={r}',
                template=i,
            )

    fig.save(output_path)
    print(f'Created: {output_path}')


def main():
    small_instances = load_instance_names(f'{SCRIPT_DIR}/target_small.txt')
    medium_instances = load_instance_names(f'{SCRIPT_DIR}/target_medium.txt')
    large_instances = load_instance_names(f'{SCRIPT_DIR}/target_large.txt')
    all_instances = small_instances + medium_instances + large_instances

    # load data
    runtime_df = pd.DataFrame()

    for dataset, _ in all_instances:
        runtime_df = pd.concat([runtime_df, pd.read_csv(f'{DATA_DIR}/stage5/{dataset}/domset_runtime.csv')])
        runtime_df = pd.concat([runtime_df, pd.read_csv(f'{DATA_DIR}/stage5/{dataset}/domset_sgc_runtime.csv')])
        runtime_df = pd.concat([runtime_df, pd.read_csv(f'{DATA_DIR}/stage5/{dataset}/domset_ilp_runtime.csv')])

    part_runtime_df, part_runtime_df_agg = create_runtime_df(all_instances)
    part_stats_df = pd.concat([pd.read_csv(f'{DATA_DIR}/stage6/{dataset}/part_stats.csv') for dataset, _ in all_instances])

    # compute closure sizes
    closure_size = {}
    for dataset, _ in all_instances:
        df = pd.read_csv(f'{DATA_DIR}/stage4/{dataset}/closure_size.csv')
        for _, row in df.iterrows():
            closure_size[(dataset, row['r'])] = row['m']

    quality_df = load_domset_quality(runtime_df, all_instances, small_instances)

    # hu data
    algs = ['sgc-sgc', 'sgc-weight', 'degree-sgc', 'degree-weight', 'ratio-sgc', 'ratio-weight']

    freq_r1_df = {}
    result_r1_df = {}
    freq_r2_df = {}
    result_r2_df = {}

    for alg in algs:
        if os.path.exists(f'{DATA_DIR}/stage7/hu-r1-{alg}/freq.csv'):
            freq_r1_df[alg] = pd.read_csv(f'{DATA_DIR}/stage7/hu-r1-{alg}/freq.csv')
            result_r1_df[alg] = pd.read_csv(f'{DATA_DIR}/stage7/hu-r1-{alg}/results.csv')

    for alg in algs:
        if os.path.exists(f'{DATA_DIR}/stage7/hu-r2-{alg}/freq.csv'):
            freq_r2_df[alg] = pd.read_csv(f'{DATA_DIR}/stage7/hu-r2-{alg}/freq.csv')
            result_r2_df[alg] = pd.read_csv(f'{DATA_DIR}/stage7/hu-r2-{alg}/results.csv')

    # create figures
    os.mkdir(OUTPUT_DIR)
    create_figure_1(runtime_df, small_instances, closure_size, f'{OUTPUT_DIR}/paper_figure_1.svg')
    create_figure_2(runtime_df, small_instances, quality_df, f'{OUTPUT_DIR}/paper_figure_2.svg')
    create_figure_3(part_runtime_df, part_stats_df, large_instances, f'{OUTPUT_DIR}/paper_figure_3.svg')
    create_figure_4(part_runtime_df, part_stats_df, large_instances, f'{OUTPUT_DIR}/paper_figure_4.svg')
    create_figure_5(freq_r1_df, algs, f'{OUTPUT_DIR}/paper_figure_5.svg')
    create_figure_6(freq_r2_df, algs, f'{OUTPUT_DIR}/paper_figure_6.svg')
    create_figure_7([result_r1_df, result_r2_df], algs, f'{OUTPUT_DIR}/paper_figure_7.svg')


if __name__ == '__main__':
    main()
