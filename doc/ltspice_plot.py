#!/usr/bin/env python3

from matplotlib import pyplot as plt
import numpy as np
import ast
import re
import csv
import math

MULTIPLIERS = {
    'a': 1e-18,
    'f': 1e-15,
    'p': 1e-12,
    'n': 1e-9,
    'u': 1e-6,
    'µ': 1e-6,
    'm': 1e-3,
    'k': 1e3,
    'M': 1e6,
    'G': 1e9,
    'T': 1e12,
    'P': 1e15,
    'E': 1e18,
}

def load_ltspice_csv(filename):
    with open(filename) as f:
        reader = csv.DictReader(f, delimiter='\t')
        fieldnames = reader.fieldnames
        return np.array([ [float(field) for field in line.values()] for line in reader ]), fieldnames

def parse_unit(val, **units):
    for unit, scale in units.items():
        if val.endswith(unit):
            val = val[:-len(unit)]
            break
    else:
        scale = 1.0

    if val[0] == '!':
        val = '-'+val[1:]

    try:
        return float(val)*scale
    except:
        match = re.match(r'(-?[0-9]*(\.[0-9]+)?)([afpnuµmkMGTPE])', val)
        if not match:
            raise ValueError(f'Invalid value: {val}')

        val, _, suffix = match.groups()
        return float(val) * MULTIPLIERS[suffix] * scale

def parse_range(text, sep='-', **units):
    if text:
        start, _, end = text.partition(sep)
        return parse_unit(start, **units), parse_unit(end, **units) if end else math.inf
    else:
        return 0, math.inf

def apply_style(ax):
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_color('#08bdf9')
    ax.spines['left'].set_color('#08bdf9')
    ax.tick_params(axis='x', colors='#01769D')
    ax.tick_params(axis='y', colors='#01769D')
    ax.xaxis.label.set_color('#01769D')
    ax.yaxis.label.set_color('#01769D')
    ax.grid(color='#08bdf9', linestyle=':')


if __name__ == '__main__':
    import argparse
    import os
    parser = argparse.ArgumentParser()
    parser.add_argument('input_txt', action='store', nargs='+', help='LTSpice .txt data export')
    parser.add_argument('-o', '--output', help='Output SVG file. Defaults to <input file name>.svg.', default=None, nargs='?')
    parser.add_argument('-s', '--span', default=None, help='Time span to plot, format: [time][unit]{-[time][unit]}')
    parser.add_argument('-c', '--channels', default=[None], action='store', nargs='*', help='List of channels to plot. Comma-separated 0-based indices or signal names. Use multiple times for vertically-stacked subplots.')
    parser.add_argument('-x', '--xlabel', default='$t\;(\mu s)$', help='Time axis label')
    parser.add_argument('-y', '--ylabel', default=[None]*100, action='store', nargs='*', help='Y axis labels. Use multiple times for subplots.')
    parser.add_argument('-r', '--yrange', default=[None]*100, action='store', nargs='*', help='Value ranges for y axes.  Use multiple times for subplots. Use ! instead of prefix minus sign.')
    parser.add_argument('-t', '--timescale', default='1us', help='Time axis unit')
    parser.add_argument('--subplot-title', default=[None]*100, action='store', nargs='*', help='Subplot titles')
    parser.add_argument('-f', '--figure-size', default='8x6', help='Plot size in [x]x[y] inches')
    args = parser.parse_args()

    start, end = parse_range(args.span, s=1)
    timescale = parse_unit(args.timescale, s=1)

    inputs = []
    for filename in args.input_txt:
        data, fieldnames = load_ltspice_csv(filename)
        data = data[(data[:,0] > start) & (data[:,0] < end)]
        data[:,0] = (data[:,0] - start) / timescale
        inputs.append((data, fieldnames))

    fig, axs = plt.subplots(len(args.channels), 1, squeeze=False, sharex=True, figsize=parse_range(args.figure_size, sep='x'))
    
    for row, (ax, channelspec) in enumerate(zip(axs.flatten(), args.channels)):
        channels = channelspec.split(',') if args.channels else range(0, 1000)

        apply_style(ax)

        n_plotted = 0
        name_plotted = 'V(out)'
        for k, (data, fieldnames) in enumerate(inputs):
            for i, name in enumerate(fieldnames[1:], start=1):
                if not any(x in channels for x in [i, f'{i}', f'{k}:{i}', f'{name}', f'{k}:{name}']):
                    print(f'Not plotting channel {i} "{name}"')
                    continue
                print(f'Plotting channel {i} "{name}"')
                ax.plot(data[:,0], data[:,i], color='#fe3ea0')
                n_plotted += 1
                name_plotted = name

        if args.yrange[row]:
            ax.set_ylim(parse_range(args.yrange[row], A=1, V=1))

        if args.ylabel[row]:
            ax.set_ylabel(args.ylabel[row])
        else: # Guess label
            unit = {'V': 'V', 'I': 'A'}[name_plotted[0]]
            if n_plotted == 1:
                ax.set_ylabel(f'${name_plotted}\;({unit})$')
            else:
                ax.set_ylabel(f'${name_plotted[0]}\;({unit})$')

        if args.subplot_title[row] not in (None, '<none>'):
            ax.set_title(args.subplot_title[row], color='#fe3ea0', fontname='Fredoka One')

        outfile = args.output if args.output else os.path.splitext(args.input_txt[0])[0] + '.svg'
        
    if args.xlabel:
        axs.flatten()[-1].set_xlabel(args.xlabel)
    plt.tight_layout()
    fig.savefig(outfile)
