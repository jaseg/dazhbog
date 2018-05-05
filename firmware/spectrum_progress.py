#!/usr/bin/env python3
import sqlite3
import argparse
import time

import tqdm

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--database', nargs='?', default='spectra.sqlite3')
    parser.add_argument('-u', '--update-delay', nargs='?', type=float, default=1.0)
    parser.add_argument('max_step', nargs='?', type=int, default=250)
    args = parser.parse_args()

    db = sqlite3.connect(args.database)
    def current_step():
        step, = db.execute(
                'SELECT MAX(step) FROM measurements WHERE run_id = (SELECT MAX(run_id) FROM runs)'
                ).fetchone()
        return int(step)

    def step_gen():
        while True:
            step = current_step()
            yield step
            if step >= args.max_step:
                break
            time.sleep(args.update_delay)

    bar = tqdm.tqdm(total=args.max_step)
    while True:
        try:
            for step in step_gen():
                bar.update(step - bar.n)
        except:
            time.sleep(args.update_delay)
