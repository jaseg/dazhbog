#!/usr/bin/env python3

TIMER_FREQ = 30e6 # MHz


with open('main.c') as f:
    lines = f.readlines()

defs = {}
for line in lines:
    if line.startswith('#define'):
        _pragma, name, val, *_comment = line.split()
        val = defs.get(val, val)
        defs[name] = val

print('Bit cycle timings:')
timings_total = 0
in_array = False
for line in lines:
    if not in_array:
        if line.startswith('static uint16_t timer_period_lookup'):
            in_array = True
    else:
        if '}' in line:
            break
        if ',' not in line:
            continue
        val, *_comment = line.split(',')
        for name, defval in defs.items():
            val = val.replace(name, defval)
        duration = eval(val)
        print(duration)
        timings_total += duration + int(defs['RESET_PERIOD_LENGTH'])

total_len = timings_total/TIMER_FREQ
print('Total cycles:', timings_total)
print('Total cycle length: {:.3f}ms'.format(total_len*1e3))
print('Frame rate: {:.3f}Hz'.format(1/total_len))
