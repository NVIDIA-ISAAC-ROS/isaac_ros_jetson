# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#
# SPDX-License-Identifier: MIT

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


TEMPERATURE_MAX = 84
TEMPERATURE_CRIT = 100


def size_min(num, divider=1.0, n=0, start=''):
    return find_unit(num, 1024.0, divider, n, start)


def unit_min(num, divider=1.0, n=0, start=''):
    return find_unit(num, 1000.0, divider, n, start)


def find_unit(size, power, divider=1.0, n=0, start=''):
    n = 0
    power_labels = ['m', '', 'k', 'M', 'G', 'T']
    while size > power - 1:
        divider *= power
        size /= power
        n += 1
    idx = power_labels.index(start)
    return round(size, 1), divider, power_labels[n + idx]


def size_to_string(value, unit):
    return value_to_string(value, unit, '', size_min)


def unit_to_string(value, unit, system_unit):
    return value_to_string(value, unit, system_unit, unit_min)


def value_to_string(value, unit, system_unit, func):
    value, _, unit = func(value, start=unit)
    value_string = str(value)
    if value >= 100:
        value_string = value_string[:3].rstrip('.')
    return f'{value_string}{unit}{system_unit}'


def strfdelta(tdelta, fmt):
    d = {'days': tdelta.days}
    d['hours'], rem = divmod(tdelta.seconds, 3600)
    d['minutes'], d['seconds'] = divmod(rem, 60)
    return fmt.format(**d)


def other_status(hardware, jetson, version):
    values = []
    nvpmodel = jetson.nvpmodel
    text = ''
    if nvpmodel is not None:
        nvp_name = nvpmodel.name
        nvpmodel_id = nvpmodel.id
        values.append(KeyValue(key='NV Power-ID', value=str(nvpmodel.id)))
        values.append(KeyValue(key='NV Power-Mode', value=nvp_name))
        text += f'NV Power[{nvpmodel_id}] {nvp_name}'
    jc = jetson.jetson_clocks
    level = DiagnosticStatus.OK
    if jetson.jetson_clocks is not None:
        if jetson.jetson_clocks.status in ['running', 'inactive']:
            level = DiagnosticStatus.OK
        elif 'ing' in jc.status:
            level = DiagnosticStatus.WARN
        else:
            level = DiagnosticStatus.ERROR
        # Show if JetsonClock is enabled or not
        values.append(KeyValue(key='jetson_clocks', value=str(jc.status)))
        values.append(KeyValue(key='jetson_clocks on boot', value=str(jc.boot)))
        text += f' - JC {jc.status}'
    # Uptime
    uptime_string = strfdelta(
        jetson.uptime, '{days} days {hours}:{minutes}:{seconds}')
    values.append(KeyValue(key='Up Time', value=str(uptime_string)))
    # Jtop version
    values.append(KeyValue(key='interval', value=str(jetson.interval)))
    values.append(KeyValue(key='jtop', value=str(version)))
    # Make board diagnostic status
    status = DiagnosticStatus(
        level=level,
        name='jetson_stats/board/Status',
        message=text,
        hardware_id=hardware,
        values=values)
    return status


def board_status(hardware, board, dgtype):
    if board['platform']['Machine'] == 'x86_64':
        platform = board['platform']
        system = platform['System'].upper()
        machine = platform['Machine']
        distribution = platform['Distribution']
        release = platform['Release'].split('-')[0]
        message = f'{system} {machine} machine - {distribution} [{release}]'
        level = DiagnosticStatus.OK
    elif 'L4T' in board['hardware']:
        model = board['hardware']['Model']
        jetpack = board['hardware']['Jetpack']
        message = f'{model} - Jetpack {jetpack}'
        level = DiagnosticStatus.OK
    else:
        message = 'Unrecognized hardware'
        level = DiagnosticStatus.WARN
    values = []
    for key, value in board['hardware'].items():
        values.append(KeyValue(key=key, value=str(value)))
    for key, value in board['libraries'].items():
        values.append(KeyValue(key='lib ' + key, value=str(value)))
    # Make board diagnostic status
    d_board = DiagnosticStatus(
        level=level,
        name=f'jetson_stats/{dgtype}/Config',
        message=message,
        hardware_id=hardware,
        values=values)
    return d_board


def disk_status(hardware, disk, dgtype):
    value = int(float(disk['used']) / float(disk['total']) * 100.0)
    used = size_to_string(disk['used'], disk['unit'])
    total = size_to_string(disk['total'], disk['unit'])
    # Set level status
    if value >= 90:
        level = DiagnosticStatus.ERROR
    elif value >= 70:
        level = DiagnosticStatus.WARN
    else:
        level = DiagnosticStatus.OK
    # Make board diagnostic status
    d_board = DiagnosticStatus(
        level=level,
        name=f'jetson_stats/{dgtype}/Disk',
        message=f'{used}/{total}',
        hardware_id=hardware,
        values=[
            KeyValue(key='Used', value=used),
            KeyValue(key='Total', value=total)
        ])
    return d_board


def cpu_status(hardware, name, cpu):
    message = 'OFF'
    values = []
    if cpu:
        if 'idle' in cpu:
            # Decode utilizaiton status
            val = 100 - cpu['idle']
            freq_cpu = unit_to_string(cpu['freq']['cur'], 'k', 'Hz')
            # Make Diagnostic Status message with cpu info
            values = [
                KeyValue(key='Idle', value=f"{cpu['idle']:6.2f}%"),
                KeyValue(key='User', value=f"{cpu['user']:6.2f}%"),
                KeyValue(key='Nice', value=f"{cpu['nice']:6.2f}%"),
                KeyValue(key='System', value=f"{cpu['system']:6.2f}%"),
                KeyValue(key='Governor', value=cpu['governor']),
                KeyValue(key='Freq', value=freq_cpu)]
            # Update message
            message = f'{val:6.2f}%'
        if 'model' in cpu and cpu['model']:
            values.append(KeyValue(key='Model', value=cpu['model']))

    # Build diagnostic message
    d_cpu = DiagnosticStatus(
        name=f'jetson_stats/cpu/{name}',
        message=message,
        hardware_id=hardware,
        values=values)
    return d_cpu


def gpu_status(hardware, name, gpu):
    gpu_load = gpu['status']['load']
    used_gpu = f'{gpu_load:6.2f}%'
    freq_gpu = unit_to_string(gpu['freq']['cur'], 'k', 'Hz')
    railgate_string = 'Active' if gpu['status']['railgate'] else 'Disable'
    scaling_string = 'Active' if gpu['status']['3d_scaling'] else 'Disable'
    # Build diagnostic message
    values = [
        KeyValue(key='Used', value=used_gpu),
        KeyValue(key='Freq', value=freq_gpu),
        KeyValue(key='Railgate', value=railgate_string),
        KeyValue(key='3D scaling', value=scaling_string),
        KeyValue(key='governor', value=gpu['freq']['governor']),
    ]

    if 'GPC' in gpu['freq']:
        for i, gpc in enumerate(gpu['freq']['GPC']):
            values.append(KeyValue(key=f'GPC {i}', value=f'{gpc:6.2f}%'))

    if 'power_control' in gpu and gpu['power_control']:
        values.append(KeyValue(key='Power Control', value=gpu['power_control']))

    d_gpu = DiagnosticStatus(
        name=f'jetson_stats/gpu/{name}',
        message=used_gpu,
        hardware_id=hardware,
        values=values)
    return d_gpu


def fan_status(hardware, name, fan):
    values = []
    # List of all speeds
    for idx, speed in enumerate(fan['speed']):
        values.append(KeyValue(key=f'PWM {idx}', value=f'{speed: >3.0f}%'))
        if 'rpm' in fan:
            rpm_fan = fan['rpm'][idx]
            values.append(KeyValue(key=f'RPM {idx}', value=f'{rpm_fan}RPM'))
    values.append(KeyValue(key='Profile', value=fan['profile']))
    first_fan_speed = fan['speed'][0]
    message = 'Fan0: ' if len(fan['speed']) > 1 else ''
    message += f'{first_fan_speed: >3.0f}%'
    # Make fan diagnostic status
    d_fan = DiagnosticStatus(
        name=f'jetson_stats/fan/{name}',
        message=message,
        hardware_id=hardware,
        values=values)
    return d_fan


def ram_status(hardware, ram, dgtype):
    # Build label string
    used = size_to_string(ram['used'], 'k')
    total = size_to_string(ram['tot'], 'k')
    percent = f'{used}/{total}B'
    lfb = ram['lfb']
    label_lfb = f'(lfb {lfb}x4MB)'
    # Make ram diagnostic status
    d_ram = DiagnosticStatus(
        name=f'jetson_stats/{dgtype}/ram',
        message=f'{percent} - {label_lfb}',
        hardware_id=hardware,
        values=[
            KeyValue(key='Use', value=size_to_string(ram['used'], 'k')),
            KeyValue(key='Shared', value=size_to_string(ram['shared'], 'k')),
            KeyValue(key='Buffers', value=size_to_string(ram['buffers'], 'k')),
            KeyValue(key='Cached', value=size_to_string(ram['cached'], 'k')),
            KeyValue(key='Free', value=size_to_string(ram['free'], 'k')),
            KeyValue(key='Total', value=size_to_string(ram['tot'], 'k')),
            KeyValue(key='lfb', value=f'{lfb}x4MB')])
    return d_ram


def swap_status(hardware, swap, dgtype):
    used = size_to_string(swap['used'], 'k')
    total = size_to_string(swap['tot'], 'k')
    cached = size_to_string(swap['cached'], 'k')
    # Build list of keys
    values = [
        KeyValue(key='Use', value=used),
        KeyValue(key='Total', value=total),
        KeyValue(key='Cached', value=cached)
    ]
    for swap_name in swap['table']:
        swap_data = swap['table'][swap_name]
        prio = swap_data['prio']
        used = size_to_string(swap_data['used'], 'k')
        total = size_to_string(swap_data['size'], 'k')
        boot = '- Boot' if swap_data['boot'] else ''
        values.append(KeyValue(key=f'swap {swap_name}', value=f'{prio} - {used}/{total} {boot}'))
    # Make swap diagnostic status
    d_swap = DiagnosticStatus(
        name=f'jetson_stats/{dgtype}/swap',
        message=f'{used}/{total} (Cached {cached})',
        hardware_id=hardware,
        values=values)
    return d_swap


def power_status(hardware, name, power):
    values = []
    # name = name.replace("VDDQ_", "").replace("VDD_", "").replace("_", " ")
    unit_power = unit_to_string(power['power'], 'm', 'W')
    unit_avg = unit_to_string(power['avg'], 'm', 'W')
    # Add values
    if 'volt' in power:
        values.append(KeyValue(key='Volt', value=unit_to_string(power['volt'], 'm', 'W')))
    if 'curr' in power:
        values.append(KeyValue(key='Current', value=unit_to_string(power['curr'], 'm', 'W')))
    values.append(KeyValue(key='Power', value=f'{unit_power}'))
    values.append(KeyValue(key='Average', value=f'{unit_avg}'))
    if 'warn' in power:
        values.append(KeyValue(key='Warning', value=unit_to_string(power['warn'], 'm', 'W')))
    if 'crit' in power:
        values.append(KeyValue(key='Critical', value=unit_to_string(power['crit'], 'm', 'W')))
    # Set status
    level = DiagnosticStatus.OK
    d_volt = DiagnosticStatus(
        level=level,
        name=f'jetson_stats/power/{name}',
        message=f'Power: {unit_power} - Avg: {unit_avg}',
        hardware_id=hardware,
        values=values)
    return d_volt


def temp_status(hardware, name, sensor):
    temperature = sensor['temp']
    # Set color temperature
    max_value = sensor['max'] if 'max' in sensor else TEMPERATURE_MAX
    crit_value = sensor['crit'] if 'crit' in sensor else TEMPERATURE_CRIT
    message = f'{temperature:3.2f}C' if sensor['online'] else 'Offline'
    # Set temperature level
    level = DiagnosticStatus.OK
    if temperature >= crit_value:
        level = DiagnosticStatus.ERROR
        message = f'{temperature:3.2f}C more than {crit_value:3.2f}C'
    elif temperature >= max_value:
        level = DiagnosticStatus.WARN
        message = f'{temperature:3.2f}C more than {max_value:3.2f}C'
    # Build diagnostic message
    d_temp = DiagnosticStatus(
        level=level,
        name=f'jetson_stats/temp/{name}',
        message=message,
        hardware_id=hardware,
        values=[
            KeyValue(key='Warning', value=f'{max_value:3.2f}C'),
            KeyValue(key='Critical', value=f'{crit_value:3.2f}C')
        ])
    return d_temp


def emc_status(hardware, emc, dgtype):
    frequency = unit_to_string(emc['cur'], 'k', 'Hz')
    emc_val = emc['val']
    # Make EMC diagnostic status
    d_emc = DiagnosticStatus(
        name=f'jetson_stats/{dgtype}/emc',
        message=frequency,
        hardware_id=hardware,
        values=[
            KeyValue(key='Freq', value=frequency),
            KeyValue(key='Bandwidth', value=f'{emc_val:6.2f}%')])
    return d_emc


def engine_status(hardware, name_group, engines):
    values = []
    message = ''
    for name, engine in engines.items():
        if engine['online']:
            frequency = unit_to_string(engine['cur'], 'k', 'Hz')
            values.append(KeyValue(key=name, value=frequency))
            message += f'{name}: {frequency} ' if len(engines) > 1 else frequency
        else:
            values.append(KeyValue(key=name, value='Offline'))
            message += f'{name}: Offline ' if len(engines) > 1 else 'Offline'
    d_engine = DiagnosticStatus(
        name=f'jetson_stats/engine/{name_group}',
        message=message,
        hardware_id=hardware,
        values=values)
    return d_engine
# EOF
