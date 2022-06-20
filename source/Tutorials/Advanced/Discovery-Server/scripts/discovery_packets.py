#!/usr/bin/python3 discovery_server

"""Script to count number of rtps packages in a tcpdump capture."""
import os
import subprocess
import pandas as pd

import matplotlib.pyplot as plt


def plot_packets(data):
    """Plot a graph with the result obtained."""
    ax = data.plot.bar(x='discovery_protocol', y='discovery_packets', rot=0)
    plt.title('Packet traffic during discovery')
    plt.xlabel('Discovery Protocol')
    plt.ylabel('Number of packets')
    plt.legend().remove()
    # plt.show()
    plt.savefig('discovery_packets.png')
    plt.close()


def count_packets(filename):
    """Count the discovery packets filtering them by Entity id."""
    print('Processing {}'.format(filename))
    command = 'tshark -r {} -Y'.format(filename)
    command += ' "rtps && ('
    command += ' (rtps.sm.rdEntityId == 0x000002c2) ||'
    command += ' (rtps.sm.rdEntityId == 0x000002c7) ||'
    command += ' (rtps.sm.rdEntityId == 0x000003c2) ||'
    command += ' (rtps.sm.rdEntityId == 0x000003c7) ||'
    command += ' (rtps.sm.rdEntityId == 0x000004c2) ||'
    command += ' (rtps.sm.rdEntityId == 0x000004c7) ||'
    command += ' (rtps.sm.rdEntityId == 0x000100c2) ||'
    command += ' (rtps.sm.rdEntityId == 0x000100c7) ||'
    command += ' (rtps.sm.rdEntityId == 0x000200C2) ||'
    command += ' (rtps.sm.rdEntityId == 0x000200C7) ||'
    command += ' (rtps.sm.rdEntityId == 0x000201C3) ||'
    command += ' (rtps.sm.rdEntityId == 0x000201C4))" | wc -l'

    res = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
    n_packets = int(res.communicate()[0].decode().rstrip())
    return n_packets


if __name__ == '__main__':

    files = [
        'simple.pcapng',
        'discovery_server.pcapng',
    ]
    data = pd.DataFrame()
    protocols = []
    packets = []
    for f in files:
        if not os.path.isfile(f):
            print('Cannot find file {}'.format(f))
            continue

        protocol = 'Simple'
        if 'server' in f:
            protocol = 'Discovery Server'

        protocols.append(protocol)
        packets.append(count_packets(f))

    data = pd.DataFrame(
        {
            'discovery_protocol': protocols,
            'discovery_packets': packets
        }
    )

    print(data)
    plot_packets(data)
