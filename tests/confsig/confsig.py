#!/usr/bin/env python3
"""
Compute MCCONF_SIGNATURE / APPCONF_SIGNATURE from a VESC Tool parameter XML.

The firmware rejects a configuration whose signature does not match
(confgenerator.c), and the signature is normally produced by VESC Tool's
ConfigParams::getSignature(). This reimplements it so that a signature can be
derived from the XML and checked without building the tool.

getSignature() concatenates, in serialisation order, each parameter's name, its
type as a decimal, its vTx as a decimal, and every one of its enum names, then
takes CRC32-C over the UTF-8 of that string.

Usage:
    confsig.py <parameters_appconf.xml> [...]
"""

import sys
import xml.etree.ElementTree as ET


def crc32c(data):
    """Utility::crc32c from vesc_tool: reflected Castagnoli, poly 0x82F63B78."""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            mask = -(crc & 1) & 0xFFFFFFFF
            crc = (crc >> 1) ^ (0x82F63B78 & mask)
    return (~crc) & 0xFFFFFFFF


def signature(xml_path, verbose=False):
    root = ET.parse(xml_path).getroot()

    params = root.find('Params')
    ser_order = root.find('SerOrder')
    if params is None or ser_order is None:
        raise SystemExit('%s: missing Params or SerOrder' % xml_path)

    # Parameters are elements named after the C field path.
    by_name = {child.tag: child for child in params}

    parts = []
    for ser in ser_order.findall('ser'):
        name = (ser.text or '').strip()
        parts.append(name)

        p = by_name.get(name)
        if p is None:
            # getSignature() appends the name alone when the parameter is absent.
            if verbose:
                print('  (no param entry) %s' % name)
            continue

        # An absent tag leaves the ConfigParam constructor default, which is
        # CFG_T_UNDEFINED / VESC_TX_UNDEFINED, both zero. 45 of the appconf
        # parameters have no <vTx>, so this is not an edge case.
        ptype = (p.findtext('type') or '0').strip() or '0'
        vtx = (p.findtext('vTx') or '0').strip() or '0'
        parts.append(ptype)
        parts.append(vtx)
        for e in p.findall('enumNames'):
            parts.append(e.text or '')

    sig_str = ''.join(parts)
    return crc32c(sig_str.encode('utf-8')), len(list(ser_order.findall('ser')))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    for path in sys.argv[1:]:
        sig, n = signature(path)
        print('%s\n  fields: %d\n  signature: %d\n' % (path, n, sig))
