#!/usr/bin/env python3
"""
Check that confgenerator.c/.h agree with a VESC Tool parameter XML.

confgenerator.c is generated output, and nothing in a build detects a
disagreement between it and the XML it came from. A wrong signature is rejected
silently by the firmware, and a field order that differs by so much as one
position corrupts every field after that point while still loading.

    checkconf.py <bldc dir> <vesc_tool config dir>

Exits non-zero on any disagreement.
"""

import re
import sys
import os

from confsig import signature


def fw_signature(header, which):
    """Read a *CONF_SIGNATURE out of confgenerator.h."""
    text = open(header).read()
    m = re.search(r'#define\s+%s_SIGNATURE\s+(\d+)' % which, text)
    if not m:
        raise SystemExit('%s: no %s_SIGNATURE' % (header, which))
    return int(m.group(1))


def fw_order(source, func):
    """Field order as serialised by confgenerator.c, in C member syntax."""
    text = open(source).read()
    m = re.search(r'int32_t %s\([^)]*\)\s*\{(.*?)\n\}' % func, text, re.S)
    if not m:
        raise SystemExit('%s: no %s' % (source, func))
    # Skip the signature word itself, which is not a config field.
    body = m.group(1)
    return re.findall(r'conf->([A-Za-z0-9_.\[\]]+)', body)


def canon(name):
    """XML uses name__0 where the C source uses name[0]."""
    name = re.sub(r'__(\d+)$', r'[\1]', name)
    return name


def check(bldc, cfgdir):
    ok = True

    for which, xml_name, func in (
            ('APPCONF', 'parameters_appconf.xml', 'confgenerator_serialize_appconf'),
            ('MCCONF', 'parameters_mcconf.xml', 'confgenerator_serialize_mcconf')):

        xml = os.path.join(cfgdir, xml_name)
        sig, _ = signature(xml)
        fw_sig = fw_signature(os.path.join(bldc, 'confgenerator.h'), which)

        if sig == fw_sig:
            print('%s signature: %d  OK' % (which, sig))
        else:
            print('%s signature: firmware has %d, XML gives %d  MISMATCH'
                  % (which, fw_sig, sig))
            ok = False

        import xml.etree.ElementTree as ET
        root = ET.parse(xml).getroot()
        xml_fields = [canon((s.text or '').strip())
                      for s in root.find('SerOrder').findall('ser')]
        c_fields = fw_order(os.path.join(bldc, 'confgenerator.c'), func)

        if xml_fields == c_fields:
            print('%s field order: %d fields  OK' % (which, len(c_fields)))
        else:
            ok = False
            print('%s field order: MISMATCH (%d in XML, %d in confgenerator.c)'
                  % (which, len(xml_fields), len(c_fields)))
            for i in range(max(len(xml_fields), len(c_fields))):
                x = xml_fields[i] if i < len(xml_fields) else '<none>'
                c = c_fields[i] if i < len(c_fields) else '<none>'
                if x != c:
                    print('  first difference at %d: XML %s, confgenerator.c %s'
                          % (i, x, c))
                    break

    return ok


if __name__ == '__main__':
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    sys.exit(0 if check(sys.argv[1], sys.argv[2]) else 1)
