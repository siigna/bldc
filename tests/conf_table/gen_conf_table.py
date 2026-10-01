#!/usr/bin/env python3
"""
Derive the Lua conf-parameter table from the LispBM extensions, which are the
authoritative mapping of parameter name to configuration field.

Two modes:

  --emit    write script/lua_vesc_conf_table.h
  --check   compare the committed header against the source and fail on any
            difference

The point of --check is that a round-trip test cannot catch a mis-mapping:
set-then-get agrees whether "l-current-max" is wired to l_current_max or to
l_current_min. An independent source of truth is the only thing that does, and
lisp's own chain is one that already exists and is already shipped.
"""

import re
import sys
import os

HERE = os.path.dirname(os.path.abspath(__file__))
TOP = os.path.normpath(os.path.join(HERE, '..', '..'))
LISP = os.path.join(TOP, 'lispBM', 'lispif_vesc_extensions.c')
DATATYPES = os.path.join(TOP, 'datatypes.h')
HEADER = os.path.join(TOP, 'script', 'lua_vesc_conf_table.h')

# Scaled or computed in lisp rather than read straight out of the struct, so
# they are hand-written in lua_vesc_conf.c instead of living in the table.
SPECIAL = {
    'foc-motor-l', 'foc-motor-ld-lq-diff', 'foc-motor-r',
    'foc-motor-flux-linkage', 'foc-observer-gain',
    'min-speed', 'max-speed',
}


def structs(text):
    """Every `typedef struct {...} name;` in the file, as name -> {field: type}."""
    out = {}
    for m in re.finditer(r'typedef struct \{(.*?)\}\s*([A-Za-z_][A-Za-z0-9_]*);',
                         text, re.S):
        body, name = m.group(1), m.group(2)
        fields = {}
        for line in body.split('\n'):
            line = line.split('//')[0].strip()
            mm = re.match(r'^([A-Za-z_][A-Za-z0-9_ ]*?)\s+'
                          r'([A-Za-z_][A-Za-z0-9_]*)\s*(\[[0-9]+\])?\s*;$', line)
            if mm:
                fields[mm.group(2)] = mm.group(1).strip()
        out[name] = fields
    return out


def resolve(path, root, all_structs):
    """Type of a possibly-nested, possibly-indexed field path."""
    cur = root
    for part in path.split('.'):
        base = part.split('[')[0]
        fields = all_structs.get(cur)
        if fields is None or base not in fields:
            return None
        cur = fields[base]
    return cur


def set_transforms(lisp, name_of):
    """
    How conf-set writes each parameter.

    Returned as name -> 'plain' or 'neg_abs'. Anything else is returned as the
    assignment text so the caller can refuse to generate: an unrecognised
    transform means the set and get sides would disagree, which is worse than
    not supporting the parameter at all.
    """
    start = lisp.index('static lbm_value ext_conf_set(')
    body = lisp[start:lisp.index('\n}\n', start)]
    arms = re.split(r'compare_symbol\(name, &syms_vesc\.([a-z0-9_]+)\)', body)

    plain = re.compile(r'^(?:mcconf|appconf)->([a-z0-9_.\[\]]+)\s*=\s*'
                       r'(?:lbm_dec_as_float|lbm_dec_as_i32|lbm_dec_as_u32)'
                       r'\(args\[1\]\)$')
    negabs = re.compile(r'^(?:mcconf|appconf)->([a-z0-9_.\[\]]+)\s*=\s*'
                        r'-fabsf\(lbm_dec_as_float\(args\[1\]\)\)$')

    out = {}
    for i in range(1, len(arms), 2):
        name = name_of.get(arms[i])
        if name is None:
            continue
        chunk = arms[i + 1].split('compare_symbol')[0]
        assigns = [a.strip() for a in re.findall(
            r'((?:mcconf|appconf)->[a-z0-9_.\[\]]+\s*=\s*[^;]+);', chunk)]

        # 1 = written straight to the live configuration, 2 = needs a full
        # reconfigure through mc_interface_set_configuration/app_set_
        # configuration. 87 of the 143 are 2s, and writing one of those
        # through the fast path makes it appear to be set without taking
        # effect -- so this has to be derived too, not assumed.
        changed = re.findall(r'changed_(?:mc|app)\s*=\s*([12])', chunk)
        apply_needed = bool(changed) and changed[0] == '2'

        if len(assigns) == 1 and plain.match(assigns[0]):
            how = 'plain'
        elif len(assigns) == 1 and negabs.match(assigns[0]):
            how = 'neg_abs'
        else:
            how = assigns[0] if assigns else '(no assignment)'
        out[name] = (how, apply_needed)
    return out


def parse():
    lisp = open(LISP).read()
    dt = open(DATATYPES).read()
    all_structs = structs(dt)

    name_of = {}
    for m in re.finditer(r'comp == &syms_vesc\.([a-z0-9_]+)\)\s*\{\s*'
                         r'lbm_add_symbol_const\("([a-z0-9-]+)"', lisp):
        name_of[m.group(1)] = m.group(2)

    sets = set_transforms(lisp, name_of)

    start = lisp.index('static lbm_value ext_conf_get(')
    body = lisp[start:lisp.index('\n}\n', start)]

    entries = []
    unresolved = []
    arms = re.split(r'compare_symbol\(name, &syms_vesc\.([a-z0-9_]+)\)', body)
    for i in range(1, len(arms), 2):
        sym = arms[i]
        chunk = arms[i + 1].split('compare_symbol')[0]
        name = name_of.get(sym)
        if name is None or name in SPECIAL:
            continue

        mc = sorted(set(re.findall(r'mcconf->([a-z0-9_.\[\]]+)', chunk)))
        app = sorted(set(re.findall(r'appconf->([a-z0-9_.\[\]]+)', chunk)))
        if len(mc) + len(app) != 1:
            unresolved.append((name, mc, app))
            continue

        owner, field = ('CONF_MC', mc[0]) if mc else ('CONF_APP', app[0])
        root = 'mc_configuration' if mc else 'app_configuration'
        # The type is resolved only to prove the field path is real. It is
        # deliberately not recorded: the emitted code names the field, so the
        # compiler picks the width and the signedness. An earlier version
        # stored an offset plus a width classification, and got both wrong --
        # uint8_t and uint16_t fields were read four bytes wide, which is
        # unaligned as well as reading the neighbours, and arm-none-eabi
        # defaults to -fshort-enums so an enum's width is not knowable from
        # the source at all.
        ctype = resolve(field, root, all_structs)
        if ctype is None:
            unresolved.append((name, field, 'type not resolved'))
            continue

        entry = sets.get(name)
        if entry is None:
            unresolved.append((name, field, 'readable but conf-set has no arm'))
            continue
        how, apply_needed = entry
        if how == 'plain':
            flags = ['CONF_PLAIN']
        elif how == 'neg_abs':
            flags = ['CONF_NEG_ABS']
        else:
            unresolved.append((name, field,
                               'unrecognised conf-set transform: %s' % how))
            continue
        if apply_needed:
            flags = [f for f in flags if f != 'CONF_PLAIN'] + ['CONF_APPLY']
        flags = ' | '.join(flags)

        entries.append((name, owner, field, flags))

    entries.sort(key=lambda e: e[0])
    return entries, unresolved


def render(entries):
    mc = [e for e in entries if e[1] == 'CONF_MC']
    app = [e for e in entries if e[1] == 'CONF_APP']

    out = []
    out.append('/*')
    out.append(' * Parameter name to configuration field, for vesc.conf_get and')
    out.append(' * vesc.conf_set.')
    out.append(' *')
    out.append(' * GENERATED by tests/conf_table/gen_conf_table.py from the LispBM')
    out.append(' * extensions, which are the authoritative mapping. Do not edit by hand:')
    out.append(' * tests/conf_table/run.sh regenerates it and fails if this file differs,')
    out.append(' * so a hand edit is reported as a mismatch.')
    out.append(' *')
    out.append(' * Derived rather than retyped because a round-trip test cannot catch a')
    out.append(' * mis-mapping -- set-then-get agrees whether a name is wired to the')
    out.append(' * right field or the one next to it -- and a parameter silently reading')
    out.append(' * the wrong setting is the whole failure mode worth preventing here.')
    out.append(' *')
    out.append(' * X-macro lists naming the struct member, rather than a table of byte')
    out.append(' * offsets and widths. The field name lets the compiler choose the load,')
    out.append(' * which is the only way to get this right: uint8_t and uint16_t members')
    out.append(' * read four bytes wide are both unaligned and overlapping their')
    out.append(' * neighbours, and arm-none-eabi defaults to -fshort-enums, so an enum\'s')
    out.append(' * width cannot be known from the declaration either.')
    out.append(' *')
    out.append(' * Flags, both derived from the conf-set arm:')
    out.append(' *')
    out.append(' *   CONF_NEG_ABS  stored negative, given as a positive magnitude, which')
    out.append(' *                 is what lisp conf-set does (-fabsf). conf_get returns')
    out.append(' *                 the stored value either way, again matching lisp.')
    out.append(' *   CONF_APPLY    needs a full reconfigure rather than a write to the')
    out.append(' *                 live struct. Writing one of these through the fast')
    out.append(' *                 path leaves it looking set without taking effect.')
    out.append(' *')
    out.append(' * Seven parameters are absent because lisp scales or computes them')
    out.append(' * rather than reading a field: the five FOC motor constants and the two')
    out.append(' * speed limits. Those are written out in lua_vesc_conf.c.')
    out.append(' */')
    out.append('')
    out.append('// clang-format off')
    out.append('')
    out.append('#define CONF_MC_PARAMS(X) \\')
    for name, _owner, field, flags in mc:
        out.append('\tX("%s", %s, %s) \\'
                   % (name.replace('-', '_'), flags, field))
    out.append('\t/* end */')
    out.append('')
    out.append('#define CONF_APP_PARAMS(X) \\')
    for name, _owner, field, flags in app:
        out.append('\tX("%s", %s, %s) \\'
                   % (name.replace('-', '_'), flags, field))
    out.append('\t/* end */')
    out.append('')
    out.append('#define CONF_MC_PARAM_COUNT %d' % len(mc))
    out.append('#define CONF_APP_PARAM_COUNT %d' % len(app))
    out.append('')
    out.append('// clang-format on')
    return '\n'.join(out) + '\n'


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else '--check'
    entries, unresolved = parse()

    if unresolved:
        print('gen_conf_table: %d parameter(s) could not be derived:'
              % len(unresolved))
        for u in unresolved:
            print('   ', u)
        return 2

    text = render(entries)

    if mode == '--emit':
        open(HEADER, 'w').write(text)
        print('gen_conf_table: wrote %d entries to %s'
              % (len(entries), os.path.relpath(HEADER, TOP)))
        return 0

    if not os.path.exists(HEADER):
        print('gen_conf_table: %s is missing; run with --emit' % HEADER)
        return 1

    have = open(HEADER).read()
    if have != text:
        print('gen_conf_table: %s does not match the LispBM extensions.'
              % os.path.relpath(HEADER, TOP))
        import difflib
        diff = list(difflib.unified_diff(
            have.split('\n'), text.split('\n'),
            'committed', 'derived', lineterm=''))
        for line in diff[:40]:
            print('  ' + line)
        if len(diff) > 40:
            print('  ... %d more diff lines' % (len(diff) - 40))
        return 1

    print('gen_conf_table: %d entries, identical to the LispBM extensions'
          % len(entries))
    return 0


if __name__ == '__main__':
    sys.exit(main())
