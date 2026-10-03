#!/usr/bin/env python3
"""Who wrote the content of the files this fork adds?

git blame -C -C -C follows lines copied from other files, so for a file that
was created by lifting code out of somewhere else it reports the original
author and the file it came from. That is the provenance question, answered by
the history we already have rather than by guessing at textual similarity.

The licence boilerplate is skipped: every file in these trees starts with the
same GPL notice, so blame attributes those lines to whichever file git decides
it was copied from, which says nothing about the code.
"""
import re
import subprocess
import sys

repo = sys.argv[1]
upstream_ref = sys.argv[2]
ours = {"siigna", "Stephen Bouche", "Steve Bouché", "Steve Bouche"}


def sh(*args):
    return subprocess.run(args, cwd=repo, capture_output=True, text=True).stdout


added = [f for f in sh("git", "diff", "--name-only", "--diff-filter=A",
                       upstream_ref + "...HEAD").splitlines()
         if re.search(r"\.(c|h|cpp|hpp|pri|py|lisp|lua)$", f)]

findings = []

for f in added:
    porc = sh("git", "blame", "-C", "-C", "-C", "--line-porcelain", "--", f)
    if not porc:
        continue

    # Walk the porcelain output, pairing each line's author with its content.
    author = None
    counts = {}
    files = {}

    for line in porc.splitlines():
        if line.startswith("author "):
            author = line[len("author "):]
        elif line.startswith("filename "):
            src = line[len("filename "):]
        elif line.startswith("\t"):
            content = line[1:].strip()

            # Skip the shared licence header and trivia: braces, blank lines,
            # includes and comment bodies carry no authorship signal.
            if (len(content) < 25
                    or content.startswith(("*", "/*", "//", "#include", "#ifndef",
                                           "#define", "#endif", ";", "}", "{"))
                    or "Copyright" in content or "GNU General Public" in content
                    or "WITHOUT ANY WARRANTY" in content
                    or "MERCHANTABILITY" in content
                    or "free software" in content
                    or "This file is part of" in content
                    or "along with this program" in content
                    or "Free Software Foundation" in content
                    or "SPDX" in content):
                continue

            counts[author] = counts.get(author, 0) + 1
            files.setdefault(author, set()).add(src)

    foreign = {a: n for a, n in counts.items() if a not in ours}

    if not foreign:
        continue

    # What the file's own header says.
    head = sh("git", "show", "HEAD:" + f)[:2000]
    declared = set(re.findall(r"Copyright[^\n]*?([A-Z][a-zA-Z]+ [A-Z][a-zA-Z]+)", head))

    missing = {a: n for a, n in foreign.items()
               if not any(a.split()[-1] in d for d in declared)}

    missing = {a: n for a, n in missing.items() if n >= 3}

    if missing:
        findings.append((f, missing, files, sorted(declared)))

for f, missing, files, declared in sorted(findings, key=lambda x: -sum(x[1].values())):
    print("%s" % f)
    print("    header credits: %s" % (", ".join(declared) or "nobody"))
    for a, n in sorted(missing.items(), key=lambda x: -x[1]):
        src = ", ".join(sorted(files[a])[:3])
        print("    %-26s %3d lines   from %s" % (a, n, src))
    print()

print("%d file(s) with uncredited content" % len(findings))
