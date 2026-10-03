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

# Optional: restrict to these paths. On a pull request only the files it
# touches are worth auditing, and that turns several minutes into a few
# seconds -- which is the difference between a check that runs on every
# contribution and one that runs weekly and catches things late.
only = set(sys.argv[3:])
ours = {"siigna", "Stephen Bouche", "Steve Bouché", "Steve Bouche"}

# Contributors who commit under more than one git identity. Without this the
# audit reports somebody as uncredited while their other name sits in the
# header, which is noise that hides the real findings.
ALIAS = {
    "JFriesen": "Jeffrey M. Friesen",
    "DovPearX": "DovPear",
}

# Blame attributes uncommitted lines to this; it is not a person.
IGNORE = {"Not Committed Yet"}


def sh(*args):
    return subprocess.run(args, cwd=repo, capture_output=True, text=True).stdout


added = [f for f in sh("git", "diff", "--name-only", "--diff-filter=A",
                       upstream_ref + "...HEAD").splitlines()
         if re.search(r"\.(c|h|cpp|hpp|pri|py|lisp|lua)$", f)]

if only:
    added = [f for f in added if f in only]

    if not added:
        print("no added source files among the paths given; nothing to audit")
        sys.exit(0)

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
            author = ALIAS.get(author, author)
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

    foreign = {a: n for a, n in counts.items()
               if a not in ours and a not in IGNORE}

    if not foreign:
        continue

    # What the file's own header says.
    #
    # The decision is a substring test against the header text, not a parse of
    # it. The first version extracted holders with a Firstname-Lastname regex
    # and then asked whether the author was among them, which failed on
    # "Jeffrey M. Friesen" -- a middle initial is not a surname -- and on
    # "r3n33", which has no capital letter. Both were already credited and both
    # were reported as missing, by the tool whose job is to not make that
    # mistake. The regex survives only to show what a header says.
    head = sh("git", "show", "HEAD:" + f)[:4000]
    declared = [l.strip() for l in head.splitlines() if "Copyright" in l]

    missing = {a: n for a, n in foreign.items()
               if a.split()[-1] not in head}

    missing = {a: n for a, n in missing.items() if n >= 3}

    if missing:
        findings.append((f, missing, files, sorted(declared)))

for f, missing, files, declared in sorted(findings, key=lambda x: -sum(x[1].values())):
    print("%s" % f)
    if declared:
        for d in declared:
            print("    declares: %s" % d)
    else:
        print("    declares: nothing")
    for a, n in sorted(missing.items(), key=lambda x: -x[1]):
        src = ", ".join(sorted(files[a])[:3])
        print("    %-26s %3d lines   from %s" % (a, n, src))
    print()

print("%d file(s) with uncredited content" % len(findings))

# Non-zero on findings, so this can gate a pull request. A reviewer should see
# it as a failure to fix, not a line in a log.
sys.exit(1 if findings else 0)
