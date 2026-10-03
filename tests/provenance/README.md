# Provenance audit

Who actually wrote the content of the files this fork adds.

```sh
python3 tests/provenance/audit.py . upstream/master
python3 tests/provenance/audit.py ../vesc_tool upstream/master
python3 tests/provenance/audit.py ../vesc_pkg upstream/main
python3 tests/provenance/audit.py ../vesc_express upstream/main
```

It takes a repository path, so one copy serves all four. Needs
`git fetch upstream` first.

## How it works, and why not textual similarity

`git blame -C -C -C` follows lines copied *between* files, so for a file
created by lifting code out of somewhere else it reports the original author
and the file it came from. That is provenance answered from the history that is
already there. The first plan was to compare each new file's lines against the
upstream tree and score the overlap, which would have been a worse version of
something git already does exactly. The three `-C`s matter: one looks within a
commit, three look across the whole history.

The audit then compares that against what the file's header claims, and reports
files where somebody's work is in the content and not in the notice.

## Why it exists

Because that had happened, repeatedly, and nobody noticed. Every time a
function was lifted out of an upstream file to make it testable or shareable,
the new file was created with this fork's copyright header and only this fork's
name on it. The first two found were `util/mc_limits.c` and
`util/imu_freeze.c`; the audit then found eighteen more across four
repositories, including `vesc_tool/appstyle.cpp`, which is 87 lines of Jeffrey
M. Friesen's, Benjamin Vedder's and r3n33's work.

GPL-3 section 5 asks that those notices be kept intact. Beyond the licence, it
is simply not true that we wrote it.

## Two ways it lied at first, both worth knowing

**Identity.** The first run flagged about 1,500 lines in `vesc_pkg` and
`vesc_express` as foreign, because the author of this fork commits under
`siigna`, `Stephen Bouche` and `Steve Bouché` and the tool only knew two of
them. It reported the author as a stranger to his own files. The known-ours set
is at the top of the script and will need extending again.

**Boilerplate.** Every file in these trees opens with the same GPL notice, so
blame attributes those lines to whichever file git decides they were copied
from — which says nothing at all. The filter skips the licence text, short
lines, braces, includes and comment bodies, and only counts findings of three
lines or more. Below that it is almost always one stray line of shared
boilerplate, and the noise buries the real cases.

A third trap was mine rather than the tool's: when first reading the output I
filtered it through `grep '^    [A-Z]'`, which silently dropped `r3n33` —
contributors are not all called Firstname Lastname.

## What it does not do

It finds content moved *within* the history it can see, which means after an
upstream merge. Code retyped from somewhere else entirely, or copied in before
a merge brought the original, is invisible to it. It also says nothing about
whether a notice is well-formed; `reuse lint` is the tool for that, and this
tree does not use SPDX headers consistently enough yet for it to be quiet.
