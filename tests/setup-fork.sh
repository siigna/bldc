#!/usr/bin/env bash
# Copyright 2026 Stephen Bouche
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Local git settings that make merging upstream less painful. Run once per
# clone, in each of the four repositories -- the settings are per-repository
# and cannot be committed.
#
#   ./tests/setup-fork.sh
#
# None of this changes history or touches a remote. It only sets config in the
# repository it is run from, and prints what it set.

set -eu
cd "$(git rev-parse --show-toplevel)"

echo "configuring $(basename "$(pwd)")"

# Records how a conflict was resolved and replays that resolution when the
# same conflict appears again. This fork carries a rebrand across 80 files, so
# the same hunks will conflict on merge after merge; without this each one is
# resolved from scratch and inconsistently.
#
# It is keyed on the conflict's own content, so it cannot misfire on a
# different conflict -- and for the same reason it does nothing for the
# generated signature constants, whose content differs every time. Those are
# handled by .gitattributes instead.
git config rerere.enabled true
git config rerere.autoupdate true
echo "  rerere:            on, resolutions replayed automatically"

# Conflict markers that also show the common ancestor. For a rebrand the
# three-way view is the difference between seeing what upstream changed and
# guessing: with the default style, ours and theirs often differ in two ways
# at once and only the base says which is the rename.
git config merge.conflictStyle zdiff3
echo "  conflict style:    zdiff3, shows the common ancestor"

# The driver .gitattributes names for generated files. `true` means "keep
# ours, report success": for confgenerator.{h,c} that is right because the
# file is regenerated afterwards, and the signature stage in tests/check.sh
# fails if it is not.
git config merge.ours.driver true
echo "  merge=ours driver: registered (see .gitattributes, MERGING.md)"

echo
echo "done. Nothing was committed and no remote was contacted."
