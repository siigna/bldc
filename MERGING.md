# Merging upstream

This is a fork of [vedderb/bldc](https://github.com/vedderb/bldc) that carries
a rebrand and a parameter set of its own. Two things make a merge more than a
`git merge`, and both have a check that catches the mistake.

Run `./tests/setup-fork.sh` once per clone first. It turns on `rerere`, sets
`zdiff3` conflict markers, and registers the merge driver `.gitattributes`
asks for. All three are local settings that cannot be committed.

## The procedure

```sh
git fetch upstream
git merge upstream/master
```

Then, in order:

**1. Port any new configuration parameters into the Tool's XML.** Upstream
adds fields to `mc_configuration` or `app_configuration` in `datatypes.h` and
to its own `res/config/<their version>` in vesc_tool. This fork ships its own
version directory, which upstream never touches, so those fields arrive in the
firmware struct with nothing describing them. `git diff HEAD upstream/master --
datatypes.h` is the list.

**2. Regenerate.** `confgenerator.{h,c}` are written by the Tool from that
XML, and `.gitattributes` keeps our copies through the merge precisely so they
can be regenerated rather than hand-resolved:

```sh
vesc_tool --xmlConfToCode ../vesc_tool/res/config/<version>/parameters_mcconf.xml
vesc_tool --xmlConfToCode ../vesc_tool/res/config/<version>/parameters_appconf.xml
```

**3. Check, in both repositories.** `./tests/check.sh` here recomputes both
signatures from the Tool's XML and fails if the header disagrees — that is the
guard against a half-done merge, and it is why step 2 can be automated at all.
In vesc_tool, `configSignatureIsPinned` holds the expected pair and will fail
with the new numbers; update them in the same commit as the regeneration, not
separately.

## Why a stale signature matters

The signature is a CRC over every parameter's name, type, transmit type and
enum labels, and `confgenerator_deserialize_appconf` rejects the **entire
blob** on a mismatch — not the changed field, the whole configuration, in both
directions, with `Invalid signature` as the only symptom. A firmware and a
Tool that disagree cannot exchange configuration at all.

## What the rebrand costs

Less than expected, so far. A trial merge of 22 upstream commits produced one
conflict, in the generated header above; four upstream commits into vesc_tool
produced none. The display strings this fork renames are mostly in files
upstream is not editing. `rerere` is set up for when that stops being true: it
replays a resolution only when the same conflict reappears, which is the shape
a recurring rebrand conflict has.

It does nothing for the generated constants, whose content differs on every
regeneration — that is what `.gitattributes` is for, and the two are solving
different halves on purpose.

## Upstream arrives without tests

Worth knowing before trusting a merge. Of the 22 commits in the last upstream
batch, one carried a test. New configuration fields, IMU attitude seeding and
CAN frame validation all arrived bare, and the first two are testable in
`tests/` here without hardware.
