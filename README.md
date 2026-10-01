# mpc-vst-riffbox

RiffBox: an electric guitar for the Akai MPC OS plugin host (Force, MPC Live/One/X/Key) that plays
a whole chord per note, through five amp models, with aftertouch and a tempo delay. Built as a
rhythm/riff layer on top of electronic tracks.

The strings are extended Karplus-Strong, the principle of [DISTRHO Kars](https://github.com/DISTRHO/Kars)
(Chris Cannam's karplong), written new: in-tune fractional delay lines that bend, a pick-position
comb, per-string decay, palm mute. `vst/riffbox_vst.cpp` plus the RAT model `vst/rat_core.h`, no dependencies.

| Tab | What it does |
|---|---|
| PLAY | CHORD per note (POWER 5, POWER 8, OCTAVES, MAJOR, MINOR, SUS2, SUS4, DOM7, MIN7, MAJ7, ADD9 - barre voicings on the played root), OCTAVE, STRUM 0-60 ms with STROKE down/up/alternating, PALM MUTE, SUSTAIN, PICK TONE, RELEASE, DOUBLE (a second, detuned and later guitar through its own amp, panned apart). One guitar: a new note's chord takes over the strings. |
| PEDAL | a RAT in front of the amp (the circuit model from `mpc-vst-rat`, `vst/rat_core.h`): OFF, RAT, TURBO RAT (LEDs), GE RAT (germanium), RUETZ RAT (tighter bass); DISTORTION, FILTER, LEVEL. Into CLEAN it is the distortion; into CLASSIC ROCK at low distortion and high level, a boost. |
| AMP | CLEAN, CLASSIC ROCK, BRIT GARAGE, FUZZ, STONER; GAIN, BASS, MID, TREBLE, VOLUME. 2x oversampled clipping, tone stack, cabinet. |
| TOUCH | what channel or poly aftertouch does, each with its own amount: VIBRATO, BEND (up to a whole tone), FEEDBACK (endless sustain that blooms to the octave), WAH (heel to toe), GAIN. |
| DELAY | TIME from the host tempo (1/16, 1/8T, 1/8, 3/16, 1/4, 1/4D, 1/2), FEEDBACK, MIX, PING PONG. |

Also: pitch bend +-2 semitones, mod wheel -> vibrato, sustain pedal, velocity -> attack and brightness.

`vst/test.sh` runs the offline test (tuning, chord thirds, amp order and levels, strum, damping,
aftertouch, delay timing, stereo double, project restore) and a benchmark: on x86 the heaviest case
(6-string chord, double, stoner, delay, aftertouch) takes ~1 % of a core.

Build/deploy workflow: see `sd88me/mpc-vst-plugins`' `docs/PORTING.md`. License: MIT.

## Hinweis

Entwickelt mit Unterstützung von Claude (Anthropic)
