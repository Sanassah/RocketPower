# RocketPower — LinkedIn Posts

4 posts. Post 1-3 are ready now. Post 4 is reserved, write it after the
actual flight (two branches below, use whichever happened). Only the
`[MEDIA: ...]` lines (and the `[DATE]`/`[X]` fields in post 4) are
placeholders.

---

## Post 1

In 2024 I started designing a custom flight computer for a rocket. Then I
didn't touch it again for almost a year.

My capstone project was a drone built from scratch with a great team, and
it was insane. When it ended, I missed it more than I expected to. So I
leaned on my background in rocketry at Concordia and started something of
my own: an active fin-control rocket, custom PCB and all. I got as far as
the MCU section of the schematic, no sensors yet, before life took over.

Graduation, a trip through Asia with friends, a new job, moving out of my
parents' house. All good things. But every few months I'd open that
folder, look at it, and close it again without touching anything.

I came back to it in 2025 because I missed having something that was
entirely mine, my own decisions, my own mistakes, start to finish. Bought
a $200 refurbished 3D printer and a solder station, relearned electronics
and KiCad from YouTube and forums (I'd forgotten almost everything), and
pushed the schematic through ten revisions solo instead of paying someone
to check my work.

Then came the layout, which turned out to be the harder problem. The
rocket's body is 67mm in diameter, and I wanted the board to sit clean and
horizontal inside it. I picked a BGA MCU on purpose, more capable, more of
a real challenge than a basic package, which meant almost no space to
route around it. A 2-layer board didn't work even using both sides. Went
to 4-layer. Still had to fit servo channels, telemetry, a camera, GPS, EMI
shielding, power conversion, sensors, and three pyro channels on one small
board.

Rewrote the whole architecture more than once. There were nights I stared
at the ratsnest wondering if I'd picked a form factor that just didn't
work. Then, at some point, it did.

[MEDIA: photo of the $200 refurb 3D printer / solder station setup or the
early schematic, plus a PCB layout screenshot showing the density]

#engineering #aerospace #pcbdesign #embeddedsystems

---

## Post 2

The board arrived and I got genuinely emotional holding it. Plugged it in:
nothing. No COM port, no device.

I'd ordered it pre-assembled through PCBWay for about $400 CAD, generated
the Gerbers while traveling in Peru, and handled the supplier back and
forth from there. One design mistake at that stage meant losing that
money and hundreds of hours with it. So when it showed up dead, my
stomach dropped.

Two nights of debugging, barely sleeping. At one point I thought I'd found
something that needed pulling the MCU and reworking two microscopic BGA
pads by hand, and I genuinely considered that the whole thing might be
over. I slept on it instead of spiraling, came back, and checked the logic
of every component from scratch.

Found it: the reset button was shorted to ground by design. Cut the trace
with a knife, plugged it back in, and heard Windows recognize a new
device. I don't think I've ever been that relieved over a sound. The two
BGA issues I was afraid of turned out to be non-blocking, fixed properly
in a V2 revision instead.

[MEDIA: macro shot of the knife-cut trace on the real board, plus a photo
of the board arriving/unboxing]

#pcbdesign #embeddedsystems #debugging

---

## Post 3

While the board debugging was happening, everything else on the rocket
had to move in parallel.

The airframe came together in Fusion 360: carbon fiber rods, a mostly
3D-printed body, 4.4g servos on 3D-printed fins, all self-taught,
including the hard way of losing a CAD version I needed and never getting
it back. Procurement turned into its own slog: motor, camera, batteries,
parachute charges, ematches, radios, and all the small stuff nobody warns
you takes just as long to source. I built the ground station software in
the same stretch, the kind of project where you look up and it's 3am.

Sensors came online one at a time, each one a small win: barometer, GPS,
the IMU. I shorted a connector at one point and blew a fuse, exactly what
it's there for.

Last piece was simulation, the one part that didn't feel brand new since
I'd modeled vehicle dynamics before. Fed real mass properties from Fusion
360 and aero numbers from OpenRocket into a Simulink model, and I'm tuning
the PID gains there before any of it touches real hardware.

Everything's built now. Standing next to a finished rocket I designed
myself still doesn't feel entirely real.

Next step: finding a launch site. To be continued.

[MEDIA: Fusion 360 render or assembly photo, plus a Simulink scope
screenshot]

#cad #groundstation #controlsystems #aerospace

---

## Post 4 (write after the actual flight, use whichever branch happened)

**If it flew clean:**

[DATE]. After [X] months of nights, the rocket flew.

Custom flight computer, custom PCB, active fin control, hand-built ground
station, all of it ran like it did on the bench, just [X]m up instead of
on a table.

Apogee at [X]m, recovery [X]m from the pad. Every system worked.

Two years ago this was a notebook sketch. Today it's a flight log.

**If something went wrong:**

[DATE]. The rocket flew today, and it didn't go the way I planned.

[ONE OR TWO SENTENCES: what happened]

I'll tell that story in full, it's more useful than a highlight reel, and
everything else in this project failed once before it worked too.

[MEDIA: launch photo/video, ignition, ascent, recovery, plus a screenshot
of the actual telemetry plot from the flight]

#aerospace #rocketry #embeddedsystems
