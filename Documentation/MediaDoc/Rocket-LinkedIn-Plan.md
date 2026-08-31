# RocketPower — LinkedIn Posts

4 posts. Post 1-3 are ready now. Post 4 is reserved, write it after the
actual flight (two branches below, use whichever happened). Only the
`[MEDIA: ...]` lines (and the `[DATE]`/`[X]` fields in post 4) are
placeholders.

---

## Post 1

In 2024, I started designing a custom flight computer for a rocket 🚀
Then I didn't touch it again for almost a year.

My capstone project was a drone built from scratch with a great team.
When it ended, I missed the challenge, so I leaned on my background in
rocketry at Concordia and started something of my own: an active
fin-control rocket, custom PCB and flight control code in C++. I got as
far as the MCU section of the schematic before life took over.

Graduation, a trip through Asia, a new job, moving out of my parents'
house. A year passed. I'd open KiCad, feel discouraged by the amount of
work ahead, and close my laptop.

The itch never went away, and I came back to it in 2025. I bought a $200
refurbished 3D printer, a soldering station, and a power supply. I had to
relearn electronics from books, forums, YouTube, and PJRC. I often felt
the need to go on Fiverr and hire an EE for a review of my work, but I
decided to trust myself, and earned that trust through 15 revisions 😆

Then came the layout, the hardest part. Fitting that many sensors, servo
channels, telemetry, camera, GPS, EMI shielding, power conversion, and
pyro channels into a 50mm diameter was the challenge, I wasn't willing to
compromise. I picked a BGA MCU (MIMXRT1062) on purpose, smaller footprint
and more of a headache. A 2-layer board wouldn't work. I went to 4 layers
and rewrote the whole architecture more than once. There were nights I
stared at the ratsnest wondering if I'd picked a form factor that just
didn't work.
Then, at some point, it did.

[MEDIA: photo of the $200 refurb 3D printer / solder station setup or the
early schematic, plus a PCB layout screenshot showing the density]

#engineering #aerospace #pcbdesign #embeddedsystems

---

## Post 2

The preassembled board finally arrived ($400 later). It was pure magic right up until I plugged it in... then crickets. 
Total radio silence: no COM port, no USB handshake, no signs of life.

I'd ordered it through PCBWay, generated the Gerbers while vacationing in Peru, and managed supplier Q&A from 
there (really impressed by their service btw). But one design mistake at that stage meant flushing money and effort down 
the drain. So when it showed up dead, my stomach dropped.

Cue two nights of full-blown engineering existential dread. At one point, I was staring down the barrel of manually 
reworking microscopic BGA pads. But after sleeping on it, I stopped overthinking and audited the schematic line by line.



Turns out, I didn't need surgical BGA mastery. I had accidentally designed the reset button to be hard-wired to ground. 
Basically, I gave my own flight computer a perpetual coma. One quick trace surgical strike with an X-Acto knife later, 
I plugged it in and heard the glorious Windows USB connect chime. I don't think a single sound has ever brought me that much inner peace.

[MEDIA: macro shot of the knife-cut trace on the real board, plus a photo
of the board arriving/unboxing]

#pcbdesign #embeddedsystems #debugging

---

## Post 3

While the board was busy pretending to be dead, the rest of the rocket had
to keep moving in parallel.

The airframe came together inside Fusion 360: carbon fiber rods, a
modular 3D-printed body, and 4.4g servos driving 3D-printed fins. The
mechanical design was entirely self-taught, which included learning the
importance of revision control after corrupted CAD files wiped out hours
of work.

Meanwhile, procurement turned into its own logistical side-quest: tracking
down motors, cameras, LiPos, parachute ejection charges, e-matches,
telemetry radios, and all the tiny hardware that eats up your budget.
During that same stretch, I built the ground station software: one of
those "I'll just fix one quick bug" projects where suddenly it's 3:00 AM,
you're staring into the void, and your terminal is staring back.

One by one, the sensors finally blinked to life (barometer, GPS, IMU),
each one delivering a sweet hit of dopamine. (I did manage to short a
connector and blow a fuse along the way, but hey, at least the protection
circuitry works as advertised.)

The final boss was the 6DOF flight simulation. This was the one area
where I actually felt at home, having modeled vehicle dynamics before. I
dumped the real mass properties from Fusion and the aero coefficients from
OpenRocket straight into a custom Simulink model to tune the fin control
gains before letting the flight computer anywhere near actual hardware.

Now, the entire thing is assembled and standing in front of me. Looking at
a fully functional rocket that started as random ideas and late-night CAD
sketches still doesn't feel completely real.

Next up: finding a suitable launch site and prepping for flight testing.
To be continued.

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
