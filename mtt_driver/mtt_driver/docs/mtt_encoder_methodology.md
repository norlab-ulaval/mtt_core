# Encoder & Gear Ratio Methodology (Clean Mechanical Description)

This document defines the theoretical tick-to-distance relation for the MTT drivetrain, written in terms of the **real mechanical components**. 

---

## 1) Mechanical Reference

On the motor shaft there are two distinct elements:

- A **5-tooth sprocket** driving the reduction gear train  
  (`MTT_SPROCKET_TEETH = 5`)
- A **tachometer plate with 10 slots**, producing **10 ticks per motor revolution**  
  (`MTT_TACHO_TEETH = 10`) **Not in official docs, visually confirmed**

The manufacturer’s C headers list `MTT_Encoder_TEET = 5`, which reflects the sprocket teeth. 
However, in practice the tachometer disc gives **10 ticks per motor revolution**. 
For clarity, both are noted separately here.

Other gears in the chain:

```c
#define MTT_GEAR1    16
#define MTT_GEAR2    36
#define MTT_GEAR3    15
#define MTT_GEAR4    32
#define MTT_GEAR_DRIVE   8
#define MTT_GEAR_TRACK   54
```

Track loop length for reference:

```c
#define MTT_TRACK_LENGTH_CM 393
```

---

## 2) Theoretical Gear Reduction

The overall motor-to-sprocket reduction:

\[
G_{theory} 
= \frac{36}{16} 
\times \frac{32}{15} 
\times \frac{54}{8} 
= \mathbf{32.400000}
\]

This means: one sprocket revolution requires about **32.40 motor revolutions**.

---

## 3) Ticks per Track (Sprocket) Revolution

Since the tachometer disc produces **10 ticks per motor revolution**, the number of ticks per sprocket (track) revolution is:

32.4 motor rev/sprocket rev × 10 ticks/motor rev = 324.00 ticks per sprocket rev

This is the **canonical value** used for speed and distance conversion.

---

## 4) Tick-to-Distance Relation

Let:

- \( C_{pitch} \): sprocket pitch circumference (m), measured on-ground  
- \( G \): theoretical gear reduction (motor rev / sprocket rev)  
- \( T \): ticks per motor rev (here 10)  

Then:

\[
\Delta s_{tick}\,[mm] = 1000 \cdot \frac{C_{pitch}}{G \cdot T}\,.
\]

Equivalently, given a measured ground resolution \( \Delta s_{tick} \):

\[
C_{pitch}\,[m] = \frac{\Delta s_{tick}}{1000} \cdot G \cdot T
\]

---

## 5) Field Verification

1. **Measure one tick on ground** → get \( \Delta s_{tick} \).  
2. **Compute implied sprocket circumference** using formula above.  
3. **Compare with theory**: if result is close, constants are correct.  
   If result is off by ~×2, check if firmware was mistakenly using 5 vs 10 ticks per motor rev.

---

## 6) Reference Code

**Python:**

```python
G_theory = (36/16) * (32/15) * (54/8)

def mm_per_tick(C_pitch_m, G=G_theory, ticks_per_motor_rev=10):
    return 1000.0 * C_pitch_m / (G * ticks_per_motor_rev)

def implied_C_pitch_m(mm_per_tick_val, G=G_theory, ticks_per_motor_rev=10):
    return (mm_per_tick_val / 1000.0) * G * ticks_per_motor_rev
```

---

## 7) Final Values

- Gear reduction: \( G_{theory} = 32.400000 \) (motor rev / sprocket rev)  
- Ticks per motor rev: 10  
- Final ticks per sprocket revolution: **324.00**  

This is the correct reference for driver code and documentation.
